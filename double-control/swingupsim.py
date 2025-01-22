import json
import numpy as np
import casadi as ca
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import control as ct
from dynamics import to_casadi
from eoms import get_eoms
from all_settings import HybridSimulatorSettings


class SwingUpController:
    def __init__(self, model_params, eoms, horizon, dt, A_MAX):
        self.params = model_params
        self.p = list(model_params.values())
        self.eoms = eoms
        self.horizon = horizon
        self.dt = dt
        if ca.__version__ == '3.5.5':
            self._eoms_silly = {
                'x': self.eoms['x'],
                'p': self.eoms['u'],
                'ode': ca.substitute(self.eoms['ode'], self.eoms['p'], list(model_params.values())),
            }
            self.intfunc_bad = ca.integrator('integrator', 'rk', self._eoms_silly, {'t0': 0.0, 'tf': dt})
            self.intfunc = lambda x0, u, p: self.intfunc_bad(x0=x0, p=u)
        else:
            self.intfunc = ca.integrator('integrator', 'rk', self.eoms, 0.0, dt)
        self.A_MAX = A_MAX

    @staticmethod
    def normalize_angle(angle):
        return ca.fmod(angle + ca.pi, 2 * ca.pi) - ca.pi

    def solve_full_trajectory(self, x0, target, tf, filename="trajectory.json"):
        x = self.eoms['x']
        u = self.eoms['u']
        ode = self.eoms['ode']
        params = self.eoms['p']

        Q = np.diag([100, 10, 100, 10, 100, 10])
        R = np.diag([10])
        Q_f = Q * 10
        total_steps = int(tf / self.dt)

        opti = ca.Opti()
        X = opti.variable(x.shape[0], total_steps + 1)
        U = opti.variable(u.shape[0], total_steps)
        opti.subject_to(X[:, 0] == x0)
        opti.subject_to(X[:, -1] == target)

        for t in range(total_steps):
            next_state = self.intfunc(x0=X[:, t], u=U[:, t], p=self.p)['xf']
            opti.subject_to(X[:, t + 1] == next_state)

        #* additional bounds to make this reasonable
        #* uncomment if you'd like; these are the real system's bounds
        opti.subject_to(opti.bounded(-0.8, X[0, :], 0.8))
        opti.subject_to(opti.bounded(-5.0, X[1, :], 5.0))

        opti.subject_to(opti.bounded(-self.A_MAX, U, self.A_MAX))
        cost = 0
        for t in range(total_steps):
            #* testing energy-based costs for the swing up.
            #* they seem converge a bit quicker but I couldn't get it fully there
            #* takes too long to iterate when each solve is like 3 years
            # q1, dq1, q2, dq2, dx = X[2, t], X[3, t], X[4, t], X[5, t], X[1, t]

            # Lin. KE of joint 1
            # cost += 0.5*self.params.M_1*((self.params.L_1*ca.sin(q1)*dq1)**2 + (dx+self.params.L_1*ca.cos(q1)*dq1)**2)
            # Lin. KE of joint 2
            # cost += 0.5*self.params.M_2*((self.params.L_PEND*ca.sin(q1)*dq1 + self.params.L_2*(dq1+dq2)*ca.sin(dq1+dq2))**2 + (dx+self.params.L_PEND*ca.cos(q1)*dq1 + self.params.L_2*(dq1+dq2)*ca.cos(q1+q2))**2)
            # RKE
            # cost += 0.5*(self.params.I_1*dq1**2 + self.params.I_2*(dq1+dq2)**2)
            # PE
            # cost -= 100*9.81*(self.params.L_1*self.params.M_1*(1-ca.cos(q1))+self.params.M_2*(self.params.L_2*(1-ca.cos(q1+q2)) + self.params.L_PEND*(1-ca.cos(q1))))
            # KE = (m1 * (dx1[0]**2 + dx1[1]**2))/2      +     (m2 * (dx2[0]**2 + dx2[1]**2))/2
            # RKE = (I1 * diff(q1, t)**2)/2     +     (I2 * diff(q1+q2, t)**2)/2
            # PE = m1 * g * x1[1]     +     m2 * g * x2[1]
            deviation = X[:, t] - target
            deviation[2] = self.normalize_angle(deviation[2])
            deviation[4] = self.normalize_angle(deviation[4])
            cost += ca.mtimes([deviation.T, Q, deviation]) + ca.mtimes([U[:, t].T, R, U[:, t]])
        final_deviation = X[:, -1] - target
        final_deviation[2] = self.normalize_angle(final_deviation[2])
        final_deviation[4] = self.normalize_angle(final_deviation[4])
        # opti.subject_to(ca.sumsqr(final_deviation)==0)
        cost += ca.mtimes([final_deviation.T, Q_f, final_deviation])
        opti.minimize(cost)
        opti.solver('ipopt', dict(), dict(linear_solver='ma57'))
        sol = opti.solve()

        trajectory = np.array(sol.value(X))
        controls = np.array(sol.value(U))

        with open(filename, 'w') as f:
            json.dump({'trajectory': trajectory.tolist(), 'controls': controls.tolist()}, f)

        return trajectory, controls

    def load_trajectory(self, filename="trajectory.json"):
        with open(filename, 'r') as f:
            data = json.load(f)
        trajectory = np.array(data['trajectory'])
        controls = np.array(data['controls'])
        return trajectory, controls
    def make_mpc_controller(self, mpc, filename="trajectory.json", target=[0.0, 0.0, np.pi, 0.0, 0.0, 0.0, 0.0]):
        x_traj, u_traj = self.load_trajectory(filename)
        self.x_traj, self.u_traj = x_traj, u_traj
        print('<here>')
        print(x_traj.shape, u_traj.shape)
        print('</here>')
        u_traj = np.array([u_traj.flatten().tolist() + [0.0]]) # screw efficiency. this only happens once
        x_traj = np.concatenate([x_traj, u_traj], axis=0)
        u_traj = u_traj.flatten()
        print("SHAPE:", x_traj.shape)
        default_traj = np.array([list(target)]*mpc.N)

        mult = int(mpc.dt/self.dt)

        P_traj = []
        for xf, uf in zip(self.x_traj.T, self.u_traj.T):
            _, P, _ = ct.lqr(mpc.A_func(xf, uf), mpc.B_func(xf, uf), mpc.Q, mpc.R)
            P_traj.append(P)
        P_traj.append(mpc.P)
        def controller(t, x):
            t_idx = int(t/self.dt)
            if t_idx>=len(u_traj): 
                print("DEFAULT TRAJ")
                traj = default_traj
            elif t_idx+mpc.N*mult > len(u_traj):
                print("HERE!!!")
                traj = x_traj[:, t_idx::mult]
                traj = np.concatenate([traj.T, default_traj[:(mpc.N-traj.shape[1])]], axis=0)
            else:
                traj = x_traj[:, t_idx:(t_idx + mpc.N*mult):mult].T
            
            P = P_traj[t_idx+mpc.N*mult] if t_idx+mpc.N*mult < len(P_traj) else None
            # if t>1.5: traj = default_traj
            # return u_traj[t_idx]
            return mpc.solve(np.array(x), traj, P)
        return controller
    def make_controller(self, filename="trajectory.json"):
        x_traj, u_traj = self.load_trajectory(filename)
        # print(x_traj.shape, u_traj.shape)
        follower = TrajectoryFollower(self.eoms, self.p, Q=np.diag([100, 10, 100, 10, 100, 10]), R=np.diag([10]))
        def controller(t, x):
            print(t)
            t_idx = int(t/self.dt)
            if t_idx>=len(u_traj): t_idx=len(u_traj)-1
            return follower.track(x, x_traj[:, t_idx], u_traj[t_idx])
        return controller

class HybridSimulator:
    def __init__(self, model_params, dt, horizon, integration_method, A_MAX):
        self.dt = dt
        self.horizon = horizon
        self.eoms = to_casadi(get_eoms())
        self.intfunc = ca.integrator('integrator', integration_method, self.eoms, 0, dt)
        self.model_params = model_params
        self.p = list(self.model_params.values())
        self.A_MAX = A_MAX

    def run(self, x0, target, tf, trajectory_file="trajectory.json"):
        controller = SwingUpController(self.model_params, self.eoms, horizon=10, dt=self.dt, A_MAX=10)
        trajectory, controls = controller.load_trajectory(trajectory_file)
        follower = TrajectoryFollower(self.eoms, self.p, Q=np.diag([100, 10, 100, 10, 100, 10]), R=np.diag([10]))

        self.x = [ca.DM(x0)]
        self.u = []
        for t in range(trajectory.shape[1] - 1):
            current_state = self.x[-1].full().flatten()
            x_target = trajectory[:, t + 1]
            u_target = controls[t] if t < controls.shape[0] else 0
            control_input = follower.track(current_state, x_target, u_target)
            control_input = np.clip(control_input, -self.A_MAX, self.A_MAX)
            self.u.append(control_input)
            next_state = self.intfunc(x0=current_state, u=control_input, p=self.p)['xf']
            self.x.append(next_state)

    def plot(self):
        x = np.array(ca.horzcat(*self.x))
        u = np.array(ca.horzcat(*self.u))

        print(x.shape, u.shape)

        num_frames = len(u[0])
        fig, ax = plt.subplots()
        mat, = ax.plot(*self.get_xy(*x[::2, 0], self.p[0], self.p[1]), marker='o')
        times = np.arange(0, num_frames * self.dt, self.dt)
        assert len(times) == num_frames

        ax.axhline(0, linestyle='dashed')

        frame_mult = ((1 / 30) / self.dt)

        def animate(i):
            mat.set_data(self.get_xy(*x[::2, int(i * frame_mult) % num_frames], self.p[0], self.p[1]))
            return mat

        ax.axis([-1, 1, -0.75, 0.75])
        self.anim = animation.FuncAnimation(fig, animate, interval=1000 / 30, frames=num_frames)
        plt.show()

    def get_xy(self, x, th1, th2, l1, l2):
        cart = np.array([[x], [0]])
        pend_1 = cart + self.rot(th1) @ np.array([[0], [-l1]])
        pend_2 = pend_1 + self.rot(th1 + th2) @ np.array([[0], [-l2]])
        return np.concatenate([cart, pend_1, pend_2], axis=1)

    @staticmethod
    def rot(th):
        return np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])


class TrajectoryFollower:
    def __init__(self, eoms, p, Q, R):
        self.eoms = eoms
        self.p = p
        self.Q = Q
        self.R = R

    def compute_lqr_gain(self, x_target, u_target):
        A = np.array(ca.Function('A', [self.eoms['x'], self.eoms['u'], self.eoms['p']],
                                 [ca.jacobian(self.eoms['ode'], self.eoms['x'])])(x_target, u_target, self.p))
        B = np.array(ca.Function('B', [self.eoms['x'], self.eoms['u'], self.eoms['p']],
                                 [ca.jacobian(self.eoms['ode'], self.eoms['u'])])(x_target, u_target, self.p))
        K, _, _ = ct.lqr(A, B, self.Q, self.R)
        return K

    def track(self, x_current, x_target, u_target):
        K = self.compute_lqr_gain(x_target, u_target)
        u_feedback = -K @ (x_current - x_target)
        return u_feedback + u_target


if __name__ == '__main__':
    sim = HybridSimulator(
        **HybridSimulatorSettings.withopt(horizon=10)
    )
    x0 = [0, 0, 0, 0, 0, 0]
    target = [0, 0, np.pi, 0, 0, 0]
    sim.run(x0, target, tf=5.0, trajectory_file="trajectory.json")
    sim.plot()