#%%
from sympy import zeros, symbols
from sympy.solvers.ode import dsolve, systems
from sympy.physics.mechanics import Body, PinJoint, PrismaticJoint, JointsMethod, inertia
from sympy.physics.mechanics import dynamicsymbols
import sympy as sp
from sympy.physics.mechanics import *
from scipy.integrate import solve_ivp, solve_bvp
import numpy as np
import matplotlib.pyplot as plt
import control as ct
import matplotlib.animation as animation
# from numba import njit
#             ----> x
# ___________III______
#             \   |
#              \ / θ_1
#               \
#                l
#                 \
#                  mb (dist from top is lcom*l)
#                   \
#                    /\
#                   /  \
#                  /    \
#                 /(-θ_2)\
#                l
#               /
#             mc


#%%
# print(np.linalg.matrix_rank(ct.ctrb(A, B))==6) #this should be 6 since we have 6 degrees of freedom


class dipc_model:
    def __init__(self, constants={ # [0.15, 40, 9.8, 0.125, 0.125, 1.74242, 0.04926476]
            'L':  0.15, # length of pend com, in meters
            'ma': 0.125, # mass of cart (kg)
            'mb': 0.125, # mass of pendulum (kg)
            'kE': 0.00474448, # constant for motor force proportional to voltage applied
            'kF': 0.04926477 # friction
        }): 
        self.K = {}
        self.constants = list(constants.values())
        # self.R_a = 0.142301 # armature resistance
        self.R_a = 0.9 # actual measured armature resistance
        self.y = dynamicsymbols('y:4')
        # y1, y3 is cart pos, vel
        # y2, y4 is pend ang, vel
        self.l, self.t = symbols('l, t')
        # l is first pend length
        # lcom is pos of COM from cart pivot joint of first pend (fraction of L)
        # g is gravity (positive)
        # t is time

        self.ma, self.mb = symbols('m_a, m_b')
        # ma is mass of cart
        # mb is mass of first pendulum

        self.kE, self.kF = symbols('k_V, k_W')
        # kE is constant for motor, kF is friction


        self.track = Body('N')
        self.cart = Body('C', mass=self.ma)
        self.top_pend = Body('P_1', mass=self.mb)

        self.slider = PrismaticJoint('slider', self.track, self.cart, coordinates=self.y[0], speeds=self.y[2])
        self.rev1 = PinJoint('r1', self.cart, self.top_pend, coordinates=self.y[1], speeds=self.y[3],
                        child_axis=self.top_pend.z, child_joint_pos=self.l*self.top_pend.y,
                        parent_axis=self.cart.z)

        self.joints = (self.slider, self.rev1)
        self.bodies = (self.track, self.cart, self.top_pend)

        self.F = dynamicsymbols('F')
        #                                    k_E        *          I                 - friction
        #                              k_E    *   (V_actual        /      R_armature)   - friction
        #                         ((k_E * (V_applied - k_E   *   w))   /  R_armature) - k_f  *   w
        self.cart.apply_force(((16*sp.pi/0.06)*(self.kE*(12*self.F - self.kE * self.y[2])/self.R_a) - self.kF * self.y[2])*self.cart.x)

        # gravity
        # self.cart.apply_force(-self.track.y*self.cart.mass*self.g)
        self.top_pend.apply_force(-self.track.y*self.top_pend.mass*9.8)

        # get equations of motion
        self.method = JointsMethod(self.track, self.slider, self.rev1)
        self.method.form_eoms()
        self.ydot = self.method.rhs()
        self.A_sym, self.B_sym = self.method.method.to_linearizer().linearize(A_and_B=True, simplify=True)
    def set_constants(self, new_constants):
        self.constants = new_constants
        return self
    def linearize(self, op_point=[0,sp.pi,0,0]):
        op_point=dict(zip(
            self.y+[
               self.F,
               self.l,
               self.ma,
               self.mb,
               self.kE,
               self.kF,
            ], 
            op_point+[0]+list(self.constants)))
        self.A, self.B = msubs(self.A_sym, op_point), msubs(self.B_sym, op_point)
        self.A, self.B = np.array(self.A).astype(np.float64), np.array(self.B).astype(np.float64)
        return self
    def lambdify(self):
        self.func = sp.lambdify([self.y, self.F, self.l, self.ma, self.mb, self.kE, self.kF], self.ydot, 'numpy')
        # self.func = sp.lambdify([self.y, self.F, self.lb, self.lc, self.lcom, self.c, self.g, self.ma, self.mb, self.mc, self.IBzz, self.ICzz], self.ydot, 'numpy')
        return self
    def construct_PP(self, eigs):
        self.K['PP'] = ct.place(self.A, self.B, eigs)
        return self
    def construct_LQR(self, Q, R):
        self.K['LQR'], _, _ = ct.lqr(self.A, self.B, Q, R)
        return self
    def get_eigs(self, controller):
        return np.linalg.eig(self.A - self.B@self.K[controller])
    def integrate_with_scipy(self, y_0 = [0, (7/8)*np.pi, 0, 0], 
                            targets = np.array([[  0, np.pi, 0, 0],
                                                [0.5, np.pi, 0, 0]]),
                            target_timestamps=[0, 5, 10], 
                            constants = None,
                            tspan=15,
                            framerate=60,
                            lag_seconds=0.25,
                            controller='LQR',
                            t_eval = None,
                            remember_forces = False):
        if constants is None: constants = self.constants
        lag_buffer = [y_0]*int(framerate*lag_seconds)
        forces = []
        # if controller == 'FFF':
        #     ctrl = lambda t, e: eval_fourier(t, dipc_model.fourier_weights)
        # elif controller == 'FFBF':
        #     ctrl = lambda t, e: dipc_model.ff_path[int(t*framerate)]
        # else:
        #     ctrl = lambda t, e: (-self.K[controller]@e)[0]
        if isinstance(controller, str): 
            ctrl = lambda t, e: (-self.K[controller]@e)[0]
        else: 
            ctrl = lambda t, dx: controller(t)

        # def func_for_scipy(t, x):
        #     lag_buffer.append(x)
        #     for counter, i in enumerate(target_timestamps[1:]):
        #         if t<i:
        #             forces.append([t, ctrl(t, lag_buffer.pop(0)-targets[counter])])
        #             break
        #         if i>target_timestamps[-1]: 
        #             forces.append([t, 0]) # stop control.
        #             lag_buffer.pop(0)
        #     return self.func(x, forces[-1][1], *self.constants[:-2]).flatten() # forces[-1][1]
        
        def func_for_scipy(t, x):
            if remember_forces:
                # ((16*sp.pi/0.06)*(self.kE*(12*self.F - self.kE * self.y[2])/self.R_a) - self.kF * self.y[2])*self.cart.x
                forces.append([t, ctrl(t, targets[0] - x), (16*np.pi/0.06)*(constants[-2]*(12*ctrl(t, x[2]) - constants[-2]*x[2])/self.R_a) - constants[-1]*((1/8)*sp.asinh(10*x[2]))])# + 2*x[2]])
                return self.func(x, forces[-1][1], *constants).flatten()
            else:
                return self.func(x, ctrl(t, x), *constants).flatten()
        soln = solve_ivp(func_for_scipy, (0, tspan), y_0, dense_output=True, t_eval = t_eval if not (t_eval is None) else np.arange(0, tspan, 1/framerate))
        if remember_forces:
            forces.sort(key = lambda x: x[0])
            self.soln, self.soln_forces = soln.sol, np.array(forces)
        else:
            self.soln = soln.sol
        return self
    def plot_graph(self):
        plt.plot(self.soln_t, self.soln_y[0], label='$x$ (m)')
        plt.plot(self.soln_t, self.soln_y[2], label='$\dot{x}$ (m/s)')
        plt.plot(self.soln_t, self.soln_y[1], label='$\\theta_1$ (rad)')
        plt.plot(self.soln_t, self.soln_y[3], label='$\dot\\theta_1$ (rad/s)')
        plt.plot(self.soln_forces[:, 0], self.soln_forces[:, 1], label='$u$ (N)')
        plt.xlabel('Time (s)')
        plt.legend()
    def plot_animation(self, colors, times):
        num_frames = len(self.soln_y[0])
        fig, ax = plt.subplots()
        mat, = ax.plot(*self.get_xy(self.soln_y[0, 0], self.soln_y[1, 0], self.constants[0]), marker='o')
        ax.axhline(color='tab:blue')
        time_label = ax.text(0, -0.6, '0')
        num_frames = len(self.soln_y[0])
        def animate(i):
            mat.set_data(self.get_xy(*(self.soln_y[(0, 1), i%(num_frames+30)-30 if i%(num_frames+30) > 29 else 0]), self.constants[0]))
            time_label.set_text(f't={round((i%(num_frames+30)-30)/60, 4)}')
            for counter, t in enumerate(times[1:]):
                if t*60 - 10 < i < t*60 + 10:
                    mat.set_color(colors[counter+1])
                    break
            return mat
        ax.axis([-0.5,1.5,-0.75,0.75])
        self.anim = animation.FuncAnimation(fig, animate, interval=(self.soln_t[1]-self.soln_t[0])*1000, frames=num_frames+30)
        plt.show()
    def show_plots(self, block = True):
        plt.show(block = block)
    @classmethod
    def rot(self, th):
        return np.array([
            [np.cos(th), -np.sin(th)],
            [np.sin(th),  np.cos(th)]
        ])
    def get_xy(self, x, th1, l1):
        cart = np.array([[x], [0]])
        pend_1 = cart + dipc_model.rot(th1)@np.array([[0],[-l1]])
        return np.concatenate([cart, pend_1], axis=1)   
    def print_eoms(self):
        dx, dth1, dth2, x, th1, th2 = dynamicsymbols('\dot{x}, \dot\\theta_1, \dot\\theta_2, x, \\theta_1, \\theta_2')
        print("""
        \\begin{bmatrix}
        \dot x \\
        \dot \\theta_1 \\
        \ddot x \\
        \ddot \\theta_1 \\
        \end{bmatrix} = 
        """ + sp.latex(self.ydot.subs([
            (self.y[0].diff(self.t), dx),
            (self.y[1].diff(self.t), dth1),
            
            (self.y[0], x),
            (self.y[1], th1),
            (self.y[2], dx),
            (self.y[3], dth1),
        ])))
#%%
if __name__ == '__main__':
    eigs = np.array([
        [-3.50], 
        [-3.51],
        [-3.52],
        [-3.53],
    ])

    Q = np.diag([100, 10, 1, 100])
    R = np.diag([10])
    model = dipc_model().linearize().lambdify()
    print(repr(model.A), '\n\n', repr(model.B))
    model.construct_PP(eigs).construct_LQR(Q, R)
    print(model.get_eigs('PP')[0])
    print(model.B@model.K['PP'])
    # print(model.K['PP'])
    model.integrate_with_scipy(
        y_0 = [0, (7/8)*np.pi, 0, 0], 
        targets = np.array([[  0, np.pi, 0, 0],
                            [0.5, np.pi, 0, 0]]),
                            controller = 'PP', lag_seconds=0.0165)
    
    print(model.soln_y[:, :10].T)
    # model.print_eoms()
    # normal = model.ydot.subs(zip([model.lb, model.c, model.g, model.ma, model.mb, model.IBzz], model.constants))
    # print(normal)
    # print(systems.dsolve_system([sp.Eq(model.y[i].diff(model.t), normal[i]) for i in range(6)], 
    # ics=dict(zip([i.subs(model.t, 0) for i in model.y]+[i.subs(model.t, 1.5) for i in model.y],[0, 0, 0, 0, 0, 0]+[0,sp.pi,0,0,0,0]))))
    # print(systems.dsolve_system([sp.Eq(model.y[i].diff(model.t), normal[i]) for i in range(4)]))
    #%%
    model.plot_graph()
    model.plot_animation(['tab:blue', 'tab:orange', 'tab:blue'], [0, 5, 10])
    model.show_plots()