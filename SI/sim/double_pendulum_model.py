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


# func = sp.lambdify([y, F, lb, lc, lcom, c, g, ma, mb, mc, IBzz, ICzz], ydot)
#%%
# @njit
def eval_fourier(t, weights):
    return np.array([1]+[np.cos(n*t) for n in range(1, 50)]+[np.sin(n*t) for n in range(1, 50)])@weights
class dipc_model:
       #                                                            0.084            0.0007621
    def __init__(self, constants=[0.3, 0.2,   0.67,   1, 9.81, 0.125, 0.1, 0.07, 0.0035, 0.002]): #0.333333333*0.070348348*(0.292^2)
        self.K = {}
        self.constants = constants
        self.y = dynamicsymbols('y:6')
        # y1, y4 is cart pos, vel
        # y2, y5 is cart's pivot joint pos, vel
        # y3, y6 is mid-pendulum joint
        self.lb, self.lc, self.lcom, self.c, self.g, self.t= symbols('l_b, l_c, l_{com}, c, g, t')
        # l is first pend length
        # lcom is pos of COM from cart pivot joint of first pend (fraction of L)
        # c is friction of cart
        # g is gravity (positive)
        # t is time

        self.ma, self.mb, self.mc, self.IBzz, self.ICzz= symbols('m_a, m_b, m_c, I_{Bzz}, I_{Czz}')
        # ma is mass of cart
        # mb is mass of first pendulum
        # mc is mass of end pendulum
        # IBzz is the moment of inertia of the first pendulum
        # moment of inertia of end pendulum is assumed to just come from mass at a radius

        self.track = Body('N')
        self.cart = Body('C', mass=self.ma)
        self.IB = inertia(self.cart.frame, 0, 0, self.IBzz)
        self.IC = inertia(self.cart.frame, 0, 0, self.ICzz)
        self.top_pend = Body('P_1', mass=self.mb, central_inertia=self.IB)
        self.end_pend = Body('P_2', mass=self.mc, central_inertia=self.IC)

        self.slider = PrismaticJoint('slider', self.track, self.cart, coordinates=self.y[0], speeds=self.y[3])
        self.rev1 = PinJoint('r1', self.cart, self.top_pend, coordinates=self.y[1], speeds=self.y[4],
                        child_axis=self.top_pend.z, child_joint_pos=self.lb*self.lcom*self.top_pend.y,
                        parent_axis=self.cart.z)

        self.rev2 = PinJoint('r2', self.top_pend, self.end_pend, coordinates=self.y[2], speeds=self.y[5],
                        child_axis=self.end_pend.z, parent_joint_pos=-self.lb*(1-self.lcom)*self.top_pend.y,
                        parent_axis=self.top_pend.z, child_joint_pos=self.lc*self.end_pend.y)

        self.joints = (self.slider, self.rev1, self.rev2)
        self.bodies = (self.track, self.cart, self.top_pend, self.end_pend)

        self.F = dynamicsymbols('F')
        self.cart.apply_force(self.F*self.cart.x) # motor
        # self.cart.apply_force(-self.c*self.y[3]*self.cart.x, reaction_body=self.track) # friction

        # gravity
        self.cart.apply_force(-self.track.y*self.cart.mass*self.g)
        self.top_pend.apply_force(-self.track.y*self.top_pend.mass*self.g)
        self.end_pend.apply_force(-self.track.y*self.end_pend.mass*self.g)

        # get equations of motion
        self.method = JointsMethod(self.track, self.slider, self.rev1, self.rev2)
        self.method.form_eoms()
        self.ydot = self.method.rhs()

        a = sp.Symbol('a')
        print('ydot[3]:')
        print(sp.Eq(self.ydot[3], a))
        print(sp.solveset(sp.Eq(self.ydot[3], a), self.F, sp.S.Reals), open("sp_soln.txt", "w"))
    def linearize(self, op_point=[0,sp.pi,0,0,0,0]):
        op_point=dict(zip(
            self.y+[self.F,
               self.lb,
               self.lc,
               self.lcom,
               self.c,
               self.g,
               self.ma,
               self.mb,
               self.mc,
               self.IBzz,
               self.ICzz], 
            op_point+[0]+self.constants))
        self.A, self.B = self.method.method.to_linearizer().linearize(A_and_B=True, op_point=op_point)
        self.A, self.B = np.array(self.A).astype(np.float64), np.array(self.B).astype(np.float64)
        return self
    def lambdify(self):
        self.func = sp.lambdify([self.y, self.F, self.lb, self.lc, self.lcom, self.c, self.g, self.ma, self.mb, self.mc, self.IBzz, self.ICzz], self.ydot, 'numpy')
        return self
    def construct_PP(self, eigs):
        self.K['PP'] = ct.place(self.A, self.B, eigs)
        return self
    def construct_LQR(self, Q, R):
        self.K['LQR'], _, _ = ct.lqr(self.A, self.B, Q, R)
        return self
    def get_eigs(self, controller):
        return np.linalg.eig(self.A - self.B@self.K[controller])
    def integrate_with_scipy(self, y_0 = [0, (7/8)*np.pi, 0, 0, 0, 0], 
                            targets = np.array([[  0, np.pi, 0, 0, 0, 0],
                                                [0.5, np.pi, 0, 0, 0, 0]]),
                            target_timestamps=[0, 5, 10], 
                            tspan=15,
                            framerate=60,
                            lag_seconds=0.25,
                            controller='LQR',):

        lag_buffer = [y_0]*int(framerate*lag_seconds)
        forces = []
        if controller == 'FFF':
            ctrl = lambda t, e: eval_fourier(t, dipc_model.fourier_weights)
        elif controller == 'FFBF':
            ctrl = lambda t, e: dipc_model.ff_path[int(t*framerate)]
        else:
            ctrl = lambda t, e: (-self.K[controller]@e)[0]

        def func_for_scipy(t, x):
            lag_buffer.append(x)
            for counter, i in enumerate(target_timestamps[1:]):
                if t<i:
                    forces.append([t, ctrl(t, lag_buffer.pop(0)-targets[counter])])
                    break
                if i>target_timestamps[-1]:
                    forces.append([t, 0]) # stop control.
                    lag_buffer.pop(0)
            return self.func(x, forces[-1][1], *self.constants).flatten()

        soln = solve_ivp(func_for_scipy, (0, tspan), y_0, t_eval = np.arange(0, tspan, 1/framerate))
        forces.sort(key = lambda x: x[0])
        self.soln_t, self.soln_y, self.soln_forces = soln.t, soln.y, np.array(forces)
        return self
    def plot_graph(self):
        plt.plot(self.soln_t, self.soln_y[0], label='$x$ (m)')
        plt.plot(self.soln_t, self.soln_y[3], label='$\dot{x}$ (m/s)')
        plt.plot(self.soln_t, self.soln_y[1], label='$\\theta_1$ (rad)')
        plt.plot(self.soln_t, self.soln_y[4], label='$\dot\\theta_1$ (rad/s)')
        plt.plot(self.soln_t, self.soln_y[2], label='$\\theta_2$ (rad)')
        plt.plot(self.soln_t, self.soln_y[5], label='$\dot\\theta_2$ (rad/s)')
        plt.plot(self.soln_forces[:, 0], self.soln_forces[:, 1], label='$u$ (N)')
        plt.xlabel('Time (s)')
        plt.legend()
    def plot_animation(self, colors, times):
        num_frames = len(self.soln_y[0])
        fig, ax = plt.subplots()
        mat, = ax.plot(*self.get_xy(*self.soln_y[0:3, 0], self.constants[0], self.constants[1]), marker='o')
        num_frames = len(self.soln_y[0])
        def animate(i):
            mat.set_data(self.get_xy(*self.soln_y[0:3, i%(num_frames+30)-30 if i%(num_frames+30) > 29 else 0], self.constants[0], self.constants[1]))
            for counter, t in enumerate(times[1:]):
                if t*60 - 10 < i < t*60 + 10:
                    mat.set_color(colors[counter+1])
                    break
            return mat
        ax.axis([-0.5,1.5,-0.75,0.75])
        self.anim = animation.FuncAnimation(fig, animate, interval=(self.soln_t[1]-self.soln_t[0])*1000, frames=num_frames+30)
        plt.show()
    def show_plots(self):
        plt.show()
    @classmethod
    def rot(self, th):
        return np.array([
            [np.cos(th), -np.sin(th)],
            [np.sin(th),  np.cos(th)]
        ])
    def get_xy(self, x, th1, th2, l1, l2):
        cart = np.array([[x], [0]])
        pend_1 = cart + dipc_model.rot(th1)@np.array([[0],[-l1]])
        pend_2 = pend_1 + dipc_model.rot(th1+th2)@np.array([[0], [-l2]])
        return np.concatenate([cart, pend_1, pend_2], axis=1)   
    def print_eoms(self):
        dx, dth1, dth2, x, th1, th2 = dynamicsymbols('\dot{x}, \dot\\theta_1, \dot\\theta_2, x, \\theta_1, \\theta_2')
        print("""
        \\begin{bmatrix}
        \dot x \\
        \ddot x \\
        \dot \\theta_1 \\
        \ddot \\theta_1 \\
        \dot \\theta_2 \\
        \ddot \\theta_2
        \end{bmatrix} = 
        """ + sp.latex(self.ydot.subs([
            (self.y[0].diff(self.t), dx),
            (self.y[1].diff(self.t), dth1),
            (self.y[2].diff(self.t), dth2),
            
            (self.y[0], x),
            (self.y[1], th1),
            (self.y[2], th2),
            (self.y[3], dx),
            (self.y[4], dth1),
            (self.y[5], dth2),
        ])))
#%%
if __name__ == '__main__':
    eigs = np.array([
        [-3.50], 
        [-3.51],
        [-3.52],
        [-3.53],
        [-3.54],
        [-3.55],
    ])

    Q = np.diag([100, 10, 5, 1, 100, 50])
    R = np.diag([10])
    model = dipc_model().linearize(op_point=[0,sp.pi,sp.pi,0,0,0]).lambdify()
    # exit()
    model.construct_PP(eigs).construct_LQR(Q, R)
    print(model.K)
    print(sp.latex(model.ydot))
    exit()
    model.integrate_with_scipy(y_0 = [0,0,0,0,0,0], targets = np.array([[  0, np.pi, 0, 0, 0, 0]]), target_timestamps = [0, 1.5], controller = 'FFBF')
    # model.print_eoms()
    print("ydot:")
    print(model.ydot)
    print("f:")
    a = sp.Symbol('a')
    print(sp.solve(sp.Eq(model.ydot[3], a), model.F, simplify=False))
    # print(sp.simplify(sp.trigsimp(sp.expand(sp.solve(model.ydot[3], model.F)))))

    # print(sp.simplify(sp.trigsimp(sp.expand(model.ydot))))
    normal = model.ydot.subs(zip([model.lb, model.lc, model.lcom, model.c, model.g, model.ma, model.mb, model.mc, model.IBzz, model.ICzz], model.constants))
    # print(normal)
    # print(systems.dsolve_system([sp.Eq(model.y[i].diff(model.t), normal[i]) for i in range(6)], 
    # ics=dict(zip([i.subs(model.t, 0) for i in model.y]+[i.subs(model.t, 1.5) for i in model.y],[0, 0, 0, 0, 0, 0]+[0,sp.pi,0,0,0,0]))))
    print(systems.dsolve_system([sp.Eq(model.y[i].diff(model.t), normal[i]) for i in range(6)]))
    #%%
    model.plot_graph()
    model.plot_animation(['tab:blue', 'tab:orange', 'tab:blue'], [0, 5, 10])
    model.show_plots()