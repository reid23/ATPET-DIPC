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
                               # [lb,  c,              g,   ma,    mb,    IBzz] c=23.08014302
    def __init__(self, constants=[0.3, 40.0645    , 9.80, 0.25, 0.125, 0.00065]):
        self.K = {}
        self.constants = constants
        self.y = dynamicsymbols('y:4')
        # y1, y3 is cart pos, vel
        # y2, y4 is pend ang, vel
        self.lb, self.c, self.g, self.t = symbols('l_b, c, g, t')
        # l is first pend length
        # lcom is pos of COM from cart pivot joint of first pend (fraction of L)
        # c is friction of cart
        # g is gravity (positive)
        # t is time

        self.ma, self.mb, self.IBzz = symbols('m_a, m_b, I_{Bzz}')
        # ma is mass of cart
        # mb is mass of first pendulum
        # mc is mass of end pendulum
        # IBzz is the moment of inertia of the first pendulum
        # moment of inertia of end pendulum is assumed to just come from mass at a radius

        self.track = Body('N')
        self.cart = Body('C', mass=self.ma)
        self.IB = inertia(self.cart.frame, 0, 0, self.IBzz)
        self.top_pend = Body('P_1', mass=self.mb, central_inertia=self.IB)

        self.slider = PrismaticJoint('slider', self.track, self.cart, coordinates=self.y[0], speeds=self.y[2])
        self.rev1 = PinJoint('r1', self.cart, self.top_pend, coordinates=self.y[1], speeds=self.y[3],
                        child_axis=self.top_pend.z, child_joint_pos=self.lb*self.top_pend.y,
                        parent_axis=self.cart.z)

        self.joints = (self.slider, self.rev1)
        self.bodies = (self.track, self.cart, self.top_pend)

        self.F = dynamicsymbols('F')
        self.cart.apply_force(self.F*self.cart.x) # motor
        self.cart.apply_force(-self.c*(self.y[2])*self.cart.x, reaction_body=self.track) # friction

        # gravity
        self.cart.apply_force(-self.track.y*self.cart.mass*self.g)
        self.top_pend.apply_force(-self.track.y*self.top_pend.mass*self.g)

        # get equations of motion
        self.method = JointsMethod(self.track, self.slider, self.rev1)
        self.method.form_eoms()
        self.ydot = self.method.rhs()
    def linearize(self, op_point=[0,sp.pi,0,0]):
        op_point=dict(zip(
            self.y+[self.F,
               self.lb,
               self.c,
               self.g,
               self.ma,
               self.mb,
               self.IBzz], 
            op_point+[0]+self.constants))
        self.A, self.B = self.method.method.to_linearizer().linearize(A_and_B=True, op_point=op_point)
        self.A, self.B = np.array(self.A).astype(np.float64), np.array(self.B).astype(np.float64)
        return self
    def lambdify(self):
        self.func = sp.lambdify([self.y, self.F, self.lb, self.c, self.g, self.ma, self.mb, self.IBzz], self.ydot, 'numpy')
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
                            tspan=15,
                            framerate=60,
                            lag_seconds=0.25,
                            controller='LQR',):

        lag_buffer = [y_0]*int(framerate*lag_seconds)
        forces = []
        # if controller == 'FFF':
        #     ctrl = lambda t, e: eval_fourier(t, dipc_model.fourier_weights)
        # elif controller == 'FFBF':
        #     ctrl = lambda t, e: dipc_model.ff_path[int(t*framerate)]
        # else:
        #     ctrl = lambda t, e: (-self.K[controller]@e)[0]
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
        plt.plot(self.soln_t, self.soln_y[2], label='$\dot{x}$ (m/s)')
        plt.plot(self.soln_t, self.soln_y[1], label='$\\theta_1$ (rad)')
        plt.plot(self.soln_t, self.soln_y[3], label='$\dot\\theta_1$ (rad/s)')
        plt.plot(self.soln_forces[:, 0], self.soln_forces[:, 1], label='$u$ (N)')
        plt.xlabel('Time (s)')
        plt.legend()
    def plot_animation(self, colors, times):
        num_frames = len(self.soln_y[0])
        fig, ax = plt.subplots()
        mat, = ax.plot(*self.get_xy(self.soln_y[0, 0], self.soln_y[2,0], self.constants[0]), marker='o')
        time_label = ax.text(0, -0.6, '0')
        num_frames = len(self.soln_y[0])
        def animate(i):
            mat.set_data(self.get_xy(*(self.soln_y[(0, 2), i%(num_frames+30)-30 if i%(num_frames+30) > 29 else 0]), self.constants[0]))
            time_label.set_text(f't={round((i%(num_frames+30)-30)/60, 4)}')
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
    def get_xy(self, x, th1, l1):
        cart = np.array([[x], [0]])
        pend_1 = cart + dipc_model.rot(th1)@np.array([[0],[-l1]])
        return np.concatenate([cart, pend_1], axis=1)   
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
    model.construct_PP(eigs).construct_LQR(Q, R).integrate_with_scipy(
        y_0 = [0, (7.5/8)*np.pi, 0, 0], 
        targets = np.array([[  0, np.pi, 0, 0],
                            [0.5, np.pi, 0, 0]]),
                            controller = 'LQR', lag_seconds=0.05)
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