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
from numba import njit
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
@njit
def eval_fourier(t, weights):
    return np.array([1]+[np.cos(n*t) for n in range(1, 50)]+[np.sin(n*t) for n in range(1, 50)])@weights
class dipc_model:
    fourier_weights = np.array([-1.01059992, -5.15912399,  0.62330237,  3.05759723, -1.12530636,
        2.70284226, -4.50405468,  0.28368342,  3.24455156, -5.35317389,
        3.10959512, -5.81990927,  1.45885037,  5.02717026,  0.97317233,
       -5.30639522, -5.30537116, -5.74147928, -5.90698523,  4.36302429,
       -1.87339229,  3.72996092,  5.44033578, -4.19383407,  3.49324388,
       -3.74749909, -3.10188143, -1.38929699,  5.45965118, -1.14199923,
       -1.87365414,  2.05730946, -0.74948988,  2.88238477,  5.3534978 ,
       -0.59731288, -2.68040456,  2.17956034,  5.77212432,  3.89544017,
       -1.44203953,  0.19349814,  0.0610279 , -3.5676886 ,  3.82201409,
        2.77510269, -2.73921327, -2.8959084 , -4.42520525,  3.39951133,
        0.20293385,  4.02532746,  5.37061355,  4.05480086, -1.28061653,
       -2.10952495, -5.78233763, -2.77562075,  4.59822595,  2.28902516,
        2.91955732,  1.15328398,  2.89499228,  0.90932704, -3.17843466,
       -0.17923359, -1.7568002 ,  5.8974776 ,  5.54273894,  1.2979972 ,
       -1.32922312,  4.24627956, -2.28375478, -0.47280421,  3.07124023,
       -2.24965711,  5.40198739,  1.11280345, -0.98245419,  5.93988155,
        1.76381701, -1.80283728,  5.52826215, -2.60865246,  4.02604869,
        0.87696243,  3.0560031 ,  5.042354  , -5.79265693, -0.13775956,
       -3.17305324, -0.35643387,  0.94615916, -4.0704026 , -5.48553411,
        2.27536486, -0.32290616,  5.9038042 ,  5.16601646])
    ff_path = np.array([ 9.84133178e-03, -1.37981774e-04, -3.41530340e-03, -2.51843290e-08,
        1.17409120e-08,  7.69891949e-04, -3.97038688e-03, -3.34800518e-08,
       -3.25464305e-08,  9.53878506e-13, -2.93953002e-14,  7.38233369e-13,
        2.73852493e-15, -3.29089743e-13, -6.18642696e-14, -7.22536599e-14,
       -8.49240943e-14,  1.27940816e-13, -7.24897440e-13,  2.58591411e-13,
       -4.53016736e-13, -2.04762762e-13, -1.23002510e-13, -1.05671607e-13,
        5.53440758e-03, -3.86502371e-13,  4.11185959e-10, -4.26621904e-13,
        6.10836677e-13, -4.72457331e-14, -5.48102077e-13, -1.66754174e-13,
        4.72834320e-17, -5.72465668e-13,  6.45213359e-13, -7.81809334e-13,
        4.90981537e-13,  5.68689720e-13, -6.13978869e-13,  1.88572428e-13,
        0.00000000e+00,  0.00000000e+00,  4.00537664e-13,  2.34027836e-13,
       -2.87178212e-13,  1.69502661e-13,  2.78644422e-13,  1.22674576e-13,
        1.62503241e-13,  0.00000000e+00,  3.29257519e-13, -1.39694279e-13,
        2.85864398e-13,  3.27273753e-15,  4.93918434e-06,  2.92334947e-13,
       -1.83803349e-13, -5.42774322e-14,  2.17378534e-14,  3.34277020e-04,
       -6.42325638e-14,  4.32368726e-14, -8.84487515e-13,  1.16364233e-13,
       -1.73115321e-13, -6.60862984e-13, -2.37121220e-04,  0.00000000e+00,
       -7.37475162e-13, -9.25642545e-14, -3.57962026e-13,  1.14446795e-04,
       -2.21508689e-13, -1.33152916e-04, -8.12170564e-13,  7.95597127e-14,
       -4.65881486e-13, -1.71569117e-13, -4.21710013e-15, -2.67467577e-13,
       -1.53194506e-13, -3.43884938e-13,  3.40466933e-14,  6.73865169e-15,
       -1.13626826e-13, -2.46224758e-05,  0.00000000e+00, -1.60802578e-08,
       -1.27854804e-14, -3.14870994e-15,  1.21924364e-06])
       #                                                            0.084            0.0007621
    def __init__(self, constants=[0.3, 0.2,   0.5,   1, 9.81, 0.25, 0.125, 0.125, 0.00065, 0.00065]):
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
        self.cart.apply_force(-self.c*self.y[3]*self.cart.x, reaction_body=self.track) # friction

        # gravity
        self.cart.apply_force(-self.track.y*self.cart.mass*self.g)
        self.top_pend.apply_force(-self.track.y*self.top_pend.mass*self.g)
        self.end_pend.apply_force(-self.track.y*self.end_pend.mass*self.g)

        # get equations of motion
        self.method = JointsMethod(self.track, self.slider, self.rev1, self.rev2)
        self.method.form_eoms()
        self.ydot = self.method.rhs()
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
    model = dipc_model().linearize().lambdify()
    # .construct_PP(eigs).construct_LQR(Q, R).integrate_with_scipy(y_0 = [0,0,0,0,0,0], targets = np.array([[  0, np.pi, 0, 0, 0, 0]]), target_timestamps = [0, 1.5], controller = 'FFBF')
    # model.print_eoms()
    normal = model.ydot.subs(zip([model.lb, model.lc, model.lcom, model.c, model.g, model.ma, model.mb, model.mc, model.IBzz, model.ICzz], model.constants))
    print(normal)
    # print(systems.dsolve_system([sp.Eq(model.y[i].diff(model.t), normal[i]) for i in range(6)], 
    # ics=dict(zip([i.subs(model.t, 0) for i in model.y]+[i.subs(model.t, 1.5) for i in model.y],[0, 0, 0, 0, 0, 0]+[0,sp.pi,0,0,0,0]))))
    print(systems.dsolve_system([sp.Eq(model.y[i].diff(model.t), normal[i]) for i in range(6)]))
    #%%
    model.plot_graph()
    model.plot_animation(['tab:blue', 'tab:orange', 'tab:blue'], [0, 5, 10])
    model.show_plots()