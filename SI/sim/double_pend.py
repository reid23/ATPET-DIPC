#%%
import sympy as sym
from sympy import symbols, sin, cos
from sympy.physics.mechanics import Body, JointsMethod, inertia, dynamicsymbols, LagrangesMethod
from sympy.physics.mechanics.joint import WeldJoint, PinJoint, PrismaticJoint
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from casadi import MX, DM, vertcat, vertsplit, Function, integrator, mpower, horzcat, inv
# for testing
from control import lqr
from numpy.random import normal

sym.init_printing()
sym.physics.mechanics.mechanics_printing()
#%%
class double_pend_model:
    def __init__(self, params=None, use_saved_sage=True, sage_saved_f_path='f_solved.txt'):
        self.use_saved_sage = use_saved_sage
        self.sage_saved_f_path = sage_saved_f_path
        self.param_names = ['c0', 'I0', 'l0', 'a0', 'm0', 'c1', 'I1', 'l1', 'a1', 'm1', 'm3']
        #* nice printing version
        # q = dynamicsymbols('q:3')
        # dq = dynamicsymbols('{\dot{q_0}} {\dot{q_1}} {\dot{q_2}}')
        #* translate to casadi version
        q = dynamicsymbols('q[0] q[1] q[2]')
        dq = dynamicsymbols('q[3] q[4] q[5]')

        f, u = dynamicsymbols('f u') # not meant to be aggressive

        m = symbols('m:4')
        l = symbols('l:2')
        a = symbols('a:2')
        I = symbols('I:2')
        c = symbols('c:2')

        #* known:
        #* l[0]
        #* m[1]

        track = Body('N')
        cart = Body('C', mass=m[2])
        # cart.masscenter.set_acc(cart.frame, u*track.x)
        I0 = inertia(cart.frame, 0, 0, (1/12)*m[0]*(l[0]**2))#I[0])
        I1 = inertia(cart.frame, 0, 0, (1/6)*m[1]*(a[1]**2))#I[1])
        
        top_pend = Body('P_0', mass=m[0], central_inertia=I0)
        end_pend = Body('P_1', mass=m[1], central_inertia=I1)
        encoder  = Body('end', mass=m[3])

        slider = PrismaticJoint('s', track, cart, coordinates=q[0], speeds=dq[0], parent_axis=track.x, child_axis=cart.x)

        rev1 = PinJoint('r_1', cart, top_pend, coordinates=q[1], speeds=dq[1],
                        child_axis=top_pend.z, child_joint_pos=a[0]*top_pend.y,
                        parent_axis=cart.z)

        rev2 = PinJoint('r_2', top_pend, end_pend, coordinates=q[2], speeds=dq[2],
                        child_axis=end_pend.z, parent_joint_pos=-(l[0]-a[0])*top_pend.y,
                        parent_axis=top_pend.z, child_joint_pos=a[1]*end_pend.y)

        weld = WeldJoint('w', end_pend, encoder, parent_point=a[1]*end_pend.y)
        joints = (slider, rev1, rev2, weld)
        bodies = (track, cart, top_pend, end_pend)

        cart.apply_force(f*cart.x) # motor
        top_pend.apply_torque(-c[0]*dq[1]*top_pend.z)
        end_pend.apply_torque(-c[1]*dq[2]*end_pend.z, reaction_body=top_pend)


        # gravity
        g = 9.800 # in san francisco this is the true value

        cart.apply_force(-track.y*encoder.mass*g)
        top_pend.apply_force(-track.y*top_pend.mass*g)
        end_pend.apply_force(-track.y*end_pend.mass*g)

        # get equations of motion
        method = JointsMethod(track, *joints)
        self.method = method
        method.form_eoms()

        M = method.mass_matrix_full
        F = method.forcing_full
        M[3,3] = 1
        M[3,4] = 0
        M[3,5] = 0
        F[3] = f
        ydot = M.LUsolve(F)

        expression_strings = [str(i).replace('f(t)', 'self.u')
                                    .replace('(t)', '')
                                    .replace('sin', 'ca.sin')
                                    .replace('cos', 'ca.cos')
                                    .replace('q', 'self.q') for i in ydot]
        self.q = MX.sym('q', 6)
        self.u = MX.sym('u')
        m2 = 1

        self.params = {}
        self.params_symbolic = {'m2': 1}
        for i in self.param_names:
            self.params_symbolic[i] = MX.sym(i)
        
        things = {}
        things.update(globals())
        things.update(locals())
        things.update(self.params_symbolic)

        self.ydot = vertcat(*[eval(i, things) for i in expression_strings])
        if params != None:
            assert sum([int(i in params) for i in self.param_names]) == len(self.param_names), 'not all parameters accounted for!'
            self.params = params
        self.ydot_func = Function('ydot_func', [self.q, self.u] + [self.params_symbolic[i] for i in self.param_names], [self.ydot])
    def update_params(self, params):
        self.params.update(params)
    def subs_params(self):
        assert sum([int(i in self.params) for i in self.param_names]) == len(self.param_names), 'not all parameters accounted for!'
        self.ydot_with_params = self.ydot_func(self.q, self.u, *[self.params[i] for i in self.param_names]) # create list with param_names to ensure order is correct (named parameters won't work bc names are erased with Function())
    def get_A_func(self):
        return Function('A_func', [self.q, self.u], [ca.jacobian(self.ydot_with_params, self.q)])
    def get_B_func(self):
        return Function('B_func', [self.q, self.u], [ca.jacobian(self.ydot_with_params, self.u)])
    def get_integrator(self, grid, method='rk'):
        ode = {'x': self.q, 'u': self.u, 'ode': self.ydot_with_params}
        return integrator('int_func', method, ode, 0.0, grid)
    def get_pole_placement_func(self):
        eigs = symbols('e:6')
        x = symbols('x')

        prod = [(x-i) for i in eigs]
        prod = prod[0]*prod[1]*prod[2]*prod[3]*prod[4]*prod[5]
        coeffs = prod.expand().as_poly(x).all_coeffs()

        x = MX.sym('x')
        eigs = MX.sym('eigs', 6)
        e0, e1, e2, e3, e4, e5 = vertsplit(eigs)
        a = eval(str(coeffs))
        
        A_func = self.get_A_func()
        B_func = self.get_B_func()
        q = MX.sym('q', 6)
        u = MX.sym('u')

        A = A_func(q, u)
        B = B_func(q, u)
        print((A@B).shape)
        K = horzcat(0,0,0,0,0,1)@(inv(horzcat(*[(mpower(A, n))@B for n in range(6)]))@sum([mpower(A, (6-n))*a[n] for n in range(7)]))
        return Function('pole_placement_func', [q, u, eigs], [K])
    @classmethod
    def rot(self, th):
        return np.array([
            [np.cos(th), -np.sin(th)],
            [np.sin(th),  np.cos(th)]
        ])
    def get_xy(self, q):
        cart = np.array([[q[0]], [0]])
        top = cart + double_pend_model.rot(q[1])@np.array([[0],[-self.params['l0']]])
        end = top + double_pend_model.rot(q[1]+q[2])@np.array([[0],[-self.params['l1']]])
        return np.concatenate([cart, top, end], axis=1)
    def plot_animation(self, grid, x0, u):
        res = np.array(self.get_integrator(grid)(x0=x0, u=u)['xf']).T
        num_frames = len(res)
        fig, ax = plt.subplots()

        mat, = ax.plot(*self.get_xy(res[0, 0:3].flatten()), marker='o')
        ax.axhline(color='tab:blue')
        time_label = ax.text(0, -0.6, '0')
        def animate(i):
            mat.set_data(*self.get_xy((res[i%(num_frames+30)-30 if i%(num_frames+30) > 29 else 0, 0:3].flatten())))
            time_label.set_text(f't={round((i%(num_frames+30)-30)/60, 4)}')
            return mat
        ax.axis([-0.5,1.5,-0.75,0.75])
        self.anim = animation.FuncAnimation(fig, animate, interval=np.mean(np.diff(grid))*1000, frames=num_frames+30)
        plt.show()
    def animate_data(self, q, fig, ax, tf, dt=1/60, color='tab:blue', show=True):
        q = np.array(q)
        mat, = ax.plot(*self.get_xy(q[0].flatten()), marker='o', color = color)
        ax.axhline(color='tab:blue')
        time_label = ax.text(0, -0.6, '0')
        num_frames = len(q)
        def animate(i):
            mat.set_data(*self.get_xy((q[i%(num_frames+30)-30 if i%(num_frames+30) > 29 else 0].flatten())))
            time_label.set_text(f't={round((i%(num_frames+30)-30)*dt, 4)}')
            return mat
        ax.axis([-0.5,1.5,-0.75,0.75])
        self.anim = animation.FuncAnimation(fig, animate, interval=dt*1000, frames=num_frames+30)
        if show: plt.show()
        return self.anim
    
    def __str__(self):
        return f'{self.__class__.__name__}(params={self.params})'
    def __repr__(self):
        return f'{self.__class__.__name__}(params={self.params}, use_saved_sage={self.use_saved_sage}, sage_saved_f_path={self.sage_saved_f_path})'
# %%
if __name__ == '__main__':
    from matplotlib import pyplot as plt
    # res = int_func(x0=[0.1,0.1,0.1,0.1,0.1,0.1], u=[1]*250 + [-1]*500 + [1]*250)['xf']
    model = double_pend_model()
    model.update_params({'c0': 0.01, 'I0': 0.01, 'l0': 0.3, 'a0': 0.15, 'm0': 0.1, 'c1': 0.01, 'I1': 0.01, 'l1': 0.3, 'a1': 0.15, 'm1': 0.1})
    params = {
        'c0': 2.5339930154189007e-5,
        'I0': 0.001848,
        'l0': 0.3048,
        'a0': 0.12895,
        'm0': 2.075,
        'c1': 9.960299252043114e-5,
        'I1': 0.0005999,
        'l1': 0.3,
        'a1': 0.140322,
        'm1': 0.077771,
        'm3': 0.01
    }
    model.update_params(params)
    model.subs_params()
    model.plot_animation(np.arange(0, 10, 1/60), [0, np.pi, 0, 0, 0, 0], [2]*30 + ([-2]*60 + [2]*60)*3 + [-2]*30 + [0]*180)
    # print(model.get_integrator(np.arange(0, 10, 0.1)))
    f = model.get_pole_placement_func()
    # f.generate('gen.c')

    tf = 0.01
    intfunc = model.get_integrator(tf)
    op_pt = ([0, ca.pi, 0, 0, 0, 0], 0)
    eigs = np.linspace(-1,-1.1,6)[::-1]*2.3
    K = f(*op_pt, eigs)
    A = model.get_A_func()(*op_pt)
    B = model.get_B_func()(*op_pt)
    Q = np.diag([1000, 200, 50, 1, 50, 40])
    R = np.diag([10])
    K, _, E = lqr(A, B, Q, R)
    print(f'feedback gains: {K}')
    print(f'requested eigs: {eigs}')
    print(f'actual eigs:    {E}')
    # print(f'actual eigs:    {np.linalg.eig(A-B@K)[0]}')
    # K = DM([0,0,0,0,0,0]).T
    res = []
    us = []
    ubuf = [DM(0.0)]*1
    x = DM([0.1,np.pi*0.99,0.01,0,0,0])
    sp = DM([0,np.pi,0,0,0,0])
    nsteps = int(5/tf)
    for i in range(nsteps):
        # print(-K@(x-sp))
        ubuf.append(-K@(x-sp+np.concatenate((np.random.normal(0, 0.0015, 4), np.random.normal(0, 0.003, 2)))))
        # print(repr(np.array(ubuf[-1])))
        u = ubuf.pop(0)
        x = intfunc(x0=x, u=u)['xf']
        res.append(x)
        us.append(np.array(u)[0][0])


    # grid = np.arange(0, 10, 0.01)
    # intfunc = model.get_integrator(grid)
    # res = intfunc(x0=[0,ca.pi,0,0,0,0], u=0.1)['xf'].T
    plt.plot(np.arange(0, nsteps*tf, tf), np.array(res)[:, :, 0], label=['$x$', '$\\theta_1$', '$\\theta_2$', '$\dot x$', '$\dot \\theta_1$', '$\dot \\theta_2$'])
    plt.plot(np.arange(0, nsteps*tf, tf), us, label='$u$')
    plt.legend()
    plt.show(block=False)
    input('[enter] to close plot: ')
    plt.close()

    fig, ax = plt.subplots()
    model.animate_data(np.array(res)[:, 0:3, 0], fig, ax, tf*nsteps, tf)
# %%
# %%
