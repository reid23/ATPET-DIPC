#%%
import sympy as sym
from sympy import symbols, sin, cos
from sympy.solvers.ode import dsolve, systems
from sympy.physics.mechanics import Body, PinJoint, PrismaticJoint, JointsMethod, inertia, dynamicsymbols
import casadi as ca
import numpy as np
from casadi import MX, DM, vertcat, vertsplit, Function, integrator, mpower, horzcat, inv
from time import perf_counter
from control import lqr


sym.init_printing()
sym.physics.mechanics.mechanics_printing()
#%%
class double_pend_model:
    def __init__(self, params=None, use_saved_sage=True, sage_saved_f_path='f_solved.txt'):
        self.use_saved_sage = use_saved_sage
        self.sage_saved_f_path = sage_saved_f_path
        self.param_names = ['c0', 'I0', 'l0', 'a0', 'm0', 'c1', 'I1', 'l1', 'a1', 'm1']
        #* nice printing version
        # q = dynamicsymbols('q:3')
        # dq = dynamicsymbols('{\dot{q_0}} {\dot{q_1}} {\dot{q_2}}')
        #* translate to casadi version
        q = dynamicsymbols('q[0] q[1] q[2]')
        dq = dynamicsymbols('q[3] q[4] q[5]')

        f, u = dynamicsymbols('f u') # not meant to be aggressive

        m = symbols('m:3')
        l = symbols('l:2')
        a = symbols('a:2')
        I = symbols('I:2')
        c = symbols('c:2')

        #* known:
        #* l[0]
        #* m[1]
        # %%

        track = Body('N')
        cart = Body('C', mass=m[2])
        # cart.masscenter.set_acc(cart.frame, u*track.x)
        I0 = inertia(cart.frame, 0, 0, I[0])
        I1 = inertia(cart.frame, 0, 0, I[1])
        top_pend = Body('P_0', mass=m[0], central_inertia=I0)
        end_pend = Body('P_1', mass=m[1], central_inertia=I1)

        slider = PrismaticJoint('s', track, cart, coordinates=q[0], speeds=dq[0])

        rev1 = PinJoint('r_1', cart, top_pend, coordinates=q[1], speeds=dq[1],
                        child_axis=top_pend.z, child_joint_pos=a[0]*top_pend.y,
                        parent_axis=cart.z)

        rev2 = PinJoint('r_2', top_pend, end_pend, coordinates=q[2], speeds=dq[2],
                        child_axis=end_pend.z, parent_joint_pos=-(l[0]-a[0])*top_pend.y,
                        parent_axis=top_pend.z, child_joint_pos=a[1]*end_pend.y)


        joints = (rev1, rev2, slider)
        bodies = (track, cart, top_pend, end_pend)

        cart.apply_force(f*cart.x) # motor
        top_pend.apply_torque(-c[0]*q[1]*top_pend.z)
        end_pend.apply_torque(-c[1]*q[2]*end_pend.z, reaction_body=top_pend)

        # %%

        # gravity
        g = 9.800 # in san francisco this is the true value

        cart.apply_force(-track.y*cart.mass*g)
        top_pend.apply_force(-track.y*top_pend.mass*g)
        end_pend.apply_force(-track.y*end_pend.mass*g)

        # get equations of motion
        method = JointsMethod(track, slider, rev1, rev2)
        method.form_eoms()
        ydot = method.rhs()

        if not self.use_saved_sage:
            print(f"""
        # enter into sage:
        q = var('q0 q1 q2 q3 q4 q5')
        var('f u')
        var('m0 m1 m2 l0 l1 a0 a1 I0 I1 c0 c1')
        res = solve([u == ({str(ydot[3]).replace('(t)', '')})], f)
        assert len(res) == 1
        res[0].rhs().full_simplify()._sympy_()
        """)
            sage_result = input('enter sage result: ')
        else:
            with open(self.sage_saved_f_path, 'r') as file:
                sage_result = file.read()
        f_solved = eval(sage_result.replace('q0', 'q[0]')
                                .replace('q1', 'q[1]')
                                .replace('q2', 'q[2]')
                                .replace('q3', 'dq[0]')
                                .replace('q4', 'dq[1]')
                                .replace('q5', 'dq[2]')
                                .replace('m0', 'm[0]')
                                .replace('m1', 'm[1]')
                                .replace('m2', 'm[2]')
                                .replace('l0', 'l[0]')
                                .replace('l1', 'l[1]')
                                .replace('a0', 'a[0]')
                                .replace('a1', 'a[1]')
                                .replace('I0', 'I[0]')
                                .replace('I1', 'I[1]')
                                .replace('c0', 'c[0]')
                                .replace('c1', 'c[1]'))
        #%%
        ydot[3] = u
        ydot[4] = ydot[4].subs(f, f_solved) - c[0]*dq[0]
        ydot[5] = ydot[5].subs(f, f_solved) - c[1]*dq[1]
        # %%
        expression_strings = [str(i).replace('(t)', '')
                                    .replace('sin', 'ca.sin')
                                    .replace('cos', 'ca.cos')
                                    .replace('q', 'self.q')
                                    .replace('u', 'self.u') for i in ydot]

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
    def get_integrator(self, grid):
        ode = {'x': self.q, 'u': self.u, 'ode': self.ydot_with_params}
        return integrator('int_func', 'rk', ode, 0.0, grid)
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
    model.subs_params()
    # print(model.get_integrator(np.arange(0, 10, 0.1)))
    f = model.get_pole_placement_func()
    # f.generate('gen.c')

    tf = 0.01
    intfunc = model.get_integrator(tf)
    op_pt = ([0, ca.pi, 0, 0, 0, 0], 0)
    eigs = np.linspace(-1,-1.5,6)[::-1]*3
    K = f(*op_pt, eigs)

    A = model.get_A_func()(*op_pt)
    B = model.get_B_func()(*op_pt)
    print(f'feedback gains: {K}')
    print(f'requested eigs: {eigs}')
    print(f'actual eigs:    {np.linalg.eig(A-B@K)[0]}')

    res = []
    x = DM([0,np.pi*0.95,0,0,0,0])
    sp = DM([0,np.pi,0,0,0,0])
    for i in range(1000):
        # print(-K@(x-sp))
        x = intfunc(x0=x, u=-K@(x-sp))['xf']
        res.append(x)


    # grid = np.arange(0, 10, 0.01)
    # intfunc = model.get_integrator(grid)
    # res = intfunc(x0=[0,ca.pi,0,0,0,0], u=0.1)['xf'].T
    plt.plot(np.arange(0, 1000*tf, tf), np.array(res)[:, :, 0], label=['$x$', '$\\theta_1$', '$\\theta_2$', '$\dot x$', '$\dot \\theta_1$', '$\dot \\theta_2$'])
    plt.legend()
    plt.show()
# %%
# %%
