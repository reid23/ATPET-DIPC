#%%
import sympy as sym
from sympy.physics.mechanics import dynamicsymbols
from sympy import *
import casadi as ca

def construct_eoms():
    # setup variables
    # l1, l2 are distance from pivot to COM. 
    # lpend is length of first arm (to mid pivot)
    # m1, m2 are masses of the arms
    # I1, I2 are moments of inertia of the arms
    # c1, c2 are friction coefficients of the pivots
    l1, l2, lpend = symbols('l_1,l_2,l_p', positive=True)
    m1, m2 = symbols('m_1,m_2', positive=True)
    I1, I2 = symbols('I_1,I_2', positive=True)
    c1, c2 = symbols('c_1,c_2', positive=True)
    g = -9.81

    t = Symbol('t') # time
    a = Symbol('a') # acceleration of our reference frame (cart)

    # generalized coordinates for the pendulum angles
    q1, q2 = dynamicsymbols('q_1, q_2')

    # get cartesian coords from pend angles
    # two parts: position, assuming pivot is (0, 0), then
    # additional 1/2 at^2 to account for accelerating reference frame
    # Math: x_1 = \begin{bmatrix}l_1\sin q_1\\-l_1(1-\cos q_1)\end{bmatrix}+\begin{bmatrix}\frac{1}{2}at^2\\0\end{bmatrix}
    x1 = Matrix([
        (a * t**2)/2 + (l1 * sin(q1)),
        -l1 * (1 - cos(q1)),
    ])
    x2 = Matrix([
        (a * t**2)/2 + (lpend*sin(q1)) + l2*sin(q1+q2),
        -lpend*(1 - cos(q1)) - l2*(1 - cos(q1 + q2))
    ])

    # get cartesian velocities
    dx1 = diff(x1, t)
    dx2 = diff(x2, t)

    # now formulate the lagrangian KE-PE using
    # the cartesian positions and velocities.

    # Math: KE_\text{lin} = \sum\frac{1}{2}mv^2
    # Math: KE_\text{rot} = \sum\frac{1}{2}I\omega^2
    # Math: PE = \sum mgh
    # Math: L = KE - PE

    KE = (m1 * (dx1[0]**2 + dx1[1]**2))/2      +     (m2 * (dx2[0]**2 + dx2[1]**2))/2
    RKE = (I1 * diff(q1, t)**2)/2     +     (I2 * diff(q1+q2, t)**2)/2
    PE = m1 * g * x1[1]     +     m2 * g * x2[1]
    L = simplify(KE+RKE) - simplify(PE)

    # formulate euler-lagrange equations
    dldq = Matrix([
        diff(L, q1),
        diff(L, q2),
    ])
    dlddq = Matrix([
        diff(L, diff(q1, t)),
        diff(L, diff(q2, t)),
    ])
    dlddqdt = diff(dlddq, t)

    ELeq = Eq(simplify(dldq), simplify(dlddqdt))

    # solve them for the accelerations
    ddq = Matrix([diff(q1, t, 2), diff(q2, t, 2)])
    eoms = solve(ELeq, ddq)
    eoms = simplify(Matrix([
        eoms[ddq[0]],
        eoms[ddq[1]]
    ]))

    # 
    # Now we want to put it in this format:
    #   d/dt y = f(y)
    # which, expanded, is:
    # d/dt [x    ]    = [xdot            ]
    # d/dt [xdot ]    = [a               ] 
    # d/dt [q1   ]    = [q1dot           ]
    # d/dt [q1dot]    = [[thing1]-c1q1dot]
    # d/dt [q2   ]    = [q2dot           ]
    # d/dt [q2dot]    = [[thing2]-c2q1dot]
    #


    # fix variables and make them nicer
    y = MatrixSymbol('y', 6, 1)
    eoms = Subs(eoms, [q1, diff(q1, t), q2, diff(q2, t)], [y[2], y[3], y[4], y[5]]).doit()
    eoms = simplify(Matrix([
        y[1],
        a,
        y[3],
        eoms[0]-c1*y[3],
        y[5],
        eoms[1]-c2*y[5],
    ]))

    return {
        'params': Matrix([l1, l2, lpend, m1, m2, c1, c2, I1, I2]),
        'y': y,
        'u': a,
        'ode': eoms,
    }

# %%
def get_functions(eoms, params):
    ode = eoms['ode']
    y = eoms['y']
    a = eoms['u']
    parameters = (eoms['params'], params)

    A = lambdify([a, y], Subs(ode.jacobian(y),   *parameters).doit(), cse=True)
    B = lambdify([a, y], Subs(ode.jacobian([a]), *parameters).doit(), cse=True)
    f = lambdify([a, y], Subs(ode,               *parameters).doit(), cse=True)
    return A, B, f

def to_casadi(eoms):
    params = []
    for i in eoms['params']:
        exec(f'{i} = ca.SX.sym("{i}")')
        params.append(eval(str(i)))
    exec(f'{eoms["y"]} = ca.SX.sym("{eoms["y"]}", *{eoms["y"].shape})')
    exec(f'{eoms["u"]} = ca.SX.sym("{eoms["u"]}")')
    ode = eval(str(eoms['ode']).replace('Matrix', '')
                               .replace('sin', 'ca.sin')
                               .replace('cos', 'ca.cos'))
    ode = ca.vertcat(*[i[0] for i in ode])
    return {
        'p': ca.vertcat(*params),
        'u': eval(str(eoms['u'])),
        'x': eval(str(eoms['y'])),
        'ode': ode
    }
    
# %%
eoms = construct_eoms()
#%%
if __name__ == '__main__':
    import numpy as np
    import scipy as sp
    import matplotlib.pyplot as plt
    from time import perf_counter
    p = [
        0.2,      # l1
        0.15,     # l2
        0.3,      # lpend
        0.09,     # m1
        0.05,     # m2
        0.01,     # c1
        0.01,     # c2
        0.00035,  # I1
        0.0002    # I2
    ]
    A, B, f = get_functions(eoms, p)
    ca_eoms = to_casadi(eoms)
    x0 = np.array([0, 0, 1, 0, -1, 0])
    u = 0
    tgrid = np.linspace(0, 5, 100, dtype=float).flatten().tolist()
    ca_intfunc = ca.integrator('intfunc', 'cvodes', ca_eoms, 0, tgrid)
    Zip = perf_counter()
    fun = lambda t, x: f(u, np.array(x).reshape((6, 1))).T
    res = sp.integrate.solve_ivp(fun, [0, 5], x0, rtol=1e-8, atol=1e-8)
    Zap = perf_counter()
    res2 = ca_intfunc(x0=x0, p=p, u=u)['xf']
    Zop = perf_counter()
    print(f'scipy:  {Zap-Zip:.5f}s')
    print(f'casadi: {Zop-Zap:.5f}s')
    # plotting stuff
    if True:
        fig, axs = plt.subplots(3, 2, sharex=True)
        for idx, ax in enumerate(axs.flatten()):
            label = ['x', '$\\dot x$', '$\\theta_1$', '$\\dot \\theta_1$', '$\\theta_2$', '$\\dot \\theta_2$'][idx]
            ax.plot(res['t'], res['y'][idx], label=label+' (sp)', color='tab:blue')
            ax.plot(tgrid, np.array(res2)[idx], label=label+' (ca)', color='tab:orange')
            ax.legend()
        axs[2][0].set_xlabel('time (s)')
        axs[2][1].set_xlabel('time (s)')
        plt.show()

a, b = A(0, np.zeros((6, 1))), B(0, np.zeros((6, 1)))
eigs = np.linalg.eigvals(np.hstack([b, a@b, a@a@b, a@a@a@b, a@a@a@a@b, a@a@a@a@a@b]))
print(*np.round(eigs, 3))
# %%
