#%%
import sympy as sym
from sympy.physics.mechanics import dynamicsymbols
from sympy import *

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
    RKE = (I1 * q1**2)/2     +     (I2 * (q1+q2)**2)/2
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
# %%
eoms = construct_eoms()
#%%
if __name__ == '__main__':
    import numpy as np
    import scipy as sp
    import matplotlib.pyplot as plt

    A, B, f = get_functions(eoms, [0.2, 0.15, 0.3, 0.1, 0.1, 1, 1, 0.00035, 0.0002])
    fun = lambda t, x: f(0, np.array(x).reshape((6, 1))).T
    # tgrid = np.linspace(0, 10, 100)
    res = sp.integrate.solve_ivp(fun, [0, 10], np.array([0, 0, 1, 0, -1, 0]), rtol=1e-5, atol=1e-5)

    # plotting stuff
    fig, axs = plt.subplots(3, 2, sharex=True)
    for idx, ax in enumerate(axs.flatten()):
        ax.plot(res['t'], res['y'][idx], label=['x', '$\\dot x$', '$\\theta_1$', '$\\dot \\theta_1$', '$\\theta_2$', '$\\dot \\theta_2$'][idx])
        ax.legend()
    axs[2][0].set_xlabel('time (s)')
    axs[2][1].set_xlabel('time (s)')
    # plt.legend()
    plt.show()
# %%
