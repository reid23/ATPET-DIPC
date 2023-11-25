#%%
import casadi as ca
from casadi import MX, DM, vertcat, horzcat, integrator, sin, cos, Function
import numpy as np
import matplotlib.pyplot as plt
import os

debug_integrator_times = False

params = (0.220451, 0.139185)
nsteps = 10
tstep = 0.1

Q = DM(np.diag([1., 10., 1., 1.]))
R = DM(np.diag([0.]))
def get_integrator(t_step, p):
    l, c = p[0], p[1]
    f = MX.sym('f')
    y = MX.sym('y', 4)
    ydot = vertcat(
        y[2],
        y[3],
        f,
        -c*y[3] + (-9.8*sin(y[1])-f*cos(y[1]))/l
    )
    dynamics = ca.Function('ydot', [y, f], [ydot])


    ode = {'x':y, 'u':f, 'ode': ydot}
    return integrator('F_i', 'rk', ode, 0, t_step, {'expand': True, 'simplify': True})
if debug_integrator_times:
    print('integrator timing results:')
    x0 = DM([0., 1., 0., 0.])
    u = DM([3.])
    intfunc = get_integrator(0.1, params)
    # %timeit intfunc(x0=x0, u=u)
# %%
def fix_integrator(integrator):
    _x = MX.sym('x0', 5)
    return Function('integrator', [_x], [integrator(x0=_x[:4], u=_x[4])['xf']])
def construct_solver(nsteps, tstep):
    x = MX.sym('x', 5, nsteps)
    x0 = MX.sym('x0', 5)

    integrator = fix_integrator(get_integrator(tstep, params))
    res = integrator.map(nsteps-1)(x[:, :nsteps-1])
    diff = ca.sumsqr(res-x[:4, 1:])
    diff += ca.sumsqr(x[:, 0]-x0)
    cost = 1e2*ca.sum2(cos(x[1, :])) + 1e-4*(ca.sumsqr(x[2, :]) + ca.sumsqr(x[3, :])) + ca.sumsqr(x[0, :])
    g = horzcat(
        diff,
        x[0, :],
        9*x[2, :]**4 + 50*x[2, :]**2 * x[4, :]**2 + 0.26*x[4, :]**4,
    )

    nlp = {
        'x': ca.vec(x),
        'p': x0,
        'f': cost,
        'g': g
    }
    opts = {
        'ipopt.linear_solver': 'MA27'
    }
    return ca.nlpsol('solver', 'ipopt', nlp, opts)






# %%
sol = construct_solver(nsteps, tstep)
sol.generate_dependencies('gen.c', {'main': True, 'with_header': True})
os.system('gcc -shared -o gen.so -fPIC gen.c')
solver = ca.nlpsol('solver', 'ipopt', 'gen.so')
#%%
res = np.array(ca.reshape(solver(
    p=[0, 0.99*np.pi, 0, 0, 0], 
    lbg=[0] + [-0.7]*nsteps + [-1]*nsteps, 
    ubg=[0] + [0.7]*nsteps + [100]*nsteps
)['x'], (5, nsteps)))
fig, axs = plt.subplots(2, 2)
for i, ax in enumerate(axs.flatten()):
    ax.plot(res[i])
    ax.set_title(['cart pos', 'pend pos', 'cart vel', 'pend vel'][i])
plt.show()
# %%
