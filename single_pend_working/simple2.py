#%%
import casadi as ca
from casadi import MX, DM, vertcat, horzcat, integrator, sin, cos, Function
import numpy as np
import matplotlib.pyplot as plt
import os
from time import sleep

debug_integrator_times = False
compile_new_solver = False

params = (0.220451, 0.139185)
nsteps = 10
tstep = 0.05

Q = DM(np.diag([1., 10., 1., 1.]))
R = DM(np.diag([0.]))
def get_integrator(t_step, p, quad=False, nfiniteelements=4):
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
    if quad:
        ode['quad'] = 1e2*cos(y[1]) + 1e-4*(y[2]**2) + 1e-4*(y[3]**2) + y[0]**2

    return ca.integrator('F_i', 'rk', ode, 0, t_step, {'expand': True, 'simplify': True, 'number_of_finite_elements': nfiniteelements})
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
    x0 = MX.sym('x0', 4)
    u = MX.sym('u', nsteps)
    g_cart_pos = []
    g_power = []
    cost = 0
    integrator = get_integrator(tstep, params, quad=False)
    x = x0
    for i in range(nsteps):
        res = integrator(x0=x, u=u[i])
        x = res['xf']
        if i%2==0: continue
        g_cart_pos.append(x[0])
        g_power.append(9*x[2, :]**4 + 50*x[2, :]**2 * u[i]**2 + 0.26*u[i]**4)
        PE = 1-cos(x[1])
        KE = x[3]**2+x[2]**2
        # cost += 1e3*cos(x[1]) + 1e-4*(x[2]**2) + 1e1*(x[3]**2) + 1e-2*x[0]**2
        cost += KE-1e2*PE + 1e5*ca.sqrt(x[0]**2+1e-10)

    g = horzcat(*g_cart_pos, *g_power)


    nlp = {
        'x': u,
        'p': x0,
        'f': cost,
        'g': g
    }
    opts = {
        'ipopt.linear_solver': 'MA27',
        'ipopt.sb': 'no',
        'ipopt.print_level': 0,
        'print_time': False,
        'expand': True
    }
    return ca.nlpsol('solver', 'ipopt', nlp, opts)


def construct_stupid_solver(nsteps, tstep):
    x0 = MX.sym('x0', 4)
    u = MX.sym('u', nsteps)
    integrator = get_integrator(tstep, params, nfiniteelements=4)
    x = x0
    cart_pos = []
    for i in range(nsteps):
        x = integrator(x0=x, u=u[i])['xf']
        cart_pos.append(x[0])

    cost = (1e3 * cos(x[1]) 
          + 1e-2 * x[0]**2
          + 1e-2 * x[2]**2
          + 1e-2 * x[3]**2)
    nlp = {
        'x': u,
        'p': x0,
        'f': cost,
        'g': horzcat(*cart_pos),
    }
    opts = {
        'ipopt.linear_solver': 'MA27',
        'ipopt.sb': 'no',
        'ipopt.print_level': 0,
        'print_time': False,
        'expand': True
    }
    return ca.nlpsol('solver', 'ipopt', nlp, opts)





# %%
opts = {
    'ipopt.linear_solver': 'MA27',
    'ipopt.sb': 'no',
    'ipopt.print_level': 0,
    'print_time': False,
}
sol = construct_solver(nsteps, tstep)
sol = construct_stupid_solver(nsteps, tstep)
if compile_new_solver:
    sol.generate_dependencies('gen.c', {'main': True, 'with_header': True})
    os.system('gcc -shared -o gen.so -fPIC gen.c')
solver = ca.nlpsol('solver', 'ipopt', 'gen.so', opts)

#%%
test_int = get_integrator(tstep, params)
x0 = [0, 0.97*np.pi, 0, 0]

def test(x0, u):
    x = x0
    hist = [x0]
    for i in range(nsteps):
        x = test_int(x0=x, u=u[i])['xf']
        hist.append(np.array(x, dtype=float).flatten().tolist())
    return np.array(hist).T
#%%
if __name__ == '__main__':
    res = solver(
        p=x0, 
        lbg=[-0.9]*nsteps + [-1]*nsteps, 
        ubg=[0.9]*nsteps + [100]*nsteps
    )
    u_res = np.array(res['x'], dtype=float).flatten().tolist()
    x_res = test(x0, u_res)
    x_res = np.array(x_res)
    #%%
    fig, axs = plt.subplots(2, 2)
    for i, ax in enumerate(axs.flatten()):
        ax.plot(x_res[i])
        ax.set_title(['cart pos', 'pend pos', 'cart vel', 'pend vel'][i])
    plt.show()
# %%
