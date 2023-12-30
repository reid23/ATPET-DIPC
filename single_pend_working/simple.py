#%%
import casadi as ca
from casadi import MX, DM, vertcat, horzcat, integrator, sin, cos
import numpy as np

params = (0.220451, 0.139185)
nsteps = 10
tstep = 0.01
tgrid = np.linspace(0, nsteps*tstep, nsteps+1, endpoint=False)
Q = DM(np.diag([1., 10., 1., 1.]))
R = DM(np.diag([0.]))
def get_integrator(t_step, p, simple=True, div: int=1):
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

    if simple:
        t_step = t_step/div
        half = y+0.5*t_step*dynamics(y, f)
        intfunc = ca.Function('intfunc', [y, f], [y+dynamics(half, f)*t_step])
        res = y
        for _ in range(div):
            res = intfunc(res, f)
        intfunc = ca.Function('intfunc_2', [y, f], [res])
        return intfunc
    else:
        ode = {'x':y, 'u':f, 'ode': ydot}
        return integrator('F_i', 'rk', ode, 0, t_step)
    
#%%
import matplotlib.pyplot as plt
def test_integrators_and_plot(n=10, steps=10):
    tgrid=np.linspace(0, tstep*steps, steps+1, endpoint=True)
    # print(tgrid)
    truth = get_integrator(tstep, params, simple=False)
    true_res = [DM([0, 1, 0, 0])]
    for i in range(steps):
        true_res.append(truth(x0=true_res[-1], u=i)['xf'])
    truth = np.array(true_res)
    integrators = [get_integrator(tstep, params, div=i) for i in range(1, n+1)]
    fig, axs = plt.subplots(2, 2)
    results = []
    for i in range(n):
        results.append([DM([0, 1, 0, 0])])
        for j in range(steps):
            results[-1].append(integrators[i](results[-1][-1], DM([j])))
    results = np.array(results)
    for i, ax in enumerate(axs.flatten()):
        ax.set_title(['cart pos', 'pend ang', 'cart vel', 'pend vel'][i])
        ax.plot(tgrid, truth[:, i], label='truth', color='tab:orange')
        ax.plot(tgrid, np.array(r)[i], color='black')
        for j in range(n):
            ax.plot(tgrid, results[j, :, i], label=f'div={j+1}', color=(j/n, 1-j/n, 0))
        ax.legend()
    plt.plot()

test_integrators_and_plot()
#%%

intfunc = get_integrator(tstep, params, simple=True, div=2)
y = MX.sym('y', 4)
quad_form_cost = ca.Function('quad', [y], [y.T@Q@y])

fancy_intfunc = get_integrator(tgrid, params, simple=False)


x = MX.sym('x', 5, nsteps)
x0 = MX.sym('x0', 4)
# g = ca.sumsqr(fancy_intfunc(x0=x0, u=x[0, :])['xf']-x[1:, :])
g = (
    [ca.sumsqr(x0-x[1:, 0])]
  + [ca.sumsqr(intfunc(x[1:, i], x[0, i])-x[1:, i+1]) for i in range(nsteps-1)]
#   + [ca.sumsqr(ca.vec(fancy_intfunc(x0=x0, u=x[0, :])['xf']-x[1:, :]))]
  + [x[1, i] for i in range(nsteps)]
  + [9*x[3, i]**4 + 50*x[3, i]**2 * x[0, i]**2 + 0.26*x[0, i]**4 for i in range(nsteps)]
)

cost = 1e2*ca.sum2(cos(x[2, :])) + 1e-4*(ca.sumsqr(x[3, :]) + ca.sumsqr(x[4, :])) + ca.sumsqr(x[1, :])

nlp = {
    'x': ca.vec(x),
    'f': cost,
    'g': horzcat(*g),
    'p': x0,
}
solver = ca.nlpsol('solver', 'ipopt', nlp, {'print_in': 0, 'print_out': 0, 'print_time': 0, 'ipopt.linear_solver': 'MA27', 'ipopt.print_level':5, 'ipopt.max_iter': 1e5, 'expand': False})

soln = solver(
    # x0=DM([0]*nsteps*), 
    # p=DM([0.0, np.pi*0.95, 0.0, 0.0]).T,
    p=DM([0.0, 0.001, 0.0, 0.0]).T,
    lbg=DM([0.]*(nsteps) + [-0.7]*nsteps + [0]*nsteps),
    ubg=DM([0.]*(nsteps) + [0.7]*nsteps + [100]*nsteps)
)
print(*[f'{key}: {val}' for key, val in soln.items()], sep='\n')
#%%
res = np.array(ca.reshape(soln['x'], (5, nsteps)))
test_int = get_integrator(tgrid.tolist(), params, simple=False, div=2)
test_res = np.array(test_int(x0=[0.0, 0.001, 0.0, 0.0], u=res[0, :].flatten().tolist()+[0])['xf'])

from time import perf_counter
def plot(tgrid, res, offset=1, plot_true=False):
    fig, axs = plt.subplots(2, 2)
    for i, ax in enumerate(axs.flatten()):
        ax.plot(tgrid[:-1], res[i+offset, :], color='tab:blue')
        if plot_true: ax.plot(tgrid, test_res[i, :], color='tab:orange')
        ax.set_title(['cart pos', 'pend angle', 'cart vel', 'pend vel'][i])
plot(tgrid, res, plot_true=True)
plt.show()
# print(tgrid)
#%%
### simulation
def make_step(x0, initial_guess):
    return solver(x0=initial_guess, p=DM(x0), lbg=DM([0.]*(2) + [-0.7]*nsteps + [0]*nsteps), ubg=DM([0.]*(2) + [0.7]*nsteps + [100]*nsteps))

sim_integrator = get_integrator(tstep, params, simple=False)
x0 = soln['x']
x = np.array([0, np.pi*0.97, 0, 0])
history = [x]
u_history = []
times = []
N=50
print()
for i in range(N):
    tic = perf_counter()
    soln = solver(x0=x0, p=DM(x), lbg=DM([0.]*(2) + [-0.7]*nsteps + [0]*nsteps), ubg=DM([0.]*(2) + [0.7]*nsteps + [100]*nsteps))
    toc = perf_counter()
    times.append(toc-tic)
    x = np.array(sim_integrator(x0=x, u=soln['x'][0])['xf']).flatten()
    u_history.append(soln['x'][0])
    history.append(x)
    x0 = vertcat(soln['x'][5:], DM([0]), intfunc(soln['x'][-4:], soln['x'][-5]))
    print(f'\rtimestep {i}', end='')
print()

history = np.vstack(history)
#%%
plot(np.linspace(0, tstep*(N+1), N+1), history.T, offset=0)
plt.show()
# %%
