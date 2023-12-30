import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from simple2 import get_integrator, nsteps, tstep, params
from time import perf_counter
opts = {
    'ipopt.linear_solver': 'MA27',
    'ipopt.sb': 'no',
    'ipopt.print_level': 0,
    'print_time': False,
}

solver = ca.nlpsol('solver', 'ipopt', 'gen.so', opts)

integrator = get_integrator(tstep, params)

hist = []
u_hist = []
times = []
N = 200
x = ca.DM([0, 0.2*np.pi, 0, 0])
for i in range(N):
    hist.append(np.array(x, dtype=float).flatten().tolist())
    tic = perf_counter()
    res = solver(
        p=x, 
        lbx=-1.5,
        ubx=1.5,
        lbg=-0.7,
        ubg=0.7
        # lbg=[-0.9]*int(nsteps/2) + [-1]*int(nsteps/2), 
        # ubg=[0.9]*int(nsteps/2) + [100]*int(nsteps/2)
    )
    toc = perf_counter()
    u_hist.append(np.array(res['x'][0], dtype=float).flatten()[0])
    x = integrator(x0=x, u=res['x'][0])['xf']
    times.append(toc-tic)

fig, axs = plt.subplots(6, 1, sharex=True)

hist = np.array(hist).T
grid = np.arange(0, N*tstep, tstep)
for i, ax in enumerate(axs.flatten()):
    if i<=3: 
        ax.plot(grid, hist[i])
        ax.set_title(['cart pos', 'pend angle', 'cart vel', 'pend angular vel'][i])
        ax.set_ylabel(['$x\:\: ([m])$', '$\\theta \:\: ([rad])$', '$\\dot x \:\: ([m][s]^{-1})$', '$\\dot \\theta \:\: ([rad][s]^{-1})$'][i])
    elif i==4:
        ax.plot(grid, u_hist)
        ax.set_title('cart acceleration')
        ax.set_ylabel('$u\:\: ([m][s]^{-2})$')
    elif i==5:
        ax.plot(grid, times)
        ax.set_title('solve times')
        ax.set_ylabel('solve time ($[s]$)')

fig.suptitle(f'Simulation Results for MPC\$\\Delta t=${tstep}, $n=${nsteps}')
plt.show()