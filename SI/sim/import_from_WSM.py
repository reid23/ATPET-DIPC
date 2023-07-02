#%%
import casadi as ca
from casadi import DaeBuilder, integrator, MX
import numpy as np
from matplotlib import pyplot as plt
from scipy.integrate import solve_ivp

dae = DaeBuilder('f', 'DIPC8')
#%%
params = [0.001, 0.01, 0.3, 0.15, 0.1,
          0.001, 0.01, 0.3, 0.15, 0.1]
for param in zip(['top_friction', 'top_inertia', 'top_L', 'top_lc', 'top_mass', 
              'end_friction', 'end_inertia', 'end_L', 'end_lc', 'end_mass'],
              params):
    dae.register_p(param[0])
    # dae.set(*param)
#%%
# f = dae.create('banana')
# soln = solve_ivp(lambda t, y: np.array(f(x=y, u=0.1, p=params)['ode']).flatten(), t_span=(0, 10), y0=[0,0,0,0,0,0])
# plt.plot(soln.t, soln.y.T)
# f2 = dae.create('f', ['t', 'x', 'z', 'p', 'u'], ['ode'], {'aux': ['y0', 'y1', 'y2', 'y3', 'y4', 'y5']})
# f3 = dae.create('f2', ['t', 'x', 'z', 'p', 'u'], ['ode', 'alg', 'quad'])
# %%
# test = dae.create('test')
grid=np.arange(0, 10, 0.01)
intfunc = integrator('intfunc', 'rk', dae.create('for_intfunc'), 0, grid)
# %%
res = intfunc(x0=[0, 0, ca.pi/2, 0, 0, 0], u=[1]*250 + [-1]*500 + [1]*250, p=params)
plt.plot(grid, res['xf'].T, label=['$x$', '$\dot x$', '$\\theta_1$', '$\dot \\theta_1$', '$\\theta_2$', '$\dot \\theta_2$'])
plt.legend()
plt.show()
# %%