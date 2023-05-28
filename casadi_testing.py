#%%
from casadi import DaeBuilder, sin, cos, pi, integrator, vertcat, SX, MX
import numpy as np
from time import perf_counter
from MPC_testing import get_mpc
import sys
#%%
# l, ma, mb, I, c = [0.23116035, 0.00625   , 0.05      , 0.        , 0.10631411]
p = MX.sym('p', 6)
l, ma, mb, I, c = p[0], p[1], p[2], p[3], p[4]
f = p[5]#MX.sym('f')
y = MX.sym('y', 4)

ydot = vertcat(
    y[2],
    y[3],
    f,
    -c*y[3] + (-9.8*l*mb*sin(y[1]) - l*mb*(l*mb*y[3]**2*sin(y[1]) + (-I*l*mb*y[3]**2*sin(y[1]) + I*ma*f + I*mb*f - l**3*mb**2*y[3]**2*sin(y[1]) + l**2*ma*mb*f - l**2*mb**2*f*cos(y[1])**2 + l**2*mb**2*f - 4.9*l**2*mb**2*sin(2.0*y[1]))/(I + l**2*mb))*cos(y[1])/(ma + mb))/(I - l**2*mb**2*cos(y[1])**2/(ma + mb) + l**2*mb)
)

ode = {'x':y, 'p': p, 'ode': ydot}
def eval_ode(t_step, t_f, x0=np.array([0,np.pi/2,0,0]), p=np.array([0.23116035, 0.00625   , 0.05      , 0.        , 0.10631411, 0])):
    int_func = integrator('F_i', 'cvodes', ode, dict(t0=0, tf=t_step))
    x0 = np.array([0,np.pi/2,0,0])
    p=np.array([0.23116035, 0.00625   , 0.05      , 0.        , 0.10631411, 0])
    res = []
    for i in range(int(t_f/t_step)):
        x0 = int_func(x0=x0, p=p)['xf']
        res.append(x0)
    return np.array(res)

#%%

# dydot = [
#         (296.296296296296*pi*ke*(-ke*-dy[0]+12*f)-kf*-dy[0]-((l*mb*(-9.8*l*mb*sin(y[1])-(l*mb*(296.296296296296*pi*ke*(-ke*-dy[0]+12*f)-kf*-dy[0]+l*mb*(dy[1]**2)*sin(y[1])))*cos(y[1])))/((-((l**2)*(mb**2)*(cos(y[1])**2))/(ma+mb))+(l**2)*mb)) + l*mb*(dy[1]**2)*sin(y[1]))/(ma+mb),
#         -((-9.8*l*mb*sin(y[1]))-((l*mb*(296.296296296296*pi*ke*(-ke*-dy[0]+12*f)-kf*-dy[0]+l*mb*(dy[1]**2)*sin(y[1])))/(ma+mb)))/((-((l**2)*(mb**2)*(cos(y[1])**2))/(ma+mb))+(l**2)*mb)
# ]

d.add_ode('xdot', ydot[0])
d.add_ode('thetadot', ydot[1])
d.add_ode('dxdot', dydot[0])
d.add_ode('dthetadot', dydot[1])

d.set_start('x', 0)
d.set_start('theta', 0)
d.set_start('dx', 0)
d.set_start('dtheta', 0)

# d.set_unit('l', 'm')
# d.set_unit('ma', 'kg')
# d.set_unit('mb', 'kg')
# d.set_unit('ke', '')

print(d)

# %%
func = d.create('f', ['x', 'u', 'p'], ['ode'])
x = vertcat(
    y[0],
    y[1],
    dy[0],
    dy[1]
)

ode = vertcat(
    ydot[0],
    ydot[1],
    dydot[0],
    dydot[1]
)
# dae = SXFunction(daeIn(x=[y[0], y[1], dy[0], dy[1]]), daeOut(ode=func))
# dae = {'x':x, 'ode':ode}
# i = integrator('i', 'idas', dae, {'l':l, 'ma':ma, 'mb':mb, 'ke':ke, 'kf':kf, 'f':f})
# # I = integrator()
# %%
import scipy as sp
mpc, sim, _ = get_mpc()
null = open('/dev/null', 'w')
start = perf_counter()

#%%
def get_power(t, y):
    start = perf_counter()
    # mpc.reset_history()
    sys.stdout = null
    step = mpc.make_step(y)[0, 0]
    sys.stdout = sys.__stdout__
    print(t, step, perf_counter()-start, sep='\t')
    return step

consts = [0.18, 1, 0.12, 0.00299,  22]
# consts = [0.17, 0.8, 0.1, 0.00299, 25]
K = np.array([1.1832159566007476, 4.373058536495648, 1.0524256130913159, 0.9350445494914013])
# K = np.array([-5.1728044235924315, 1.6984511727579807, -1.1415374803753107, 0.20032634799088658])
target = np.array([0, np.pi, 0, 0])
y0 = np.array([0, 2*np.pi/8, 0, 0])
sim.x0 = y0

tspan = 8
# soln = sp.integrate.solve_ivp(lambda t, y: np.array(func(y, (K@(y-target)), consts)).flatten(), y0 = y0, t_span=(0, tspan), dense_output = True)
soln = sp.integrate.solve_ivp(lambda t, y: np.array(func(y, get_power(t, y), consts)).flatten(), y0 = y0, t_span=(0, tspan), dense_output = True)
# %%
import matplotlib.pyplot as plt
t = np.arange(0, tspan, 0.05)
solutions = soln.sol(t)
plt.plot(t[1:], solutions[0][1:], label='x')
plt.plot(t[1:], solutions[1][1:], label='theta')
plt.plot(t[1:], solutions[2][1:], label='dx')
plt.xlabel('time (s)')
plt.ylabel('x (m), theta (rad), $\dot x$ (m/s)')
plt.title('MPC Controller Simulation With Parameter Noise\n$P_{real}$ = [0.17, 0.8, 0.1, 0.00299, 25]\n$P_{ctrl}$ = [0.18, 1, 0.12, 0.00299,  22]')
# plt.plot(t[1:], solutions[3][1:], label='dtheta') # don't plot by default bc it looks wonky and kinda big
mpc.reset_history()
plt.plot(t[1:], [get_power(0, i) for i in solutions.T][1:], label='u')
plt.legend()
plt.show()


#%%
# now with euler's method:
