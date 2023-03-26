#%%
from casadi import *
import numpy as np
from time import perf_counter
from MPC_testing import get_mpc

d = DaeBuilder()
l = d.add_p('l')
ma = d.add_p('ma')
mb = d.add_p('mb')
ke = d.add_p('ke')
kf = d.add_p('kf')
f = d.add_u('f')

y = [d.add_x('x'), d.add_x('theta')]
dy = [d.add_x('dx'), d.add_x('dtheta')]

ydot = dy
dydot = [
    (296.296296296296*pi*ke*(-ke*dy[0]+12*f)-kf*dy[0]-((l*mb*(-9.8*l*mb*sin(y[1])-(l*mb*(296.296296296296*pi*ke*(-ke*dy[0]+12*f)-kf*dy[0]+l*mb*(dy[1]**2)*sin(y[1])))*cos(y[1])))/((-((l**2)*(mb**2)*(cos(y[1])**2))/(ma+mb))+(l**2)*mb)) + l*mb*(dy[1]**2)*sin(y[1]))/(ma+mb),
    ((-9.8*l*mb*sin(y[1]))-((l*mb*(296.296296296296*pi*ke*(-ke*dy[0]+12*f)-kf*dy[0]+l*mb*(dy[1]**2)*sin(y[1])))/(ma+mb)))/((-((l**2)*(mb**2)*(cos(y[1])**2))/(ma+mb))+(l**2)*mb)
]

d.add_ode('xdot', ydot[0])
d.add_ode('thetadot', ydot[1])
d.add_ode('dxdot', dydot[0])
d.add_ode('dthetadot', dydot[1])

d.set_start('x', 0)
d.set_start('theta', pi/2)
d.set_start('dx', 0)
d.set_start('dtheta', 0)

# d.set_unit('l', 'm')
# d.set_unit('ma', 'kg')
# d.set_unit('mb', 'kg')
# d.set_unit('ke', '')

print(d)

#%%



# %%
func = d.create('f', ['x', 'u', 'p'], ['ode'])

# i = integrator('i', 'idas', func)
# print(i)

# f = Function('f1', func, )
# %%
import scipy as sp
mpc = get_mpc()
null = open('/dev/null', 'w')
start = perf_counter()
def get_power(t, y):
    mpc.reset_history()
    sys.stdout = null
    step = mpc.make_step(y)[0, 0]
    sys.stdout = sys.__stdout__
    print(t, step, perf_counter()-start, sep='\t')
    return step
consts = [0.22, 1, 0.12, 0.00299,  22]
K = np.array([1.1832159566007476, 4.373058536495648, 1.0524256130913159, 0.9350445494914013])
# K = np.array([-5.1728044235924315, 1.6984511727579807, -1.1415374803753107, 0.20032634799088658])
target = np.array([0, np.pi, 0, 0])
y0 = np.array([0, 2*np.pi/8, 0, 0])
tspan = 5
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
plt.title('MPC Controller Simulation')
# plt.plot(t[1:], solutions[3][1:], label='dtheta')
# plt.plot(t[1:], K@(solutions[:, 1:] - target[:, np.newaxis]), label='u')
plt.legend()
plt.show()
exit()
# %%

# time to do actual fitting!

with open('data4.txt', 'r') as f:
    data = list(map(np.array, eval(f.read())))

interpolators = []
for i in range(len(data)):
    # crop bits of start and end and scale time to be seconds
    data[i] = data[i][10:-10, (0,1,2,3,5,6)]
    data[i][:, 0] -= data[i][0, 0]
    data[i][:, 0] *= 1e-9
    
    # delete outliers
    for j in range(1, len(data[i])-1):
        if np.abs(data[i][j, 3] - data[i][j-1, 3])>1:
            print('here', i, j)
            data[i][j, 3] = (data[i][j+1, 3] + data[i][j-1, 3])/2
    interpolators.append(sp.interpolate.interp1d(data[i][:, 0], data[i][:, 1], assume_sorted=True))

def single_cost(trial, consts):
    # print('func: ', func)
    soln = sp.integrate.solve_ivp(
        lambda t, y: np.array(func(
            y, 
            interpolators[trial](t), 
            consts)).flatten(), 
        y0 = data[trial][0, 2:],
        t_span=(0, data[trial][-1, 0]), 
        t_eval = data[trial][:, 0]).y.T
    return np.sum((soln - data[trial][:, 2:])**2)

def cost(consts, pool):
    return sum(pool.starmap(single_cost, [(i, consts) for i in range(len(data))]))

from multiprocessing import Pool
if __name__ == '__main__':
                #    L   ma  mb      K_E   K_f
    y_0 = np.array([0.22, 1, 0.12, 0.00299,  22])
    cur = y_0.copy()
    dx = 0.01
    step_scale = 1
    step = 0.05
    old_time = perf_counter()
    best_cost = np.Inf
    best_coeffs = []
    with Pool(len(data)) as p:
        try:
            while True:
                base = cost(cur, p)
                if base > 1_000_000_000: 
                    print(f'died, final cost was {base} with weights {cur}')
                    break
                if base < best_cost:
                    best_cost = base
                    best_coeffs = cur
                print('[', *[np.format_float_positional(i, 5) for i in cur], ']',
                    np.round(base, 1), 
                    np.round(perf_counter()-old_time, 3), sep='\t')
                d = np.zeros(len(y_0))
                old_time = perf_counter()
                for i in range(len(y_0)):
                    test = cur.copy()
                    test[i] += dx*y_0[i]
                    d[i] = (cost(test, p) - base)
                d = d/np.linalg.norm(d)
                # print('d: ', d)
                # print('step: ', step_scale*d*step)
                cur -= step_scale*d*step
                cur[cur<0] = 0.00005
                step_scale *= 0.995
        except KeyboardInterrupt:
            print('done!')
            print(best_coeffs, best_cost)
# %%
