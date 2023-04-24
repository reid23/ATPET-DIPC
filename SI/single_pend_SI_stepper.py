#%%
from multiprocessing import Pool
import numpy as np
import matplotlib.pyplot as plt
from sim.single_pendulum_model_stepper import dipc_model
import scipy
from time import perf_counter

#%%
with open('mpc_data.txt', 'r') as f:
    data = np.array(eval(f.read()))

data = data[10:-10]
data[0, :] -= data[0, 0] 


model = dipc_model().lambdify()
func = model.func
weights = np.array([0.217, 0.125, 0.05, 0.005])
best_cost = 10000000
best_consts = np.ones(4)
def stepper_cost(t0, constants):
    istart, iend = np.searchsorted(data[:, 0], t0), np.searchsorted(data[:, 0], t0+2)
    # print(istart, iend)
    soln = scipy.integrate.solve_ivp(fun = lambda t, y: func(y, data[np.searchsorted(data[:, 0], t), -1], *(constants*weights), 10, 10).flatten(), 
                                    y0=data[istart, 1:5], 
                                    t_span=(t0, t0+2), 
                                    t_eval=data[istart:iend, 0])
    # print(istart, iend, soln)
    return np.sum((soln.y.T - data[istart:iend, 1:5])**2)

def cost(consts):
    cost = sum(stepper_cost(i, consts) for i in np.arange(0, 225, int(225/16)))
    # cost = sum(np.array(pool.starmap(stepper_cost, [(i, consts) for i in np.arange(0, 225, int(225/16))])))
    global best_cost
    global best_consts
    if best_cost > cost: 
        best_cost = cost
        best_consts = consts
        # print('here!')
    print(cost, consts, best_cost, best_consts)
    return cost
try:
    print(scipy.optimize.minimize(cost, np.ones(4)))
except KeyboardInterrupt:
    pass

# %%
plt.plot(data[:450, 0], data[:450, 1:5])
soln = scipy.integrate.solve_ivp(fun = lambda t, y: func(y, data[np.searchsorted(data[:, 0], t), -1], *best_consts, 10, 10).flatten(), 
                                    y0=data[0, 1:5], 
                                    t_span=(0, 5), 
                                    t_eval=data[4:430, 0])
plt.plot(soln.t, soln.y.T)
plt.show()