#%%
import numpy as np
from numpy import sin, cos
from scipy.integrate import solve_ivp, quad
from scipy.optimize import minimize
from double_pendulum_model import dipc_model
from numba import njit
from time import time
#%%
eigs = np.array([
    [-3.50],
    [-3.51],
    [-3.52],
    [-3.53],
    [-3.54],
    [-3.55],
])

Q = np.array([50, 100, 50, 10, 70, 50]).astype(np.float64)
R = 10
# R = np.diag([10])
model = dipc_model().linearize().lambdify().construct_PP(eigs).construct_LQR(np.diag(Q), R)
tspan = (0, 1.5)
framerate = 60
y_0 = np.array([0, 0, 0, 0, 0, 0])
target = np.array([0,np.pi,0,0,0,0]).T
t_eval = np.arange(0, tspan[1], 1/framerate)

@njit
def eval_fourier(t, weights):
    # print(t, weights)
    return weights[0]+sum([cos(n*t)*weights[n] for n in range(1, 50)])+sum([sin(n*t)*weights[n+49] for n in range(1, 50)])

eval_fourier = np.vectorize(eval_fourier, excluded=['weights'], signature='(),(99)->()')


# @njit
def impulse_cost(times, weights):
    return sum((eval_fourier(times, weights)**2)*R/framerate)

@njit
def times_q(state):
    return sum(((state**2)*Q))/framerate #instead of state.T@Q@state bc it's diagonal

times_q = np.vectorize(times_q, signature='(6)->()')

# @njit
def cost(times, y_states, weights):
    # state_cost = 0
    # for state in (y_states-target): 
    #     state_cost += (state.T@Q@state)/framerate
    return sum((eval_fourier(times, weights)**2)*R/framerate)+sum(times_q(y_states-target))
start_time = time()


def calc_cost(weights):
    soln = solve_ivp(lambda t, x: model.func(x, weights[int(t*60)], *model.constants).flatten(), tspan, y_0, t_eval=t_eval)
    c = sum(weights**2)*R/framerate + sum(times_q(soln.y.T-target))
    print(f'time: {round(time()-start_time, 5)}, cost: {c}', np.round(weights[:10], 3))
    return c
# %%
# initial_weights = np.zeros(3*60)
# initial_weights[1] = 1

# print(len(model.constants))

initial_weights = -np.cos(np.linspace(0, 10, int(tspan[1]*framerate)+1))
# initial_weights = min(initial_weights, key=calc_cost)
print(initial_weights)
print(minimize(calc_cost, initial_weights, options={'disp':True}), file=open('optimization_result.txt', 'w'))
# %%
# import scipy

# def fun(x, y, p):
#     return model.func(y, p[int(x*60)], *model.constants)

# def bc(ya, yb, p):
#     return np.array([ya-y_0, yb-target])

# scipy.integrate.solve_bvp(model.func, bc, x, y_0)