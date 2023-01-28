#%%
import numpy as np
from numpy import sin, cos
from scipy.integrate import solve_ivp, quad
from scipy.optimize import minimize
from double_pendulum_model import dipc_model
from numba import njit
#%%
eigs = np.array([
    [-3.50],
    [-3.51],
    [-3.52],
    [-3.53],
    [-3.54],
    [-3.55],
])

Q = np.diag([100, 10, 5, 1, 100, 50]).astype(np.float64)
R = 10
# R = np.diag([10])
model = dipc_model().linearize().lambdify().construct_PP(eigs).construct_LQR(Q, R)
tspan = (0, 3)
framerate = 60
y_0 = np.array([0, 0, 0, 0, 0, 0])
target = np.array([0,np.pi,0,0,0,0]).T
t_eval = np.arange(0, tspan[1], 1/framerate)

@njit
def eval_fourier(t, weights):
    return np.array([1]+[cos(n*t) for n in range(1, 50)]+[sin(n*t) for n in range(1, 50)])@weights

@njit
def impulse_cost(times, weights):
    return sum([(eval_fourier(t, weights)**2)*R/framerate for t in times])

@njit
def cost(times, y_states, weights):
    state_cost = 0
    for state in (y_states-target): 
        state_cost += (state.T@Q@state)/framerate
    return sum([(eval_fourier(t, weights)**2)*R/framerate for t in times])+state_cost


def calc_cost(weights):
    soln = solve_ivp(lambda t, x: model.func(x, eval_fourier(t, weights), *model.constants).flatten(), tspan, y_0, t_eval=t_eval)
    print('1')
    return cost(soln.t, soln.y.T, weights)
# %%
print(minimize(calc_cost, np.ones(99)), file=open('optimization_result.txt', 'w'))
# %%
