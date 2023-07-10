#%%
import numpy as np
import casadi as ca
from casadi import MX, vertcat, sin, cos, Function
import control as ct
import scipy
from scipy.optimize import minimize
from MPC_testing_stepper import get_mpc
#%%
y = MX.sym('y', 4)
f = MX.sym('f')
l, c = 0.220451, 0.139187
ydot = vertcat(
    y[2],
    y[3],
    f,
    -c*y[3] + (-9.8*sin(y[1])-f*cos(y[1]))/l
)

#%%
A = ca.jacobian(ydot, y)
B = ca.jacobian(ydot, f)
ydot_eval = Function('ydot', [y, f], [ydot])
def get_u(state=[0,ca.pi,0,0]):
    return minimize(Function('blah', [u], [ydot_eval(state, u)[[0,1,3]].T@(ydot_eval(state, u))[0,1,3]]), x0=0).x[0]
    return ca.rootfinder('G','kinsol',Function('blah', [u], [ydot_eval(state, u).T@ydot_eval(state, u)]))()['o0']
# %%
A_func = Function('f_j', [y, f], [A])
B_func = Function('f_b', [y, f], [B])
# %%

def get_lqr(op_pt, Q, R):
    K, _, eigs = ct.lqr(A_func(*op_pt), B_func(*op_pt), Q, R)
    return K, eigs

def get_pp(op_pt, eigs):
    K = ct.place(A_func(*op_pt), B_func(*op_pt), eigs)
    return K

sp = ([0, ca.pi*7/8, 0, 0], 0)
# up
Q = np.diag([500, 100, 40, 100])
R = np.diag([10])
# down
# Q = np.diag([10000, 70, 1, 700])
# R = np.diag([50])
if __name__ == '__main__':
    K, _, eigs = ct.lqr(A_func(*sp), B_func(*sp), Q, R)
    print(K)
    print(eigs)

# %%
    print('here')
    mpc, _, _ = get_mpc(thoriz=10, compile_nlp=False)
    print('1')
    mpc.make_step(np.array([[0.0, 0.0, 0.0, 0.0]]))
    mpc.make_step(np.array([[0.0, 0.0, 0.0, 0.0]]))

    upath = mpc.data.prediction(('_u', 'f'))[0,:,0]
    ypath = mpc.data.prediction(('_x',))[:, :, 0].T

    print(ypath.shape, upath.shape)
    print(ypath[:5])
    print(upath[:5])
# %%
