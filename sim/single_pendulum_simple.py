#%%
import numpy as np
import control as ct
import matplotlib.pyplot as plt
import scipy.linalg as la
# % clear all, close all, clc
pi = np.pi
m = 0.12 # mass of pendulum (kg)
M = 0.2 # mass of cart (kg)
L = 0.2 # length of pendulum (m)
g = -9.81 # gravity (dm/s^2)
d = 0 # friction
s = 1 # pendulum is up (this is the state we linearize around)
# A = np.array([
#     [0,1,0,0],
#     [0,-d/M,-m*g/M,0],
#     [0,0,0,1],
#     [0,-s*d/(M*L),-s*(m+M)*g/(M*L),0]
# ])
# B = np.array([
#     [0],
#     [1/M],
#     [0],
#     [s/(M*L)]
# ])

A = np.array([[  0.        ,   0.        ,   1.        ,   0.        ],
              [  0.        ,   0.        ,   0.        ,   1.        ],
              [  0.        ,   4.51380368,   0.        , -29.95541398],
              [  0.        ,  45.13803681,   0.        , -94.39731297]]) 

B = np.array([[ 0.        ],
              [ 0.        ],
              [ 3.89366053],
              [12.26993865]])


# [[  0.           0.           1.           0.        ]
#  [  0.           0.           0.           1.        ]
#  [  0.           4.51380368   0.         -29.95541398]
#  [  0.          45.13803681   0.         -94.39731297]] 
# [[ 0.        ]
#  [ 0.        ]
#  [ 3.89366053]
#  [12.26993865]]


eigs = np.array([
    [-5.3], 
    [-5.31],
    [-5.32],
    [-5.33]
])
K = ct.place(A, B, eigs)
print(np.round(la.eig(A-B)[0], 3))
print(np.round(la.eig(A-B@K)[0], 3))
print(ct.ctrb(A, B))
print(np.linalg.eigh(ct.ctrb(A, B)))
print('A, B, K:')
print(A)
print(B)
print(K)
print(B@K)
#%%
#               x    t    vx   vt
y0 = np.array([[0], [pi], [0], [0]])
target = np.array([[0.1], [pi], [0], [0]])

dt = 0.03
t_span = 10
forces = []

def real_physics_acc(y, B, K, target):
    bu = B@K@(y-target)
    return np.array([
        [y[2]],
        []
    ]) + bu

def get_acc(y, A, B, K, target):
    forces.append(-(K@(y-target))[0][0])
    return (A-B@K)@(y-target)
def sim(t_span, dt, y0, target):
    forces = []
    y = np.zeros((int(t_span/dt + 1), 4, 1))
    y[0] = y0
    for t in np.arange(int(t_span/dt)):
        y[t+1] = y[t]+(get_acc(y[t], A, B, K, target)*dt)
    return y

data = sim(t_span, dt, y0, target).T
print(data[0][0][-1])

x = np.arange(0, t_span, dt)

fig, ax = plt.subplots(3, 2, sharex=True)
ax[2][0].set_xlabel('time (s)')
ax[2][1].set_xlabel('time(s)')



ax[0][0].plot(x, data[0][0], label='$x$')
ax[0][0].set_ylabel('x ($[m]$)')

ax[1][0].plot(x, data[0][1], label = '$\dot{x}$')
ax[1][0].set_ylabel('$\dot{x}$ ($[m][s]^{-1}$)')

ax[2][0].plot(x[:-1], np.diff(data[0][1])/np.diff(x), label='$\ddot{x}$')
ax[2][0].plot(x[:-1], forces, label='$f_x$')
ax[2][0].legend()
ax[2][0].set_ylabel('$\ddot{x}$ ($[m][s]^{-2}$')

ax[0][1].plot(x, data[0][2], label='$\\theta$')
ax[0][1].set_ylabel('$\\theta$ ($[rad]$)')

ax[1][1].plot(x, data[0][3], label='$\dot\\theta$')
ax[1][1].set_ylabel('$\dot\\theta$ ($[rad][s]^{-1}$)')

ax[2][1].plot(x[:-1], np.diff(data[0][3])/np.diff(x), label='$\ddot\\theta$')
ax[2][1].set_ylabel('$\ddot\\theta$ ($[rad][s]^{-2}$)')

# ax[2].plot(x[:-1], forces, label='$f_x$')
# fig.legend()
plt.show()
# end
# %%
import scipy

u_list = []
def f(t, y):
    Sx = np.sin(y[2])
    Cx = np.cos(y[2])
    D = m*L*L*(M+m*(1-Cx**2))
    u =  (K@(y[:, np.newaxis]-(target-np.array([[0],[0],[pi],[0]]))))[0][0]
    return np.array([
            y[1],
            (-(m**2)*(L**2)*g*Cx*Sx + m*(L**2)*((m*L*y[3])**2)*Sx - d*y[1])/D + m*L*L*(1/D)*u, 
            y[3],
            (m+M)*m*g*L*Sx - m*L*Cx*(m*L*(y[3]**2)*Sx - d*y[1])/D - m*L*Cx*(1/D)*u
    ])

soln = scipy.integrate.solve_ivp(f, [0, t_span], y0.flatten()-np.array([0,0,pi,0]), max_step=dt)

print(soln['y'][0][-1])
plt.plot(soln['t'], soln['y'][0], label='$x(t)$')
plt.plot(soln['t'], soln['y'][2], label='$\\theta(t)$')
plt.plot(soln['t'], soln['y'][1], label='$\dot{x}(t)$')
plt.plot(soln['t'], soln['y'][3], label='$\dot\\theta(t)$')
# plt.plot(soln['t'][1:], np.diff(soln['y'][1])/np.diff(soln['t']), label='$\ddot{x}(t)$')
plt.legend()



# %%
