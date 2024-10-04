#%%
from sim.double_pend_properly import to_casadi
from sim.eoms import get_eoms
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
eoms = to_casadi(get_eoms())
#%%
dt = 0.02
thoriz = 0.2
tgrid = np.arange(0, thoriz, dt).flatten().tolist()
intfunc = ca.integrator('intfunc', 'rk', eoms, 0, tgrid)
#%%
def plot(params, start=0):
    fix_angles = np.array([0, 0, params[-2], 0, params[-1], 0])
    res = intfunc(x0=data[1:, start]+fix_angles, u=data[0, start:start+len(tgrid)], p=full_params(params[:-2]))['xf']
    fig, axs = plt.subplots(2, 3)
    for idx, ax in enumerate(axs.T.flatten()):
        ax.plot(tgrid, np.array(res[idx, :]).flatten(), color='tab:orange')
        ax.plot(tgrid, data[idx+1, start:start+len(tgrid)].flatten()+fix_angles[idx], color='tab:blue')
    plt.show()
def load_data(file, dt=dt, STEPS_PER_MM=128*200/80, TICKS_PER_RAD=(2**13)/(np.pi)):
    with open(file, 'r') as f:
        data = np.array(eval(f.read()))
    # convert us to s
    uneven = (data[:, 0]-data[0, 0]).astype(np.float64)/1_000_000.0
    tgrid = np.arange(0, max(uneven[:-1]), dt).tolist()
    pos = -data[:, 2].astype(np.float64)/STEPS_PER_MM
    vel = -data[:, 3].astype(np.float64)/STEPS_PER_MM
    end = -data[:, 4].astype(np.float64)/TICKS_PER_RAD
    top = data[:, 6].astype(np.float64)/TICKS_PER_RAD

    u = np.diff(vel)/np.diff(uneven)
    dend = np.diff(end)/np.diff(uneven)
    dtop = np.diff(top)/np.diff(uneven)
    uneven = uneven[:-1]
    pos = pos[:-1]
    vel = vel[:-1]
    top = top[:-1]
    end = end[:-1]

    evendata = np.empty((7, len(tgrid)))
    for idx, col in enumerate([u, pos, vel, top, dtop, end, dend]):
        # print(uneven.shape, col.shape, idx)
        interp = ca.interpolant('interp', 'linear', [uneven], col)
        # return interp
        evendata[idx] = np.array(interp(tgrid)).flatten()
        # plt.plot(uneven, col)
        # plt.plot(tgrid, evendata[idx])
        # plt.show()
    return evendata

# %%
data = load_data('data1.txt')

offsets = ca.MX.sym('offsets', 2)
offsets = [2*np.pi*6817.5/(2**14 - 1), 2*np.pi*6259.5/(2**14 - 1)]
fix_angles = [0, 0, offsets[0], 0, offsets[1], 0]

cost = 0
p = ca.MX.sym('p', 5)
params = ca.vertcat(
    p[0],     # l1
    0.140631, # l2 #
    0.3048,   # lpend #
    p[1],     # m1
    0.045477, # m2 #
    p[2],     # c1
    p[3],     # c2
    p[4],     # I1
    0.0004115,# I2 #
)
cadata = ca.horzcat(
    data[0, :],
    data[1, :],
    data[2, :],
    ca.DM(data[3, :])+offsets[0],
    data[4, :],
    ca.DM(data[5, :])+offsets[1],
    data[6, :]
).T
full_params = ca.Function('params', [p], [params])
weights = ca.DM([0, 0, 100, 0, 100, 0]).T # state error weights
for i in range(data.shape[1]):
    if i%(data.shape[1]//20) != 0: continue
    if data.shape[1]-len(tgrid) <= i: continue
    res = intfunc(x0=cadata[1:, i], u=data[0, i:i+len(tgrid)], p=params)['xf']
    print(res.shape)
    err = (res-cadata[1:, i:i+len(tgrid)])**2
    err = ca.sum2(err)
    print(err.shape)
    cost += weights@err
    
nlp = {
    'f': cost,
    'x': ca.vertcat(p),
}
# solver = ca.nlpsol('solver', 'ipopt', nlp, {'ipopt.linear_solver': 'MA57'})
solver = ca.nlpsol('solver', 'worhp', nlp)
#%%
x0  = [0.2, 0.08, 0.0, 0.0, 0.0008, -4.0,     10.0-np.pi]
lbx = [0.1, 0.05, 0,    0,    0.0004, -ca.inf, -ca.inf]
ubx = [0.3, 0.8,  0.1,  0.1,  0.1,    ca.inf,  ca.inf]
fxd = [1,   1,    1,    1,    1,      0,       0]
soln = solver(x0=[0.2, 0.08, 0.01, 0.01, 0.0008, 0.0, 0.0], lbx=[x0[i] if fxd[i] else lbx[i] for i in range(len(x0))], ubx=[x0[i] if fxd[i] else ubx[i] for i in range(len(x0))])
#%%
plot(np.array(soln['x']).flatten(), 70)
print(soln)
# %%
def plot_dataset(file):
    data = load_data(file)
    fig, axs = plt.subplots(2, 3)
    for idx, ax in enumerate(axs.T.flatten()):
        ax.plot(data[idx+1, :].flatten(), color='tab:blue')
    plt.show()

# for file in ['data.txt', 'data1.txt', 'data2.txt', 'data3.txt', 'static0.txt', 'static1.txt', 'static2.txt']:
#     print(file)
#     plot_dataset(file)
# %%
# PEND DOWN ANGLES
# top: 6817.5 ticks
# end: 6259.5 ticks