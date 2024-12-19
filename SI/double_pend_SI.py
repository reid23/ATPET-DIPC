#%%
from sim.double_pend_properly import to_casadi
from sim.eoms import get_eoms
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
eoms = to_casadi(get_eoms())
#%%
dt = 0.01
thoriz = 3
tgrid = np.arange(0, thoriz, dt).flatten().tolist()
intfunc = ca.integrator('intfunc', 'rk', eoms, 0, tgrid)
#%%
def plot(params, start=0):
    fix_angles = np.array([0, 0, 0*params[-2], 0, 0*params[-1], 0])
    res = intfunc(x0=data[1:, start], u=data[0, start:start+len(tgrid)], p=full_params(params[:-2]))['xf']
    fig, axs = plt.subplots(2, 3)
    for idx, ax in enumerate(axs.T.flatten()):
        ax.plot(tgrid, np.array(res[idx, :]).flatten(), color='tab:orange')
        ax.plot(tgrid, data[idx+1, start:start+len(tgrid)].flatten()+fix_angles[idx], color='tab:blue')
    plt.show()
def load_data(file, dt=dt, STEPS_PER_MM=128*200/80, TICKS_PER_RAD=(2**13)/(np.pi)):
    if isinstance(file, str):
        with open(file, 'r') as f:
            data = np.array(eval(f.read()))
    else:
        data = file
    # convert us to s
    uneven = (data[:, 0]-data[0, 0]).astype(np.float64)/1_000_000.0
    tgrid = np.arange(0, max(uneven[:-1]), dt).tolist()
    pos = data[:, 1].astype(np.float64)#/STEPS_PER_MM
    vel = data[:, 2].astype(np.float64)#/STEPS_PER_MM
    top = data[:, 3].astype(np.float64)#/TICKS_PER_RAD
    dtop = data[:, 4].astype(np.float64)#/TICKS_PER_RAD
    end = data[:, 5].astype(np.float64)#/TICKS_PER_RAD
    dend = data[:, 6].astype(np.float64)#/TICKS_PER_RAD

    u = np.diff(vel)/np.diff(uneven)
    print(u)
    print(np.mean(u))
    # u = np.clip(np.round(u/5)*5, -5, 5)
    u = np.concatenate([np.array([0]), u])
    # u = data[:, 1]
    # dend = np.diff(end)/np.diff(uneven)
    # dend = -data[:, 7]
    # dtop = np.diff(top)/np.diff(uneven)
    # dtop = data[:, 5]
    # uneven = uneven[:-1]
    # pos = pos[:-1]
    # vel = vel[:-1]
    # top = top[:-1]
    # end = end[:-1]

    print('here:')
    print(np.where(np.diff(uneven)<0))

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
def load_data_new(top_home=2.0597783333333335, end_home=2.6163720625):
    with open('../teensy-code/src/devttyACM0_2024_12_10.00.47.14.553.txt', 'r') as f:
        def eval_handle_error(s):
            try:
                out = list(eval(s).values())
                assert len(out)==9
                return out
            except:
                return None
        data = np.array(list(filter(lambda x: x is not None, [eval_handle_error(i) for i in f.readlines()])))
        data[:, 4] -= top_home
        data[:, 6] -= end_home
        datasmooth = np.zeros((data.shape[0], 2))
        for i in range(data.shape[0]):
            lower_idx = i-49 if i-49 > 0 else 0
            datasmooth[i, 0] = np.mean(data[lower_idx:i+1, 4])
            datasmooth[i, 1] = np.mean(data[lower_idx:i+1, 6])
        data[:, (4, 6)] = datasmooth
        print(datasmooth)
        return data
# data2 = np.vstack(sorted(load_data_new(), key=lambda x: x[0]))
data2 = np.load('../data.npy')
data2[:, 1] /= 1000.0
data2[:, 2] /= 1000.0
data = load_data(data2).astype(np.float64)

offsets = ca.MX.sym('offsets', 2)
# offsets = [2*np.pi*6817.5/(2**14 - 1), 2*np.pi*6259.5/(2**14 - 1)]
fix_angles = [0, 0, offsets[0], 0, offsets[1], 0]

cost = 0
p = ca.MX.sym('p', 6)
params = ca.vertcat(
    p[0],     # l1
    p[5],     # l2 #
    0.3048,   # lpend #
    p[1],     # m1
    p[2], # m2 #
    0.0,     # c1
    0.0,     # c2
    p[4],     # I1
    p[3],# I2 #
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
weights = ca.DM([0, 0, 100, 100, 100, 100]).T # state error weights
for i in range(data.shape[1]):
    if i%(data.shape[1]//20) != 0: continue
    if data.shape[1]-len(tgrid) <= i: continue
    res = intfunc(x0=cadata[1:, i], u=data[0, i:i+len(tgrid)], p=params)['xf']
    print('res shape:', res.shape)
    err = (res-cadata[1:, i:i+len(tgrid)])**2
    cost += ca.sumsqr(err)
    err = ca.sum2(err)
    print('err shape:', err.shape)
    # cost += weights@err
    
nlp = {
    'f': cost,
    'x': ca.vertcat(p, offsets),
}
solver = ca.nlpsol('solver', 'ipopt', nlp, {'ipopt.linear_solver': 'ma57'})
# solver = ca.nlpsol('solver', 'ipopt', nlp)
#%%     l1   m1     m2       I2     I1     l2
x0  = [0.2, 0.08, 0.045, 0.0008, 0.0008, 0.14, 0, 0]
x0 = [0.152265, 0.150243, 0.1616, 0.00161724, 0.0014427, 0.103731, 0, 0]
lbx = [0.05, 0.01,  0.02, 0.0001, 0.0001,  0.01, 0, 0]
ubx = [0.3,  0.2,   0.3,   0.02,   0.02,  0.2, 0, 0]
fxd = [0.,   0,    0,    0,    0,      0,       0, 0, 0]
soln = solver(x0=x0, lbx=[x0[i] if fxd[i] else lbx[i] for i in range(len(x0))], ubx=[x0[i] if fxd[i] else ubx[i] for i in range(len(x0))])
#%%
plot(np.array(soln['x']).flatten(), 250)
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

#%%

res = np.array(intfunc(x0=[0, 0, np.pi/2, 0, 0, 0], p=full_params(soln['x'][:-2]), u=[0]*len(tgrid))['xf'])
fig, axs = plt.subplots(6)
for i in range(6):
    axs[i].plot(res[i, :])
plt.show()
# %%
# PEND DOWN ANGLES
# top: 6817.5 ticks
# end: 6259.5 ticks
params = soln['x'][:-2]
# params = x0[:-2]
tgrid2 = np.array(list(range(data.shape[1])))*dt
intfunc2 = ca.integrator('intfunc', 'idas', eoms, 0, tgrid2)
full_integration_result = intfunc2(x0=data[1:, 0], u=data[0, :], p=full_params(params))['xf']
full_res = np.array(full_integration_result)
fig, axs = plt.subplots(6)
for i in range(6):
    axs[i].plot(full_res[0+i, :], label='$\\hat x$'); axs[i].plot(data[1+i, :], label='$x$'); axs[i].legend()
# %%
