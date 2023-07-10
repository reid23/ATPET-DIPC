#%%
from sim.double_pend import double_pend_model as Model
from lmfit import Parameters, minimize, fit_report
from IPython.display import clear_output
from matplotlib import pyplot as plt
from multiprocessing.pool import Pool
import numpy as np
#%%
with open('../data/dp_run1.txt', 'r') as f:
    data = np.array(eval(f.read()))
data[:, 0] = data[:, 0]/1e9 # convert t from ns to s
data[:, 1] = data[:, 1]/1e6 # convert power to m/s^2
idxs = [-1] + list(np.nonzero(np.array(np.diff(data[:, 0])<0))[0]) + [len(data)-1]
data = [data[idxs[i]+1:idxs[i+1]+1] for i in range(len(idxs)-1)]
for i in range(len(data)):
    data[i] = data[i][10:-10, :]
#%%
model = Model(sage_saved_f_path='sim/f_solved.txt')
# %%
params = Parameters()
for i in model.param_names:
    params.add(i, min=0)
#* names: c, l, I, a, m
#* top = 0, end = 1
params['c0'].set(value=0.1080466, vary=True,  is_init_value=True)#, min=-0.01, max=0)
params['l0'].set(value=0.3048,     vary=False, is_init_value=True)
params['I0'].set(value=0.00308971, vary=False, is_init_value=True)
params['a0'].set(value=0.155,      vary=False, is_init_value=True)
params['m0'].set(value=0.075,      vary=False, is_init_value=True)

params['c1'].set(value=0.15,       vary=True,  is_init_value=True)#, min=-0.01, max=0)
params['l1'].set(value=0.3,        vary=False, is_init_value=True)
params['I1'].set(value=0.0005,     vary=False, is_init_value=True)
params['a1'].set(value=0.125323,   vary=False, is_init_value=True)
params['m1'].set(value=0.066,      vary=False, is_init_value=True)
# %%
run = 1
t0 = 0
tf = len(data[run])
tf = 800
n = 0
int_length = 0.5
num_int_segments = 20
def lmfit_func(params, data):
    global n
    model.update_params(dict(zip(model.param_names, [params[i]*1.0 for i in model.param_names])))
    model.subs_params()
    # res = model.get_integrator(grid=data[:, 0]-data[0, 0])(x0=data[0, 2:8], u=data[:, 1])['xf']
    tgrid = np.arange(0, int_length, np.mean(np.diff(data[:, 0])))
    intfunc = model.get_integrator(grid=tgrid)
    res = []
    start_vals = np.linspace(0, len(data), num_int_segments, endpoint=False, dtype=int)
    train = [data[i:i+tgrid.shape[0]] for i in start_vals if i+tgrid.shape[0]<len(data)]
    for subset in train:
        res += [intfunc(x0=subset[0, 2:8], u=subset[:, 1])['xf'].T]
    res = np.concatenate(res, axis=0)
    train = np.concatenate(train, axis=0)

    # print(res.shape)
    clear_output(wait=True)
    print(f'function has been called {n} times')
    n+=1
    return res[:, 1:6] - train[:, 3:8]

fitted_params = minimize(
    lmfit_func, 
    params, 
    args=(data[run][t0:tf],), 
    method = "least_squares",
)

print(fit_report(fitted_params))
fitted_params
# %%
plotdata = data[run][t0:tf]
plt.plot(plotdata[:, 0], plotdata[:, 5], label='$x$ ($m$)', color='tab:blue')
plt.plot(plotdata[:, 0], plotdata[:, 3], label='$\\theta_1$ ($rad$)', color='tab:orange')
plt.plot(plotdata[:, 0], plotdata[:, 4], label='$\\theta_2$ ($rad$)', color='tab:green')
plt.plot(plotdata[:, 0], plotdata[:, 1], label='$u$ ($m \\cdot s^{-2}$)', color='black')

fit_res = dict(zip(model.param_names, [fitted_params.params[i].value for i in model.param_names]))
fit_res = dict(zip(model.param_names, [params[i].value for i in model.param_names]))
model.update_params(fit_res)
model.subs_params()
sim_data = model.get_integrator(grid=plotdata[:, 0]-plotdata[0, 0])(x0=list(plotdata[0, 2:8]), u=list(plotdata[:, 1]))['xf'].T

plt.plot(plotdata[:, 0], sim_data[:, 3], label='$\hat x$ ($m$)', linestyle='dashed', color='tab:blue')
plt.plot(plotdata[:, 0], sim_data[:, 1], label='$\hat \\theta_1$ ($rad$)', linestyle='dashed', color='tab:orange')
plt.plot(plotdata[:, 0], sim_data[:, 2], label='$\hat \\theta_2$ ($rad$)', linestyle='dashed', color='tab:green')

plt.xlabel('time (s)')
# plt.ylabel('$x$ (m), $\\theta$ (rad), u ($m \\times {s^2}$)')
plt.title('True and Predicted System State vs. Time')
plt.legend()
plt.plot()
# %%
