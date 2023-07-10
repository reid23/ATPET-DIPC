#%%
from sim.double_pend import double_pend_model as Model
from lmfit import Parameters, minimize, fit_report
from matplotlib import pyplot as plt
from IPython.display import clear_output
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
params['l0'].vary = False
params['a1'].vary = False
params['I1'].vary = False
params['a0'].vary = False
params['l0'].value = 0.3048 # 1 ft
params['a1'].value = 0.125323
params['I1'].value = 5.999e-4
params['a0'].value = 0.155 # not confident
# params['a0'].set(value=0.155, is_init_value=True)
params['c0'].set(value=0.10804660, is_init_value=True)
params['I0'].set(value=0.0011, is_init_value=True)
params['m0'].set(value=0.1, is_init_value=True)
params['c1'].set(value=0.0015, is_init_value=True)
params['l1'].set(value=0.3, is_init_value=True)
# params['a1'].set(value=0.125, is_init_value=True)
params['m1'].set(value=0.1, is_init_value=True)
# %%
run = 1
t0 = 0
tf = len(data[run])
tf = 800
n=0
def lmfit_func(params, data):
    global n
    model.update_params(dict(zip(model.param_names, [params[i]*1.0 for i in model.param_names])))
    model.subs_params()
    # res = model.get_integrator(grid=data[:, 0]-data[0, 0])(x0=data[0, 2:8], u=data[:, 1])['xf']
    intfunc = model.get_integrator(grid=np.arange(0, 1, np.mean(np.diff(data[:, 0]))))
    t_start=0
    res = []
    l = len(np.arange(0, 1, np.mean(np.diff(data[:, 0]))))
    while t_start<len(data)-200:
        # print(len(res))
        res += [intfunc(x0=data[t_start, 2:8], u=data[t_start:t_start+l, 1])['xf']]
        t_start += l
    res = np.concatenate(res, axis=1)

    # print(res.shape)
    clear_output(wait=True)
    print(f'function has been called {n} times')
    n+=1
    return res.T[:, 1:] - data[:len(res.T), 3:8]
    # return res.T[:, 1:] - data[:, 3:8]

fitted_params = minimize(
    lmfit_func, 
    params, 
    args=(data[run][t0:tf],), 
    method = "least_squares",
)

print(fit_report(fitted_params))
# %%
plotdata = data[run][t0:tf]
plt.plot(plotdata[:, 0], plotdata[:, 5], label='$x$', color='tab:blue')
plt.plot(plotdata[:, 0], plotdata[:, 3], label='$\\theta_1$', color='tab:orange')
plt.plot(plotdata[:, 0], plotdata[:, 4], label='$\\theta_2$', color='tab:green')
plt.plot(plotdata[:, 0], plotdata[:, 1], label='$u$', color='black')

fit_res = dict(zip(model.param_names, [fitted_params.params[i].value for i in model.param_names]))
model.update_params(fit_res)
model.subs_params()
sim_data = model.get_integrator(grid=plotdata[:, 0]-plotdata[0, 0])(x0=list(plotdata[0, 2:8]), u=list(plotdata[:, 1]))['xf'].T

plt.plot(plotdata[:, 0], sim_data[:, 3], label='$\hat x$', linestyle='dashed', color='tab:blue')
plt.plot(plotdata[:, 0], sim_data[:, 1], label='$\hat \\theta_1$', linestyle='dashed', color='tab:orange')
plt.plot(plotdata[:, 0], sim_data[:, 2], label='$\hat \\theta_2$', linestyle='dashed', color='tab:green')

plt.legend()
plt.plot()
# %%
