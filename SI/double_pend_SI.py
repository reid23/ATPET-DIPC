#%%
import importlib
from sim.double_pend import double_pend_model as Model
from lmfit import Parameters, minimize, fit_report
from IPython.display import clear_output
from matplotlib import pyplot as plt
from multiprocessing.pool import Pool
import numpy as np
from time import perf_counter
#%%
with open('../data/dp_run3.txt', 'r') as f:
    data = np.array(eval(f.read()))
data[:, 0] = data[:, 0]/1e9 # convert t from ns to s
# data[:, 1] = data[:, 1]/1e6 # convert power to m/s^2
data[:, 1] = data[:, -1]
idxs = [-1] + list(np.nonzero(np.array(np.diff(data[:, 0])<0))[0]) + [len(data)-1]
data = [data[idxs[i]+1:idxs[i+1]+1] for i in range(len(idxs)-1)]
for i in range(len(data)):
    data[i] = data[i][10:-10, :]
    # shift = 1
    # data[i][:, 1] = np.concatenate((data[i][:shift, 1], data[i][:-shift, 1]), axis=0)
#%%
model = Model(sage_saved_f_path='sim/f_solved.txt')
# %%
params = Parameters()
for i in model.param_names:
    params.add(i, min=0)
#* names: c, l, I, a, m
#* top = 0, end = 1
params['c0'].set(value=0.0000,     vary=True,  is_init_value=True)
params['l0'].set(value=0.3048,     vary=False, is_init_value=True)
params['I0'].set(value=0.0001848,   vary=True, is_init_value=True)
params['a0'].set(value=0.16895,    vary=False, is_init_value=True)
params['m0'].set(value=0.075,      vary=True, is_init_value=True)

params['c1'].set(value=0.0000,     vary=True,  is_init_value=True)
params['l1'].set(value=0.3,        vary=False, is_init_value=True)
params['I1'].set(value=0.00005,    vary=True, is_init_value=True)
params['a1'].set(value=0.140322,   vary=False, is_init_value=True)
params['m1'].set(value=0.03,       vary=True, is_init_value=True)

scale = np.array([params[i].value*0.1 for i in model.param_names])

for i in model.param_names:
    params[i].set(value=10, is_init_value=True, min=1, max=100)
# %%
run = 4
t0 = 500
tf = len(data[run])
tf = 2000
n = 0
int_length = 1
num_int_segments = 5
start = 0
def lmfit_func(params, data):
    global start
    global n
    model.update_params(dict(zip(model.param_names, [params[i]*scale[idx] for idx, i in enumerate(model.param_names)])))
    model.subs_params()

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
    print(f'function has been called {n} times. total runtime: {round(perf_counter()-start)}s')
    n+=1
    return (res[:, 1:6] - train[:, 3:8])**2
start = perf_counter()
fitted_params = minimize(
    lmfit_func, 
    params, 
    args=(data[run][t0:tf],), 
    method = "least_squares",
)

# print(fit_report(fitted_params))
fitted_params
# %%

def plot_results(time, data, hat=True, vars=[0,1,1,1,0,0,1]):
    if vars[0]: plt.plot(time, data[:, 0], label='$\hat x$ ($m$)', linestyle='dashed' if hat else '-', color='tab:blue')
    if vars[1]: plt.plot(time, data[:, 1], label='${} \\theta_1$ ($rad$)'.format("\hat" if hat else ""), linestyle='dashed' if hat else '-', color='tab:orange')
    if vars[2]: plt.plot(time, data[:, 2], label='${} \\theta_2$ ($rad$)'.format("\hat" if hat else ""), linestyle='dashed' if hat else '-', color='tab:green')
    if vars[3]: plt.plot(time, data[:, 3], label='${} \dot x$ ($m \cdot s^{}$)'.format("\hat" if hat else "", '{-1}'), linestyle='dashed' if hat else '-', color='tab:purple')
    if vars[4]: plt.plot(time, data[:, 4], label='${} \dot \\theta_1$ ($rad \cdot s^{}$)'.format("\hat" if hat else "", '{-1}'), linestyle='dashed' if hat else '-', color='tab:red')
    if vars[5]: plt.plot(time, data[:, 5], label='${} \dot \\theta_2$ ($rad \cdot s^{}$)'.format("\hat" if hat else "", '{-1}'), linestyle='dashed' if hat else '-', color='tab:olive')
    if vars[6]: plt.plot(time, data[:, 6], label='$u$ ($m \\cdot s^{-2}$)', color='black')
#%%
plotdata = data[run][t0:tf]

fit_res = dict(zip(model.param_names, [fitted_params.params[i].value*scale[idx] for idx, i in enumerate(model.param_names)]))
model.update_params(fit_res)
model.subs_params()
sim_data = model.get_integrator(grid=plotdata[:, 0]-plotdata[0, 0])(x0=list(plotdata[0, 2:8]), u=list(plotdata[:, -1]))['xf'].T

initial_params = dict(zip(model.param_names, [params[i].value*scale[idx] for idx, i in enumerate(model.param_names)]))
model.update_params(initial_params)
model.subs_params()
initial_param_sim = model.get_integrator(grid=plotdata[:, 0]-plotdata[0, 0])(x0=list(plotdata[0, 2:8]), u=list(plotdata[:, -1]))['xf'].T

fig, ax = plt.subplots()

anim = model.animate_data(sim_data[:, 0:3], fig, ax, tf=plotdata[-1, 0], dt=np.mean(np.diff(plotdata[:, 0])), color='tab:blue', show=True)
model.animate_data(plotdata[:, 2:5], fig, ax, tf=plotdata[-1, 0], dt=np.mean(np.diff(plotdata[:, 0])), color='tab:orange', show=True)
plt.show()
#%%
plot_results(plotdata[:, 0], plotdata[:, (2,3,4,5,6,7,1)], vars=[1,1,1,1,1,1,1], hat=False)
plot_results(plotdata[:, 0], sim_data, vars=[1,1,1,0,0,0,0], hat=True)
# plt.plot(plotdata[:, 0][:-1], np.diff(plotdata[:, 3])/np.diff(plotdata[:, 0]))
# plot_results(plotdata[:, 0], initial_param_sim, vars=[0,1,1,0,0,0,0], hat=False)
plt.xlabel('time (s)')
# plt.ylabel('$x$ (m), $\\theta$ (rad), u ($m \\times {s^2}$)')
plt.title('True and Predicted System State vs. Time')
plt.legend()
plt.plot()
print(fit_res)
# %%