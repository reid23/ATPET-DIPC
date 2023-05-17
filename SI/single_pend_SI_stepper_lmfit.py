#%%
import numpy as np
from single_pend_SI_stepper import func
import scipy
from lmfit import Parameters, minimize, fit_report
from matplotlib import pyplot as plt
#%%
with open('../final_data_for_plotting_mpc.txt', 'r') as f:
    data = np.array(eval(f.read()))

data = data[10:-10, (0, 1, 2, 4, 5, 7)]
data[:, 0] = np.cumsum(data[:, 0]+0.00000035)
data[:, 0] -= data[0, 0] 

print('data loaded')

#%%
n=0
params = Parameters()
params.add('l', value=20, min=5, max=30)
params.add('ma', value=20, min=0, max=60)
params.add('mb', value=20, min=0, max=60)
params.add('I', value=20, min=0, max=60)
params.add('c', value=20, min=0)
def lmfit_func(params, data):
    global n
    consts = np.array([0.217, 0.125, 0.05, 0.005, 0.1])*np.array([params['l'], params['ma'], params['mb'], params['I'], params['c']])/20
    y_fit = scipy.integrate.solve_ivp(lambda t, y: func(*y, data[np.searchsorted(data[:, 0], t), -1], *consts).flatten(),
                                        y0=data[0, 1:5],
                                        t_span=(0, data[-1, 0]),
                                        t_eval=data[:, 0])
    print(f'here! {n}')
    n+=1
    return y_fit.y.T - data[:, 1:5]

fitted_params = minimize(
    lmfit_func, 
    params, 
    args=(data[:600],), 
    method = "powell",
)
print(fitted_params.__dict__)
print(fit_report(fitted_params))
# %%
tfinal = 600
plt.plot(data[:tfinal, 0], data[:tfinal, 1], label='x')
plt.plot(data[:tfinal, 0], data[:tfinal, 2], label='theta')
# plt.plot(data[:tfinal, 0], data[:tfinal, 3], label='x dot')
# plt.plot(data[:tfinal, 0], data[:tfinal, 4], label='theta dot')
plt.plot(data[:tfinal, 0], data[:tfinal, 5], label='u')

fit_result = np.array([0.217, 0.125, 0.05, 0.005, 0.1])*np.array([
    fitted_params.params['l'].value,
    fitted_params.params['ma'].value,
    fitted_params.params['mb'].value,
    fitted_params.params['I'].value,
    fitted_params.params['c'].value,
])/20

soln = scipy.integrate.solve_ivp(fun = lambda t, y: func(*y, data[np.searchsorted(data[:, 0], t), -1], *fit_result).flatten(), 
                                    y0=data[0, 1:5], 
                                    t_span=(0, data[tfinal, 0]), 
                                    t_eval=data[:tfinal, 0])
plt.plot(soln.t, soln.y.T[:, 0], linestyle='dashed', label='x (pred)')
plt.plot(soln.t, soln.y.T[:, 1], linestyle='dashed', label='theta (pred)')
# plt.plot(soln.t, soln.y.T[:, 2], linestyle='dashed', label='x dot')
# plt.plot(soln.t, soln.y.T[:, 3], linestyle='dashed', label='theta dot')
plt.legend()
# %%
