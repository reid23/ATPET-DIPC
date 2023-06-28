#%%
import numpy as np
from single_pend_SI_stepper import func
import scipy
from lmfit import Parameters, minimize, fit_report
from matplotlib import pyplot as plt
from IPython.display import clear_output
import sys
sys.path.append('../')
from casadi_testing import get_integrator
#%%
if False:
    with open('../final_data_for_plotting_mpc.txt', 'r') as f:
        data = np.array(eval(f.read()))

    data = data[10:-10, (0, 1, 2, 4, 5, 7)]
    data[:, 0] = np.cumsum(data[:, 0]+0.00000035)
    data[:, 0] -= data[0, 0] 
else:
    with open('../ethan_data.txt', 'r') as f:
        data = np.array(eval(f.read()))

    data = data[10:-10, (0, 2, 3, 5, 6, 1)]
    data[:, 0] -= data[0, 0] 
    data[:, 0] /= 1e9
    data[:, 5] /= 1e4

print('data loaded')

from casadi import DaeBuilder, sin, cos, pi, integrator, vertcat, SX, MX
import numpy as np
import casadi
#%%
# l, ma, mb, I, c = [0.23116035, 0.00625   , 0.05      , 0.        , 0.10631411]
p = MX.sym('p', 6)
l, ma, mb, I, c = p[0], p[1], p[2], p[3], p[4]
f = p[5]#MX.sym('f')
y = MX.sym('y', 4)

# ydot = vertcat(
#     y[2],
#     y[3],
#     f,
#     -c*y[3] + (-9.8*l*mb*sin(y[1]) - l*mb*(l*mb*y[3]**2*sin(y[1]) + (-I*l*mb*y[3]**2*sin(y[1]) + I*ma*f + I*mb*f - l**3*mb**2*y[3]**2*sin(y[1]) + l**2*ma*mb*f - l**2*mb**2*f*cos(y[1])**2 + l**2*mb**2*f - 4.9*l**2*mb**2*sin(2.0*y[1]))/(I + l**2*mb))*cos(y[1])/(ma + mb))/(I - l**2*mb**2*cos(y[1])**2/(ma + mb) + l**2*mb)
# )

p = MX.sym('p', 2)
f = MX.sym('f', len(data))
t = MX.sym('t', 1)
l, c = p[0], p[1]
ydot = vertcat(
    y[2],
    y[3],
    f,
    -c*y[3] + (-9.8*sin(y[1])-f*cos(y[1]))/l
)
ode = {'x':y, 'p': [p, f], 't':t, 'ode': ydot}

intfunc = integrator('f_i', 'rk', ode, dict(grid=data[:, 0]))

def eval_ode_casadi(t_step, t_f, x0=np.array([0,np.pi/2,0,0]), p=np.array([0.23116035, 0.00625   , 0.05      , 0.        , 0.10631411, 0])):
    int_func = integrator('F_i', 'cvodes', ode, dict(t0=0, tf=t_step))
    # x0 = np.array([0,np.pi/2,0,0])
    p=np.array([0.23116035, 0.00625   , 0.05      , 0.        , 0.10631411, 0])
    res = []
    for i in range(int(t_f/t_step)):
        x0 = int_func(x0=x0, p=p)['xf']
        res.append(x0)
    return np.array(res)

#%%
n=0
params = Parameters()
params.add('l', value=21.1715266, min=5, max=30)
# params.add('ma', value=1, min=0, max=60, vary=False)
# params.add('mb', value=41.8029909, min=0, max=60, vary=True)
# params.add('mb', value=20, min=0, max=60, vary=True)
# params.add('I', value=0.06439586, min=0, max=60, vary=True)
# params.add('I', value=0, min=0, max=40, vary=True)
params.add('c', value=21.2145859, min=0)
params.add('offset', value=0, min=-0.03, max=0.03)

def func(y0, y1, dy0, dy1, f, l, c):
    return np.array([
        dy0,
        dy1,
        f,
        -c*dy1 + (-9.8*sin(y1)-f*cos(y1))/l
    ])


def lmfit_func(params, data):
    global n
    consts = np.array([0.217, 0.1])*np.array([params['l'], params['c']])/20
    y_fit = scipy.integrate.solve_ivp(lambda t, y: func(*y, data[np.searchsorted(data[:, 0], t), -1], *consts).flatten(),
                                        y0=data[0, 1:5],
                                        t_span=(data[0, 0], data[-1, 0]),
                                        t_eval=data[:, 0],
                                        max_step=0.005)
    # y_fit = eval_ode_casadi(t_step=0.01, t_f=data[-1, 0], x0=data[0, 1:5], p=consts)
    # intfunc = get_integrator(data[:, 0], p=consts)
    # y_fit = intfunc()
    clear_output()
    print(f'function has been called {n} times')
    n+=1
    res = y_fit.y.T
    res[:, 3] -= params['offset']
    return res - data[:, 1:5]

t_0 = 1000
t_f = 1400

t_0 = 10
t_f = 230

fitted_params = minimize(
    lmfit_func, 
    params, 
    args=(data[t_0:t_f],), 
    method = "least_squares",
)
print(fitted_params.__dict__)
print(fit_report(fitted_params))
# %%
plotdata = data[t_0:t_f]
plt.plot(plotdata[:, 0], plotdata[:, 1], label='$x$')
plt.plot(plotdata[:, 0], plotdata[:, 2]/5, label='$\\theta$')
plt.plot(plotdata[:, 0], plotdata[:, 3], label='$\dot x$')
# plt.plot(plotdata[:tfinal, 0], plotdata[:tfinal, 4], label='theta dot')
# plt.plot(plotdata[:tfinal, 0], plotdata[:tfinal, 5], label='u')

fit_result = np.array([0.217, 0.1])*np.array([
    fitted_params.params['l'].value,
    # fitted_params.params['ma'].value,
    # fitted_params.params['mb'].value,
    # fitted_params.params['I'].value,
    fitted_params.params['c'].value,
])/20

soln = scipy.integrate.solve_ivp(fun = lambda t, y: func(*y, plotdata[np.searchsorted(plotdata[:, 0], t), -1], *fit_result).flatten(), 
                                    y0=plotdata[0, 1:5], 
                                    t_span=(plotdata[0, 0], plotdata[-1, 0]), 
                                    t_eval=plotdata[:, 0],
                                    max_step=0.005)
plt.plot(soln.t, soln.y.T[:, 0], linestyle='dashed', label='$\hat x$')
plt.plot(soln.t, soln.y.T[:, 1]/5, linestyle='dashed', label='$\hat \\theta$')
plt.plot(soln.t, soln.y.T[:, 2], linestyle='dashed', label='$\hat \dot x$')
# plt.plot(soln.t, soln.y.T[:, 3], linestyle='dashed', label='theta dot')
plt.title(f'Predicted vs. Real State over Time\nParameters: {list(np.round(fit_result, 6))}')
plt.legend()
# %%
