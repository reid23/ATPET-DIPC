with open('double_pend_mpc_data_3.txt', 'r') as f:
    data = np.array(eval(f.read()))

data = data[10:-10, (0, 1, 2, 4, 5, 7)]
data[:, 0] = np.cumsum(data[:, 0]+0.00000035)
data[:, 0] -= data[0, 0] 

print('data loaded')

n=0

from lmfit import Parameters, minimize, fit_report
params = Parameters()
params.add('l', value=20, min=10)
params.add('ma', value=20, min=0)
params.add('mb', value=20, min=0)
params.add('I', value=20, min=0)
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

fitted_params = minimize(lmfit_func, params, args=(data[:400],), method = "leastsq")
print(fitted_params.__dict__)