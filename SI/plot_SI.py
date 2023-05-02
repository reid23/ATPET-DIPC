import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
from single_pend_SI_stepper import func

with open('double_pend_mpc_data_3.txt', 'r') as f:
    data = np.array(eval(f.read()))

data = data[10:-10, (0, 1, 2, 4, 5, 7)]
data[:, 0] = np.cumsum(data[:, 0]+0.00000035)
data[:, 0] -= data[0, 0] 
# data[:, -1] = np.zeros(len(data))
print(data.shape)

plt.plot(data[:250, 0], data[:250, 1], label='x')
plt.plot(data[:250, 0], data[:250, 2], label='theta')
plt.plot(data[:250, 0], data[:250, 3], label='x dot')
plt.plot(data[:250, 0], data[:250, 4], label='theta dot')
plt.plot(data[:250, 0], data[:250, 5], label='u')

soln = sp.integrate.solve_ivp(fun = lambda t, y: func(*y, data[np.searchsorted(data[:, 0], t), -1], *[0.22698345, 0.19621574, 0.05477813, 0.00604388]), 
                                    y0=data[0, 1:5], 
                                    t_span=(0, 5))
                                    # t_eval=data[4:430, 0])
plt.plot(soln.t, soln.y.T[:, 0], linestyle='dashed', label='x')
plt.plot(soln.t, soln.y.T[:, 1], linestyle='dashed', label='theta')
plt.plot(soln.t, soln.y.T[:, 2], linestyle='dashed', label='x dot')
plt.plot(soln.t, soln.y.T[:, 3], linestyle='dashed', label='theta dot')
plt.legend()
plt.show()