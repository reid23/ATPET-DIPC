#%%
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from sim.single_pendulum_model import dipc_model
from scipy.optimize import minimize, curve_fit
from scipy.interpolate import interp1d
#%%
with open('SI/data2.txt', 'r') as f:
    data = list(map(np.array, eval(f.read())))
model = dipc_model()
for i in range(len(data)):
    data[i] = data[i][10:-10, (0,1,2,3,5,6)]
    data[i][:, 0] -= data[i][0, 0]
    data[i][:, 0] *= 1e-9
    model.K[f'FF{i}'] = lambda t: data[i][np.argmin(data[i][:, 0] - t), 1]
    

#%%
def cost(consts):
    model.update_constants_and_relineaarize(consts)
    # error = # figure out interpolation of data to get least squares error accumulation
    for counter, trial in enumerate(data):
        model.integrate_with_scipy(y_0 = trial[0, 2:], controller=f'FF{counter}')
    

# %%
trials = len(data)
fig, ax = plt.subplots(trials, 2, sharex=True)
fig.suptitle("Pendulum Angle and Velocity vs. Time (s)")
ax[0][0].set_title("Position (rad)")
ax[0][1].set_title("Velocity (rad/s)")
for i in range(trials):
    ax[i][0].axvline(1*47.44, color='orange', linestyle='dashed')
    ax[i][0].axvline(1.5*47.44, color='orange', linestyle='dashed')
    ax[i][1].axvline(1*47.44, color='orange', linestyle='dashed')
    ax[i][1].axvline(1.5*47.44, color='orange', linestyle='dashed')
    # data[i][:, 1][data[i][:, 1] > 100] -= 5400
    ax[i][0].plot(data[i][:, 2], label = '$\\theta$')
    # ax[i][1].plot(np.diff(np.convolve(data[i][:, 1]*np.pi/180, np.ones(5)/5, 'same')), label = '$\dot \\theta$')
    ax[i][1].plot(data[i][:, 5], label = '$\dot \\theta$')
    ax[i][0].xaxis.set_major_formatter(ticker.FuncFormatter(lambda x, pos: '{0:g}'.format(x/47.44)))
    ax[i][1].xaxis.set_major_formatter(ticker.FuncFormatter(lambda x, pos: '{0:g}'.format(x/47.44)))
fig.tight_layout()
try:
    plt.show()
except:
    plt.close()

# L = 0.22403
# %%
# constants:


# f_motor, in newtons, = 20.909*V_actual (found experimentally from stall torque)
# V_actual = V_applied - K_e*w # w (omega) is rotational speed of motor
# V_actual = V_applied - K_e * x' / (0.12*16)
# Torque = K_e * i, so 0.38 N-m = K_e * 84 A, so K_e = 0.38/84 = 0.00452381 N*m*A^-1
# i = V_actual / R_armature = (V_applied - K_e * x'/(0.12*16))/R_armature
# R_armature = V/I = 0.14285714
# F = 20.909*(V_applied - 0.00452381*(1/(0.12*16)*x')
# F = 20.909*V_applied - 0.04926476x'

# ...assuming datasheet is right and this is a global property
# but this gives me confidence at least that the general form is
# F = k_V(V_applied) - k_W(x')

# oh also oops this assumes voltage varies from 0..1, it's really 0..12, so k_V = 1.74242

# Constants are:
#   k_V
#   k_W
#   m_c
#   m_p
#   L
#   k_F



