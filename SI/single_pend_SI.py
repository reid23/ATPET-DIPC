#%%
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
#%%
with open('data2', 'r') as f:
    data = list(map(np.array, eval(f.read())))

# %%
trials = 2
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
    ax[i][0].plot(data[i][:, 1]*np.pi/180, label = '$\\theta$')
    # ax[i][1].plot(np.diff(np.convolve(data[i][:, 1]*np.pi/180, np.ones(5)/5, 'same')), label = '$\dot \\theta$')
    ax[i][1].plot(data[i][:, 4]*np.pi/180, label = '$\dot \\theta$')
    ax[i][0].xaxis.set_major_formatter(ticker.FuncFormatter(lambda x, pos: '{0:g}'.format(x/47.44)))
    ax[i][1].xaxis.set_major_formatter(ticker.FuncFormatter(lambda x, pos: '{0:g}'.format(x/47.44)))
fig.tight_layout()
try:
    plt.show()
except:
    plt.close()

# L = 0.22403
# %%
