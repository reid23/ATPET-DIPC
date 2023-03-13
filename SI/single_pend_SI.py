#%%
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
#%%
with open('data', 'r') as f:
    data = list(map(np.array, eval(f.read())))

# %%
fig, ax = plt.subplots(7, 2, sharex=True)
for i in range(7):
    data[i][:, 1][data[i][:, 1] > 100] -= 5400
    ax[i][0].plot(data[i][:, 1]*np.pi/180, label = '$\\theta$')
    ax[i][1].plot(np.diff(np.convolve(data[i][:, 1]*np.pi/180, np.ones(5)/5, 'same')), label = '$\dot \\theta$')
    ax[i][0].xaxis.set_major_formatter(ticker.FuncFormatter(lambda x, pos: '{0:g}'.format(x/47.44)))
    ax[i][1].xaxis.set_major_formatter(ticker.FuncFormatter(lambda x, pos: '{0:g}'.format(x/47.44)))

fig.legend()
plt.show()

# L = 0.22403
# %%
