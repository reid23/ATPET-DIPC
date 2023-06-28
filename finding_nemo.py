#%%
import numpy as np
import matplotlib.pyplot as plt
#%%
# with open('../ethan_data.txt', 'r') as f:
with open('ethan_data.txt', 'r') as f:
    data = np.array(eval(f.read()))

data = data[10:-10, (0, 2, 3, 5, 6, 1)]
data[:, 0] -= data[0, 0] 
data[:, 0] /= 1e9
data[:, 5] /= 1e4

#%%
plt.plot(data[:, 0], data[:, 1], label='$x$')
plt.plot(data[:, 0], data[:, 2], label='$\\theta$')
plt.plot(data[:, 0], data[:, 3], label='$\dot x$')
plt.plot(data[:, 0], data[:, 4]/5, label='$\dot \\theta$')
plt.plot(data[:, 0], data[:, 5], label='$u$')
plt.legend()


# %%
