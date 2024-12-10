#%%
import numpy as np
import matplotlib.pyplot as plt

#%%
with open('devttyACM0_2024_12_10.00.47.14.553.txt', 'r') as f:
    def eval_handle_error(s):
        try:
            out = list(eval(s).values())
            assert len(out)==9
            return out
        except:
            return None
    data = np.array(list(filter(lambda x: x is not None, [eval_handle_error(i) for i in f.readlines()])))

# %%
fig, axs = plt.subplots(2, sharex='all')
axs[0].plot(data[:, 0], data[:, 2], label="$x$")
axs[0].plot(data[:, 0], data[:, 4], label="$\\theta_1$")
axs[0].plot(data[:, 0], data[:, 6], label="$\\theta_2$")
axs[1].plot(data[:, 0], data[:, 3], label='$\\dot x$')
axs[1].plot(data[:, 0], data[:, 5], label='$\\dot \\theta_1$')
axs[1].plot(data[:, 0], data[:, 7], label='$\\dot \\theta_2$')
axs[0].legend()
axs[1].legend()
plt.show()
# %%
