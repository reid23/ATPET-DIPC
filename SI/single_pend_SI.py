#%%
import numpy as np
import matplotlib.pyplot as plt
#%%
with open('data', 'r') as f:
    data = list(map(np.array, eval(f.read())))

# %%
fig, ax = plt.subplots(7, 1)
for i in range(7):
    ax[i].plot(data[i][:, 1])
# %%
