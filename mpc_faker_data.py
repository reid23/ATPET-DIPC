#%%
# x             (-0.4, 0.4)
# x dot         (-0.7, 0.7)
# theta         (0, 2pi)
# theta dot     (-16, 16)
# u             (-1.5, 1.5)

from MPC_testing_stepper import get_mpc
import numpy as np
from multiprocessing import pool
import itertools
#%%
mpc, _, _ = get_mpc(0.05, 3, compile_nlp=False)
params = np.array(list(itertools.product(
    np.linspace(-0.7, 0.7, 10),
    np.linspace(0, 2*np.pi, 10),
    np.linspace(-0.7, 0.7, 10, endpoint=False),
    np.linspace(-20**(1/3), 20**(1/3), 10)**3,
))).reshape(8,1,1250,4)

#%%
# def get_list(x):
#     return list(map(mpc.make_step, x))

# if __name__ == '__main__':
#     with pool.Pool(8) as p:
#         with open('results.txt', 'w') as f:
#             print(list(itertools.chain.from_iterable(p.starmap(get_list, params))), file=f)



# mpc.make_step()
# %%
input_data = params.reshape(10000,4)
with open('results.txt', 'r') as f: 
    output_data = np.array(eval(f.read()))
# %%
y = output_data.reshape(10,10,10,10)
grad = np.gradient(y,     
    np.linspace(-0.7, 0.7, 10),
    np.linspace(0, 2*np.pi, 10),
    np.linspace(-0.7, 0.7, 10, endpoint=False),
    np.linspace(-20**(1/3), 20**(1/3), 10)**3
)

#%%
import matplotlib.pyplot as plt
import matplotlib.animation as anim
fig, ax = plt.subplots(2, 3, sharex=True, sharey=True, layout='tight')
ax[0][0].set_xlabel('dx')
ax[0][0].set_ylabel('dtheta')
ax[0][1].set_xlabel('theta')
ax[0][1].set_ylabel('dtheta')
ax[0][2].set_xlabel('theta')
ax[0][2].set_ylabel('dx')
ax[1][0].set_xlabel('x')
ax[1][0].set_ylabel('dtheta')
ax[1][1].set_xlabel('x')
ax[1][1].set_ylabel('dx')
ax[1][2].set_xlabel('x')
ax[1][2].set_ylabel('theta')

def plot(h):
    for a in ax:
        for b in a:
            b.clear()
    return [
        ax[0][0].imshow(y[h, h, :, :], cmap='gray'),
        ax[0][1].imshow(y[h, :, h, :], cmap='gray'),
        ax[0][2].imshow(y[h, :, :, h], cmap='gray'),
        ax[1][0].imshow(y[:, h, h, :], cmap='gray'),
        ax[1][1].imshow(y[:, h, :, h], cmap='gray'),
        ax[1][2].imshow(y[:, :, h, h], cmap='gray'),
    ]

# plot(5)
ani = anim.FuncAnimation(fig, plot, repeat=True,
                                    frames=10, interval=50)
plt.show()

# %%
