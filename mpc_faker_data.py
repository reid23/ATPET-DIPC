#%%
# x             (-0.4, 0.4)
# x dot         (-0.7, 0.7)
# theta         (0, 2pi)
# theta dot     (-16, 16)
# u             (-1.5, 1.5)

from MPC_testing_stepper import get_mpc
import numpy as np
from multiprocessing import Pool
import itertools
#%%
mpc, _, _ = get_mpc()
params = np.array(list(itertools.product(
    np.linspace(-0.4, 0.4, 10),
    np.linspace(-0.7, 0.7, 10),
    np.linspace(0, 2*np.pi, 100),
    np.linspace(-20**(1/3), 20**(1/3), 64)**3,
    np.linspace(-1.5**(1/3), 1.5**(1/3), 50)**3,
))).reshape(64,500000,5,1)

#%%
def get_list(x):
    return list(map(mpc.make_step, x))
with Pool(64) as p:
    with open('results.txt', 'w') as f:
        print(sum(p.starmap(get_list, params)), file=f)



# mpc.make_step()
# %%
