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
def get_list(x):
    return list(map(mpc.make_step, x))

if __name__ == '__main__':
    with pool.Pool(8) as p:
        with open('results.txt', 'w') as f:
            print(list(itertools.chain.from_iterable(p.starmap(get_list, params))), file=f)



# mpc.make_step()
# %%
