#%%
import numpy as np
from MPC_testing_stepper import get_mpc
mpc, simulator, model = get_mpc(0.05, 10, False)
# x0 = np.array([[0.1],[0],[0],[0]])
# mpc.x0 = x0
# simulator.x0
# mpc.set_initial_guess()
pwr = mpc.make_step(np.array([[0], [0.001], [0], [0]]))
print(np.concatenate([
    mpc.data.prediction(('_u', 'f'))[:, :200],
    mpc.data.prediction(('_x', 'y_1'))[:, :200],
    mpc.data.prediction(('_x', 'dy'))[:, :200],
], axis=1))
# %%
