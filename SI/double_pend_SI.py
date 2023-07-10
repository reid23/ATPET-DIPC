#%%
from sim.double_pend import double_pend_model as Model
from lmfit import Parameters, minimize, fit_report
from matplotlib import pyplot as plt
from IPython.display import clear_output
import numpy as np
#%%
with open('../data/dp_run1.txt', 'r') as f:
    data = np.array(eval(f.read()))
#%%
model = Model(sage_saved_f_path='sim/f_solved.txt')

# %%
params = Parameters()
for i in model.param_names:
    params.add(i, min=0)
#* names: c, l, I, a, m
#* top = 0, end = 1
params['l0'].vary = False
params['a1'].vary = False
params['I1'].vary = False
params['a0'].vary = False
params['l0'].value = 0.3048 # 1 ft
params['a1'].value = 0.125323
params['I1'].value = 5.999e-4
params['a0'].value = 0.155 # not confident
# %%
