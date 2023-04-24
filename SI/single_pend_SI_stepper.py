#%%
from multiprocessing import Pool
import numpy as np
import matplotlib.pyplot as plt
from sim.single_pendulum_model_stepper import dipc_model
import scipy
from time import perf_counter

#%%
with open('mpc_data.txt', 'r') as f:
    data = np.array(eval(f.read()))

data = data[10:-10]
data[0, :] -= data[0, 0] 


model = dipc_model().lambdify()
func = model.func
weights = np.array([0.217, 0.125, 0.05, 0.005])
best_cost = 10000000
best_consts = np.ones(4)
dt = 2
def stepper_cost(t0, constants):
    istart, iend = np.searchsorted(data[:, 0], t0), np.searchsorted(data[:, 0], t0+dt)
    # print(istart, iend)
    soln = scipy.integrate.solve_ivp(fun = lambda t, y: func(y, data[np.searchsorted(data[:, 0], t), -1], *(constants*weights), 10, 10).flatten(), 
                                    y0=data[istart, 1:5], 
                                    t_span=(t0, t0+dt), 
                                    t_eval=data[istart:iend, 0])
    # print(istart, iend, soln)
    return np.sum((soln.y.T - data[istart:iend, 1:5])**2)

def cost(consts):
    cost = sum(stepper_cost(i, consts) for i in np.arange(0, 225, int(225/16)))
    # cost = sum(np.array(pool.starmap(stepper_cost, [(i, consts) for i in np.arange(0, 225, int(225/16))])))
    global best_cost
    global best_consts
    if best_cost > cost: 
        best_cost = cost
        best_consts = consts
        # print('here!')
    print(cost, consts, best_cost, best_consts)
    return cost

# if __name__ == '__main__':
#     import pygad

#     function_inputs = np.array([1,1,1,1])
#     fitness_function = cost

#     num_generations = 50
#     num_parents_mating = 10

#     sol_per_pop = 30
#     num_genes = len(function_inputs)

#     init_range_low = -2
#     init_range_high = 5

#     parent_selection_type = "sss"
#     keep_parents = 1

#     crossover_type = "single_point"

#     mutation_type = "random"
#     mutation_percent_genes = 30

#     ga_instance = pygad.GA(num_generations=num_generations,
#                         num_parents_mating=num_parents_mating,
#                         fitness_func=fitness_function,
#                         sol_per_pop=sol_per_pop,
#                         num_genes=num_genes,
#                         init_range_low=init_range_low,
#                         init_range_high=init_range_high,
#                         parent_selection_type=parent_selection_type,
#                         keep_parents=keep_parents,
#                         crossover_type=crossover_type,
#                         mutation_type=mutation_type,
#                         mutation_percent_genes=mutation_percent_genes,)
                        # parallel_processing=['process', 4])

    # print(ga_instance.run())
    # print(ga_instance.best_solution())

# exit()
try:
    print(scipy.optimize.minimize(cost, np.ones(4)))
except:
    pass



# %%
plt.plot(data[:450, 0], data[:450, 1:5])
soln = scipy.integrate.solve_ivp(fun = lambda t, y: func(y, data[np.searchsorted(data[:, 0], t), -1], *np.array([0.217, 0.125, 0.05, 0.005]), 10, 10).flatten(), 
                                    y0=data[0, 1:5], 
                                    t_span=(0, 5), 
                                    t_eval=data[4:430, 0])
plt.plot(soln.t, soln.y.T, linestyle='dashed')
plt.show()