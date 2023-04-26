#%%
from multiprocessing import Pool
import numpy as np
import matplotlib.pyplot as plt
from sim.single_pendulum_model_stepper import dipc_model
import scipy
from time import perf_counter

#%%
with open('double_pend_mpc_data_3.txt', 'r') as f:
    data = np.array(eval(f.read()))

data = data[10:-10, (0, 1, 2, 4, 5, 7)]
data[:, 0] = np.cumsum(data[:, 0]+0.00000035)
data[:, 0] -= data[0, 0] 
# data[:, -1] = np.clip(data[:, -1], -0.5, 0.5)
print(data[-10:])

model = dipc_model().lambdify()
func = model.func
weights = np.array([0.217, 0.125, 0.05, 0.005])
best_cost = 10000000
best_consts = np.ones(4)
dt = 5
def stepper_cost(data, t0, constants):

    # print(istart, iend)
    soln = scipy.integrate.solve_ivp(fun = lambda t, y: func(y, data[np.searchsorted(data[:, 0], t), -1], *(constants*weights), 10, 10).flatten(), 
                                    y0=data[0, 1:5], 
                                    t_span=(data[0, 0], data[0, -1]), 
                                    t_eval=data[:, 0])
    # print(istart, iend, soln)
    print('start shapes:', soln.y.T.shape, data[:, 1:5].shape)
    print('result shape:', (soln.y.T - data[:, 1:5]).shape)
    return np.sum((soln.y.T - data[:, 1:5])**2)

def cost(consts):
    cost = sum(stepper_cost(i, consts) for i in np.arange(0, 200, int(200/64)))
    # cost = sum(np.array(pool.starmap(stepper_cost, [(i, consts) for i in np.arange(0, 225, int(225/16))])))
    global best_cost
    global best_consts
    if best_cost > cost: 
        best_cost = cost
        best_consts = consts*weights
        # print('here!')
    print(cost, consts, best_cost, best_consts)
    return cost

if __name__ == '__main__':
    import pygad

    function_inputs = np.array([1,1,1,1])
    fitness_function = cost

    num_generations = 50
    num_parents_mating = 10

    sol_per_pop = 30
    num_genes = len(function_inputs)

    init_range_low = -2
    init_range_high = 5

    parent_selection_type = "sss"
    keep_parents = 1

    crossover_type = "single_point"

    mutation_type = "random"
    mutation_percent_genes = 30

    ga_instance = pygad.GA(num_generations=num_generations,
                        num_parents_mating=num_parents_mating,
                        fitness_func=fitness_function,
                        sol_per_pop=sol_per_pop,
                        num_genes=num_genes,
                        init_range_low=init_range_low,
                        init_range_high=init_range_high,
                        parent_selection_type=parent_selection_type,
                        keep_parents=keep_parents,
                        crossover_type=crossover_type,
                        mutation_type=mutation_type,
                        mutation_percent_genes=mutation_percent_genes,
                        parallel_processing=['process', 4])

    print(ga_instance.run())
    print(ga_instance.best_solution())

# exit()
# try:
#     print(scipy.optimize.minimize(cost, np.ones(4)))
# except KeyboardInterrupt:
#     pass



# %%
#[ 1.41713816,  3.48891797, -1.69523664,  1.60384412][0.217, 0.125, 0.05, 0.005]
plt.plot(data[:450, 0], data[:450, 1], label='x')
plt.plot(data[:450, 0], data[:450, 2], label='theta')
plt.plot(data[:450, 0], data[:450, 3], label='x dot')
plt.plot(data[:450, 0], data[:450, 4], label='theta dot')
plt.plot(data[:450, 0], data[:450, 5], label='u')

soln = scipy.integrate.solve_ivp(fun = lambda t, y: func(y, data[np.searchsorted(data[:, 0], t), -1], *(best_consts), 10, 10).flatten(), 
                                    y0=data[0, 1:5], 
                                    t_span=(0, 5), 
                                    t_eval=data[4:430, 0])
plt.plot(soln.t, soln.y.T[:, 0], linestyle='dashed', label='x')
plt.plot(soln.t, soln.y.T[:, 1], linestyle='dashed', label='theta')
plt.plot(soln.t, soln.y.T[:, 2], linestyle='dashed', label='x dot')
plt.plot(soln.t, soln.y.T[:, 3], linestyle='dashed', label='theta dot')
plt.legend()
plt.show()