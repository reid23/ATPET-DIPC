#%%
from multiprocessing import Pool, freeze_support
import numpy as np
import matplotlib.pyplot as plt
from sim.single_pendulum_model_stepper import dipc_model
import scipy
from time import perf_counter
import pygad
from numpy import sin, cos

def func(y0, y1, dy0, dy1, f, l, ma, mb, I):
    return np.array([
        dy0,
        dy1,
        f,
        l*mb*(-9.8*(I + l**2*mb)*(ma + mb)*sin(y1) - (1.0*I*ma*f + 1.0*I*mb*f + 1.0*l**2*ma*mb*f + 1.0*l**2*mb**2*f*sin(y1)**2 - 4.9*l**2*mb**2*sin(2.0*y1))*cos(y1))/((I + l**2*mb)*(-l**2*mb**2*cos(y1)**2 + (I + l**2*mb)*(ma + mb))) 
    ])
#%%
def stepper_cost(data, constants):
    # print('stepper_cost', data.shape)
    soln = scipy.integrate.solve_ivp(fun = lambda t, y: func(*y, data[np.searchsorted(data[:, 0], t), -1], *(constants*np.array([0.217, 0.125, 0.05, 0.005])/20)), 
                                    y0=data[0, 1:5], 
                                    t_span=(data[0, 0], data[-1, 0]), 
                                    t_eval=data[:, 0])
    # print(istart, iend, soln)
    # print('start shapes:', soln.y.T.shape, data[:, 1:5].shape)
    # print('result shape:', (soln.y.T - data[:, 1:5]).shape)
    return np.sum((soln.y.T - data[:, 1:5])**2)

def cost(data, consts, pool):
    cost = sum(pool.starmap(stepper_cost, [(data[int(start):int(start+3/0.015)], consts) for start in np.linspace(0, len(data), 64, endpoint=False)]))
    print(cost, consts*np.array([0.217, 0.125, 0.05, 0.005])/20)
    return cost

if __name__ == '__main__':
    freeze_support()

    with open('double_pend_mpc_data_3.txt', 'r') as f:
        data = np.array(eval(f.read()))

    data = data[10:-10, (0, 1, 2, 4, 5, 7)]
    data[:, 0] = np.cumsum(data[:, 0]+0.00000035)
    data[:, 0] -= data[0, 0] 
    # data[:, -1] = np.clip(data[:, -1], -0.5, 0.5)
    print(data[-10:])
    print('here')
    with Pool(64) as p:
        def fitness_func(a, consts, b):
            print('here')
            res = 10000/cost(data, consts, p)
            print(consts, res)
            return res
        # def func(a, consts, b): return 10000/cost(data, consts, model.func)
        function_inputs = np.array([1,1,1,1])*20
        fitness_function = fitness_func

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
                            mutation_percent_genes=mutation_percent_genes)

        print(ga_instance.run())
        print(ga_instance.best_solution())

    # exit()
    # try:
    #     print(scipy.optimize.minimize(cost, np.ones(4), args=()))
    # except KeyboardInterrupt:
    #     pass

    # %%
#[ 1.41713816,  3.48891797, -1.69523664,  1.60384412][0.217, 0.125, 0.05, 0.005]
# exit()