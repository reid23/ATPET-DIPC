#%%
from multiprocessing import Pool, freeze_support
import numpy as np
import matplotlib.pyplot as plt
from sim.single_pendulum_model_stepper import dipc_model
import scipy
from time import perf_counter
import pygad
from numpy import sin, cos
import wandb

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
    print('sim:', soln.y.T.shape)
    print('irl:', data.shape)
    return np.sum((soln.y.T[:, 1] - data[:, 2])**2)
best_cost = np.Inf
best_consts = []
gen_best_cost = np.Inf
gen_best_consts = []
def cost(data, consts, pool):
    global best_cost
    global best_consts
    global gen_best_cost
    global gen_best_consts
    cost = sum(pool.starmap(stepper_cost, [(data[int(start):int(start+5/0.015)], consts) for start in np.linspace(0, len(data), 64, endpoint=False)]))
    #print(cost, consts*np.array([0.217, 0.125, 0.05, 0.005])/20)
    if cost < best_cost:
        best_cost = cost
        best_consts = np.array(consts)*np.array([0.217, 0.15, 0.05, 0.005])/20
        print(best_cost, best_consts)
    if cost < gen_best_cost:
        gen_best_cost = cost
        gen_best_consts = np.array(consts)*np.array([0.217, 0.15, 0.05, 0.005])/20
    return cost

def on_gen(ga_instance):
    global gen_best_cost
    global gen_best_consts
    
    plt.clf()

    plt.plot(data[:450, 0], data[:450, 1], label='x')
    plt.plot(data[:450, 0], data[:450, 2], label='theta')
    # plt.plot(data[:450, 0], data[:450, 3], label='x dot')
    # plt.plot(data[:450, 0], data[:450, 4], label='theta dot')
    plt.plot(data[:450, 0], data[:450, 5], label='u')

    soln = scipy.integrate.solve_ivp(fun = lambda t, y: func(*y, data[np.searchsorted(data[:, 0], t), -1], *(best_consts)).flatten(), 
                                        y0=data[0, 1:5], 
                                        t_span=(0, data[451, 0]), 
                                        t_eval=data[:450, 0])
    plt.plot(soln.t, soln.y.T[:, 0], linestyle='dashed', label='x (pred)')
    plt.plot(soln.t, soln.y.T[:, 1], linestyle='dashed', label='theta (pred)')
    # plt.plot(soln.t, soln.y.T[:, 2], linestyle='dashed', label='x dot')
    # plt.plot(soln.t, soln.y.T[:, 3], linestyle='dashed', label='theta dot')
    plt.legend()

    l, ma, mb, I = gen_best_consts
    wandb.log({
        'cost': gen_best_cost, 
        'l': l, 
        'ma': ma, 
        'mb': mb, 
        'I': I, 
        'graph': plt,
    })

    print("Generation", ga_instance.generations_completed)
    gen_best_cost = np.Inf
    gen_best_consts = []
    plt.cla()
    plt.clf()
    # best_soln = ga_instance.best_solution()
    # print("Best Solution:", np.array(best_soln[0])*np.array([0.217, 0.125, 0.05, 0.005])/20)
    # print("Best Solution Cost:", best_soln[1])

if __name__ == '__main__':
    freeze_support()
    wandb.init(project='ATPET-DIPC')

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
            #print('here')
            res = -cost(data, consts, p)
            #print(consts, res)
            return res
        # def func(a, consts, b): return 10000/cost(data, consts, model.func)
        function_inputs = np.array([1,1,1,1])
        fitness_function = fitness_func

        num_generations = 1000
        num_parents_mating = 40

        sol_per_pop = 100
        num_genes = len(function_inputs)

        init_range_low = 15
        init_range_high = 25

        parent_selection_type = "sss"
        keep_parents = 1

        crossover_type = "single_point"

        mutation_type = "random"
        mutation_percent_genes = 50

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
                            on_generation=on_gen)

        print(ga_instance.run())
        print(ga_instance.best_solution())
        wandb.finish()

    # exit()
    # try:
    #     print(scipy.optimize.minimize(cost, np.ones(4), args=()))
    # except KeyboardInterrupt:
    #     pass

    # %%
#[ 1.41713816,  3.48891797, -1.69523664,  1.60384412][0.217, 0.125, 0.05, 0.005]
# exit()
# plt.plot(data[:450, 0], data[:450, 1], label='x')
# plt.plot(data[:450, 0], data[:450, 2], label='theta')
# plt.plot(data[:450, 0], data[:450, 3], label='x dot')
# plt.plot(data[:450, 0], data[:450, 4], label='theta dot')
# plt.plot(data[:450, 0], data[:450, 5], label='u')

# soln = scipy.integrate.solve_ivp(fun = lambda t, y: func(y, data[np.searchsorted(data[:, 0], t), -1], *(best_consts), 10, 10).flatten(), 
#                                     y0=data[0, 1:5], 
#                                     t_span=(0, 5), 
#                                     t_eval=data[4:430, 0])
# plt.plot(soln.t, soln.y.T[:, 0], linestyle='dashed', label='x')
# plt.plot(soln.t, soln.y.T[:, 1], linestyle='dashed', label='theta')
# plt.plot(soln.t, soln.y.T[:, 2], linestyle='dashed', label='x dot')
# plt.plot(soln.t, soln.y.T[:, 3], linestyle='dashed', label='theta dot')
# plt.legend()
# plt.show()