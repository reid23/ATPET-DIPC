#%%
from multiprocessing import Pool
import numpy as np
import matplotlib.pyplot as plt
from sim.single_pendulum_model import dipc_model
import scipy
from time import perf_counter

from casadi import DaeBuilder, sin, cos, pi, integrator, vertcat

d = DaeBuilder()
l = d.add_p('l')
ma = d.add_p('ma')
mb = d.add_p('mb')
ke = d.add_p('ke')
kf = d.add_p('kf')
f = d.add_u('f')

y = [d.add_x('x'), d.add_x('theta')]
dy = [d.add_x('dx'), d.add_x('dtheta')]

ydot = dy
dydot = [
    (296.296296296296*pi*ke*(-ke*dy[0]+12*f)-kf*dy[0]-((l*mb*(-9.8*l*mb*sin(y[1])-(l*mb*(296.296296296296*pi*ke*(-ke*dy[0]+12*f)-kf*dy[0]+l*mb*(dy[1]**2)*sin(y[1])))*cos(y[1])))/((-((l**2)*(mb**2)*(cos(y[1])**2))/(ma+mb))+(l**2)*mb)) + l*mb*(dy[1]**2)*sin(y[1]))/(ma+mb),
    ((-9.8*l*mb*sin(y[1]))-((l*mb*(296.296296296296*pi*ke*(-ke*dy[0]+12*f)-kf*dy[0]+l*mb*(dy[1]**2)*sin(y[1])))/(ma+mb)))/((-((l**2)*(mb**2)*(cos(y[1])**2))/(ma+mb))+(l**2)*mb)
]

d.add_ode('xdot', ydot[0])
d.add_ode('thetadot', ydot[1])
d.add_ode('dxdot', dydot[0])
d.add_ode('dthetadot', dydot[1])

d.set_start('x', 0)
d.set_start('theta', pi/2)
d.set_start('dx', 0)
d.set_start('dtheta', 0)

print(d)

func = d.create('f', ['x', 'u', 'p'], ['ode'])

#%%
with open('data4.txt', 'r') as f:
    data = list(map(np.array, eval(f.read())))

model = dipc_model().lambdify()
interpolators = []
powers = []
splines = np.empty((len(data), 6), dtype=object)
for i in range(len(data)):
    data[i] = data[i][10:-10, (0,1,2,3,5,6)]
    data[i][:, 0] -= data[i][0, 0]
    data[i][:, 0] *= 1e-9


#%%
def single_cost(trial, constants):
    soln = scipy.integrate.solve_ivp(lambda t, y: np.array(func(y, data[trial][np.searchsorted(data[trial][:, 0], t)-1, 1], constants)).flatten(), (0, data[trial][-1, 0]), data[trial][0, 2:], t_eval = data[trial][:, 0])
    return np.sum((soln.y.T[:, (1,2,3)] - data[trial][:, (3,4,5)])**2)

def cost(consts, pool, trials_to_use=[0]):
    return sum(np.array(pool.starmap(single_cost, [(i, consts) for i in trials_to_use])))
    error = 0
    for counter, trial in enumerate([data[0]]):
        model.integrate_with_scipy(y_0 = trial[0, 2:], controller=lambda t: powers3(t, powers[counter]), tspan = max(trial[:, 0]), constants = consts)
        error += np.linalg.norm(np.linalg.norm(model.soln(trial[:, 0])[0:4].T - trial[:, 2:6], axis=1)**2)**2
    # print(consts, error)
    return error

def cost_single_threaded(consts, trials_to_use):
    return sum(map(lambda x: single_cost(x, consts.astype(np.float64)), trials_to_use))

def integration_grad_descent():
                #    L   ma  mb      K_E   K_f
    # y_0 = np.array([0.22, 1, 0.12, 0.00299, 22])
    y_0 = np.array([0.17, 0.8, 0.1, 0.00299, 25])
    cur = y_0/y_0
    dx = 0.01
    step = 0.005
    old_time = perf_counter()
    best_cost = np.Inf
    best_coeffs = y_0
    print(len(data))
    trials_to_use=[0,3,4,5,7,9,10,11,12] # not [1,2,6,8]
    with Pool(len(y_0)*len(trials_to_use) + 1) as p:
        try:
            print('here')
            for i in range(10):
                step_scale = 0.5**i
                for _ in range(500):
                    d = np.zeros(len(y_0))
                    old_time = perf_counter()
                    test = [cur] + [np.insert(np.delete(cur, i), i, cur[i] + dx) for i in range(len(y_0))]
                    d = np.array(p.starmap(single_cost, [(i, j.astype(np.float64)*y_0) for i in trials_to_use for j in test]))
                    base = d[0]
                    d = d[1:].reshape((len(trials_to_use), len(y_0)))
                    
                    if base > 1_000_000_000: print(f'died, final cost was {base} with weights {cur}')
                    if base < best_cost:
                        best_cost = base
                        best_coeffs = cur
                    print('[', *[np.format_float_positional(i, 5) for i in cur*y_0], ']',
                        np.round(base, 1), 
                        np.round(perf_counter()-old_time, 3), sep='\t')
                    #print(d, test)
                    d = np.sum(d, axis=0) - base
                    # d = np.array(p.starmap(cost_single_threaded, [(test[i], trials_to_use) for i in range(len(test))])) - base
                    # for i in range(len(y_0)):
                    #     test = cur.copy()
                    #     test[i] += dx*y_0[i]
                    #     d[i] = (cost(test, p, trials_to_use) - base)
                    d = d/np.linalg.norm(d)
                    # print('d: ', d, 'step: ', step_scale*d*step)
                    # print('step: ', step_scale*d*step)
                    cur -= step_scale*d*step
                    cur[cur<0] = 0.00005
                    step_scale *= 0.99
        except KeyboardInterrupt:
            print('done!')
            print(best_coeffs, best_cost)
    return best_coeffs

if __name__ == '__main__':

    best_coeffs = integration_grad_descent()

    
    #%%
    # print(cost([0.15, 40, 0.125, 0.125, 1.74242, 0.04926476]))
    #%%
    # y_0 = np.array([0.22, 1, 0.12, 0.00299, 22])
    # res = scipy.optimize.minimize(fun = cost, x0 = y_0, args = (, [0, 4]), options = {'disp': True})
    # print(res)
    # print(res)
    # %%
    # trials=4
    # fig, ax = plt.subplots(trials, 6, sharex=True, gridspec_kw={'hspace':0})
    # fig.suptitle("pend ang , vel, acc vs. time (s)")

    #%%
    trials = len(data)
    # trials = 4
    fig, ax = plt.subplots(trials, 4, sharex=True, gridspec_kw={'hspace': 0})
    fig.suptitle("Pendulum Angle and Velocity vs. Time (s)")
    ax[0][0].set_title("Pend Angle (rad)")
    ax[0][1].set_title("Pend Vel (rad/s)")
    ax[0][2].set_title("Cart Pos (m)")
    ax[0][3].set_title("Cart Vel (m/s)")
    model = dipc_model().lambdify()
    model.set_constants(best_coeffs)

    print('here')
    for i in range(trials):
        model.integrate_with_scipy(y_0 = data[i][0, 2:], controller=lambda t: data[i][np.searchsorted(data[i][:, 0], t)-1, 1], tspan = max(data[i][:, 0]), remember_forces = True)
        print(f'applied power: {max(model.soln_forces[:, 1])}, actual force: {max(model.soln_forces[:, 2])}')
        ax[i][0].plot(data[i][:, 0], model.soln(data[i][:, 0])[1], color='orange', label='$\hat \\theta$')
        ax[i][0].plot(data[i][:, 0], data[i][:, 3], label = '$\\theta$')
        ax[i][1].plot(data[i][:, 0], model.soln(data[i][:, 0])[3], color='orange', label='$\hat \dot \\theta$')
        ax[i][1].plot(data[i][:, 0], data[i][:, 5], label = '$\dot \\theta$')
        ax[i][2].plot(data[i][:, 0], model.soln(data[i][:, 0])[0], color='orange', label='$\hat x$')
        ax[i][2].plot(data[i][:, 0], data[i][:, 2], label = '$x$')
        ax[i][3].plot(data[i][:, 0], model.soln(data[i][:, 0])[2], color='orange', label='$\hat \dot x$')
        ax[i][3].plot(data[i][:, 0], data[i][:, 4], label = '$\dot x$')
    fig.legend()
    plt.show()
    #%%
    run_number = 0
    model = dipc_model().lambdify()
    model.set_constants(y_0)
    # model.integrate_with_scipy(y_0 = data[run_number][0, 2:], controller=lambda t: powers3(t, powers[run_number]), tspan = max(data[run_number][:, 0]), remember_forces = True)
    # model.soln_y = model.soln(data[0][:, 0])
    # model.soln_t = data[0][:, 0]
    eigs = np.array([
        [-4.50], 
        [-4.51],
        [-4.52],
        [-4.53],
    ])

    Q = np.diag([3, 5, 6, 5])
    R = np.diag([70])
    model.linearize().lambdify().construct_LQR(Q, R).construct_PP(eigs)
    print(model.K)
    #%%
    # model.construct_lqr().construct_PP()
    model.integrate_with_scipy(lag_seconds = 0.05)
    model.soln_t, model.soln_y = np.arange(0, 15, 0.05), model.soln(np.arange(0, 15, 0.05))
    model.plot_animation(['tab:blue']*10, [])
    # fig.tight_layout(pad=0)
    model.show_plots(block=False)


    plt.show(block=False)

    # L = 0.22403
    #%%
    run_number = 0
    model = dipc_model()
    model.set_constants([0.15, 0.2, 0.07, 0.02995, 0.002])
    # model.set_constants([0.3, 0.125, 0.125, 20.909, 0.04926476])
    model.soln_t = data[run_number][:, 0].T
    model.soln_y = data[run_number][:, 2:].T
    # model.plot_animation(['tab:blue']+[('green', 'red')[i%2] for i in range(len(powers[run_number])-2)]+['tab:blue'], powers[2][:, 0])
    model.plot_animation(['tab:blue']*len(powers[run_number]), powers[run_number][:, 0])

    model.show_plots()



    # %%
    # constants:


    # f_motor, in newtons, = 20.909*V_actual (found experimentally from stall torque)
    # V_actual = V_applied - K_e*w # w (omega) is rotational speed of motor
    # V_actual = V_applied - K_e * x' / (0.12*16)
    # Torque = K_e * i, so 0.38 N-m = K_e * 84 A, so K_e = 0.38/84 = 0.00452381 N*m*A^-1
    # i = V_actual / R_armature = (V_applied - K_e * x'/(0.12*16))/R_armature
    # R_armature = V/I = 0.14285714
    # F = 20.909*(V_applied - 0.00452381*(1/(0.12*16)*x')
    # F = 20.909*V_applied - 0.04926476x'

    # ...assuming datasheet is right and this is a global property
    # but this gives me confidence at least that the general form is
    # F = k_V(V_applied) - k_W(x')

    # oh also oops this assumes voltage varies from 0..1, it's really 0..12, so k_V = 1.74242

    # Constants are:
    #   k_V
    #   k_W
    #   m_c
    #   m_p
    #   L
    #   k_F



