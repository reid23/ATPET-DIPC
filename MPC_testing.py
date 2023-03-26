#%%
import numpy as np
import do_mpc
from casadi import *

# from SI.sim import single_pendulum_model as model
import matplotlib.pyplot as plt
from SI import data_collection as Pendulum
import sys
from time import sleep
# sys.stdout = open('/dev/null', 'w')
#%%
def get_mpc():
    model = do_mpc.model.Model('continuous')
    # y = model.set_variable('_x', 'y', shape=(2, 1))
    y0 = model.set_variable('_x', 'y_0')
    y1 = model.set_variable('_x', 'y_1')
    dy = model.set_variable('_x', 'dy', shape=(2, 1))
    f = model.set_variable('_u', 'f')

    l = model.set_variable('_p', 'l')
    ma = model.set_variable('_p', 'ma')
    mb = model.set_variable('_p', 'mb')
    ke = model.set_variable('_p', 'ke')
    kf = model.set_variable('_p', 'kf')

    # pend = model.dipc_model()
    # pend.print_eoms()
    expr = vertcat(
        (296.296296296296*pi*ke*(-ke*dy[0]+12*f)-kf*dy[0]-((l*mb*(-9.8*l*mb*sin(y1)-(l*mb*(296.296296296296*pi*ke*(-ke*dy[0]+12*f)-kf*dy[0]+l*mb*(dy[1]**2)*sin(y1)))*cos(y1)))/((-((l**2)*(mb**2)*(cos(y1)**2))/(ma+mb))+(l**2)*mb)) + l*mb*(dy[1]**2)*sin(y1))/(ma+mb),
        ((-9.8*l*mb*sin(y1))-((l*mb*(296.296296296296*pi*ke*(-ke*dy[0]+12*f)-kf*dy[0]+l*mb*(dy[1]**2)*sin(y1)))/(ma+mb)))/((-((l**2)*(mb**2)*(cos(y1)**2))/(ma+mb))+(l**2)*mb)
    )
    expr1 = vertcat(
        dy[0],
        dy[1],
        (296.296296296296*pi*ke*(-ke*dy[0]+12*f)-kf*dy[0]-((l*mb*(-9.8*l*mb*sin(y1)-(l*mb*(296.296296296296*pi*ke*(-ke*dy[0]+12*f)-kf*dy[0]+l*mb*(dy[1]**2)*sin(y1)))*cos(y1)))/((-((l**2)*(mb**2)*(cos(y1)**2))/(ma+mb))+(l**2)*mb)) + l*mb*(dy[1]**2)*sin(y1))/(ma+mb),
        ((-9.8*l*mb*sin(y1))-((l*mb*(296.296296296296*pi*ke*(-ke*dy[0]+12*f)-kf*dy[0]+l*mb*(dy[1]**2)*sin(y1)))/(ma+mb)))/((-((l**2)*(mb**2)*(cos(y1)**2))/(ma+mb))+(l**2)*mb)
    )
    #%%

    # print(f(0,1,0,0))



    #%%
    model.set_rhs('y_0', dy[0])
    model.set_rhs('y_1', dy[1])
    model.set_rhs('dy', expr)

    model.setup()

    mpc = do_mpc.controller.MPC(model)

    tstep = 0.2
    thorizon = 2
    nhorizon = int(thorizon/tstep)
    setup_mpc = {
        'n_horizon': nhorizon,
        't_step': tstep,
        'n_robust': 1,
        'store_full_solution': True,
    }
    mpc.set_param(**setup_mpc)

    l_term = 1-sin(y1) + y0**2
    m_term = 1-sin(y1) + y0**2

    mpc.set_objective(lterm=l_term, mterm=m_term)
    mpc.set_rterm(f=0.1)

    # bounds on state:
    mpc.bounds['lower','_x', 'y_0'] = -0.5
    mpc.bounds['upper','_x', 'y_0'] = 0.5

    # bounds on input:
    mpc.bounds['lower','_u', 'f'] = -0.5
    mpc.bounds['upper','_u', 'f'] = 0.5

    # scaling
    mpc.scaling['_x', 'y_0'] = 10

    mpc.set_uncertainty_values(
        # l = np.array([0.17, 0.15, 0.2]),
        # ma = np.array([1]),
        # mb = np.array([0.1, 0.13]),
        # ke = np.array([0.00299, 0.004, 0.002]),
        # kf = np.array([20, 15, 25])
        l = np.array([0.17]),
        ma = np.array([1]),
        mb = np.array([0.1]),
        ke = np.array([0.00299]),
        kf = np.array([20])
    )
    mpc.setup()

    simulator = do_mpc.simulator.Simulator(model)
    simulator.set_param(t_step = setup_mpc['t_step'])

    p_template = simulator.get_p_template()
    def p_fun(t_now):
        p_template['l'] = 0.17
        p_template['ma'] = 1
        p_template['mb'] = 0.1
        p_template['ke'] = 0.00299
        p_template['kf'] = 20
        return p_template


    simulator.set_p_fun(p_fun)

    simulator.setup()

    x0 = np.zeros((4, 1))
    simulator.x0 = x0
    mpc.x0 = x0

    mpc.set_initial_guess()
    return mpc

if __name__ == '__main__':
    mpc = get_mpc()
#%%
    try:
        with Pendulum.Pendulum(file = '/dev/null') as p:
            p.set_mode('usb')
            sleep(0.05)
            p.set(0)
            sleep(0.05)
            input('press [enter] to start balancing')
            try:
                while True:
                    sys.stdout = open('/dev/null', 'w')
                    u = mpc.make_step(np.delete(p.y, (2, 5)))
                    sys.stdout = open('/dev/stdout', 'w')
                    print(np.delete(p.y, (2, 5)), float(u))
                    p.set(-float(u))
                    sleep(0.03)
            except KeyboardInterrupt:
                p.set(0)
                print('Stopping.')
    except:
        pass


    # #%%
    # # %%capture
    # mpc_graphics = do_mpc.graphics.Graphics(mpc.data)
    # sim_graphics = do_mpc.graphics.Graphics(simulator.data)
    # fig, ax = plt.subplots(2, sharex=True, figsize=(16,9))
    # fig.align_ylabels()
    # #%%
    # # %%capture
    # for g in [sim_graphics, mpc_graphics]:
    #     # Plot the angle positions (phi_1, phi_2, phi_2) on the first axis:
    #     g.add_line(var_type='_x', var_name='y_0', axis=ax[0], label='cart')
    #     g.add_line(var_type='_x', var_name='y_1', axis=ax[0], label='pend')

    #     # Plot the set motor positions (phi_m_1_set, phi_m_2_set) on the second axis:
    #     g.add_line(var_type='_u', var_name='f', axis=ax[1])


    # ax[0].set_ylabel('x, ang')
    # ax[1].set_ylabel('motor power [-1, 1]')
    # ax[1].set_xlabel('time [s]')
    # fig.legend()

    #%%
    # %%capture

    x = x0
    states = []
    for i in range(int((3/thorizon)/tstep)):
        with open('/dev/null', 'w') as f:
            sys.stdout = f
            x = simulator.make_step(mpc.make_step(x))
            sys.stdout = sys.__stdout__
            states.append(x.flatten())
            print(x.flatten())
    times = np.array(list(range(int((3/thorizon)/tstep))))*tstep
    states = np.array(states)
    plt.plot(times, states[:, 0], label='$x$')
    plt.plot(times, states[:, 1], label='$\\theta$')
    plt.legend()
    plt.show()


    # %%
