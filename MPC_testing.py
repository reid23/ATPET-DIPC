#%%
import numpy as np
import do_mpc
from casadi import *
from SI import data_collection as Pendulum
import sys
from time import sleep, perf_counter
from multiprocessing import Process, Array, Value
#%%
def get_mpc():
    model = do_mpc.model.Model('continuous')
    y0 = model.set_variable('_x', 'y_0')
    y1 = model.set_variable('_x', 'y_1')
    dy = model.set_variable('_x', 'dy', shape=(2, 1))
    f = model.set_variable('_u', 'f')

    l = model.set_variable('_p', 'l')
    ma = model.set_variable('_p', 'ma')
    mb = model.set_variable('_p', 'mb')
    ke = model.set_variable('_p', 'ke')
    kf = model.set_variable('_p', 'kf')

    # -ke*dy[0]+12*f)-kf*dy[0]
    expr = vertcat(
        (296.296296296296*pi*ke*(-ke*dy[0]+12*f)-kf*dy[0]-((l*mb*(-9.8*l*mb*sin(y1)-(l*mb*(296.296296296296*pi*ke*(-ke*dy[0]+12*f)-kf*dy[0]+l*mb*(dy[1]**2)*sin(y1)))*cos(y1)))/((-((l**2)*(mb**2)*(cos(y1)**2))/(ma+mb))+(l**2)*mb)) + l*mb*(dy[1]**2)*sin(y1))/(ma+mb),
        ((-9.8*l*mb*sin(y1))-((l*mb*(296.296296296296*pi*ke*(-ke*dy[0]+12*f)-kf*dy[0]+l*mb*(dy[1]**2)*sin(y1)))/(ma+mb)))/((-((l**2)*(mb**2)*(cos(y1)**2))/(ma+mb))+(l**2)*mb)
    )
    # expr = vertcat(
    #    (296.296296296296*pi*k_E*(-k_E*y2(t) + 12*F(t)) - k_F*(-15.1548*tanh(20*y2(t)) + 15.1548*tanh(18297.8*y2(t)) + 5.688*asinh(25*y2(t))) - l*m_b*(-9.8*l*m_b*sin(y1(t)) - l*m_b*(296.296296296296*pi*k_E*(-k_E*y2(t) + 12*F(t)) - k_F*(-15.1548*tanh(20*y2(t)) + 15.1548*tanh(18297.8*y2(t)) + 5.688*asinh(25*y2(t))) + l*m_b*y3(t)**2*sin(y1(t)))*cos(y1(t))/(m_a + m_b))*cos(y1(t))/(-l**2*m_b**2*cos(y1(t))**2/(m_a + m_b) + l**2*m_b) + l*m_b*y3(t)**2*sin(y1(t)))/(m_a + m_b),
    #    (-9.8*l*m_b*sin(y1(t)) - l*m_b*(296.296296296296*pi*k_E*(-k_E*y2(t) + 12*F(t)) - k_F*(-15.1548*tanh(20*y2(t)) + 15.1548*tanh(18297.8*y2(t)) + 5.688*asinh(25*y2(t))) + l*m_b*y3(t)**2*sin(y1(t)))*cos(y1(t))/(m_a + m_b))/(-l**2*m_b**2*cos(y1(t))**2/(m_a + m_b) + l**2*m_b)
    # )
    #%%
    model.set_rhs('y_0', dy[0])
    model.set_rhs('y_1', dy[1])
    model.set_rhs('dy', expr)

    model.setup()

    mpc = do_mpc.controller.MPC(model)

    tstep = 0.1
    thorizon = 3
    nhorizon = int(thorizon/tstep)
    setup_mpc = {
        'n_horizon': nhorizon,
        't_step': tstep,
        'n_robust': 1,
        'store_full_solution': True,
    }
    mpc.set_param(**setup_mpc)
    mpc.set_param(nlpsol_opts = {'ipopt.linear_solver': 'MA27'})

    # suppress printing
    # suppress_ipopt = {'ipopt.print_level':0, 'ipopt.sb': 'yes', 'print_time':0}
    mpc.set_param(nlpsol_opts = {'ipopt.print_level':0})
    mpc.set_param(nlpsol_opts = {'ipopt.sb':'yes'})
    mpc.set_param(nlpsol_opts = {'print_time':0})

    l_term = 4*cos(y1) + y0**2 + 0.1*dy[0]**2 + 0.1*dy[1]**2 + 0.15*f
    m_term = 4*cos(y1) + y0**2 + 0.1*dy[0]**2 + 0.1*dy[1]**2 + 0.15*f
    m_term = 0*y1

    mpc.set_objective(lterm=l_term, mterm = m_term)
    mpc.set_rterm(f=0.05)

    # bounds on state:
    mpc.bounds['lower','_x', 'y_0'] = -0.3
    mpc.bounds['upper','_x', 'y_0'] = 0.3

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
        ma = np.array([0.8]),
        mb = np.array([0.05]),
        ke = np.array([0.005]),
        kf = np.array([25])
    )
    mpc.setup()

    simulator = do_mpc.simulator.Simulator(model)
    simulator.set_param(t_step = setup_mpc['t_step'])

    p_template = simulator.get_p_template()
    def p_fun(t_now):
        return p_template
        # p_template['l'] = 0.17
        # p_template['ma'] = 0.8
        # p_template['mb'] = 0.075
        # p_template['ke'] = 0.005
        # p_template['kf'] = 25
        # return p_template


    simulator.set_p_fun(p_fun)

    simulator.setup()

    x0 = np.zeros((4, 1))
    simulator.x0 = x0
    mpc.x0 = x0

    mpc.set_initial_guess()
    mpc.compile_nlp(overwrite=True) #set overwrite to true if things changed
    return mpc

null = open('/dev/null', 'w')
def get_power(t, y, mpc):
    # start = perf_counter()
    mpc.reset_history()
    sys.stdout = null
    print(mpc.make_step(y)[0, 0])
    sys.stdout = sys.__stdout__

def mpc_loop(power, state, mpc):
    sleep(0.5)
    while True:
        # sleep(0.5)
        start = perf_counter()
        get_power(0, np.array(state), mpc)
        time = int((perf_counter() - start)/mpc.t_step)
        ptr.value = 0
        for i in range(time, mpc.n_horizon):
            power[i-time] = mpc.data.prediction(('_u', 'f'))[0][i]
        print('recalculated!')

def write_to_pend_loop(power, state, ptr, t_step):
    with Pendulum.Pendulum(file = '/dev/null') as p:
        try:
            p.set_mode('usb')
            sleep(0.05)
            p.set(0)
            sleep(0.05)
        except AttributeError:
            pass
        try:
            while True:
                state[0] = p.y[0]
                state[1] = p.y[1]
                state[2] = p.y[3]
                state[3] = p.y[4]
                # power = -get_power(0, np.array([p.y[0],p.y[1],p.y[3],p.y[4]]), mpc)
                try:
                    u = float(power[ptr.value])
                    p.set(-u)
                    ptr.value += 1
                    print(u, *state, sep = '\t', file=sys.__stdout__)
                except IndexError:
                    pass
                sleep(0.95*t_step)
        except KeyboardInterrupt:
            p.set(0)
            print('Stopping.')
            sys.stdout = null
            exit()
if __name__ == '__main__':
    mpc = get_mpc()
    power = Array('d', mpc.n_horizon)
    state = Array('d', 4)
    ptr = Value('I', 0)
    input('press enter to start balancing')
    write_thread = Process(target = write_to_pend_loop, args = [power, state, ptr, mpc.t_step])
    write_thread.start()
    mpc_loop(power, state, mpc)
