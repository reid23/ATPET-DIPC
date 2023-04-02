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

    # sp = model.set_variable('_tvp', 'sp')
    sp = 0

    # -ke*dy[0]+12*f)-kf*dy[0]
    expr = vertcat(
        (296.296296296296*pi*ke*(-ke*dy[0]+12*f)-kf*dy[0]-((l*mb*(-9.8*l*mb*sin(y1)-(l*mb*(296.296296296296*pi*ke*(-ke*dy[0]+12*f)-kf*dy[0]+l*mb*(dy[1]**2)*sin(y1)))*cos(y1)))/((-((l**2)*(mb**2)*(cos(y1)**2))/(ma+mb))+(l**2)*mb)) + l*mb*(dy[1]**2)*sin(y1))/(ma+mb),
        ((-9.8*l*mb*sin(y1))-((l*mb*(296.296296296296*pi*ke*(-ke*dy[0]+12*f)-kf*dy[0]+l*mb*(dy[1]**2)*sin(y1)))/(ma+mb)))/((-((l**2)*(mb**2)*(cos(y1)**2))/(ma+mb))+(l**2)*mb)
    )
    expr = vertcat(
            (-296.296296296296*pi*ke*(-ke*dy[0] + 12*f) - kf*dy[0] - l*mb*(-9.8*l*mb*sin(y1) - l*mb*(-296.296296296296*pi*ke*(-ke*dy[0] + 12*f) - kf*dy[0] + l*mb*dy[1]**2*sin(y1))*cos(y1)/(ma + mb))*cos(y1)/(-l**2*mb**2*cos(y1)**2/(ma + mb) + l**2*mb) + l*mb*dy[1]**2*sin(y1))/(ma + mb), 
            (-9.8*l*mb*sin(y1) - l*mb*(-296.296296296296*pi*ke*(-ke*dy[0] + 12*f) - kf*dy[0] + l*mb*dy[1]**2*sin(y1))*cos(y1)/(ma + mb))/(-l**2*mb**2*cos(y1)**2/(ma + mb) + l**2*mb),
    )

    PE = 100*9.8*l*mb*(1-cos(y1))
    KE = 0.5*ma*dy[0]**2 + 0.5*mb*((sin(y1)*dy[1])**2 + (dy[0]+cos(y1)*dy[1])**2)

    model.set_expression('E_kin', KE)
    model.set_expression('E_pot', PE)
    #%%
    model.set_rhs('y_0', dy[0])
    model.set_rhs('y_1', dy[1])
    model.set_rhs('dy', expr)

    # model.set_expression('tvp', sp)

    model.setup()

    mpc = do_mpc.controller.MPC(model)

    tstep = 0.1
    thorizon = 1
    nhorizon = int(thorizon/tstep)
    setup_mpc = {
        'n_horizon': nhorizon,
        't_step': tstep,
        'open_loop': 0,
        'n_robust': 0,
        # 'store_full_solution': True,
        # 'state_discretization': 'collocation',
        # 'collocation_type': 'radau',
        # 'collocation_deg': 3,
        # 'collocation_ni': 1,
        'store_full_solution': True,
    }
    mpc.set_param(**setup_mpc)
    mpc.set_param(nlpsol_opts = {'ipopt.linear_solver': 'MA27'})

    # suppress printing
    mpc.nlpsol_opts['ipopt.print_level'] = 0
    mpc.nlpsol_opts['ipopt.sb'] = 'yes'
    mpc.nlpsol_opts['print_time'] = 0

    mpc.nlpsol_opts['ipopt.linear_solver'] = 'MA27'

    # l_term = 10*cos(y1) + 0.1*y0**2 + 0.1*dy[0]**2 + 0.1*dy[1]**2 + 0.3*f**2 # step cost
    # m_term = 10*cos(y1) + 0.1*y0**2 + 0.02*dy[0]**2 + 0.5*dy[1]**2 # terminal state cost
    # # m_term = 0*y1
    l_term = model.aux['E_kin'] - model.aux['E_pot'] + 500*(model.x['y_0'])**2
    m_term = -model.aux['E_pot']+1000*(model.x['y_0'])**2 # stage cost

    mpc.set_objective(lterm=l_term, mterm = m_term)
    mpc.set_rterm(f=100)


    mpc.set_nl_cons('y_0', y0**2, 0.3**2, soft_constraint=True)
    # mpc.set_nl_cons('f', f**2, 0.5**2, soft_constraint=False)


    # bounds on state:
    mpc.bounds['lower','_x', 'y_0'] = -0.4
    mpc.bounds['upper','_x', 'y_0'] = 0.4

    # bounds on input:
    mpc.bounds['lower','_u', 'f'] = -0.3
    mpc.bounds['upper','_u', 'f'] = 0.3

    # scaling
    mpc.scaling['_x', 'y_0'] = 10
    consts = [0.17, 0.8, 0.1, 0.00299, 25]
    # 0.16043	0.7101	0.08698	0.00230	19.14743
    mpc.set_uncertainty_values(
        # l = np.array([0.17, 0.15, 0.2]),
        # ma = np.array([1]),
        # mb = np.array([0.1, 0.13]),
        # ke = np.array([0.00299, 0.004, 0.002]),
        # kf = np.array([20, 15, 25])
        # l = np.array([0.16043]),
        # ma = np.array([0.7101]),
        # mb = np.array([0.08698]),
        # ke = np.array([0.0023]),
        # kf = np.array([19.14743])
        l = np.array([consts[0]]),
        ma = np.array([consts[1]]),
        mb = np.array([consts[2]]),
        ke = np.array([consts[3]]),
        kf = np.array([consts[4]])
    )
    # 0.18319	0.77088	0.11271	0.00093	32.95541
    # 0.18319	0.77088	0.11271	0.00093	32.95541

    # template = mpc.get_tvp_template()

    # template['_tvp',:, 'sp'] = 0
    # def tvp_fun(t):
    #     return template
    
    # mpc.set_tvp_fun(tvp_fun)
    
    mpc.setup()


    simulator = do_mpc.simulator.Simulator(model)
    simulator.set_param(t_step = setup_mpc['t_step'])

    p_template = simulator.get_p_template()
    p_template['l'] = consts[0]
    p_template['ma'] = consts[1]
    p_template['mb'] = consts[2]
    p_template['ke'] = consts[3]
    p_template['kf'] = consts[4]
    def p_fun(t_now):
        return p_template
        # return p_template


    simulator.set_p_fun(p_fun)

    tvp_template = simulator.get_tvp_template()
    def tvp_fun_2(t): 
        # tvp_template['_tvp',:, 'sp'] = 0
        return tvp_template
    simulator.set_tvp_fun(tvp_fun_2)

    simulator.setup()

    # x0 = np.zeros((4, 1))
    # simulator.x0 = x0
    # mpc.x0 = x0

    mpc.set_initial_guess()
    mpc.compile_nlp(overwrite = False) #set overwrite to true if things changed

    return mpc, simulator, model

null = open('/dev/null', 'w')
def get_power(t, y, mpc):
    # start = perf_counter()
    mpc.reset_history()
    sys.stdout = null
    print(y, mpc.make_step(y)[0, 0])
    sys.stdout = sys.__stdout__

def mpc_loop(power, state, mpc):
    # sleep(0.5)
    while True:
        # sleep(0.5)
        start = perf_counter()
        get_power(0, np.array(state), mpc)
        time = int((perf_counter() - start)/mpc.t_step)
        while perf_counter()-start < mpc.t_step: pass
        ptr.value = 0
        for i in range(time, mpc.n_horizon):
            power[i-time] = mpc.data.prediction(('_u', 'f'))[0][-i]
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
                    print(u, ptr.value, *state, sep = '\t', file=sys.__stdout__)
                except IndexError:
                    pass
                sleep(0.95*t_step)
        except KeyboardInterrupt:
            p.set(0)
            print('Stopping.')
            sys.stdout = null
            exit()
if __name__ == '__main__':
    mpc, _, _ = get_mpc()
    power = Array('d', mpc.n_horizon)
    state = Array('d', 4)
    ptr = Value('I', 0)
    input('press enter to start balancing')
    write_thread = Process(target = write_to_pend_loop, args = [power, state, ptr, mpc.t_step])
    write_thread.start()
    mpc.x0 = np.array(state[:])
    mpc.set_initial_guess()
    sleep(2)
    mpc_loop(power, state, mpc)
