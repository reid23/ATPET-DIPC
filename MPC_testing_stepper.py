#%%
import numpy as np
import do_mpc
from casadi import *
from SI import data_collection as Pendulum
import sys
from time import sleep, perf_counter
from multiprocessing import Process, Array, Value
#%%

### PARAMETERS: L: 0.220451, c: 0.139187, offset: -0.01 (bottom calibrated)

def get_mpc(tstep=0.1, thoriz=1, compile_nlp=True):
    model = do_mpc.model.Model('continuous')
    y0 = model.set_variable('_x', 'y_0')
    y1 = model.set_variable('_x', 'y_1')
    dy = model.set_variable('_x', 'dy', shape=(2, 1))
    f = model.set_variable('_u', 'f')

    # l = model.set_variable('_p', 'l')
    # ma = model.set_variable('_p', 'ma')
    # mb = model.set_variable('_p', 'mb')
    # ke = model.set_variable('_p', 'ke')
    # kf = model.set_variable('_p', 'kf')

    # # sp = model.set_variable('_tvp', 'sp')
    # sp = 0

    # -ke*dy[0]+12*f)-kf*dy[0]

    # expr = vertcat(
    #     f,
    #     (-9.8*(ma + mb)*sin(y1) + (4.9*mb*sin(2*y1) - (ma + mb*sin(y1)**2)*f)*cos(y1))/(l*(ma + mb*sin(y1)**2))
    # )
    l, ma, mb, I = [0.24777857, 0.12615081, 0.06319876, 0.0036074 ]
    l, ma, mb, I = [0.24777857, 0.12615081, 0.075, 0.0015 ]
    l, ma, mb, I, c = [0.23116035, 0.00625   , 0.05      , 0.        , 0.10631411]
    l, ma, mb, I, c = [0.23116035, 0   , 0.05      , 0.        , 0.10631411]

    # l, ma, mb, I, c = [0.23116035, 0.00625, 0.05, 0.0, 0.10631411]
    # l, ma, mb, I = [0.247, 0.126, 0.063, 0.001]
    # l, ma, mb, I = [0.257, 0.126, 0.075, 0.001] # good
    # l, ma, mb, I = [0.257, 0.126, 0.063, 0.001]



    # l, ma, mb, I = np.array([1.15395098, 0.41613935, 1.76343964, 0.19017682])*np.array([0.217, 0.125, 0.05, 0.005])
    # l, ma, mb, I = np.array([1.1539, 0.41613935, 1, 0.5])*np.array([0.217, 0.125, 0.05, 0.005])



    # l = 0.22


    expr = vertcat(
        f,
        l*mb*(-9.8*(I + l**2*mb)*(ma + mb)*sin(y1) - (1.0*I*ma*f + 1.0*I*mb*f + 1.0*l**2*ma*mb*f + 1.0*l**2*mb**2*f*sin(y1)**2 - 4.9*l**2*mb**2*sin(2.0*y1))*cos(y1))/((I + l**2*mb)*(-l**2*mb**2*cos(y1)**2 + (I + l**2*mb)*(ma + mb))) 
    )
    expr = vertcat(
        f,
        -c*dy[1] + (-9.8*l*mb*sin(y1) - l*mb*(l*mb*dy[1]**2*sin(y1) + (-I*l*mb*dy[1]**2*sin(y1) + I*ma*f + I*mb*f - l**3*mb**2*dy[1]**2*sin(y1) + l**2*ma*mb*f - l**2*mb**2*f*cos(y1)**2 + l**2*mb**2*f - 4.9*l**2*mb**2*sin(2.0*y1))/(I + l**2*mb))*cos(y1)/(ma + mb))/(I - l**2*mb**2*cos(y1)**2/(ma + mb) + l**2*mb)
    )
    # l=0.21
    # expr = vertcat(
    #     f,
    #     -(f*cos(y1) + 9.8*sin(y1))/l
    # #     # -f*cos(y1)-(9.8/l)*sin(y1)
    # )

    PE = 1000*l*(1-cos(y1))
    KE = ((sin(y1)*dy[1])**2 + (dy[0]+cos(y1)*dy[1])**2)

    model.set_expression('E_kin', KE)
    model.set_expression('E_pot', PE)
    #%%
    model.set_rhs('y_0', dy[0])
    model.set_rhs('y_1', dy[1])
    model.set_rhs('dy', expr)

    # model.set_expression('tvp', sp)

    model.setup()

    mpc = do_mpc.controller.MPC(model)

    thorizon = thoriz # 1
    nhorizon = int(thorizon/tstep)
    setup_mpc = {
        'n_horizon': nhorizon,
        't_step': tstep,
        'open_loop': 0,
        'n_robust': 0,
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
    # # m_term 
    # = 0*y1
    l_term = 3*model.aux['E_kin'] - 50*model.aux['E_pot'] + 20*dy[1]**2 - 20*cos(y1)*(dy[1]**2)
    l_term = 10*model.aux['E_kin'] - 150*model.aux['E_pot'] + 200*y0**2
    # l_term = model.aux['E_kin'] - 100*model.aux['E_pot']
    m_term = l_term
    # m_term = -model.aux['E_pot']+(model.x['y_0'])**2 # stage cost

    mpc.set_objective(lterm=l_term, mterm=m_term)
    mpc.set_rterm(f=1)


    # mpc.set_nl_cons('y_0', y0**2, 0.4**2, soft_constraint=True)
    # mpc.set_nl_cons('f', f**2, 0.5**2, soft_constraint=False)


    # bounds on state:
    mpc.bounds['lower','_x', 'y_0'] = -0.7
    mpc.bounds['upper','_x', 'y_0'] = 0.7

    # mpc.bounds['lower','_x', 'y_1'] = -11
    # mpc.bounds['upper','_x', 'y_1'] = 11

    mpc.set_nl_cons('vcon', dy[0]**2<1**2)
    # mpc.set_nl_cons('pain', -cos(y1)*(dy[1]**2)<4**2, soft_constraint=True)
    # bounds on input:
    mpc.bounds['lower','_u', 'f'] = -1.5
    mpc.bounds['upper','_u', 'f'] = 1.5

    # scaling
    # mpc.scaling['_x', 'y_0'] = 10
    # consts = [0.217, 0.8, 0.1, 0.00299, 25]
    # 0.16043	0.7101	0.08698	0.00230	19.14743
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
    # p_template['l'] = l
    def p_fun(t_now):
        return p_template


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
    if compile_nlp==True:
        mpc.compile_nlp(overwrite = True) #set overwrite to true if things changed

    return mpc, simulator, model

null = open('NUL', 'w')
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
