# from multiprocessing import Process, Lock
from threading import Thread, Lock
from casadi_testing import get_integrator
from fake_pendulum import fake_pendulum
from MPC_testing_stepper import get_mpc
from time import perf_counter, sleep
import numpy as np
import serial
import struct

# settings
mpc_tstep = 0.02
mpc_thoriz = 2
recalc_steps = 5
compile_nlp = False
port = '/dev/tty.usbmodem00011'
params = [0.23116035, 0.00625, 0.05, 0.0, 0.10631411]

run_threads = True
motion_plan = [0]*recalc_steps*10
plan_idx = 0
cur_state = np.zeros(4)
# function which will be called in a new Process
def execute_motion_plan(update_lock, pendulum):
    global plan_idx # so we can edit plan_idx
    global cur_state
    while run_threads:
        # time everything so it runs at the frequency MPC expects
        start = perf_counter()

        # aquire the lock to make sure nobody's editing these
        update_lock.acquire()
        # do the minimal work needed to increment the
        # counter and get the power level to send
        # print(plan_idx)
        pwr = motion_plan[plan_idx] 
        plan_idx += 1
        # print(f'index: {plan_idx-1} power: {pwr}')
        # print(plan_idx)
        # then release the lock bc we're done with the
        # motion plan and plan index
        update_lock.release()

        # set the power over usb (outside of lock bc
        # this takes a bit of time since usb is slow)
        # cur_state = pendulum.get_state()
        pendulum.set_power(pwr)
        # if plan_idx == 1: print(f'state: {pendulum.get_state()}')

        # wait until the full time step has passed
        # before we continue to the next step of the plan
        while perf_counter()<start+mpc_tstep and plan_idx != 0: pass
        # print(f'plan_idx write loop: {plan_idx}')

if __name__ == '__main__':
    # io stuff with pendulum
    class Pendulum:
        def __init__(self, port):
            self.port = port
            self.state = np.array([0,0,0,0])
            self.pend = serial.Serial(port, 115200)
            self.pend.write(bytearray([100])) # clear reset if in reset
            sleep(0.02)
            self.pend.readline()
            sleep(0.02)
            self.set_power(0)

        def set_power(self, pwr):
            self.pend.write(bytearray([0])+struct.pack('>i', int(pwr*10000))) # set power
            y = np.array(str(p.readline())[2:-3].split(','), dtype=float)
            self.state = np.array([y[0], y[1]+0.01, y[3], y[4]]) # k^hope!

        def get_state(self): return self.state


    # get the thinky objects
    mpc, _, _ = get_mpc(tstep=mpc_tstep, thoriz=mpc_thoriz, compile_nlp=compile_nlp)
    intfunc = get_integrator(t_step=mpc_tstep, p=params)

    # stores the current predicted feedforward from mpc (init to zeros)
    # stores the current index we're executing from the plan
    # lock for updating the motion plan
    update_lock = Lock()


    # pendulum = Pendulum(port)
    pendulum = fake_pendulum(params, 0.005, 10)
    pendulum.start()

    # init solver so we can soft start it
    mpc.make_step(pendulum.get_state())
    sleep(0.25) # wait for things to die down

    # init the power-setting process
    p = Thread(target=execute_motion_plan, args=(update_lock, pendulum))

    # wrap the whole thing in a try/except so we can stop the write
    # process when we need to stop the program
    try:
        # start the writing process
        p.start()

        # main loop
        while True:
            # read the state
            # y = cur_state.copy()
            y = pendulum.get_state()
            # print(f'state: {y} (mpc)')
            # integrate the state forward until the end of the
            # recalc step time period
            # the idea is we catch up with and then pass
            # the "irl integrator" of the physical system
            # so we can run mpc on the future state
            # and then when mpc takes computation time, it's just
            # using up the lead we got from casadi integrating faster than life
            # print(motion_plan[:recalc_steps])
            for i in motion_plan[plan_idx:recalc_steps]:
                y = intfunc(x0=y, p=i)['xf']
            # print("integration done!")
            # now do the actual calculation with the future system state
            # this is what takes all the time
            mpc.make_step(np.array(y))

            # if calculation took a normal amount of time, MPC's expected
            # initial state will not have been reached yet irl
            # but if calculation took unusually long, we'll have passed it,
            # and we need to skip the first something steps of the MPC
            # calculation to catch back up to irl

            # this isn't perfect because it assumes the MPC result is
            # close enough at the end of the previous step and the 
            # start of the current one, but it's the best we can do because
            # otherwise we'd have to recalculate which would put us even 
            # more behind.
            if (dt := recalc_steps - plan_idx) < 0:
                steps_to_skip = -int(dt)+1 # plus one bc we're probably going to lose one step or so
                print(f'aaaaaa skipping {steps_to_skip} steps')
            else:
                steps_to_skip = 0 

            # grab the new plan from MPC now
            new_plan = mpc.data.prediction(('_u', 'f'))[0, steps_to_skip:, 0]

            print(f"safe by {recalc_steps - plan_idx} steps")
            # wait for the old recalculation step to finish
            while plan_idx<recalc_steps: pass

            # update the motion plan and tell the write process
            # to restart sending powers from the beginning
            # print('error:', pendulum.get_state()-y)
            update_lock.acquire()
            motion_plan = new_plan
            plan_idx = 0
            update_lock.release()

    # if something went wrong or we
    # did ctrl-c, shut everything down
    except KeyboardInterrupt:
        pendulum.stop()
        run_threads = False
        sleep(0.25)
        pendulum.plot()

