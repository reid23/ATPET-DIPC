from MPC_testing_stepper import get_mpc
import serial
import struct
import numpy as np
from time import perf_counter
from time import sleep

port = '/dev/tty.usbmodem00011'
pwr = 0
mpc, _, _ = get_mpc()
with serial.Serial(port, 115200) as p:
    try:
        p.write(bytearray([100])) # clear reset if in reset
        sleep(0.02)
        p.readline()
        print('here')
        start = perf_counter()
        with open('double_pend_mpc_data_2.txt', 'w') as f:
            while True:
                # start = perf_counter()
                # p.flush()
                # sleep(0.02)
                p.write(bytearray([0])+struct.pack('>i', int(pwr*10000))) # set power
                y = np.array(str(p.readline())[2:-3].split(','), dtype=float)
                pwr = mpc.make_step(np.array([y[0], y[1], y[3], y[4]]))
                # print(f"{[perf_counter()-start, y[0], y[1], y[2], y[3], y[4], y[5], pwr]},")#, file=f)
                # while perf_counter()-start < mpc.t_step-0.02:
                #     pass
    except Exception as e:
        print(e)
        p.write(bytearray([0])+struct.pack('>i', 0)) # set power to 0

