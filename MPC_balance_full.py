from MPC_testing import get_mpc
import serial
import struct
import numpy as np
from time import perf_counter

port = '/dev/tty.usbmodem00011'
pwr = 0
mpc, _, _ = get_mpc()
with serial.Serial(port, 115200) as p:
    try:
        p.write(bytearray([100])) # clear reset if in reset
        while True:
            start = perf_counter()
            p.write(bytearray([0])+struct.pack('>H', int((pwr+1)*32768))) # set power
            y = np.array(str(p.readline())[2:-3].split(','), dtype=float)
            pwr = mpc.make_step([y[0], y[1], y[3], y[4]])
            while perf_counter()-start < mpc.t_step:
                pass
    except:
        p.write(bytearray([0])+struct.pack('>H', 32768)) # set power to 0

