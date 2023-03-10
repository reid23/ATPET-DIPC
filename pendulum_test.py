import serial
from time import sleep, time
import numpy as np
import struct

# def get_state(serial):
#     ser.write(bytes([10]))
#     data = bytearray(ser.readline())
#     return np.array([struct.unpack('>f', data[i*4:i*4 + 4]) for i in range(6)])
start = time()
w = np.array([120/4096, 360/4096, 360/4096])
y = np.zeros(6)
with serial.Serial('/dev/tty.usbmodem00011', 115200) as ser:
    try:
        while True:
            ser.write(bytearray([0])+struct.pack('>H', int(0.6*65536) if abs(y[0])<0.25 else int(0.4*65536)))
            y = np.array(str(ser.readline())[2:-3].split(','), dtype=float)
            print(y)
            sleep(0.05)
    except KeyboardInterrupt:
        ser.write(bytes([0,0,0]))
        exit()
