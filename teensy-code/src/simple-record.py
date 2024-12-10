import struct
import serial
import numpy as np
from time import sleep
import matplotlib.pyplot as plt
states = []
with serial.Serial('/dev/ttyACM0', baudrate=250000) as ser:
    try:
        while True:
            ser.write(bytes(6))
            states.append(list(eval(ser.readline()).values()))
            # states.append(struct.unpack("<Lffffff", ser.read(7*4)))
            print(states[-1])
            # sleep(0.01)
    except KeyboardInterrupt:
        states = np.array(states)
        plt.scatter(states[:-1, 0], np.diff(states[:, 0]))
        plt.show()
        for i in range(1, 7):
            plt.scatter((states[:, 0]-states[0, 0])/1000000, states[:, i], label=i)
            # plt.scatter(states[:, i]., label=i)
        print(states.shape)
        plt.legend()
        plt.show()