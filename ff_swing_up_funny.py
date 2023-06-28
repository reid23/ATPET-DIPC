import serial
import struct
import numpy as np
from time import perf_counter
from time import sleep
from casadi_testing import controller
from casadi import DM, pi
from linearizer_casadi import get_lqr, get_pp
import matplotlib.pyplot as plt

Q = np.diag([75, 85, 1, 12])
R = np.diag([50])
eigs = np.array([
    [-3.3], 
    [-3.2],
    [-3.1],
    [-3],
])

up_sp = [0, np.pi-0.037, 0]
up_sym_sp = [0, pi, 0, 0]
down_sp = [0, -0.015333, 0]
down_sym_sp = [0, 0, 0, 0]

up_K = get_pp((up_sym_sp, 0), eigs)
down_K = get_pp((down_sym_sp, 0), eigs*2)
    
up_K = np.array(up_K)[0]
up_K = [up_K[0], up_K[1], 0, up_K[2], up_K[3], 0] #space out bc cope

down_K = np.array(down_K)[0]
down_K = [down_K[0], down_K[1], 0, down_K[2], down_K[3], 0]

ff_prog = [2.575035906172058, -3.8158905593996133, -2.0671993855312296, 4.139213882689204, 2.2243153434826377, 1.4948059154622, -4.421279471702257, -2.082844104057677, -1.819244414580912, 4.242038184131376, 2.3720656176437367, -3.938846230847921, -0.14394138719257796, 0.237489405111317, 0.3083298857013403, 0.29776815215101565, 0.23661242546355032, 0.15820602989096097]#, 0.0853957077849217, 0.029695904633668564, -0.005968777781744969, -0.023917225389918476, -0.028854169433093903, -0.02584839396694763, -0.01917379828769683, -0.011839850509807733]

port = '/dev/tty.usbmodem00011'
pwr = 0
states = []
with serial.Serial(port, 115200) as p:
    try:
        p.write(bytearray([100])) # clear reset if in reset
        p.write(bytearray([1,0]))
        sleep(0.02)
        p.readline()
        p.write(bytearray([28]))
        input("press [enter] when homing is done: ")
        while True:
            p.write(bytearray([255, 255]))
            sleep(0.05)
            response = str(p.readline())
            if len(response)>10: break
        sleep(0.5)
        p.write(bytearray([1, 0]))
        p.write(bytearray([0])+struct.pack('>i', 0))
        for idx, i in enumerate(down_K):
            p.write(bytearray([idx+2])+struct.pack('>f', i))
            sleep(0.05)
        for idx, i in enumerate(down_sp):
            p.write(bytearray([idx+11])+struct.pack('>f', i))
            sleep(0.05)
        print('here')
        p.write([1,1]) # set to local mode
        p.read_all()

        # wait until pendulum is stable in the down position
        while True:
            p.write(bytearray([255, 255]))
            y = np.array(str(p.readline())[2:-3].split(','), dtype=float)
            if np.remainder(np.abs(y[1]+0.015333), 2*np.pi)<0.02 and np.abs(y[3])<0.01 and np.abs(y[4])<0.0005:
                break
        
        p.write([1, 0]) # set to usb mode for switching K and sp

        # set LQR gain matrix for actual upwards balancing
        for idx, i in enumerate(up_K):
            p.write(bytearray([idx+2])+struct.pack('>f', i))
            sleep(0.01)
        for idx, i in enumerate(up_sp):
            p.write(bytearray([idx+11])+struct.pack('>f', i))
            sleep(0.01)

        # feedforward phase
        main_start = perf_counter()
        cont = True
        with open('/dev/null', 'w') as f:
            for u in ff_prog:
                start = perf_counter()
                p.write(bytearray([0])+struct.pack('>i', int(u*10000)))
                y = np.array(str(p.readline())[2:-3].split(','), dtype=float)
                states.append([y[0], y[1], y[3], y[4], u])
                # wait
                while perf_counter()-start < 0.2: pass

            p.write(bytearray([1, 1])) # set mode to local control once we're done
            # print('\ahere')
            while True:
                x = input('Enter next x setpoint (in m) or [q] to quit: ')
                if x == 'q':  break
                p.write(bytearray([11])+struct.pack('>f', float(x))) # set x setpoint

    except KeyboardInterrupt as e:
        pass # go to stuff below
    print('Stopping.')
    p.write(bytearray([1, 0])) # usb mode
    p.write(bytearray([0])+struct.pack('>i', 0)) #set power to 0
    p.write(bytearray([101])) # put system in RESET mode
        # p.write(bytearray([0])+struct.pack('>i', 0)) # set power to 0

plt.plot(np.arange(len(states))*0.2, states, label=['$x$', '$\\theta$', '$\dot x$', '$\dot \\theta$', '$u$'])
plt.legend()
# plt.show()