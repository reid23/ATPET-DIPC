import serial
import struct
import numpy as np
from time import perf_counter
from time import sleep
from casadi_testing import controller
from casadi import DM, pi
from linearizer_casadi import get_lqr
import matplotlib.pyplot as plt

# LQR setup
Q = np.diag([75, 85, 1, 12])
R = np.diag([50])
# LQR, eigs = get_lqr(([0, pi, 0, 0], 0), Q, R)
LQR = get_pp(([0, pi, 0, 0], 0), eigs = np.array([[-3.3],[-3.2],[-3.1],[-3]]))
LQR = np.array(LQR)[0]
LQR = [LQR[0], LQR[1], 0, LQR[2], LQR[3], 0]
print('gains: ', LQR)
print('eigs: ', eigs)

SP = [0, np.pi-0.037, 0]



ff_prog = [2.575035906172058, -3.8158905593996133, -2.0671993855312296, 4.139213882689204, 2.2243153434826377, 1.4948059154622, -4.421279471702257, -2.082844104057677, -1.819244414580912, 4.242038184131376, 2.3720656176437367, -3.938846230847921, -0.14394138719257796, 0.237489405111317, 0.3083298857013403, 0.29776815215101565, 0.23661242546355032, 0.15820602989096097]#, 0.0853957077849217, 0.029695904633668564, -0.005968777781744969, -0.023917225389918476, -0.028854169433093903, -0.02584839396694763, -0.01917379828769683, -0.011839850509807733]

port = '/dev/tty.usbmodem00011'
pwr = 0
# mpc = controller()
# mpc.make_step(DM([0, pi, 0, 0])) # initialize
states = []
with serial.Serial(port, 115200) as p:
    try:
        p.write(bytearray([100])) # clear reset if in reset
        sleep(0.02)
        p.readline()
        p.write(bytearray([28]))
        sleep(5)
        while True:
            p.write(bytearray([255, 255]))
            sleep(0.01)
            response = str(p.readline())
            if response.find('reset') == -1 and len(response)>20: break
        down_LQR, eigs = get_lqr(
            ([0, 0, 0, 0], 0), 
            np.diag([2,10,1,7]),
            np.diag([4])
        )
        down_LQR = np.array(down_LQR)[0]
        down_LQR = [down_LQR[0], down_LQR[1], 0, down_LQR[2], down_LQR[3], 0]

        for idx, i in enumerate(down_LQR):
            p.write(bytearray([idx+2])+struct.pack('>f', i))
            sleep(0.05)
        for idx, i in enumerate([0, -0.015333, 0]):
            p.write(bytearray([idx+11])+struct.pack('>f', i))
            sleep(0.05)
        
        p.write([1,1]) # set to local lqr mode
        p.read_all()
        while True:
            p.write(bytearray([255, 255]))
            y = np.array(str(p.readline())[2:-3].split(','), dtype=float)
            # print(y)
            np.remainder(np.abs(y[1]+0.015333), 2*np.pi)
            # print(np.remainder(np.abs(y[1]+0.015333), 2*np.pi), np.abs(y[3]), np.abs(y[4]))
            if np.remainder(np.abs(y[1]+0.015333), 2*np.pi)<0.01 and np.abs(y[3])<0.001 and np.abs(y[4])<0.0005:
                break
        p.write([1, 0]) # set to usb mode

        # set LQR gain matrix for actual upwards balancing
        for idx, i in enumerate(LQR):
            p.write(bytearray([idx+2])+struct.pack('>f', i))
            sleep(0.01)
        for idx, i in enumerate(SP):
            p.write(bytearray([idx+11])+struct.pack('>f', i))
            sleep(0.01)

        main_start = perf_counter()
        cont = True
        with open('/dev/null', 'w') as f:
            for u in ff_prog:
                start = perf_counter()
                p.write(bytearray([0])+struct.pack('>i', int(u*10000)))
                y = np.array(str(p.readline())[2:-3].split(','), dtype=float)
                states.append([y[0], y[1], y[3], y[4], u])
                # print(u)
                # if np.abs(y[1]-(np.pi-0.04))<(np.pi/60) and np.abs(y[3])<(np.pi/3):
                #     print('break reached!')
                #     break
                while perf_counter()-start < 0.2: pass
                #     if perf_counter()-main_start > 4.258:
                #         cont = False
                #         break
                # if not cont: break
            p.write(bytearray([1, 1])) # set mode to LQR
            # print('\ahere')
            while True: pass
            # p.write(bytearray([101])) # put system in RESET mode

    # except Exception as e:
    #     print(e)
    #     print('here')
    #     p.write(bytearray([0])+struct.pack('>i', 0))
    #     p.write(bytearray([101])) # put system in RESET mode
    #     # p.write(bytearray([0])+struct.pack('>i', 0)) # set power to 0
    except KeyboardInterrupt as e:
        print('Stopping.')
        p.write(bytearray([1, 0])) # usb mode
        p.write(bytearray([0])+struct.pack('>i', 0)) #set power to 0
        p.write(bytearray([101])) # put system in RESET mode
        # p.write(bytearray([0])+struct.pack('>i', 0)) # set power to 0

plt.plot(np.arange(len(states))*0.2, states, label=['$x$', '$\\theta$', '$\dot x$', '$\dot \\theta$', '$u$'])
plt.legend()
# plt.show()