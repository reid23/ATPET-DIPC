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
    [-2], 
    [-3.2],
    [-3.1],
    [-4],
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

ff_prog = [2.575035906172058, -3.8158905593996133, -2.0671993855312296, 4.139213882689204, 2.2243153434826377, 1.4948059154622, -4.421279471702257, -2.082844104057677, -1.819244414580912, 4.242038184131376, 2.3720656176437367, -3.938846230847921]
swing_down_prog = [-3.385853038570042, -2.291104168320536, 4.1975310847926535, 3.8602780696650716, 2.631301215914938, -4.386163191848309, -3.5911054562968814, -2.4652948343294234, -1.8780577501933995, 1.0977526545032275, 3.883398002439094, 4.100956276604702, 2.813741073637396, 2.0584780031239647, 0.585821960770236, -2.955463562063751, -4.426063147664362]

port = '/dev/tty.usbmodem00011'
pwr = 0
states = []

def stop_function(p):
    print('Stopping.')
    p.write(bytearray([1, 0])) # usb mode
    p.write(bytearray([0])+struct.pack('>i', 0)) #set power to 0
    p.write(bytearray([101])) # put system in RESET mode
    exit()
with serial.Serial(port, 115200) as p:
    try:
        p.write(bytearray([100])) # clear reset if in reset
        p.write(bytearray([1,0])) # usb mode
        sleep(0.02)
        p.write(bytearray([28])) # home x
        p.readline()
        p.readline()
        p.readline()

        input("press [enter] when homing is done: ")
        p.write(bytearray([1, 0]))
        p.write(bytearray([0])+struct.pack('>i', 0))
        for idx, i in enumerate(down_K):
            p.write(bytearray([idx+2])+struct.pack('>f', i))
            sleep(0.02)
        for idx, i in enumerate(down_sp):
            p.write(bytearray([idx+11])+struct.pack('>f', i))
            sleep(0.02)
        
        while True:
            p.write([1,1]) # set to local mode
            sleep(0.01)
            p.read_all()
            # wait until pendulum is stable in the down position
            input()
            while True:
                p.write(bytearray([255, 255]))
                y = np.array(str(p.readline())[2:-3].split(','), dtype=float)
                print(np.remainder(np.abs(y[1]+0.015333), 2*np.pi), np.abs(y[3]), np.abs(y[4]))
                if np.remainder(np.abs(y[1]+0.015333), 2*np.pi)<0.02 and np.abs(y[3])<0.005 and np.abs(y[4])<0.0005:
                    break
                sleep(0.01)
            p.write([1, 0]) # set to usb mode for switching K and sp
            sleep(1)

            # set LQR gain matrix for actual upwards balancing
            for idx, i in enumerate(up_K):
                p.write(bytearray([idx+2])+struct.pack('>f', i))
                sleep(0.005)
            p.write(bytearray([12])+struct.pack('>f', up_sp[1]))

            # feedforward phase
            for u in ff_prog:
                start = perf_counter()
                p.write(bytearray([0])+struct.pack('>i', int(u*1000000)))
                y = np.array(str(p.readline())[2:-3].split(','), dtype=float)
                states.append([y[0], y[1], y[3], y[4], u])
                # wait
                while perf_counter()-start < 0.2: pass

            p.write(bytearray([1, 1])) # set mode to local control once we're done
            # print('\ahere')
            while True:
                x = input('Enter next x setpoint (in m), [d] to swing down, or [q] to quit: ')
                if x == 'a':  stop_function(p)
                if x == 'd': break
                p.write(bytearray([11])+struct.pack('>f', float(x))) # set x setpoint

            p.read_all()
            # wait for relatively stable state
            while True:
                p.write(bytearray([255, 255]))
                y = np.array(str(p.readline())[2:-3].split(','), dtype=float)
                if np.abs(y[3])<0.02 and np.abs(y[4])<0.02:
                    break

            # set controller for down
            p.write(bytearray([1, 0])) # usb mode
            for idx, i in enumerate(down_K):
                p.write(bytearray([idx+2])+struct.pack('>f', i))
                sleep(0.01)
            p.write(bytearray([12])+struct.pack('>f', down_sp[1]))
            sleep(0.01)
            p.read_all()
            
            # do swing down program
            for u in swing_down_prog[:-2]:
                start = perf_counter()
                p.write(bytearray([0])+struct.pack('>i', int(u*1000000)))
                y = np.array(str(p.readline())[2:-3].split(','), dtype=float)
                states.append([y[0], y[1], y[3], y[4], u])
                # wait
                while perf_counter()-start < 0.1: pass
            
            p.write([1,1]) # set to local mode to balance downwards

            while True: 
                x = input('Enter next x setpoint (in m), [u] to swing up, or [q] to quit: ')
                if x == 'q': stop_function(p)
                if x == 'u': break
                p.write(bytearray([11])+struct.pack('>f', float(x))) # set x setpoint

    except KeyboardInterrupt as e:
        stop_function(p)

# plt.plot(np.arange(len(states))*0.2, states, label=['$x$', '$\\theta$', '$\dot x$', '$\dot \\theta$', '$u$'])
# plt.legend()
# plt.show()