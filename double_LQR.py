from SI.sim.double_pend import double_pend_model as Model
from SI import data_collection as Pendulum
from time import sleep
from casadi import pi
import numpy as np
import struct
import serial
import control

params = {
 'c0': 2.5339930154189007e-14,
 'I0': 0.001848,
 'l0': 0.3048,
 'a0': 0.12895,
 'm0': 0.075,
 'c1': 9.960299252043114e-15,
 'I1': 0.0005999,
 'l1': 0.3,
 'a1': 0.140322,
 'm1': 0.077771
}

m = Model(sage_saved_f_path='SI/sim/f_solved.txt')
m.update_params(params)
m.subs_params()
# f = m.get_pole_placement_func()
A, B = m.get_A_func(), m.get_B_func()
op_pt = ([0, pi, pi, 0, 0, 0], 0)
# eigs = np.linspace(-1,-1.1,6)[::-1]*0.1
# print(f)
# K = -np.array(f(*op_pt, eigs))[0]
Q = np.diag([20, 20, 50, 5, 1, 5])
Q = np.diag([1000, 200, 50, 1, 50, 40])

R = np.diag([10])
K = control.lqr(A(*op_pt), B(*op_pt), Q, R)[0][0]
print('gains: ', K)
# print('eigs: ', eigs.flatten())
# sp = [0, np.pi+0.02, np.pi]
sp = [0, np.pi+0.02, np.pi]

def stop_function(p):
    print('Stopping.')
    p.write(bytearray([1, 0])) # usb mode
    p.write(bytearray([0])+struct.pack('>i', 0)) #set power to 0
    p.write(bytearray([101])) # put system in RESET mode
    exit()

# port = '/dev/tty.usbmodem00011'
# with serial.Serial(port, 115200) as p:
#         p.write(bytearray([100])) # clear reset if in reset
#         p.write(bytearray([1,0])) # usb mode
#         p.write(bytearray([0])+struct.pack('>i', 0))
#         sleep(0.02)
#         # p.write(bytearray([28])) # home x
#         p.readline()
#         p.readline()
#         p.readline()

#         p.write(bytearray([1, 0]))
#         p.write(bytearray([0])+struct.pack('>i', 0))
#         for idx, i in enumerate(K):
#             p.write(bytearray([idx+2])+struct.pack('>f', i))
#             sleep(0.02)
#         for idx, i in enumerate(sp):
#             p.write(bytearray([idx+11])+struct.pack('>f', i))
#             sleep(0.02)
#         input('Press [enter] to start balancing. ')
#         p.write(bytearray([1, 1]))
#         while True:
#             x = input('Enter next x setpoint (m) or [q] to quit: ')s
#             if x == 'q':  break
#             p.write(bytearray([11])+struct.pack('>f', float(x)))
#         stop_function(p)
        

if __name__ == '__main__':
    with Pendulum.Pendulum(file = '/dev/null') as p:
        p.set_mode('usb')
        p.zero_all()
        p.set(0)
        sleep(0.1)
        print(K)
        p.set_K(K)
        p.set_SP(sp)
        p.ser.write(bytearray([114]))
        sleep(0.5)
        for _ in range(10):
            p.ser.readline()
        input('Press [enter] to start balancing.')
        p.set_mode('local')
        while True:
            x = input('Enter next x setpoint (m) or [q] to quit: ')
            if x == 'q':  break
            p.set_SP([float(x)]+sp[1:])
        p.set_mode('usb')
        p.set(0)
        p.estop()