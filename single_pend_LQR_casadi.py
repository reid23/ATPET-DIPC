from linearizer_casadi import get_lqr, get_pp
from SI import data_collection as Pendulum
import numpy as np
from casadi import pi
from time import sleep
# up
Q = np.diag([75, 85, 1, 12])
R = np.diag([50])
eigs = np.array([
    [-3.3], 
    [-3.2],
    [-3.1],
    [-3],
])
sp = [0, np.pi-0.037, 0]


# # down
# Q = np.diag([2, 10, 1, 7])
# R = np.diag([4])
# sp = [0, -0.015333, 0]
print(Q)
print(R)
# LQR, eigs = get_lqr(([0, pi, 0, 0], 0), Q, R)
LQR = get_pp(([0, pi, 0, 0], 0), eigs)
LQR = np.array(LQR)[0]
LQR = [LQR[0], LQR[1], 0, LQR[2], LQR[3], 0]
print('gains: ', LQR)
print('eigs: ', eigs)

if __name__ == '__main__':
    with Pendulum.Pendulum(file = '/dev/null') as p:
        p.set_mode('usb')
        p.zero_all()
        p.set(0)
        sleep(0.1)
        p.set_K(LQR)
        p.set_SP(sp)
        input('Press [enter] to start balancing.')
        p.set_mode('local')
        while True:
            x = input('Enter next x setpoint (m) or [q] to quit: ')
            if x == 'q':  break
            p.set_SP([float(x)]+sp[1:])
        p.set_mode('usb')
        p.set(0)
        p.estop()