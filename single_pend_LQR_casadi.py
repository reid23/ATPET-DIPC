from linearizer_casadi import get_lqr, get_pp
from SI import data_collection as Pendulum
import numpy as np
from casadi import pi
from time import sleep


setpoint = 'up' # 'up' or 'down'
controller = 'pole placement' # 'pp' or 'lqr'

Q = np.diag([75, 85, 1, 12])
R = np.diag([50])
eigs = np.array([
    [-3.3], 
    [-3.2],
    [-3.1],
    [-3],
])

eigs = np.array([
    [-2], 
    [-3.2],
    [-3.1],
    [-4],
])

if setpoint == 'up':
    sp = [0, np.pi-0.037, 0]
    sym_sp = [0, pi, 0, 0]
elif setpoint == 'down':
    eigs = eigs*2
    sp = [0, -0.015333, 0]
    sym_sp = [0, 0, 0, 0]
else:
    raise ValueError(f'setpoint must be "up" or "down", not {setpoint}')

if controller == 'pole placement':
    K = get_pp((sym_sp, 0), eigs)
elif controller == 'lqr':
    K, eigs = get_lqr((sym_sp, 0), Q, R)
else:
    raise ValueError(f'controller must be "pole placement" or "lqr", not {controller}')
    
K = np.array(K)[0]
K = [K[0], K[1], 0, K[2], K[3], 0] #space out bc cope
print('gains: ', K)
print('eigs: ', eigs.flatten())

if __name__ == '__main__':
    with Pendulum.Pendulum(file = '/dev/null') as p:
        p.set_mode('usb')
        p.zero_all()
        p.set(0)
        sleep(0.1)
        print(K)
        p.set_K(K)
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