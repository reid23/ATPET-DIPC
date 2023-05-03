from SI import data_collection as Pendulum
from time import sleep
from SI.sim import single_pendulum_model_stepper
import numpy as np
import sympy as sp

# up
Q = np.diag([500, 100, 40, 100])
R = np.diag([10])

eigs = np.array([
    [-3], 
    [-3.5],
    [-4],
    [-5.5],
])*5
sp = [0, sp.pi, 0]

# down
# Q = np.diag([10000, 70, 1, 700])
# R = np.diag([50])

model = (single_pendulum_model_stepper.dipc_model()
    .set_constants([0.225, 0.126, 0.13, 0.0027, 10, 10])#[0.24, 0.11, 0.1, 0.001, 10, 10])
    .set_constants([0.225, 0.126, 0.13, 0.0027, 10, 10])#[0.24, 0.11, 0.1, 0.001, 10, 10])
    .linearize(op_point = sp[:2]+[0, 0])
    .construct_PP(eigs)
    .construct_LQR(Q, R))

print(model.A)
print(model.B)
# exit()
PP, LQR = list(model.K['PP'][0]), list(model.K['LQR'][0])
print(PP, LQR)
print(model.get_eigs('PP'))
LQR.insert(2, 0)
LQR.append(0)
PP.insert(2, 0)
PP.append(0)
# LQR[0]*= 1
# LQR *= -1
# LQR[0] *= -1
# LQR[1] *= -1
# LQR[2] *= -1
# LQR[3] *= -1
print('PP eigs:')
print(np.linalg.eig(model.A - model.B@model.K['PP']))
print('LQR eigs:')
print(np.linalg.eig(model.A - model.B@model.K['LQR']))
print(PP)
print(LQR)
with Pendulum.Pendulum(file = '/dev/null') as p:
    p.set_mode('usb')
    p.set(0)
    sleep(0.1)
    print(PP)
    p.set_K(LQR)
    # down
    # p.set_SP([0, -0.01, 0])
    # up
    p.set_SP([0, np.pi-0.008, 0])
    input('Press [enter] to start balancing.')
    p.set_mode('local')
    input('Press [enter] to stop balancing.')
    p.set_mode('usb')
    p.set(0)