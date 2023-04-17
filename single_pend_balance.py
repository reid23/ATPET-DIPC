from SI import data_collection as Pendulum
from time import sleep
from SI.sim import single_pendulum_model
import numpy as np
eigs = np.array([
    [-5.5], 
    [-5.4],
    [-5.3],
    [-5.2],
][::-1])*2

Q = np.diag([10000, 10000000, 3, 10])
R = np.diag([0.1])
model = (single_pendulum_model.dipc_model()
    .linearize()
    .set_constants([0.15, 0.77088, 0.11271, 0.00093, 32.95541])
    .construct_PP(eigs)
    .construct_LQR(Q, R))
PP, LQR = list(model.K['PP'][0]), list(model.K['LQR'][0])
print(PP, LQR)
print(model.get_eigs('LQR'))
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
with Pendulum.Pendulum(file = '/dev/null') as p:
    p.set_mode('usb')
    p.set(0)
    sleep(0.1)
    print(LQR)
    p.set_K(LQR)
    p.set(0)
    input('Press [enter] to start balancing.')
    p.set_mode('local')
    input('Press [enter] to stop balancing.')
    p.set_mode('usb')
    p.set(0)