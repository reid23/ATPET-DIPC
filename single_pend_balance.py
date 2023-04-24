from SI import data_collection as Pendulum
from time import sleep
from SI.sim import single_pendulum_model_stepper
import numpy as np
eigs = np.array([
    [-5], 
    [-4],
    [-3],
    [-2],
])*2.5

Q = np.diag([500, 100, 1, 100])
R = np.diag([1])
model = (single_pendulum_model_stepper.dipc_model()
    .linearize()
    .set_constants([0.24, 0.11, 0.1, 0.001, 10, 10])
    .construct_PP(eigs)
    .construct_LQR(Q, R))
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