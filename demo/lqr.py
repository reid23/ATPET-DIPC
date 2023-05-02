import numpy as np
from SI.data_collection import Pendulum
from time import sleep
K = {
    'dlqr': [14.142135619239667, -17.94389004698371, 0, 7.684369457478255, -1.6032255627071414, 0],
    'ulqr':    [-14.142135612050481, 35.38127710091396, 0, -9.74140153678394, 7.239971846171002, 0]
}
with Pendulum(file = 'LQR_demo_data.txt') as p:
    p.set_mode('usb')
    p.set(0)
    sleep(0.1)
    p.set_K(K['dlqr'])

    p.set_SP([0, -0.01, 0]) # down
    # p.set_SP([0, np.pi-0.008, 0]) # up

    input('Press [enter] to start balancing.')
    p.set_mode('local')
    input('Press [enter] to stop balancing.')
    p.set_mode('usb')
    p.set(0)