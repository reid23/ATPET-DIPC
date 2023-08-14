from data_collection import Pendulum
from time import sleep
import numpy as np
ticks_per_step = 4096/(np.pi*2)
with Pendulum(port='COM3', file='NUL') as p:
    p.set(0)
    print('Pendulum state:')
    while True:
        print(f'\rtop vel: \t{p.y[1]*ticks_per_step:.1f}, \tend vel: \t{p.y[2]*ticks_per_step:.1f}'.rjust(100), end='')
        sleep(0.1)
