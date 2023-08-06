from data_collection import Pendulum
from time import sleep

with Pendulum(port='COM3', file='NUL') as p:
    p.set(0)
    print('Pendulum state:')
    while True:
        print(f'\rtop vel: \t{p.y[4]:.2f}, \tend vel: \t{p.y[5]:.2f}'.rjust(100), end='')
        sleep(0.1)
