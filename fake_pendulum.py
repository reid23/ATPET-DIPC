from casadi_testing import get_integrator
# from multiprocessing import Process
from matplotlib import pyplot as plt
from time import perf_counter, sleep
from threading import Thread
import numpy as np
class fake_pendulum:
    def __init__(self, params, t_step = 0.005, steps_per_save=5):
        self.steps_per_save = steps_per_save
        self.t_step = t_step
        self.integrator = get_integrator(t_step = self.t_step, p=params)
        self.x = [0,0.01,0,0]
        self.u = 0
        self.process = Thread(target = self.go)
        self.run = True
        self.states = []
    def start(self):
        self.process.start()
    def stop(self):
        self.run = False
        sleep(0.2)
    def set_power(self, power):
        self.u = power
    def get_state(self):
        return np.array(self.x).flatten()
    def go(self):
        counter = 0
        while self.run:
            start = perf_counter()
            self.x = self.integrator(x0=self.x, p=self.u)['xf']
            if counter % self.steps_per_save == 0: self.states.append(list(self.get_state()/np.array([0.2, 1, 0.5, 5]))+[self.u/2])
            counter += 1
            counter %= self.steps_per_save
            while perf_counter()-start < self.t_step: pass
    def plot(self):
        plt.plot(np.arange(0, len(self.states)*self.t_step*self.steps_per_save, self.t_step*self.steps_per_save), self.states, label=['$x$', '$\\theta$', '$\dot x$', '$\dot \\theta$', '$u$'])
        plt.legend()
        plt.xlabel('Time (seconds)')
        plt.title('Pendulum State vs. Time')
        plt.show()

