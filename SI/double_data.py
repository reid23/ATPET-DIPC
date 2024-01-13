import serial
import struct
from time import sleep, perf_counter
import matplotlib.pyplot as plt
import numpy as np

class pendulum:
    def __init__(self, port='/dev/ttyACM0', file='SI/data.txt', steps_per_mm=128*200/80):
        self.steps_per_mm = steps_per_mm
        self.ser = serial.Serial(port, 115200)
        self.file = open(file, 'a')
        self.log = []
    def parse_response(self):
        # res = self.ser.read(32)
        self.log.append(eval(self.ser.readline().decode()))
        # print(struct.unpack('>L7l', res))#, self.file)

    def send_vel(self, vel):
        vel = int(vel*self.steps_per_mm)
        self.ser.write(bytearray([1])+struct.pack('>l', vel))
        self.parse_response()
    def send_acc(self, acc):
        acc = int(acc*self.steps_per_mm)
        self.ser.write(bytearray([0])+struct.pack('>l', acc))
        self.parse_response()
    def plot(self):
        print(self.log, file = open(input('file: '), 'w'))
        data = np.array(self.log)
        fig, axs = plt.subplots(2, 3)
        axs[0, 0].plot(data[:, 0], data[:, 2])
        axs[1, 0].plot(data[:, 0], data[:, 3])
        axs[0, 1].plot(data[:, 0], data[:, 4])
        axs[1, 1].plot(data[:, 0], data[:, 5])
        axs[0, 2].plot(data[:, 0], data[:, 6])
        axs[1, 2].plot(data[:, 0], data[:, 7])
        plt.show()
        


if __name__ == '__main__':
    p = pendulum()
    n = 20
    mult = -10
    dt = 0.01
    for _ in range(100):
        p.send_vel(0)
        sleep(0.02)
    # for i in range(6):
    #     for i in range(n):
    #         p.send_vel(mult*i)
    #         sleep(dt)
    #     for i in range(n):
    #         p.send_vel(mult*(n-1-i))
    #         sleep(dt)
    #     for i in range(n):
    #         p.send_vel(-mult*i)
    #         sleep(dt)
    #     for i in range(n):
    #         p.send_vel(-mult*(n-1-i))
    #         sleep(dt)
    p.send_vel(-400)
    sleep(5)
    for _ in range(100):
        p.send_vel(0)
        sleep(0.02)
    p.plot()