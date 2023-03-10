import numpy as np
import serial
from threading import Thread, Event
from time import sleep
import struct
import sys
class Pendulum:
    def __init__(self, port = '/dev/tty.usbmodem00011'):
        self.port = port
        self.y = np.zeros(6)
        self.pwr = 0
    def __enter__(self):
        self.ser = serial.Serial(self.port, 115200)
        self.stop = Event()
        self.track_thread = Thread(target = self._track, args = (self.ser, self.stop, sys.stdout))
        self.track_thread.start()
        return self
    def __exit__(self, *args):
        self.stop.set()
        self.ser.close()
    def _track(self, ser, stop_event, file = None):
        while not stop_event.is_set():
            try:
                ser.write(bytearray([0])+struct.pack('>H', int((self.pwr+1)*32767.5)))
                sleep(0.02)
                self.y = np.array(str(ser.readline())[2:-3].split(','), dtype=float)
            except Exception as e:
                print(e)
            if not file is None: print(*list(map(lambda x: np.format_float_positional(x, 3), self.y)), sep = '\t', file = file)
    def set(self, power):
        self.pwr = power

if __name__ == '__main__':
    with Pendulum() as p:
        p.set(0)
        sleep(1)
        p.set(0.2)
        sleep(0.25)
        # p.set(0.15)
        # sleep(1)
        p.set(0)
        sleep(2)
        # sleep(0.1)
        # for i in range(-50, 50):
        #     if i%5 == 0: print(i/50, p.y)
        #     p.set(i/50)
        #     sleep(0.05)
        # p.set(0)
        # sleep(1)