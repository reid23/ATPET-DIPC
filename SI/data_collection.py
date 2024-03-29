import numpy as np
import serial
import struct
from threading import Event, Thread
from time import sleep, perf_counter_ns
import sys


class Pendulum:
    def __init__(self, port = '/dev/tty.usbmodem00011', file = 'SI/data.txt'):
        """constructor for Pendulum

        Args:
            port (str, optional): serial port to connect to. Defaults to '/dev/tty.usbmodem00011'.
        """
        self.port = port
        self.file = file
        self.y = np.zeros(6)
        self.pwr = 0
        self.modes = {'usb': 0, 'local': 1, 'pleb': 2}
        self.start = perf_counter_ns()
    def __enter__(self):
        """enter a context manager

        Returns:
            Pendulum: self object for context manager
        """
        self.f = open(self.file, 'a')
        try:
            self.ser = serial.Serial(self.port, 115200)
            self.ser.write(bytearray([100]))
        except:
            print('error in serial port initialization. continuing.')
            self.ser = None
        self.stop = Event()
        self.track_thread = Thread(target = self._track, args = (self.ser, self.stop, self.f))
        self.track_thread.start()
        return self
    def __exit__(self, *args):
        """exits the context manager
        """
        self.set(0)
        sleep(0.03)
        self.stop.set()
        sleep(0.03)
        self.ser.close()
        self.f.close()
    def _track(self, ser, stop_event, file = None):
        """tracks the encoder positions

        Args:
            ser (serial.Serial): serial port to use
            stop_event (threading.Event): event to quit on
            file (file, optional): file to print encoder positions to. Defaults to None.
        """
        while not stop_event.is_set():
            try:
                ser.write(bytearray([0])+struct.pack('>l', self.pwr))
                sleep(0.01)
                self.y = np.array(str(ser.readline())[2:-3].split(','), dtype=float)
            except AttributeError:
                pass
            except Exception as e:
                print(e, file=file)
                pass
            if not file is None: 
                # print(self.y)
                print(*([perf_counter_ns()-self.start, round(self.pwr, 3)]+list(map(lambda x: np.format_float_positional(x, precision=5, trim='k'), self.y))), sep = ',', end='],\n[', file = file)
    def set(self, power):
        """sets power to given value, accounting for deadband

        Args:
            power (float): power level in [-1, 1] to send to motor
        """
        self.pwr = int(power*10000)
    def home(self):
        self.ser.write(bytearray([28]))
    def estop(self):
        self.ser.write(bytearray([101]))
    def set_SP(self, SP):
        """sets setpoints to the given list.

        Args:
            SP (list[float]): list of length 3 containing setpoints in the format `[x, θt, θe]`
        """
        for idx, i in enumerate(SP):
            self.ser.write(bytearray([idx+11])+struct.pack('>f', i))
            sleep(0.05)
    def set_K(self, K):
        """sets gain matrix K to the given list.

        Args:
            K (list[float]): list of length 6 containing gains in the format `[x, θt, θe, ẋ, θ̇t, θ̇]`
        """
        for idx, i in enumerate(K):
            self.ser.write(bytearray([idx+2])+struct.pack('>f', i))
            sleep(0.05)
    def set_mode(self, mode):
        """sets the operating mode of the pendulum

        Args:
            mode (string): 'local' (use `u=-Kx` controller), 'usb' (read power from serial), or 'pleb' (use predefined function f(t, y))
        """
        self.ser.write(bytearray([1, self.modes[mode.lower()]]))
        sleep(0.02)
    def zero_all(self):
        """sets offsets such that current coordinates are all 0
        """
        self.ser.write(bytearray([92]))
        sleep(0.02)
    def get_status(self):
        """gets value of status registers for each encoder, in the format [_,_,MD,ML,MH,_,_,_]
        """
        self.ser.write(bytearray([10]))
        sleep(0.02)
        print(self.ser.readline())
    
if __name__ == '__main__':
    power = 0.75
    period = 1.5

    # delay = 0.2
    file = '/dev/null'
    with open(file, 'a') as f:
        print('[', file = f)
    with Pendulum(file = file) as p:
        p.home()
        sleep(10)
        p.set(0)
        sleep(0.5)
        p.set(power)
        sleep(period)
        p.set(-power)
        sleep(period*2)
        p.set(power)
        sleep(period)
        # 0.2: 0.2 (1 kg lift)
        # 0.32N for 0.2 power

        # for i in np.arange(0, 20*np.pi, 0.1):
        #     # print(np.sin(i)/5)
        #     p.set((np.sin(i)/2 + 0.5*np.sin(2*i))*0.666*power)
        #     sleep(period/(2*np.pi/0.1))
        p.set(0)
        p.estop()
        sleep(0.5)
        # def print_add(joy):
        #     print('Added', joy)

        # def print_remove(joy):
        #     print('Removed', joy)

        # def key_received(key):
        #     print('Key:', key)

        # run_event_loop(print_add, print_remove, key_received)
        # p.set(0)
        # sleep(1)

        # p.set(power)
        # sleep(delay)
        # p.set(-power)
        # sleep(delay)
        # p.set(power)
        # sleep(delay)
        # p.set(-power)
        # sleep(delay)
        # p.set(power)
        # sleep(delay)
        # p.set(-power)
        # sleep(delay)

        # p.set(0)
        # sleep(2)

    with open(file, 'a') as f:
        print('],', file = f, end = '')