from SI.data_collection import Pendulum
from time import sleep
import serial
import struct

# with Pendulum(file='/dev/null') as p:
#     p.set_K([1,2,3,4,5,6])
#     p.set_SP([0,0,0])
#     p.ser.write(bytearray(114))
#     sleep(0.5)
#     print('banananaana')
#     p.ser.write(bytearray(114))

#     sleep(1)
K = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
sp = [1.0,2.0,3.0]

port = '/dev/tty.usbmodem00011'
with serial.Serial(port, 115200) as p:
        p.write(bytearray([100])) # clear reset if in reset
        p.write(bytearray([1,0])) # usb mode
        p.write(bytearray([0])+struct.pack('>i', 0))
        sleep(0.02)
        # p.write(bytearray([28])) # home x
        p.readline()
        p.readline()
        p.readline()

        # p.write(bytearray([1, 0]))
        # p.write(bytearray([0])+struct.pack('>i', 0))
        for idx, i in enumerate(K):
            p.write(bytearray([idx+2])+struct.pack('>f', i))
            sleep(0.02)
        for idx, i in enumerate(sp):
            p.write(bytearray([idx+11])+struct.pack('>f', i))
            sleep(0.02)
        sleep(0.5)
        p.write(bytearray([114]))
        for i in range(20):
            print(p.readline())
        p.write(bytearray([114]))
        for i in range(20):
            print(p.readline())

        # p.write(bytearray([1,1]))
        # p.write(bytearray([0])+struct.pack('>i', 0))