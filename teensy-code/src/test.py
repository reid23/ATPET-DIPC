import struct
import serial
import time
import numpy as np
import matplotlib.pyplot as plt
setpoints = np.array([
    [  0.0,   0.0,   0.0,   0.0,   0.0,   0.0],
    [  0.0,   0.0,   0.0,   0.0, np.pi,   0.0],
    [  0.0,   0.0, np.pi,   0.0,   0.0,   0.0],
    [  0.0,   0.0, np.pi,   0.0, np.pi,   0.0],
]).astype(float)
K = np.array([
    [ 3.16228,  3.36727,  -6.22607,  -2.49844,   14.14236,  0.96845], # down down
    [-3.16228,  3.71965,  29.71956,   4.65688,   123.0737, 15.79952], # down up
    [-3.16228, -3.80829,  30.75454,   5.26494,  -22.79531,  -0.7553], # up down
    [ 3.16228,  4.58115, -21.19164, -11.68013, -120.54381, -14.5712], # up up
]).astype(float)
# K[:, 0] *= 1000
# K[:, 1] *= 1000
cur_setpoint = setpoints[0]
history = []
def delay_and_print(dt, ser):
    start = time.perf_counter()
    while time.perf_counter()<start+dt:
        ser.write(bytes([6]))
        time.sleep(0.01) # wait for buffer to fill
        history.append(struct.unpack("<Lffffff", ser.read(7*4)))
with serial.Serial('usb-Teensyduino_USB_Serial_15749420-if00', baudrate=250000) as ser:
    try:
        while True:
            command = input("command: ")
            if len(command)==0:
                command = "e"
            if command[0] == "x": #* command 0x00 = SET ACCELERATION (enter usb mode)
                ser.write(bytes([0]) + struct.pack(">f", float(command[1:])))
            if command[0] == "g": #* command 0x01 = SET POSITION
                ser.write(bytes([1]) + struct.pack(">f", float(command[1:])))
            if command[0] == "e": #* command 0x02 = STAHP (RESET)
                ser.write(bytes([2]))
            if command[0] == "s": #* command 0x05 = SOFT STOP
                ser.write(bytes([5]))
            if command[0] == "c": #* command 0x03 = CLEAR RESET
                ser.write(bytes([3]))
            if command[0] == "m": #* MACRO: move back and forth with given acc
                acc = float(command[1:])
                dt = 0.25
                history = []
                for _ in range(3):
                    ser.write(bytes([0])+struct.pack(">f", acc))
                    delay_and_print(dt, ser)
                    ser.write(bytes([0])+struct.pack(">f", -acc))
                    delay_and_print(dt, ser)
                    ser.write(bytes([0])+struct.pack(">f", -acc))
                    delay_and_print(dt, ser)
                    ser.write(bytes([0])+struct.pack(">f", acc))
                    delay_and_print(dt, ser)
                ser.write(bytes([5]))
            if command[0] == "k": #* command 0x07 = SET FEEDBACK GAINS
                cur_setpoint, gains = setpoints[int(command[1:])], K[int(command[1:])]
                ser.write(bytes([7]))
                for i in gains:
                    ser.write(struct.pack(">f", i))
                    print(i)
                ser.write(bytes([8]))
                for i in cur_setpoint:
                    ser.write(struct.pack(">f", i))
            if command[0] == "G": #* set X setpoint
                cur_setpoint[0] = float(command[1:])
                ser.write(bytes([8]))
                for i in cur_setpoint:
                    ser.write(struct.pack(">f", i))
            if command[0] == "r": #* command 0x09 = RUN CLOSED LOOP
                ser.write(bytes([9]))
            if command[0] == "q":
                break
            time.sleep(0.01)
            print(struct.unpack("<Lffffff", ser.read(7*4)))
    except Exception as e:
        # stop motion before letting exception propagate up
        ser.write(bytes([5]))
        raise e
    
history = np.array(history)
print(history.shape)
fig, axs = plt.subplots(2, 3)
axs[0, 0].plot(history[:, 0], history[:, 1], label='$x$')
axs[1, 0].plot(history[:, 0], history[:, 2], label='$\\dot x$')
axs[0, 1].plot(history[:, 0], history[:, 3], label='$\\theta_1$')
axs[1, 1].plot(history[:, 0], history[:, 4], label='$\\dot \\theta_1$')
axs[0, 2].plot(history[:, 0], history[:, 5], label='$\\theta_2$')
axs[1, 2].plot(history[:, 0], history[:, 6], label='$\\dot \\theta_2$')
for ax in axs.flatten(): ax.legend()
plt.show()

np.save('data.npy', history)