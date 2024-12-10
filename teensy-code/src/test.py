import struct
import serial
import time

with serial.Serial('/dev/ttyACM0', baudrate=250000) as ser:
    try:
        while True:
            command = input("command: ")
            if command[0] == "x":
                ser.write(bytes([0]) + struct.pack(">f", float(command[1:])))
            if command[0] == "g":
                ser.write(bytes([1]) + struct.pack(">f", float(command[1:])))
            if command[0] == "e":
                ser.write(bytes([2]))
            if command[0] == "s":
                ser.write(bytes([5]))
            if command[0] == "c":
                ser.write(bytes([3]))
            if command[0] == "m":
                acc = float(command[1:])
                dt = 0.5
                for _ in range(3):
                    ser.write(bytes([0])+struct.pack(">f", acc))
                    time.sleep(dt)
                    ser.write(bytes([0])+struct.pack(">f", -acc))
                    time.sleep(dt)
                    ser.write(bytes([0])+struct.pack(">f", -acc))
                    time.sleep(dt)
                    ser.write(bytes([0])+struct.pack(">f", acc))
                    time.sleep(dt)
                ser.write(bytes([5]))
            if command[0] == "q":
                break
            time.sleep(0.05)
            print(struct.unpack("<Lffffff", ser.read(7*4)))
    except Exception as e:
        # stop motion before letting exception propagate up
        ser.write(bytes([5]))
        raise e