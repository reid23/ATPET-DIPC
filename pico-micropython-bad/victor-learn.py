from machine import Pin, PWM, I2C
from time import sleep
from math import sin
import sys


pwm = PWM(Pin(1))
pwm.freq(1000//5)

def setPower(power):
    """sets power to motor, from -1 to 1

    Args:
        power (float): power in [-1,1] to write to the motor.
    """
    # direction.value(power>0) # not needed for victor
    pwm.duty_ns(int(1_500_000+500_000*power))
    print(int(1_500_000+500_000*power))

for i in range(int(1000*2*3.14159265)):
    setPower(sin(i/1000))
    sleep(0.001)


setPower(1)
sleep(3)
setPower(0)
# for i in range(1000):
#     pwm.duty_u16(abs(int(65025*sin(2*3.14159*float(i)/1000.0))))
#     if sin(2*3.14159*float(i)/1000.0)<0: direction.value(1)
#     else: direction.value(0)
#     print(i)
#     sleep(0.001)

pwm.duty_u16(0)