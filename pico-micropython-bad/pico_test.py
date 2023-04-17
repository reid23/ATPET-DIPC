#import machine
from machine import Pin, PWM, I2C, SoftI2C
from time import sleep, ticks_us, sleep_us
from math import sin
import sys
import _thread

sleep(0.1)

pwm = PWM(Pin(1))
pwm.freq(1000//5)
direction = Pin(0)


def set_power(power):
    """sets power to motor, from -1 to 1

    Args:
        power (float): power in [-1,1] to write to the motor.
    """
    # direction.value(power>0) # not needed for victor
    print('here')
    pwm.duty_ns(int(1_500_000+500_000*power))

# for i in range(int(1000*2*3.14159265)):
#     setPower(sin(i/1000))
#     sleep(0.001)
# for i in range(1000):
#     pwm.duty_u16(abs(int(65025*sin(2*3.14159*float(i)/1000.0))))
#     if sin(2*3.14159*float(i)/1000.0)<0: direction.value(1)
#     else: direction.value(0)
#     print(i)
#     sleep(0.001)

pwm.duty_u16(0)


def reg_write(i2c, addr, reg, data):
    """
    Write bytes to the specified register.
    """
    
    # Construct message
    msg = bytearray().append(data)
    
    # Write out message to register
    i2c.writeto_mem(addr, reg, msg)
    
def reg_read(i2c, addr, reg, nbytes=1):
    """
    Read byte(s) from specified register. If nbytes > 1, read from consecutive
    registers.
    """
    
    # Check to make sure caller is asking for 1 or more bytes
    if nbytes < 1:
        return bytearray()
    
    # Request data from specified register(s) over I2C
    # data = i2c.readfrom_mem(addr, reg, nbytes)
    data = i2c.readfrom_mem(addr, reg, nbytes)
    
    return data
ok = False
pend_pos, cart_pos, pend2_pos = 0, 0, 0
def track_encoders():
    global ok
    global pend_pos
    global cart_pos
    global pend2_pos
    i2c0 = SoftI2C(scl = Pin(19), sda = Pin(18), freq=10000)
    i2c1 = SoftI2C(scl = Pin(17), sda = Pin(16), freq=10000)
    i2c2 = SoftI2C(scl = Pin(15), sda = Pin(14), freq=10000)
    print(i2c0.scan())
    print(i2c1.scan())
    print(i2c2.scan())
    print('initialized')
    pend_rots, cart_rots, pend2_rots = 0, 0, 0
    pend, cart, pend2 = 0, 0, 0
    ok = True
    while True:
        oldpend, oldcart, oldpend2 = pend, cart, pend2
        try:
            pend = int.from_bytes(reg_read(i2c0, 0x36, 0x0E, 2), "big")
            cart = int.from_bytes(reg_read(i2c1, 0x36, 0x0E, 2), "big")
            pend2 = int.from_bytes(reg_read(i2c2, 0x36, 0x0E, 2), "big")
        except Exception as e:
            print(e)
        dp, dc, dp2= pend-oldpend, cart-oldcart, pend2-oldpend2
        if dp > 3500:
            pend_rots -= 1
        elif dp < -3500:
            pend_rots += 1
        
        if dc > 3500:
            cart_rots -= 1
        elif dc < -3500:
            cart_rots += 1

        if dp2 > 3500:
            cart_rots -= 1
        elif dc < -3500:
            cart_rots += 1
        
        pend_pos = pend+4096*pend_rots
        cart_pos = cart+4096*cart_rots
        pend2_pos = pend2+4096*pend2_rots
        # print(f'pend1 angle: {pend+4095*pend_rots}, cart pos: {cart+4095*cart_rots}')

try:
    encoder_thread = _thread.start_new_thread(track_encoders, ())
    print('here')
except Exception as e:
    print(e)

while not ok:
    sleep_us(10000)
sleep(0.5)
pos = []
offset = cart_pos
stop = 2*4096 # 8 is number of revolutions (120mm/rotation, 8.33 = 1m)
start = ticks_us()
set_power(0.5)
while cart_pos-offset < stop:
    # print(cart_pos*360/4096, pend_pos*360/4096, pend2_pos*360/4096, ticks_us()-start)
    sleep_us(5000)
set_power(0)
while True:
    print(cart_pos*360/4096, pend_pos*360/4096, pend2_pos*360/4096, ticks_us()-start)