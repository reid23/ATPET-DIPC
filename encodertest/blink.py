from machine import Pin;
from utime import sleep;
from machine import SPI;

sck, mosi, miso, cs = Pin(2, Pin.ALT_SPI), Pin(3, Pin.ALT_SPI), Pin(4, Pin.ALT_SPI), Pin(5, mode=Pin.OUT, value=1)
s = SPI(0, baudrate=1000000, polarity=0, phase=1, bits=8, firstbit=SPI.MSB, sck=sck, mosi=mosi, miso=miso);

buf = bytearray(int.to_bytes(0xFFFF, 2, 'big')); _ = cs(0); _ = s.write_readinto(buf, buf); _ = cs(1); print(f"{buf[0]<<8 + buf[1]:b}");
print("Finished.")
