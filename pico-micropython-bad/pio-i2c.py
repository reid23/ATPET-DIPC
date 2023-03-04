# uint8_t Rx(bool ack){
#     uint8_t dat = 0;
#     SDA_ON;
#     for( uint8_t i =0; i<8; i++){
#         dat <<= 1;
#         do{
#             SCL_ON;
#         }while(SCL_READ == 0);  //clock stretching
#         dly();
#         if(SDA_READ) dat |=1;
#         dly();
#         SCL_OFF;
#     }
#     ack ? SDA_OFF : SDA_ON;
#     SCL_ON;
#     dly();
#     SCL_OFF;
#     SDA_ON;
#     return(dat);

# void start(){
#     SDA_ON;
#     dly();
#     SCL_ON;
#     dly();
#     SDA_OFF;
#     dly();
#     SCL_OFF;
#     dly();
# }

# void start(){
#     SDA_ON;
#     dly();
#     SCL_ON;
#     dly();
#     SDA_OFF;
#     dly();
#     SCL_OFF;
#     dly();
# }


import time
from machine import Pin
import rp2

@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def blink_1hz():
    # Cycles: 1 + 1 + 6 + 32 * (30 + 1) = 1000
    irq(rel(0))
    set(pins, 1)
    set(x, 31)                  [5]
    label("delay_high")
    nop()                       [29]
    jmp(x_dec, "delay_high")

    # Cycles: 1 + 7 + 32 * (30 + 1) = 1000
    set(pins, 0)
    set(x, 31)                  [6]
    label("delay_low")
    nop()                       [29]
    jmp(x_dec, "delay_low")


# Create the StateMachine with the blink_1hz program, outputting on Pin(25).
sm = rp2.StateMachine(0, blink_1hz, freq=2000, set_base=Pin(25))

# Set the IRQ handler to print the millisecond timestamp.
sm.irq(lambda p: print(time.ticks_ms()))

# Start the StateMachine.
sm.active(1)

@rp2.asm_pio()
def i2c():
    
# .program i2c
# .side_set 1 opt pindirs

# do_nack:
#     jmp y-- entry_point        ; Continue if NAK was expected
#     irq wait 0 rel             ; Otherwise stop, ask for help

# do_byte:
#     set x, 7                   ; Loop 8 times
# bitloop:
#     out pindirs, 1         [7] ; Serialise write data (all-ones if reading)
#     nop             side 1 [2] ; SCL rising edge
#     wait 1 pin, 1          [4] ; Allow clock to be stretched
#     in pins, 1             [7] ; Sample read data in middle of SCL pulse
#     jmp x-- bitloop side 0 [7] ; SCL falling edge

#     ; Handle ACK pulse
#     out pindirs, 1         [7] ; On reads, we provide the ACK.
#     nop             side 1 [7] ; SCL rising edge
#     wait 1 pin, 1          [7] ; Allow clock to be stretched
#     jmp pin do_nack side 0 [2] ; Test SDA for ACK/NAK, fall through if ACK

# public entry_point:
# .wrap_target
#     out x, 6                   ; Unpack Instr count
#     out y, 1                   ; Unpack the NAK ignore bit
#     jmp !x do_byte             ; Instr == 0, this is a data record.
#     out null, 32               ; Instr > 0, remainder of this OSR is invalid
# do_exec:
#     out exec, 16               ; Execute one instruction per FIFO word
#     jmp x-- do_exec            ; Repeat n + 1 times