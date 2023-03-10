// lol no std (bc we don't have that much space)
#![no_std]
#![no_main]

use rp_pico::entry; // startup function macro
use embedded_hal::PwmPin; // hardware pwm
use embedded_hal::digital::v2::OutputPin; 
use rp_pico::hal::Timer;
use rp_pico::hal::prelude::*;
use rp_pico::hal::pac; // shorter pac alias
use rp_pico::hal; // shorter hal alias
use usb_device::{class_prelude::*, prelude::*}; // USB device emulation
use usbd_serial::SerialPort; // more USB stuff
use heapless::String; // bc we have no std
use core::fmt::Write; // for printing
use embedded_hal::blocking::i2c::{Operation, Transactional}; // I2C HAL traits/types
use embedded_hal::prelude::_embedded_hal_blocking_i2c_WriteRead; // actual i2c func
use fugit::RateExtU32; // for .kHz() and similar
use panic_halt as _; // make sure program halts on panic (need to mention crate for it to be linked)

enum CtrlMode{
    Local,
    USB,
    Pleb,
}
trait GetDuty{
    fn get_duty(&self, normalizer: &dyn Fn(&f32) -> Result<f32, &'static str>) -> u16;
}

impl GetDuty for f32{
    fn get_duty(&self, normalizer: &dyn Fn(&f32) -> Result<f32, &'static str>) -> u16 {
        (7820.5 * normalizer(self).unwrap_or(0.0) + 23461.5) as u16
    }
}

fn basic_norm(n: &f32) -> Result<f32, &'static str> {
    if n < &-1.0 || n > &1.0{
        Err("out of bounds, expected -1<n<1")

    // now cope because we can't change the deadband
    } else if n > &0.0 {
        Ok(0.04+0.96 * n)
    } else if n < &0.0{
        Ok(-0.04+0.96 * n)
    } else {
        Ok(0.0)
    }
}

#[allow(unused_variables)]
fn pleb_fn(t: u64, state: &[f32; 6]) -> Result<f32, &'static str>{
    //TODO: implement as needed
    if state[0] < 100.0 {
        Ok(0.1)
    } else {
        Ok(0.0)
    }
}

#[entry]
fn main() -> ! {
    // ok I have no clue what any of this next bit does, but 
    // its copied from the examples and it works I guess
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
// The single-cycle I/O block controls our GPIO pins
let sio = hal::Sio::new(pac.SIO);

// Set the pins up according to their function on this particular board
let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    
    // ok ok we're done with that funny stuff
    
    // status LED to show that board is on and not broken
    let mut led_pin = pins.led.into_push_pull_output();
    
    // usb driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    
    //allow sleeping
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    // sleep for a bit so we know if it rebooted
    delay.delay_ms(1500);

    // turn on the status LED to show the board isn't ded and all the setup worked (probably)
    led_pin.set_high().unwrap();
    // Set up the USB Communications Class Device driver thing (this is the thing we can actually write to)
    let mut serial = SerialPort::new(&usb_bus);

    // make this emulate a usb device
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Reid Dye")
        .product("ATPET Inverted Pendulum")
        .serial_number("0001")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    // get PIO and state machine for i2c over PIO (since we only have 2 hw i2c drivers)
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    // set up pio i2c
    let mut cart_i2c = i2c_pio::I2C::new(
        &mut pio,
        pins.gpio18,
        pins.gpio19,
        sm0,
        100.kHz(),
        clocks.system_clock.freq(),
    );

    // set up first hw i2c for first pendulum
    let top_sda = pins.gpio16.into_mode::<hal::gpio::FunctionI2C>();
    let top_scl = pins.gpio17.into_mode::<hal::gpio::FunctionI2C>();
    let mut top_i2c = hal::I2C::i2c0(
        pac.I2C0,
        top_sda,
        top_scl,
        100.kHz(),
        &mut pac.RESETS,
        &clocks.peripheral_clock,
    );

    // set up second hw i2c for end pendulum
    let end_sda = pins.gpio14.into_mode::<hal::gpio::FunctionI2C>();
    let end_scl = pins.gpio15.into_mode::<hal::gpio::FunctionI2C>();
    let mut end_i2c = hal::I2C::i2c1(
        pac.I2C1,
        end_sda,
        end_scl,
        100.kHz(),
        &mut pac.RESETS,
        &clocks.peripheral_clock,
    );
    
    // Init PWMs (idk what this does but it's important)
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    
    // Configure it
    // we're using hw pwm to drive the victor
    let pwm = &mut pwm_slices.pwm0;
    pwm.set_ph_correct();
    pwm.set_div_int(4u8); // 50 hz = 20, 4 = 238.42 hz (4.19ms period) 0011110100011001 = 1ms, 0111101000000000 = 2ms (nah lets just do 1ms<<1)
    pwm.enable();

    // Output channel B on PWM0 to the GPIO1 pin
    let channel = &mut pwm.channel_b;
    channel.output_to(pins.gpio1);

    // init variables to track positions
    let (mut cart_pos, mut cart_rots) = (0, 0);
    let (mut top_pos, mut top_rots) = (0, 0);
    let (mut end_pos, mut end_rots) = (0, 0);
    let (mut dc, mut dt, mut de);
    let (mut oldvc, mut oldvt, mut oldve) = (0,0,0);
    let (mut vc, mut vt, mut ve) = (0f32,0f32,0f32);
    
    
    // offsets
    let mut cart_offset = [0; 2];
    let mut top_offset = [0; 2];
    let mut end_offset = [0; 2];
    // first grab all three encoder positions
    cart_i2c
    .exec(0x36u8, &mut [
        Operation::Write(&[0x0Cu8]),
        Operation::Read(&mut cart_offset),
        ])
        .expect("Failed to run all operations");
    
    top_i2c.write_read(0x36, &[0x0Cu8], &mut top_offset).unwrap();
    end_i2c.write_read(0x36, &[0x0Cu8], &mut end_offset).unwrap();
    
    let mut k = [0.0f32; 6];
    let mut mode = CtrlMode::USB;
    let mut cur_power = 0u16;
    
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let t0 = timer.get_counter();
    let mut prev = timer.get_counter();

    let (oldc, oldt, olde) = (cart_pos, top_pos, end_pos);


    loop {
        let mut cart = [0; 2];
        let mut top = [0; 2];
        let mut end = [0; 2];
        
        // first grab all three encoder positions
        cart_i2c
            .exec(0x36u8, &mut [
                Operation::Write(&[0x0Cu8]),
                Operation::Read(&mut cart),
                ])
            .unwrap();
        
        top_i2c.write_read(0x36, &[0x0Cu8], &mut top).unwrap();
        end_i2c.write_read(0x36, &[0x0Cu8], &mut end).unwrap();
        // now update the positions
        
        (cart_pos, top_pos, end_pos) = ((u16::from_be_bytes(cart) - u16::from_be_bytes(cart_offset)) as i32, 
                                        (u16::from_be_bytes(top) - u16::from_be_bytes(top_offset)) as i32, 
                                        (u16::from_be_bytes(end) - u16::from_be_bytes(end_offset)) as i32);
        dc = cart_pos-oldc;
        dt = top_pos-oldt;
        de = end_pos-olde;
        
        // check if angle wrap happened
        if dc > 3500 { cart_rots -= 1; }
        else if dc < -3500 { cart_rots += 1; }
        
        if dt > 3500 { top_rots -= 1; }
        else if dt < -3500 { top_rots += 1; }
        
        if de > 3500 { end_rots -= 1; }
        else if de < -3500 { end_rots += 1; }
        
        let pos = [cart_pos+cart_rots*4096, top_pos+top_rots*4096, end_pos+end_rots*4096];

        let dt_real = timer.get_counter().checked_duration_since(prev).unwrap().to_micros();
        if dt_real > 10_000 {
            prev = timer.get_counter();
            (vc, vt, ve) = (
                (((pos[0]-oldvc) as f32)/(dt_real as f32)) * 29.296875, //0.120/4096 (meters/ticks) times 1_000_000 (us/s)
                (((pos[1]-oldvt) as f32)/(dt_real as f32)) * 87890.625, //360/4096 (deg/ticks) times 1_000_000 (us/s)
                (((pos[2]-oldve) as f32)/(dt_real as f32)) * 87890.625,
            );
            (oldvc, oldvt, oldve) = (pos[0], pos[1], pos[2]);
        }

        let state = [
            pos[0] as f32 * 0.000029296875,
            pos[1] as f32 * 0.087890625,
            pos[2] as f32 * 0.087890625,
            vc,
            vt,
            ve,
        ];

        // Check if we need to do usb stuff (aka did we receive a message)
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 5]; //format: 1 byte command, up to 4 bytes data
            match serial.read(&mut buf) {
                Ok(_) => {
                    match buf[0] { //32767.5
                        0 => {
                            cur_power = ((u16::from_be_bytes([buf[1], buf[2]]) as f32 / 32767.5) - 1.0).get_duty(&basic_norm);
                        },
                        1 => {
                            mode = match buf[1] {
                                0 => CtrlMode::Local,
                                1 => CtrlMode::USB,
                                2 => CtrlMode::Pleb,
                                _ => mode, // invalid, keep the same
                            }
                        }
                        2..=8 => {
                            k[(buf[0] as usize)-2] = f32::from_be_bytes([buf[1], buf[2], buf[3], buf[4]]);
                        }
                        9 => {    
                            cart_i2c
                                .exec(0x36u8, &mut [
                                    Operation::Write(&[0x0Cu8]),
                                    Operation::Read(&mut cart_offset),
                                ])
                                .expect("Failed to run all operations");
                            top_i2c.write_read(0x36, &[0x0Cu8], &mut top_offset).unwrap();
                            end_i2c.write_read(0x36, &[0x0Cu8], &mut end_offset).unwrap();
                            cart_rots = 0;
                            top_rots = 0;
                            end_rots = 0;
                        },
                        10 => {
                            let mut status = [0u8; 3];
                            cart_i2c
                                .exec(0x36u8, &mut [
                                    Operation::Write(&[0x0Cu8]),
                                    Operation::Read(&mut status[0..1]),
                                ])
                                .expect("Failed to run all operations");
                            top_i2c.write_read(0x36, &[0x0Cu8], &mut status[1..2]).unwrap();
                            end_i2c.write_read(0x36, &[0x0Cu8], &mut status[2..3]).unwrap();
                            match serial.write(&status){
                                Ok(_) => {},
                                Err(_) => {},
                            }
                        },
                        _ => {},
                    }
                    let mut message: String<100> = String::new();
                    write!(&mut message, "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}\n", state[0], state[1], state[2], state[3], state[4], state[5], u16::from_be_bytes([buf[1], buf[2]])).unwrap();
                    match serial.write(message.as_bytes()){
                        Ok(_) => {},
                        Err(_) => {},
                    }
                }
                Err(_e) => {} // do nothing, idk what to do
            }
        }
        match mode {
            CtrlMode::Local => {
                cur_power = (
                                state[0]*k[0] 
                              + state[1]*k[1]
                              + state[2]*k[2] 
                              + state[3]*k[3] //2pi/4096 (radians/ticks) times 1_000_000 (us/s)
                              + state[4]*k[4]
                              + state[5]*k[5]
                            ).get_duty(&basic_norm); //calibrated values for 0, -1, 1
            },
            CtrlMode::USB => {
                //do nothing, we already set power above
            },
            CtrlMode::Pleb => {
                //TODO: implement this as needed
                cur_power = pleb_fn(timer
                                        .get_counter()
                                        .checked_duration_since(t0)
                                        .unwrap()
                                        .to_micros(),
                                    &state)
                    .unwrap()
                    .get_duty(&basic_norm);
            },
        }
        // if state[0] > 0.8 || state[0] < -0.8 {
        //     cur_power = 0.0.get_duty(&basic_norm);
        // }
        channel.set_duty(cur_power);
    }
}