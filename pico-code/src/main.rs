// lol no std (bc we don't have that much space)
#![no_std]
#![no_main]

use rp_pico::entry; // startup function macro
use embedded_hal::PwmPin; // hardware pwm
use embedded_hal::digital::v2::{OutputPin, InputPin}; 
use rp_pico::hal::Timer;
use rp_pico::hal::gpio::Pin;
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

use rp_pico::hal::pac::interrupt;
use crate::hal::pio::SM0;

#[derive(Clone, Copy)]
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
fn pleb_fn(t: u64, cur_state: &[f32; 6]) -> Result<f32, &'static str>{
    //TODO: implement as needed
    if cur_state[0] > 100.0 || cur_state[0] < -100.0 {
        Ok(0.2)
    } else {
        Ok(0.0)
    }
}

static mut POS: Option<[i32; 3]> = None;
static mut STATE: Option<[f32; 6]> = None;
static mut MODE: Option<CtrlMode> = None;
static mut CUR_POWER: Option<u16> = None;
static mut K: Option<[f32; 6]> = None;
static mut GET_STATUS_FLAG: Option<bool> = None;
// static mut CART_I2C: Option<i2c_pio::I2C<pac::PIO0, SM0, hal::gpio::pin::bank0::Gpio18, hal::gpio::pin::bank0::Gpio19>> = None;
// static mut TOP_I2C: Option<hal::I2C<pac::I2C0, (Pin<hal::gpio::pin::bank0::Gpio16, hal::gpio::FunctionI2C>, Pin<hal::gpio::pin::bank0::Gpio17, hal::gpio::FunctionI2C>), hal::i2c::Controller>> = None;
// static mut END_I2C: Option<hal::I2C<pac::I2C1, (Pin<hal::gpio::pin::bank0::Gpio14, hal::gpio::FunctionI2C>, Pin<hal::gpio::pin::bank0::Gpio15, hal::gpio::FunctionI2C>), hal::i2c::Controller>> = None;

static mut CART_ROTS: Option<i32> = None;
static mut TOP_ROTS: Option<i32> = None;
static mut END_ROTS: Option<i32> = None;

static mut CART_OFFSET: Option<[u8; 2]> = None;
static mut TOP_OFFSET: Option<[u8; 2]> = None;
static mut END_OFFSET: Option<[u8; 2]> = None;

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

/// The USB Serial Device Driver (shared with the interrupt).
static mut USB_SERIAL: Option<SerialPort<hal::usb::UsbBus>> = None;


#[entry]
fn main() -> ! {
    let mut pos: [i32; 3] = [0; 3];
    let mut state: [f32; 6] = [0.0; 6];
    let mut get_status_flag = false;
    unsafe {
        POS = Some(pos);
        STATE = Some(state);
        GET_STATUS_FLAG = Some(get_status_flag);
    }
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
    
    // status LED to show that board is on and not broken
    let mut led_pin = pins.led.into_push_pull_output();

    // pin to check limit switches
    let reset_pin = pins.gpio11.into_pull_up_input();
    let mut in_reset = false;
    // usb driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    
    // allow sleeping
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // turn on the status LED to show the board isn't ded and all the setup worked (probably)
    led_pin.set_high().unwrap();
    channel.set_duty(0.0.get_duty(&basic_norm));
    
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_BUS = Some(usb_bus);
    }

    // Grab a reference to the USB Bus allocator. We are promising to the
    // compiler not to take mutable access to this global variable whilst this
    // reference exists!
    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    // Set up the USB Communications Class Device driver
    let serial = SerialPort::new(bus_ref);
    unsafe {
        USB_SERIAL = Some(serial);
    }
    
    // make this emulate a usb device
    let mut usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Reid Dye")
        .product("ATPET Inverted Pendulum")
        .serial_number("0001")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_DEVICE = Some(usb_dev);
    }

    
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
    );//pac::pio, pac::

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
    
    channel.set_duty(0.0.get_duty(&basic_norm));
    
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
        Operation::Write(&[0x0Eu8]),
        Operation::Read(&mut cart_offset),
        ])
        .expect("Failed to run all operations");
    
    top_i2c.write_read(0x36, &[0x0Eu8], &mut top_offset).unwrap();
    end_i2c.write_read(0x36, &[0x0Eu8], &mut end_offset).unwrap();
    
    let mut k = [0.0f32; 6];
    let mut mode = CtrlMode::USB;
    let mut cur_power = 0.0.get_duty(&basic_norm);
    
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let t0 = timer.get_counter();
    let mut prev = timer.get_counter();
    
    let (oldc, oldt, olde) = (cart_pos, top_pos, end_pos);

    unsafe {
        CUR_POWER = Some(cur_power);
        MODE = Some(mode);
        K = Some(k);

        // CART_I2C =  Some(cart_i2c);
        // TOP_I2C =  Some(top_i2c);
        // END_I2C =  Some(end_i2c);

        CART_ROTS =  Some(cart_rots);
        TOP_ROTS =  Some(top_rots);
        END_ROTS =  Some(end_rots);

        CART_OFFSET =  Some(cart_offset);
        TOP_OFFSET =  Some(top_offset);
        END_OFFSET =  Some(end_offset);
    }
    
    // Enable the USB interrupt
    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };

    loop {
        if in_reset {
            channel.set_duty(0.0.get_duty(&basic_norm));
            led_pin.set_low().unwrap();
            delay.delay_ms(500);
            led_pin.set_high().unwrap();
            delay.delay_ms(500);
            continue;
        }
        in_reset = reset_pin.is_low().unwrap();

        let mut cart = [0; 2];
        let mut top = [0; 2];
        let mut end = [0; 2];

        if get_status_flag == true{
            let mut status = [0u8; 3];
            cart_i2c
                .exec(0x36u8, &mut [
                    Operation::Write(&[0x0Bu8]),
                    Operation::Read(&mut status[0..=0]),
                ])
                .expect("Failed to run all operations");
            top_i2c.write_read(0x36, &[0x0Bu8], &mut status[1..=1]).unwrap();
            end_i2c.write_read(0x36, &[0x0Bu8], &mut status[2..=2]).unwrap();
            let mut message: String<64> = String::new();
            write!(&mut message, "cart: {:08b}, top: {:08b}, end: {:08b}\n", status[0], status[1], status[2]).unwrap();
            // let _ = serial.write(message.as_bytes());
            get_status_flag = false;
        }
        
        // first grab all three encoder positions
        cart_i2c
            .exec(0x36u8, &mut [
                Operation::Write(&[0x0Eu8]),
                Operation::Read(&mut cart),
                ])
            .unwrap();
        
        top_i2c.write_read(0x36, &[0x0Eu8], &mut top).unwrap();
        end_i2c.write_read(0x36, &[0x0Eu8], &mut end).unwrap();
        // now update the positions
        
        (cart_pos, top_pos, end_pos) = (u16::from_be_bytes(cart) as i32 - u16::from_be_bytes(cart_offset) as i32, 
                                        u16::from_be_bytes(top) as i32 - u16::from_be_bytes(top_offset) as i32, 
                                        u16::from_be_bytes(end) as i32 - u16::from_be_bytes(end_offset) as i32);
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
        
        pos = [cart_pos+cart_rots*4096, top_pos+top_rots*4096, end_pos+end_rots*4096];

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
        state = [
            pos[0] as f32 * 0.000029296875,
            pos[1] as f32 * 0.087890625,
            pos[2] as f32 * 0.087890625,
            vc,
            vt,
            ve,
        ];

        // Check if we need to do usb stuff (aka did we receive a message)
        
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

/// This function is called whenever the USB Hardware generates an Interrupt
/// Request.
///
/// We do all our USB work under interrupt, so the main thread can continue on
/// knowing nothing about USB.
#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    use core::sync::atomic::{AtomicBool, Ordering};

    /// Note whether we've already printed the "hello" message.
    static SAID_HELLO: AtomicBool = AtomicBool::new(false);

    // Grab the global objects. This is OK as we only access them under interrupt.
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let serial = USB_SERIAL.as_mut().unwrap();
    let pos = POS.as_mut().unwrap();
    let state = STATE.as_mut().unwrap();
    
    // let i2c

    // Say hello exactly once on start-up
    if !SAID_HELLO.load(Ordering::Relaxed) {
        SAID_HELLO.store(true, Ordering::Relaxed);
        let _ = serial.write(b"Hello, World!\r\n");
    }

    // Poll the USB driver with all of our supported USB Classes
    if usb_dev.poll(&mut [serial]) {
        let mut buf = [0u8; 5]; //format: 1 byte command, up to 4 bytes data
        match serial.read(&mut buf) {
            Ok(_) => {
                match buf[0] { //32767.5
                    0 => {
                        *CUR_POWER.as_mut().unwrap() = ((u16::from_be_bytes([buf[1], buf[2]]) as f32 / 32767.5) - 1.0).get_duty(&basic_norm);
                    },
                    1 => {
                        let mut mode = *MODE.as_mut().unwrap();
                        // let mode = MODE.as_mut().unwrap();
                        mode = match buf[1] {
                            0 => CtrlMode::Local,
                            1 => CtrlMode::USB,
                            2 => CtrlMode::Pleb,
                            _ => mode, // invalid, keep the same
                        }
                    }
                    2..=8 => {
                        K.as_mut().unwrap()[(buf[0] as usize)-2] = f32::from_be_bytes([buf[1], buf[2], buf[3], buf[4]]);
                    }
                    9 => {    
                        *CART_OFFSET.as_mut().unwrap() = ((pos[0]%4096) as u16).to_be_bytes();
                        *TOP_OFFSET.as_mut().unwrap() = ((pos[1]%4096) as u16).to_be_bytes();
                        *END_OFFSET.as_mut().unwrap() = ((pos[2]%4096) as u16).to_be_bytes();
                        *CART_ROTS.as_mut().unwrap() = 0;
                        *TOP_ROTS.as_mut().unwrap() = 0;
                        *END_ROTS.as_mut().unwrap() = 0;
                    },
                    10 => {
                        *GET_STATUS_FLAG.as_mut().unwrap() = true;
                    },
                    _ => {},
                }
                let mut message: String<100> = String::new();
                write!(&mut message, "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}\n", state[0], state[1], state[2], state[3], state[4], state[5]).unwrap();
                let _ = serial.write(message.as_bytes());
            }
            Err(_e) => {} // do nothing, idk what to do
        }
    }
}