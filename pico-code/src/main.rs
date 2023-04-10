// lol no std (bc we don't have that much space)
#![no_std]
#![no_main]

use rp_pico::entry; // startup function macro
use embedded_hal::PwmPin; // hardware pwm
use embedded_hal::digital::v2::OutputPin; 
use rp_pico::hal::Timer;
use rp_pico::hal::gpio::{Pin, PullUpInput};
use rp_pico::hal::gpio::bank0::Gpio11;
use rp_pico::hal::prelude::*;
use rp_pico::hal::pac; // shorter pac alias
use rp_pico::hal; use rp_pico::hal::timer::Alarm;
// shorter hal alias
use usb_device::{class_prelude::*, prelude::*}; // USB device emulation
use usbd_serial::SerialPort; // more USB stuff
use heapless::String; use core::f32::consts::PI;
// bc we have no std
use core::fmt::Write; // for printing
use embedded_hal::blocking::i2c::{Operation, Transactional}; // I2C HAL traits/types
use embedded_hal::prelude::_embedded_hal_blocking_i2c_WriteRead; // actual i2c func
use fugit::{RateExtU32, ExtU32}; // for .kHz() and similar
use panic_halt as _; // make sure program halts on panic (need to mention crate for it to be linked)
use rp_pico::hal::pac::interrupt;
use core::sync::atomic::{AtomicI32, AtomicU16, AtomicU8, AtomicBool, Ordering};

// #[derive(Clone, Copy)]
// enum CtrlMode{
//     USB,   // 0
//     Local, // 1
//     Pleb,  // 2
// }
trait GetDuty{
    fn get_duty(&self, normalizer: &dyn Fn(&f32) -> Result<f32, &'static str>) -> u16;
}

impl GetDuty for f32{
    fn get_duty(&self, normalizer: &dyn Fn(&f32) -> Result<f32, &'static str>) -> u16 {
        (7820.5 * normalizer(self).unwrap_or(0.0) + 23461.5) as u16
    }
}

fn unsafe_norm(n: &f32) -> Result<f32, &'static str> {
    let n = if n > &1.0 {&1.0} 
             else if n < &-1.0 {&-1.0} 
             else {n};
    if n < &0.0 {
        Ok(-0.04+0.96 * n)
    } else if n > &0.0{
        Ok(0.04+0.96 * n)
    } else {
        Ok(0.0)
    }
}

fn basic_norm(n: &f32) -> Result<f32, &'static str> {
    if n < &-1.0 || n > &1.0{
        Err("out of bounds, expected -1<n<1")

    // now cope because we can't change the deadband
    // also yes gt/lt signs swapped
    // it's so that the power moves
    // the cart in the right direction
    // aka + => right, - => left
    } else if n < &0.0 {
        Ok(0.04+0.96 * -n)
    } else if n > &0.0{
        Ok(-0.04+0.96 * -n)
    } else {
        Ok(0.0)
    }
}

// 5 dir, 6 step

#[allow(unused_variables)]
fn pleb_fn(t: u64) -> Result<f32, &'static str>{
    //TODO: implement as needed
    if (CART.load(Ordering::Relaxed).abs() as f32 * 0.00002929688) > 0.1 {
        Ok(0.2)
    } else {
        Ok(0.0)
    }
}



static MODE: AtomicU8 = AtomicU8::new(0);
static CUR_POWER: AtomicI32 = AtomicI32::new(0);
static IN_RESET: AtomicBool = AtomicBool::new(false);

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

/// The USB Serial Device Driver (shared with the interrupt).
static mut USB_SERIAL: Option<SerialPort<hal::usb::UsbBus>> = None;

static mut ALARM: Option<hal::timer::Alarm0> = None;

static mut RESET_PIN: Option<Pin<Gpio11, PullUpInput>> = None;

static OLD_VC: AtomicI32 = AtomicI32::new(0);
static OLD_VT: AtomicI32 = AtomicI32::new(0);
static OLD_VE: AtomicI32 = AtomicI32::new(0);

static VC: AtomicI32 = AtomicI32::new(0);
static VT: AtomicI32 = AtomicI32::new(0);
static VE: AtomicI32 = AtomicI32::new(0);

static CART: AtomicI32 = AtomicI32::new(0);
static TOP: AtomicI32 = AtomicI32::new(0);
static END: AtomicI32 = AtomicI32::new(0);

static CART_OFFSET: AtomicI32 = AtomicI32::new(0);
static TOP_OFFSET: AtomicI32 = AtomicI32::new(2954-18);
static END_OFFSET: AtomicI32 = AtomicI32::new(0);

static CART_ROTS: AtomicI32 = AtomicI32::new(0);
static TOP_ROTS: AtomicI32 = AtomicI32::new(0);
static END_ROTS: AtomicI32 = AtomicI32::new(0);
static K: [AtomicI32; 6] = [AtomicI32::new(0),
                            AtomicI32::new(0),
                            AtomicI32::new(0),
                            AtomicI32::new(0),
                            AtomicI32::new(0),
                            AtomicI32::new(0)];

const CART_M_PER_TICK: f32 = 0.12/4096.0;
const RADS_PER_TICK: f32 = 2.0*3.14159265358979323/4096.0;


#[entry]
fn main() -> ! {
    // ok I have no clue what any of this next bit does, but 
    // its copied from the examples and it works I guess
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();

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
    // let reset_pin = pins.gpio11.into_mode(hal::gpio::Interrupt::EdgeHigh);
    let reset_pin = pins.gpio11.into_pull_up_input();
    reset_pin.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, true);
    unsafe { RESET_PIN = Some(reset_pin); }
    // usb driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    
    // allow sleeping
    // let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    
    // turn on the status LED to show the board isn't ded and all the setup worked (probably)
    led_pin.set_high().unwrap();
    channel.set_duty(0.0.get_duty(&basic_norm));
    
    // Note (safety): This is safe as interrupts haven't been started yet
    unsafe { USB_BUS = Some(usb_bus); }
    
    // Grab a reference to the USB Bus allocator. We are promising to the
    // compiler not to take mutable access to this global variable whilst this
    // reference exists!
    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };
    
    // Set up the USB Communications Class Device driver
    let serial = SerialPort::new(bus_ref);
    unsafe { USB_SERIAL = Some(serial); }
    
    // make this emulate a usb device
    let mut usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Reid Dye")
        .product("ATPET Inverted Pendulum")
        .serial_number("0001")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    // Note (safety): This is safe as interrupts haven't been started yet
    unsafe { USB_DEVICE = Some(usb_dev); }


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
    let (mut cart_rots, mut top_rots, mut end_rots);
    let (mut cart_pos, mut top_pos, mut end_pos) = (0,0,0);
    let (mut dc, mut dt, mut de);
    let (mut oldc, mut oldt, mut olde);

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let t0 = timer.get_counter();
    let mut alarm0 = timer.alarm_0().unwrap();
    alarm0.schedule(10_000.micros()).unwrap();
    alarm0.enable_interrupt();
    unsafe { ALARM = Some(alarm0); }


    // Enable the USB interrupt
    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
        pac::NVIC::unmask(hal::pac::Interrupt::TIMER_IRQ_0);
    }
    unsafe { pac::NVIC::unmask(hal::pac::Interrupt::IO_IRQ_BANK0); }

    // led_pin.set_low().unwrap();
    // delay.delay_ms(1000);
    // led_pin.set_high().unwrap();

    loop {
        
        let mut cart = [0; 2];
        let mut top = [0; 2];
        let mut end = [0; 2];
        // if get_status_flag == true{
        //     let mut status = [0u8; 3];
        //     cart_i2c
        //         .exec(0x36u8, &mut [
        //             Operation::Write(&[0x0Bu8]),
        //             Operation::Read(&mut status[0..=0]),
        //         ])
        //         .expect("Failed to run all operations");
        //     top_i2c.write_read(0x36, &[0x0Bu8], &mut status[1..=1]).unwrap();
        //     end_i2c.write_read(0x36, &[0x0Bu8], &mut status[2..=2]).unwrap();
        //     let mut message: String<64> = String::new();
        //     write!(&mut message, "cart: {:08b}, top: {:08b}, end: {:08b}\n", status[0], status[1], status[2]).unwrap();
        //     // let _ = serial.write(message.as_bytes());
        //     get_status_flag = false;
        // }
        
        // first grab all three encoder positions
        let _ = cart_i2c
            .exec(0x36u8, &mut [
                Operation::Write(&[0x0Cu8]),
                Operation::Read(&mut cart),
                ]);
        let _ = top_i2c.write_read(0x36, &[0x0Cu8], &mut top);
        let _ = end_i2c.write_read(0x36, &[0x0Cu8], &mut end);
        
        if u16::from_be_bytes(cart) > 4096 || 
           u16::from_be_bytes(top)  > 4096 || 
           u16::from_be_bytes(end)  > 4096 { continue; }
        
        oldc = cart_pos;
        oldt = top_pos;
        olde = end_pos;
        // now update the positions
        cart_pos = u16::from_be_bytes(cart) as i32;
        top_pos = u16::from_be_bytes(top) as i32;
        end_pos = u16::from_be_bytes(end) as i32;
        
        
        dc = cart_pos-oldc;
        dt = top_pos-oldt;
        de = end_pos-olde;

        // check if angle wrap happened
        cart_rots = CART_ROTS.load(Ordering::Relaxed);
        top_rots = TOP_ROTS.load(Ordering::Relaxed);
        end_rots = END_ROTS.load(Ordering::Relaxed);
        
        if dc > 3500 && dc <= 4096 { CART_ROTS.store(cart_rots - 1, Ordering::Relaxed); cart_rots -= 1;}
        else if dc < -3500 && dc >= -4096 { CART_ROTS.store(cart_rots + 1, Ordering::Relaxed); cart_rots += 1;}
        
        if dt > 3500 && dt <= 4096 { TOP_ROTS.store(top_rots - 1, Ordering::Relaxed); top_rots -= 1;}
        else if dt < -3500 && dt >= -4096 { TOP_ROTS.store(top_rots + 1, Ordering::Relaxed); top_rots += 1;}
        
        if de > 3500 && de <= 4096 { END_ROTS.store(end_rots - 1, Ordering::Relaxed); end_rots -= 1;}
        else if de < -3500 && de >= -4096 { END_ROTS.store(end_rots + 1, Ordering::Relaxed); end_rots += 1;}
        
        let (c, t, e) = (cart_pos+cart_rots*4096, top_pos+top_rots*4096, end_pos+end_rots*4096);
        CART.store(c, Ordering::Relaxed);
        TOP.store(t, Ordering::Relaxed);
        END.store(e, Ordering::Relaxed);
        
        
        if IN_RESET.load(Ordering::Relaxed) {
            channel.set_duty(0.0.get_duty(&basic_norm));
            led_pin.set_low().unwrap();
            // delay.delay_ms(500);
            // led_pin.set_high().unwrap();
            // delay.delay_ms(500);
            continue;
        } else {
            led_pin.set_high().unwrap();
        }
        // Check if we need to do usb stuff (aka did we receive a message)
        
        match MODE.load(Ordering::Relaxed) {
            0 => {
                //usb
                //do nothing, we already set power above
            },
            1 => { //local
                let top_err = (t - TOP_OFFSET.load(Ordering::Relaxed)) as f32 * RADS_PER_TICK - PI;
                if top_err > PI/2.0 || top_err < -PI/2.0 {
                    CUR_POWER.store(0.0.get_duty(&basic_norm), Ordering::Relaxed);
                    MODE.store(0, Ordering::Relaxed);
                } else {
                    CUR_POWER.store((-1.0*
                        (((c - CART_OFFSET.load(Ordering::Relaxed)) as f32 * CART_M_PER_TICK) * f32::from_be_bytes(K[0].load(Ordering::Relaxed).to_be_bytes())
                        + top_err * f32::from_be_bytes(K[1].load(Ordering::Relaxed).to_be_bytes())
                        + ((e - END_OFFSET.load(Ordering::Relaxed)) as f32 * RADS_PER_TICK) * f32::from_be_bytes(K[2].load(Ordering::Relaxed).to_be_bytes())
                        + VC.load(Ordering::Relaxed) as f32 * CART_M_PER_TICK * 100.0 * f32::from_be_bytes(K[2].load(Ordering::Relaxed).to_be_bytes()) //2pi/4096 (radians/ticks) times 1_000_000 (us/s)
                        + VT.load(Ordering::Relaxed) as f32 * RADS_PER_TICK * 100.0 * f32::from_be_bytes(K[2].load(Ordering::Relaxed).to_be_bytes())
                        + VE.load(Ordering::Relaxed) as f32 * RADS_PER_TICK * 100.0 * f32::from_be_bytes(K[2].load(Ordering::Relaxed).to_be_bytes()))
                    ).get_duty(&unsafe_norm), Ordering::Relaxed); //calibrated values for 0, -1, 1
                }
            },
            2 => { //pleb
                CUR_POWER.store(pleb_fn(timer
                                    .get_counter()
                                    .checked_duration_since(t0)
                                    .unwrap()
                                    .to_micros(),
                                ).unwrap_or(0.0).get_duty(&basic_norm),
                                Ordering::Relaxed);
            },
            _ => {}
        }
        // if state[0] > 0.8 || state[0] < -0.8 {
        //     cur_power = 0.0.get_duty(&basic_norm);
        // }
        channel.set_duty(CUR_POWER.load(Ordering::Relaxed));
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
    
    // let i2c

    // Say hello exactly once on start-up
    if !SAID_HELLO.load(Ordering::Relaxed) {
        SAID_HELLO.store(true, Ordering::Relaxed);
        let _ = serial.write(b"Hello, World!\r\n");
    }

    // Poll the USB driver with all of our supported USB Classes
    if usb_dev.poll(&mut [serial]) {
        let mut buf = [0, 91, 165, 0, 0]; //format: 1 byte command, up to 4 bytes data. Default command is set power to 0.

        if IN_RESET.load(Ordering::Relaxed) {
            match serial.read(&mut buf) {
                Ok(_) => match buf[0] {
                    100 => {
                        IN_RESET.store(false, Ordering::Relaxed);
                        let _ = serial.write(b"Reset mode deactivated. Functionality restored.\n");
                        CUR_POWER.store(0.0.get_duty(&basic_norm), Ordering::Relaxed)
                    }
                    _ => {
                        let _ = serial.write(b"System in reset. Send 01100100 to reactivate.\n");
                    } //don't do anything unless we get ok message
                },
                Err(_) => {}
            }
        } else {
            match serial.read(&mut buf) {
                Ok(1) => {
                    if buf[0] == 9 {
                        CART_OFFSET.store(CART.load(Ordering::Relaxed), Ordering::Relaxed);
                        TOP_OFFSET.store(TOP.load(Ordering::Relaxed), Ordering::Relaxed);
                        END_OFFSET.store(END.load(Ordering::Relaxed), Ordering::Relaxed);
                        
                        CART_ROTS.store(0, Ordering::Relaxed);
                        TOP_ROTS.store(0, Ordering::Relaxed);
                        END_ROTS.store(0, Ordering::Relaxed);
                    }
                },
                Ok(_) => {
                    match buf[0] { //32767.5
                        0 => {
                            CUR_POWER.store(((u16::from_be_bytes([buf[1], buf[2]]) as f32 / 32767.5) - 1.0).get_duty(&basic_norm), Ordering::Relaxed);
                        },
                        1 => {
                            MODE.store(buf[1], Ordering::Relaxed);
                        }
                        2..=8 => {
                            K[(buf[0] as usize)-2].store(i32::from_be_bytes([buf[1], buf[2], buf[3], buf[4]]), Ordering::Relaxed);
                        }
                        10 => {
                            // *GET_STATUS_FLAG.as_mut().unwrap() = true;
                        },
                        _ => {},
                    }
                    let mut message: String<100> = String::new();
                    let _ = write!(&mut message, "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}\n", 
                        (CART.load(Ordering::Relaxed) - CART_OFFSET.load(Ordering::Relaxed)) as f32 * CART_M_PER_TICK, 
                        (TOP.load(Ordering::Relaxed) -  TOP_OFFSET.load(Ordering::Relaxed)) as f32 * RADS_PER_TICK, 
                        (END.load(Ordering::Relaxed) -  END_OFFSET.load(Ordering::Relaxed)) as f32 * RADS_PER_TICK, 
                        VC.load(Ordering::Relaxed) as f32 * CART_M_PER_TICK * 20.0, 
                        VT.load(Ordering::Relaxed) as f32 * RADS_PER_TICK * 20.0, 
                        VE.load(Ordering::Relaxed) as f32 * RADS_PER_TICK * 20.0,
                    );
                    let _ = serial.write(message.as_bytes());
                }
                Err(_e) => {} // do nothing, idk what to do
            }
        }
    }
}

#[interrupt]
unsafe fn IO_IRQ_BANK0() {
    CUR_POWER.store(0.0.get_duty(&basic_norm), Ordering::Relaxed);
    IN_RESET.store(true, Ordering::Relaxed);
    // USB_SERIAL.as_mut().unwrap().write(b"Limit Switch Triggered! Waiting for reset command.\n").unwrap();
    match RESET_PIN.as_mut() {
        Some(reset_pin) => {reset_pin.clear_interrupt(hal::gpio::Interrupt::EdgeHigh)},
        None => {}
    }
}

#[interrupt]
unsafe fn TIMER_IRQ_0() {
    // use core::sync::atomic::{AtomicBool, AtomicI32, Ordering};
    let vc = OLD_VC.load(Ordering::Relaxed);
    OLD_VC.store(CART.load(Ordering::Relaxed), Ordering::Relaxed);

    let vt = OLD_VT.load(Ordering::Relaxed);
    OLD_VT.store(TOP.load(Ordering::Relaxed), Ordering::Relaxed);

    let ve = OLD_VE.load(Ordering::Relaxed);
    OLD_VE.store(END.load(Ordering::Relaxed), Ordering::Relaxed);

    VC.store(CART.load(Ordering::Relaxed) - vc, Ordering::Relaxed);
    VT.store( TOP.load(Ordering::Relaxed) - vt, Ordering::Relaxed);
    VE.store( END.load(Ordering::Relaxed) - ve, Ordering::Relaxed);

    let alarm0 = ALARM.as_mut().unwrap();
    let _ = alarm0.schedule(50_000.micros());
    alarm0.clear_interrupt();
}