// lol no std (bc we don't have that much space)
#![no_std]
#![no_main]

use rp_pico::entry; // startup function macro
use rp_pico::hal;
use embedded_hal::digital::v2::OutputPin;
use hal::{Timer, pac, prelude::*};
use hal::gpio::{Pin, PullUpInput, PushPullOutput};
use hal::pio::{SM1, Running, Interrupt, SM0};
use hal::gpio::PinState;
// use hal::prelude::*;
use hal::timer::Alarm;
// shorter hal alias
use usb_device::{class_prelude::*, prelude::*}; // USB device emulation
use usbd_serial::SerialPort; // more USB stuff
use heapless::String; 
use core::f32::consts::PI;
use core::fmt::Write;
// use core::sync::atomic::AtomicU64;
use embedded_hal::blocking::i2c::{Operation, Transactional}; // I2C HAL traits/types
use embedded_hal::prelude::_embedded_hal_blocking_i2c_WriteRead; // actual i2c func
use fugit::{RateExtU32, ExtU32}; // for .kHz() and similar
use panic_halt as _; // make sure program halts on panic (need to mention crate for it to be linked)
use pac::interrupt;
use core::sync::atomic::{AtomicI32, AtomicU32, AtomicU16, AtomicU8, AtomicBool, Ordering};
use hal::pio::{PIOBuilder, ValidStateMachine, Tx};
use pio_proc::pio_file;

// const STEPS_PER_MM: i32 = 10; // unused
const PIO_FREQ: u32 = 5_000_000; // Hz
const MAX_SPEED: i32 = 5000; // mm/s
const MAX_ACCELERATION: u32 = 5000; //mm/s^2
const STEPPER_VELOCITY_UPDATE_TIME: u32 = 50_000; // in us (5 ticks/us, so max (5*this)/19 steps, min speed 0.1/5*this mm/s)


trait GetPeriod{
    fn get_period(&self) -> Result<(PinState, u32, u32), &'static str>;
}
impl GetPeriod for i32{
    // takes speed in 0.1mm/s and converts it to period for pio
    fn get_period(&self) -> Result<(PinState, u32, u32), &'static str>{
        if self == &0 {return Ok((PinState::Low, u32::MAX, 0))}
        let pwr = if self > &(MAX_SPEED*10) {MAX_SPEED*10} else {*self};
        let dir = if self < &0 {PinState::Low} else {PinState::High};
        let delay = PIO_FREQ / pwr as u32;
        if delay < 19 {return Err("speed too high (took too much of itself lmao)");}
        Ok((dir, delay-19, (STEPPER_VELOCITY_UPDATE_TIME*(PIO_FREQ/1_000_000))/(delay-19))) // 19 is len of pio program, PIO_FREQ/1_000_000 is pio ticks/us
    }
}

static mut DIR_PIN: Option<Pin<hal::gpio::bank0::Gpio5, PushPullOutput>> = None;
static mut DELAY_TX: Option<Tx<(pac::PIO1, SM0)>> = None;
static mut COUNT_TX: Option<Tx<(pac::PIO1, SM1)>> = None;

static CART_VEL: AtomicI32 = AtomicI32::new(0); // 0.1 * mm/s 
static CART_ACC: AtomicI32 = AtomicI32::new(0); // 0.1 * mm/s^2
static mut PIO1: Option<hal::pio::PIO<pac::PIO1>> = None;

static PREV_TIME: AtomicU32 = AtomicU32::new(0);
static MODE: AtomicU8 = AtomicU8::new(0);
static CUR_POWER: AtomicI32 = AtomicI32::new(0);
static IN_RESET: AtomicBool = AtomicBool::new(false);
static CART_POS: AtomicI32 = AtomicI32::new(0);

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

/// The USB Serial Device Driver (shared with the interrupt).
static mut USB_SERIAL: Option<SerialPort<hal::usb::UsbBus>> = None;

static mut ALARM: Option<hal::timer::Alarm0> = None;

static mut RESET_PIN: Option<Pin<hal::gpio::bank0::Gpio11, PullUpInput>> = None;

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
    // let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let (mut pio1, sm0, sm1, _, _) = pac.PIO1.split(&mut pac.RESETS);

    unsafe { PIO1 = Some(pio1); }

    pio1.irq1().enable_sm_interrupt(2);


    // Create a pio programs
    let step = pio_file!("steps.pio", select_program("step"),);
    let stepcaller = pio_file!("steps.pio", select_program("stepcaller"),);
    let step_installed = pio1.install(&step.program).unwrap();
    let stepcaller_installed = pio1.install(&stepcaller.program).unwrap();

    // Set gpio6 (step pin) to pio
    let _step: hal::gpio::Pin<_, hal::gpio::FunctionPio1> = pins.gpio6.into_mode();
    let step_pin_id = 6;

    // Build the pio program and set pin both for set and side set!
    // We are running with the default divider which is 1 (max speed)
    let (mut step_sm, _, mut tx0) = PIOBuilder::from_program(step_installed)
        .set_pins(step_pin_id, 1)
        .side_set_pin_base(step_pin_id)
        .clock_divisor_fixed_point(25, 0)
        .build(sm0);
    let (mut stepcaller_sm, _, mut tx1) = PIOBuilder::from_program(stepcaller_installed)
        .clock_divisor_fixed_point(25, 0)
        .build(sm1);

    // Set pio pindir for gpio6
    step_sm.set_pindirs([(step_pin_id, hal::pio::PinDir::Output)]);

    // Start state machine
    let step_sm = step_sm.start();
    let stepcaller_sm = stepcaller_sm.start();
    // setup direction pin
    let mut dir_pin = pins.gpio5.into_push_pull_output();

    let (dir, step, count) = 0.get_period().unwrap_or((PinState::Low, u32::MAX, 0));
    unsafe {
        DIR_PIN = Some(dir_pin);
        DELAY_TX = Some(tx0);
        COUNT_TX = Some(tx1);
    }

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

    // init variables to track positions
    let (mut top_rots, mut end_rots);
    let (mut top_pos, mut end_pos) = (0,0);
    let (mut dt, mut de);
    let (mut oldt, mut olde);

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
    unsafe { pac::NVIC::unmask(hal::pac::Interrupt::PIO0_IRQ_0); }

    // led_pin.set_low().unwrap();
    // delay.delay_ms(1000);
    // led_pin.set_high().unwrap();

    loop {
        let mut top = [0; 2];
        let mut end = [0; 2];
        
        // first grab all encoder positions
        let _ = top_i2c.write_read(0x36, &[0x0Cu8], &mut top);
        let _ = end_i2c.write_read(0x36, &[0x0Cu8], &mut end);
        
        if u16::from_be_bytes(top)  > 4096 || 
           u16::from_be_bytes(end)  > 4096 { continue; }
        
        oldt = top_pos;
        olde = end_pos;
        // now update the positions
        top_pos = u16::from_be_bytes(top) as i32;
        end_pos = u16::from_be_bytes(end) as i32;
        
        dt = top_pos-oldt;
        de = end_pos-olde;

        // check if angle wrap happened
        top_rots = TOP_ROTS.load(Ordering::Relaxed);
        end_rots = END_ROTS.load(Ordering::Relaxed);
        
        if dt > 3500 && dt <= 4096 { TOP_ROTS.store(top_rots - 1, Ordering::Relaxed); top_rots -= 1;}
        else if dt < -3500 && dt >= -4096 { TOP_ROTS.store(top_rots + 1, Ordering::Relaxed); top_rots += 1;}
        
        if de > 3500 && de <= 4096 { END_ROTS.store(end_rots - 1, Ordering::Relaxed); end_rots -= 1;}
        else if de < -3500 && de >= -4096 { END_ROTS.store(end_rots + 1, Ordering::Relaxed); end_rots += 1;}
        
        let (t, e) = (top_pos+top_rots*4096, end_pos+end_rots*4096);
        TOP.store(t, Ordering::Relaxed);
        END.store(e, Ordering::Relaxed);
        
        
        if IN_RESET.load(Ordering::Relaxed) {
            CART_VEL.store(0, Ordering::Relaxed);
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
                //do nothing, we already set power in usb irq
            },
            1 => { //local
                let top_err = (t - TOP_OFFSET.load(Ordering::Relaxed)) as f32 * RADS_PER_TICK - PI;
                if top_err > PI/2.0 || top_err < -PI/2.0 {
                    CART_VEL.store(0, Ordering::Relaxed);
                    CART_ACC.store(0, Ordering::Relaxed);
                    MODE.store(0, Ordering::Relaxed);
                } else {
                    //TODO: fix this
                    CART_ACC.store((10000.0*
                        (CART_POS.load(Ordering::Relaxed) as f32 * 0.00001 * f32::from_be_bytes(K[0].load(Ordering::Relaxed).to_be_bytes())
                        + top_err * f32::from_be_bytes(K[1].load(Ordering::Relaxed).to_be_bytes())
                        + ((e - END_OFFSET.load(Ordering::Relaxed)) as f32 * RADS_PER_TICK) * f32::from_be_bytes(K[2].load(Ordering::Relaxed).to_be_bytes())
                        + CART_VEL.load(Ordering::Relaxed) as f32 * 0.00001 * f32::from_be_bytes(K[2].load(Ordering::Relaxed).to_be_bytes())
                        + VT.load(Ordering::Relaxed) as f32 * RADS_PER_TICK * f32::from_be_bytes(K[2].load(Ordering::Relaxed).to_be_bytes())
                        + VE.load(Ordering::Relaxed) as f32 * RADS_PER_TICK * f32::from_be_bytes(K[2].load(Ordering::Relaxed).to_be_bytes()))
                    ) as i32, Ordering::Relaxed); //calibrated values for 0, -1, 1
                }
            },
            // 2 => { //pleb
            //     CUR_POWER.store(pleb_fn(timer
            //                         .get_counter()
            //                         .checked_duration_since(t0)
            //                         .unwrap()
            //                         .to_micros(),
            //                     ).unwrap_or(0.0).get_duty(&basic_norm),
            //                     Ordering::Relaxed);
            // },
            _ => {}
        }
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
    // use core::sync::atomic::{AtomicBool, Ordering};

    // Grab the global objects. This is OK as we only access them under interrupt.
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let serial = USB_SERIAL.as_mut().unwrap();

    // Poll the USB driver with all of our supported USB Classes
    if usb_dev.poll(&mut [serial]) {
        let mut buf = [0, 0, 0, 0, 0]; //format: 1 byte command, up to 4 bytes data. Default command is set power to 0.

        if IN_RESET.load(Ordering::Relaxed) {
            match serial.read(&mut buf) {
                Ok(_) => match buf[0] {
                    100 => {
                        IN_RESET.store(false, Ordering::Relaxed);
                        let _ = serial.write(b"Reset mode deactivated. Functionality restored.\n");
                        CART_ACC.store(0, Ordering::Relaxed);
                        CART_VEL.store(0, Ordering::Relaxed);
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
                        TOP_OFFSET.store(TOP.load(Ordering::Relaxed), Ordering::Relaxed);
                        END_OFFSET.store(END.load(Ordering::Relaxed), Ordering::Relaxed);
                        
                        TOP_ROTS.store(0, Ordering::Relaxed);
                        END_ROTS.store(0, Ordering::Relaxed);
                    }
                },
                Ok(_) => {
                    match buf[0] { //32767.5
                        0 => {
                            CART_ACC.store(i32::from_be_bytes([buf[1], buf[2], buf[3], buf[4]]), Ordering::Relaxed);
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
                    let _ = write!(&mut message, "{:.6},{:.6},{:.6},{:.6}\n",  
                        (TOP.load(Ordering::Relaxed) -  TOP_OFFSET.load(Ordering::Relaxed)) as f32 * RADS_PER_TICK, 
                        (END.load(Ordering::Relaxed) -  END_OFFSET.load(Ordering::Relaxed)) as f32 * RADS_PER_TICK, 
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
    CART_ACC.store(0, Ordering::Relaxed);
    CART_VEL.store(0, Ordering::Relaxed);
    IN_RESET.store(true, Ordering::Relaxed);
    // USB_SERIAL.as_mut().unwrap().write(b"Limit Switch Triggered! Waiting for reset command.\n").unwrap();
    match RESET_PIN.as_mut() {
        Some(reset_pin) => {reset_pin.clear_interrupt(hal::gpio::Interrupt::EdgeHigh)},
        None => {}
    }
}

#[interrupt]
unsafe fn TIMER_IRQ_0() {
    let alarm0 = ALARM.as_mut().unwrap();
    let _ = alarm0.schedule(50_000.micros());

    let vt = OLD_VT.load(Ordering::Relaxed);
    OLD_VT.store(TOP.load(Ordering::Relaxed), Ordering::Relaxed);

    let ve = OLD_VE.load(Ordering::Relaxed);
    OLD_VE.store(END.load(Ordering::Relaxed), Ordering::Relaxed);

    VT.store( TOP.load(Ordering::Relaxed) - vt, Ordering::Relaxed);
    VE.store( END.load(Ordering::Relaxed) - ve, Ordering::Relaxed);

    alarm0.clear_interrupt();
}

#[interrupt]
unsafe fn PIO0_IRQ_1() {
    let cart_vel = CART_VEL.load(Ordering::Relaxed);
    CART_VEL.store(cart_vel + CART_ACC.load(Ordering::Relaxed)/20, Ordering::Relaxed); //0.1 mm/s^2, /20 bc time step is 50 ms
    let (dir, step, count) = cart_vel.get_period().unwrap_or((PinState::Low, u32::MAX, 0u32));
    CART_POS.store(
        CART_POS.load(Ordering::Relaxed)
        + (count as i32 * match dir { 
            PinState::High => 1, 
            PinState::Low => -1,
        })
    , Ordering::Relaxed); // hacky but good enough for now

    DELAY_TX.unwrap().write(step);
    COUNT_TX.unwrap().write(count);
    DIR_PIN.unwrap().set_state(dir);
    
    PIO1.unwrap().clear_irq(32) // 32 is 00100000, so clears irq 2??? idek anymore
}