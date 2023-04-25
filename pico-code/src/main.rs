// lol no std (bc we don't have that much space)
#![no_std]
#![no_main]

use pio::MovOperation;
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
use micromath::F32Ext;

// const STEPS_PER_MM: i32 = 10; // unused
const PIO_FREQ: u32 = 5_000_000; // Hz
const MAX_SPEED: i32 = 1000; // mm/s
const MAX_ACCELERATION: i32 = 1000; //mm/s^2
const STEPPER_VELOCITY_UPDATE_TIME: u32 = 10_000; // in us (5 ticks/us, so max (5*this)/19 steps, min speed 0.1/5*this mm/s)


trait GetPeriod{
    fn get_period(&self) -> Result<(PinState, u32), &'static str>;
}
impl GetPeriod for i32{
    // takes speed in 0.1mm/s and converts it to period for pio
    fn get_period(&self) -> Result<(PinState, u32), &'static str>{
        if self == &0 {return Ok((PinState::Low, u32::MAX))}
        let pwr = if self > &(MAX_SPEED*10) {MAX_SPEED*10} 
                  else if self < &-(MAX_SPEED*10) {-MAX_SPEED*10} 
                  else {*self};
        let dir = if self < &0 {PinState::High} else {PinState::Low};
        let delay = PIO_FREQ / pwr.abs() as u32;
        if delay < 19 {return Err("speed too high (took too much of itself lmao)");}
        Ok((dir, delay-14)) // 14 is len of pio program, PIO_FREQ/1_000_000 is pio ticks/us
    }
}
static RAW_TOP: AtomicI32 = AtomicI32::new(0);

static mut DIR_PIN: Option<Pin<hal::gpio::bank0::Gpio6, PushPullOutput>> = None;
static mut DELAY_TX: Option<Tx<(pac::PIO1, SM0)>> = None;
static mut COUNT_TX: Option<Tx<(pac::PIO1, SM1)>> = None;
static mut PIO1: Option<hal::pio::PIO<pac::PIO1>> = None;
static mut STEP_SM: Option<hal::pio::StateMachine<(pac::PIO1, SM0), Running>> = None;
static IRQ_COUNTER: AtomicU32 = AtomicU32::new(0);

static CART_VEL: AtomicI32 = AtomicI32::new(0); // 0.1 * mm/s 
static CART_ACC: AtomicI32 = AtomicI32::new(0); // 0.1 * mm/s^2

static MODE: AtomicU8 = AtomicU8::new(0);
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

static OLD_VT: AtomicI32 = AtomicI32::new(0);
static OLD_VE: AtomicI32 = AtomicI32::new(0);

static VT: AtomicI32 = AtomicI32::new(0);
static VE: AtomicI32 = AtomicI32::new(0);

static TOP: AtomicI32 = AtomicI32::new(0);
static END: AtomicI32 = AtomicI32::new(0);

static TOP_OFFSET: AtomicI32 = AtomicI32::new(68);
static END_OFFSET: AtomicI32 = AtomicI32::new(0);

static TOP_ROTS: AtomicI32 = AtomicI32::new(0);
static END_ROTS: AtomicI32 = AtomicI32::new(0);
static K: [AtomicI32; 6] = [AtomicI32::new(0),
                            AtomicI32::new(0),
                            AtomicI32::new(0),
                            AtomicI32::new(0),
                            AtomicI32::new(0),
                            AtomicI32::new(0)];

static SP: [AtomicI32; 3] = [AtomicI32::new(0), //cart setpoint  (m)
                            AtomicI32::new(0),  //top setpoint (rad)
                            AtomicI32::new(0)]; //end setpoint (rad)

const RADS_PER_TICK: f32 = PI/2048.0;


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
    let mut delay = cortex_m::delay::Delay::new(_core.SYST, clocks.system_clock.freq().to_Hz());
    
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
    let (mut pio1, mut sm0, mut sm1, _, _) = pac.PIO1.split(&mut pac.RESETS);

    
    
    let simple = pio_file!("steps.pio", select_program("simple"),);
    let simple_installed = pio1.install(&simple.program).unwrap();
    
    
    // Set gpio6 (step pin) to pio
    let _step: hal::gpio::Pin<_, hal::gpio::FunctionPio1> = pins.gpio5.into_mode();
    // _step.id().num
    let step_pin_id = _step.id().num;
    // Build the pio program and set pin both for set and side set!
    // We are running with the default divider which is 1 (max speed)
    let (mut step_sm, _, mut tx0) = PIOBuilder::from_program(simple_installed)
        .set_pins(step_pin_id, 1)
        .clock_divisor_fixed_point(25, 0)
        .build(sm0);

    // Set pio pindir for gpio6
    step_sm.set_pindirs([(step_pin_id, hal::pio::PinDir::Output)]);
    
    let mut dir_pin = pins.gpio6.into_push_pull_output();
    dir_pin.set_low().unwrap();
    unsafe {
        DIR_PIN = Some(dir_pin);
        DELAY_TX = Some(tx0);
    }
    
    // step_sm.synchronize_with(&mut stepcaller_sm);
    // let irq = pio1.irq1();
    // irq.enable_sm_interrupt(2);
    // unsafe { PIO1 = Some(pio1); }
    // Start state machine
    let mut step_sm = step_sm.start();
    // let mut stepcaller_sm = stepcaller_sm.start();
    // setup direction pin
    // step_sm.drain_tx_fifo();
    
    unsafe { STEP_SM = Some(step_sm); }

    // unsafe { 
    //     let tx = DELAY_TX.as_mut().unwrap(); 
    //     let sm = STEP_SM.as_mut().unwrap();
    //     for i in 0..250{
    //         tx.write(500_000/((2*i+100)));
    //         sm.exec_instruction(
    //             pio::Instruction { 
    //                 operands: pio::InstructionOperands::PULL { 
    //                     if_empty: false, 
    //                     block: true }, 
    //                     delay: 0, 
    //                     side_set: None,
    //                 }
    //             );
    //         delay.delay_ms(10);
    //     }
    // }
    
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
    // unsafe { pac::NVIC::unmask(hal::pac::Interrupt::PIO1_IRQ_1); }

    // led_pin.set_low().unwrap();
    // delay.delay_ms(1000);
    // led_pin.set_high().unwrap();

    loop {
        let mut top = [0; 2];
        let mut end = [0; 2];
        
        // first grab all encoder positions
        let _ = top_i2c.write_read(0x36, &[0x0Eu8], &mut top);
        let _ = end_i2c.write_read(0x36, &[0x0Eu8], &mut end);
        
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
                let top_err = - ((t + TOP_OFFSET.load(Ordering::Relaxed))   as f32 + 13.5 * (1.0 + (((t + TOP_OFFSET.load(Ordering::Relaxed))   as f32) * RADS_PER_TICK).cos())) * RADS_PER_TICK - f32::from_be_bytes(SP[1].load(Ordering::Relaxed).to_be_bytes());
                let end_err =   ((e - 2206) as f32 +  3.5 * (1.0 + (((e - 2206) as f32) * RADS_PER_TICK).cos())) * RADS_PER_TICK - f32::from_be_bytes(SP[2].load(Ordering::Relaxed).to_be_bytes());
                CART_ACC.store((-10000.0*
                    (CART_POS.load(Ordering::Relaxed) as f32 * 0.0001 - f32::from_be_bytes(SP[0].load(Ordering::Relaxed).to_be_bytes()) * f32::from_be_bytes(K[0].load(Ordering::Relaxed).to_be_bytes())
                    + top_err * f32::from_be_bytes(K[1].load(Ordering::Relaxed).to_be_bytes())
                    + end_err * f32::from_be_bytes(K[2].load(Ordering::Relaxed).to_be_bytes())
                    + CART_VEL.load(Ordering::Relaxed) as f32 * 0.0001 * f32::from_be_bytes(K[3].load(Ordering::Relaxed).to_be_bytes())
                    + VT.load(Ordering::Relaxed) as f32 * RADS_PER_TICK * f32::from_be_bytes(K[4].load(Ordering::Relaxed).to_be_bytes()) * (200_000/(STEPPER_VELOCITY_UPDATE_TIME)) as f32
                    + VE.load(Ordering::Relaxed) as f32 * RADS_PER_TICK * f32::from_be_bytes(K[5].load(Ordering::Relaxed).to_be_bytes()) * (200_000/(STEPPER_VELOCITY_UPDATE_TIME)) as f32)
                ) as i32, Ordering::Relaxed); //calibrated values for 0, -1, 1
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

                        TOP_ROTS.store(0, Ordering::Relaxed);
                        END_ROTS.store(0, Ordering::Relaxed);

                        TOP_OFFSET.store(TOP.load(Ordering::Relaxed)%4096,  Ordering::Relaxed);
                        END_OFFSET.store(END.load(Ordering::Relaxed)%4096, Ordering::Relaxed);
                        

                        let _ = serial.write(b"All counts set to zero.\n");
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
                        11..=14 => {
                            SP[(buf[0] as usize-11)].store(i32::from_be_bytes([buf[1], buf[2], buf[3], buf[4]]), Ordering::Relaxed)
                        },
                        _ => {},
                    }
                    //def fix_ang(x): return x+76 + 0.5*(cos(2*pi*(x+76)/4096)+1)*27
                    let angle = (TOP.load(Ordering::Relaxed) + TOP_OFFSET.load(Ordering::Relaxed));
                    let final_angle = (angle as f32 + 13.5*(1.0+(PI*(angle as f32)/2048.0).cos())) * RADS_PER_TICK;
                    let angle2 = (END.load(Ordering::Relaxed) - 2206);
                    let final_angle2 = (angle2 as f32 + 3.5*(1.0+(PI*(angle2 as f32)/2048.0).cos())) * RADS_PER_TICK;
                    // 2206 0
                    // 4247 pi
                    let mut message: String<100> = String::new();
                    let _ = write!(&mut message, "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}\n", 
                        CART_POS.load(Ordering::Relaxed) as f32 / 10000.0,
                        -final_angle, 
                        final_angle2,
                        CART_VEL.load(Ordering::Relaxed) as f32 / 10000.0,
                        VT.load(Ordering::Relaxed) as f32 * RADS_PER_TICK * (200_000/(STEPPER_VELOCITY_UPDATE_TIME)) as f32, 
                        VE.load(Ordering::Relaxed) as f32 * RADS_PER_TICK * (200_000/(STEPPER_VELOCITY_UPDATE_TIME)) as f32,
                        CART_ACC.load(Ordering::Relaxed) as f32 / 10000.0,
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
    let _ = alarm0.schedule(STEPPER_VELOCITY_UPDATE_TIME.micros());
    let c = IRQ_COUNTER.load(Ordering::Relaxed);
    if c >= 4 {
        IRQ_COUNTER.store(0, Ordering::Relaxed);
        let angle = (TOP.load(Ordering::Relaxed) + TOP_OFFSET.load(Ordering::Relaxed));
        let final_angle = (angle as f32 + 13.5*(1.0+((angle as f32)*RADS_PER_TICK).cos())) as i32;
        let angle2 = (END.load(Ordering::Relaxed) - 2206);
        let final_angle2 = (angle2 as f32 + 3.5*(1.0+((angle2 as f32)*RADS_PER_TICK).cos())) as i32;

        let vt = (OLD_VT.load(Ordering::Relaxed) + TOP_OFFSET.load(Ordering::Relaxed));
        let final_vt = (vt as f32 + 13.5*(1.0+((vt as f32)*RADS_PER_TICK).cos())) as i32;
        let ve = (OLD_VE.load(Ordering::Relaxed) - 2206);
        let final_ve = (ve as f32 +  3.5*(1.0+((ve as f32)*RADS_PER_TICK).cos())) as i32;
        
        // let vt = OLD_VT.load(Ordering::Relaxed);
        OLD_VT.store(TOP.load(Ordering::Relaxed), Ordering::Relaxed);

        // let ve = OLD_VE.load(Ordering::Relaxed);
        OLD_VE.store(END.load(Ordering::Relaxed), Ordering::Relaxed);

        VT.store( -(final_angle - final_vt), Ordering::Relaxed);
        VE.store( final_angle2 - final_ve, Ordering::Relaxed);
    } else {
        IRQ_COUNTER.store(c+1, Ordering::Relaxed);
    }

    let tx = DELAY_TX.as_mut().unwrap(); 
    let sm = STEP_SM.as_mut().unwrap();
    let mut cart_vel = CART_VEL.load(Ordering::Relaxed);
    let mut acc = CART_ACC.load(Ordering::Relaxed);
    CART_POS.store(CART_POS.load(Ordering::Relaxed) + cart_vel/100, Ordering::Relaxed);

    // impose limits on acceleration and velocity
    if acc > MAX_ACCELERATION*10 { acc = MAX_ACCELERATION*10; }
    else if acc < (-MAX_ACCELERATION*10) { acc = -MAX_ACCELERATION*10; }
    if cart_vel > MAX_SPEED*10 { cart_vel = MAX_SPEED*10; }
    else if cart_vel < (-MAX_SPEED*10) { cart_vel = -MAX_SPEED*10; }
    
    CART_VEL.store(cart_vel + acc/100, Ordering::Relaxed); //0.1 mm/s^2, /20 bc time step is 50 ms
    let (dir, step) = cart_vel.get_period().unwrap_or((PinState::Low, u32::MAX));

    match DIR_PIN.as_mut().unwrap().set_state(dir){
        Ok(_) => {},
        Err(_) => {IN_RESET.store(true, Ordering::Relaxed);}
    }
    tx.write(step);
    sm.exec_instruction(
        pio::Instruction { 
            operands: pio::InstructionOperands::PULL { 
                if_empty: false, 
                block: true }, 
                delay: 0, 
                side_set: None,
            }
        );
    sm.exec_instruction(
        pio::Instruction { 
                operands: pio::InstructionOperands::MOV {
                     destination: pio::MovDestination::X, 
                     op: pio::MovOperation::None, 
                     source: pio::MovSource::OSR,
                },
                delay: 0, 
                side_set: None,
            }
        );
    alarm0.clear_interrupt();
}