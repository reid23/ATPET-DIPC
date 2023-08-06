// lol no std (bc we don't have that much space)
#![no_std]
#![no_main]

use hal::{spi::Spi, gpio::{Pins, FunctionSpi}, Sio};
use embedded_hal::spi::MODE_0;
use embedded_hal::prelude::_embedded_hal_blocking_i2c_Read;
use embedded_hal::prelude::_embedded_hal_blocking_i2c_Write;
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
// use panic_abort as _;
use pac::interrupt;
use core::sync::atomic::{AtomicI32, AtomicU32, AtomicU16, AtomicU8, AtomicBool, Ordering};
use hal::pio::{PIOBuilder, ValidStateMachine, Tx};
use pio_proc::pio_file;
use micromath::F32Ext;
use core::panic::PanicInfo;

// const STEPS_PER_MM: i32 = 10; // unused
const PIO_FREQ: u32 = 5_000_000; // Hz
const MAX_SPEED: i32 = 5000; // mm/s
const MAX_ACCELERATION: i32 = 5000; //mm/s^2
const STEPPER_VELOCITY_UPDATE_TIME: u32 = 10_000; // in us (5 ticks/us, so max (5*this)/19 steps, min speed 0.1/5*this mm/s)
const SPEED_MULT: i32 = 100;

trait GetPeriod{
    fn get_period(&self) -> Result<(PinState, u32), &'static str>;
}
impl GetPeriod for i32{
    // takes speed in 0.1mm/s and converts it to period for pio
    fn get_period(&self) -> Result<(PinState, u32), &'static str>{
        if self == &0 {return Ok((PinState::Low, u32::MAX))}
        let pwr = if self > &(MAX_SPEED*10*SPEED_MULT) {MAX_SPEED*10*SPEED_MULT} 
                  else if self < &-(MAX_SPEED*10*SPEED_MULT) {-MAX_SPEED*10*SPEED_MULT} 
                  else {*self};
        let dir = if self < &0 {PinState::High} else {PinState::Low};
        let delay = (SPEED_MULT as u32 * PIO_FREQ) / pwr.abs() as u32;
        // if delay > 2_000_000 {return Err("speed too low")}
        if delay < 19 {return Err("speed too high (took too much of itself lmao)");}
        Ok((dir, delay-14)) // 14 is len of pio program, PIO_FREQ/1_000_000 is pio ticks/us
    }
}

#[derive(PartialEq, Eq)]
enum Encoder {
    TOP = 4020,
    END = 2714,
}
trait AngleWrap{
    fn wrap_angle(&self) -> f32;
}
impl AngleWrap for f32 {
    fn wrap_angle(&self) -> f32 {
        (*self+PI).rem_euclid(2.0*PI)-PI
    }
}
trait ToRad{
    fn to_rad(&self, encoder: Encoder) -> f32;
}
impl ToRad for i32 {
    fn to_rad(&self, encoder: Encoder) -> f32{
        if encoder == Encoder::TOP {
            let ticks = (*self-5) as f32;
            - (ticks + 19.0 * (- 1.0 + (ticks * RADS_PER_TICK).cos())) * RADS_PER_TICK
        } else if encoder == Encoder::END {
            let ticks = (*self-5) as f32;
            ticks * RADS_PER_TICK
        } else {
            0.0
        }
    }
}

// static RAW_TOP: AtomicI32 = AtomicI32::new(0);


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

// commands
// static GET_I2C_STATUS: AtomicBool = AtomicBool::new(false);
// static SET_I2C_ZPOS: AtomicI32 = AtomicI32::new(-1);
// static SET_I2C_MPOS: AtomicI32 = AtomicI32::new(-1);


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

static AVG_LOOP_TIME: AtomicU32 = AtomicU32::new(0);
const RADS_PER_TICK: f32 = PI/2048.0;
const VEL_BUF_LEN: usize = 250;


#[entry]
fn main() -> ! {
    let mut top_buf: [f32; VEL_BUF_LEN] = [0.0; VEL_BUF_LEN];
    let mut end_buf: [f32; VEL_BUF_LEN] = [0.0; VEL_BUF_LEN];
    let mut prev_t: f32 = 0.0;
    let mut prev_e: f32 = 0.0;
    let mut vel_idx: usize = 0;


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
    let mut driver_reset_pin = pins.gpio12.into_push_pull_output();
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
    // enable stepper driver
    driver_reset_pin.set_high().unwrap();
    
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
        800.kHz(),
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
        800.kHz(),
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


    // set filter settings
    // let mut top = [0; 2];
    // let mut end = [0; 2];

    // top_i2c.read(0x07, &mut top);
    // top[8] = 1;
    // top[9] = 0;
    // top_i2c.write(0x07, &top);

    // end_i2c.read(0x07, &mut end);
    // end[8] = 1;
    // end[9] = 0;
    // end_i2c.write(0x07, &end);
    //set zero pos: 4020, 2714: 00001111 10110100, 00001010 10011010
    let mut buf = [0;2];
    top_i2c.write_read(0x36, &[0x01, 0b00001111, 0b10110100], &mut buf);
    end_i2c.write_read(0x36, &[0x01, 0b00001010, 0b10011010], &mut buf);

    //set registers to correct thing because they don't auto-increment from here
    let _ = top_i2c.write_read(0x36, &[0x0Eu8], &mut buf);
    let _ = end_i2c.write_read(0x36, &[0x0Eu8], &mut buf);
    
    // let mut time_buf = [0; 20];
    // let mut counter = 0;
    // let mut acc = 0;
    delay.delay_ms(50);
    loop {
        let tic = timer.get_counter();
        
        let mut top = [0; 2];
        let mut end = [0; 2];

        // first grab all encoder positions
        top_i2c.read(0x36, &mut top);
        end_i2c.read(0x36, &mut end);

        // let _ = top_i2c.write_read(0x36, &[0x0Eu8], &mut top);
        // let _ = end_i2c.write_read(0x36, &[0x0Eu8], &mut end);
        
        // if u16::from_be_bytes(top)  > 4096 || 
        //    u16::from_be_bytes(end)  > 4096 { continue; }
        
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
            if MODE.load(Ordering::Relaxed) != 2 {
                driver_reset_pin.set_low().unwrap();
                continue;
            }
        } else {
            driver_reset_pin.set_high().unwrap();
            led_pin.set_high().unwrap();
        }
        // Check if we need to do usb stuff (aka did we receive a message)
        
        match MODE.load(Ordering::Relaxed) {
            0 => {
                //usb
                //do nothing, we already set power in usb irq
            },
            1 => { //local
                let top_err = - (t as f32 + 19.0 * (- 1.0 + (t as f32 * RADS_PER_TICK).cos())) * RADS_PER_TICK - f32::from_be_bytes(SP[1].load(Ordering::Relaxed).to_be_bytes());
                let end_err = e as f32 * RADS_PER_TICK - f32::from_be_bytes(SP[2].load(Ordering::Relaxed).to_be_bytes());
                CART_ACC.store(((-10000*SPEED_MULT) as f32 *
                ((CART_POS.load(Ordering::Relaxed) as f32 / ((SPEED_MULT*10000) as f32) - f32::from_be_bytes(SP[0].load(Ordering::Relaxed).to_be_bytes())) * f32::from_be_bytes(K[0].load(Ordering::Relaxed).to_be_bytes())
                + top_err.wrap_angle() * f32::from_be_bytes(K[1].load(Ordering::Relaxed).to_be_bytes())
                    + end_err.wrap_angle() * f32::from_be_bytes(K[2].load(Ordering::Relaxed).to_be_bytes())
                    + (CART_VEL.load(Ordering::Relaxed) as f32 / ((SPEED_MULT*10000) as f32)) * f32::from_be_bytes(K[3].load(Ordering::Relaxed).to_be_bytes())
                    + (VT.load(Ordering::Relaxed) as f32 / 100_000.0) * f32::from_be_bytes(K[4].load(Ordering::Relaxed).to_be_bytes())
                    + (VE.load(Ordering::Relaxed) as f32 / 100_000.0) * f32::from_be_bytes(K[5].load(Ordering::Relaxed).to_be_bytes()))
                ) as i32, Ordering::Relaxed); //calibrated values for 0, -1, 1
            }, //     VT.load(Ordering::Relaxed) as f32 * RADS_PER_TICK * (200_000/(STEPPER_VELOCITY_UPDATE_TIME)) as f32
            2 => {// home
                CART_VEL.store(0, Ordering::Relaxed); //homing speed: 200 mm/s
                CART_ACC.store(10000*SPEED_MULT, Ordering::Relaxed); //500 mm/s^2
                delay.delay_ms(400); //get to 200 mm/s (takes 40mm)
                CART_ACC.store(0, Ordering::Relaxed); //stop accelerating

                while !IN_RESET.load(Ordering::Relaxed) {} //wait for button
                CART_VEL.store(0, Ordering::Relaxed);
                CART_POS.store(9000*SPEED_MULT, Ordering::Relaxed); // set location
                
                CART_ACC.store(-10000*SPEED_MULT, Ordering::Relaxed);
                delay.delay_ms(400);
                CART_ACC.store(0, Ordering::Relaxed); //stop accelerating
                delay.delay_ms(1850); //move mm, because it takes 80 to accelerate and 80 to decelerate
                CART_ACC.store(10000*SPEED_MULT, Ordering::Relaxed);
                delay.delay_ms(400);
                CART_ACC.store(0, Ordering::Relaxed);
                CART_VEL.store(0, Ordering::Relaxed);
                
                IN_RESET.store(false, Ordering::Relaxed); //fix reset
                MODE.store(0, Ordering::Relaxed);
            }
            _ => {}
        }
        // now calculate velocity
        let delta_t = timer.get_counter().checked_duration_since(tic).unwrap().to_nanos() as f32 / 10_000.0;
        // let tic = timer.get_counter();
    
        let float_t =  - (t as f32 + 19.0 * (- 1.0 + (t as f32 * RADS_PER_TICK).cos())) * RADS_PER_TICK;
        let float_e = e as f32 * RADS_PER_TICK;
    
        vel_idx += 1;
        if vel_idx >= VEL_BUF_LEN {
            vel_idx = 0;
        }
        // led_pin.set_high().unwrap();
        top_buf[vel_idx] = ((float_t - prev_t)/delta_t) * 100_000.0;
        end_buf[vel_idx] = ((float_e - prev_e)/delta_t) * 100_000.0;
        // VT.store({
        //     let mut acc = 0.0;
        //     for i in 0..VEL_BUF_LEN {
        //         acc += top_buf[i];
        //     }
        //     ((acc/VEL_BUF_LEN as f32)) as i32
        // }, Ordering::Relaxed);
        // VE.store({
        //     let mut acc = 0.0;
        //     for i in 0..VEL_BUF_LEN {
        //         acc += end_buf[i];
        //     }
        //     ((acc/VEL_BUF_LEN as f32)) as i32
        // }, Ordering::Relaxed);
        VT.store(((top_buf.iter().sum::<f32>()/(VEL_BUF_LEN as f32))*100_000.0) as i32, Ordering::Relaxed);
        VE.store(((end_buf.iter().sum::<f32>()/(VEL_BUF_LEN as f32))*100_000.0) as i32, Ordering::Relaxed);
        prev_t = - (t as f32 + 19.0 * (- 1.0 + (t as f32 * RADS_PER_TICK).cos())) * RADS_PER_TICK;
        prev_e = e as f32 * RADS_PER_TICK;
        // timer.get_counter().checked_duration_since(tic).unwrap().to_micros() as u32;
        // counter += 1;
        // if counter >= 20 { counter = 0; }
        
        // acc = 0;
        // for i in 0..20 {
        //     acc += time_buf[i];
        // }
        // AVG_LOOP_TIME.store(acc, Ordering::Relaxed);
    }

}

#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    // use core::sync::atomic::{AtomicBool, Ordering};
    
    // Grab the global objects. This is OK as we only access them under interrupt.
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let serial = USB_SERIAL.as_mut().unwrap();
    
    // Poll the USB driver with all of our supported USB Classes
    if usb_dev.poll(&mut [serial]) {
        let mode = MODE.load(Ordering::Relaxed);
        let mut buf = [0, 0, 0, 0, 0]; //format: 1 byte command, up to 4 bytes data. Default command is set power to 0.
        if IN_RESET.load(Ordering::Relaxed) && mode != 2 {
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
                Ok(size) => {
                    match buf[0] { //32767.5
                        0 => {
                            if mode == 2 {
                                let _ = serial.write(b"Error: SET_ACC prohibited during homing.");
                            } else if size < 5 {
                                let _ = serial.write(b"Error: SET_ACC requires four bytes of data.");
                            } else {
                                CART_ACC.store(i32::from_be_bytes([buf[1], buf[2], buf[3], buf[4]]), Ordering::Relaxed);
                            }
                        },
                        1 => {
                            if mode == 2 {
                                let _ = serial.write(b"Error: SET_MODE prohibited during homing.");
                            } else {
                                MODE.store(buf[1], Ordering::Relaxed);
                            }
                        }
                        2..=7 => {
                            K[(buf[0] as usize)-2].store(i32::from_be_bytes([buf[1], buf[2], buf[3], buf[4]]), Ordering::Relaxed);
                        }
                        10 => {
                            // *GET_STATUS_FLAG.as_mut().unwrap() = true;
                        },
                        11..=13 => {
                            SP[(buf[0] as usize-11)].store(i32::from_be_bytes([buf[1], buf[2], buf[3], buf[4]]), Ordering::Relaxed);
                        },
                        28 => {
                            MODE.store(2, Ordering::Relaxed);
                        },
                        92 => {
                            if mode == 2 {
                                let _ = serial.write(b"Error: SET_X prohibited during homing.");
                            } else if size == 1 {
                                CART_POS.store(0, Ordering::Relaxed);
                            } else if size == 4 {
                                CART_POS.store(i32::from_be_bytes([buf[1], buf[2], buf[3], buf[4]]), Ordering::Relaxed);
                            } else if size == 12 {

                            }
                        }
                        101 => { //e-stop, basically
                            __cortex_m_rt_IO_IRQ_BANK0();
                        },
                        114 => {
                            let mut message1: String<100> = String::new();
                            let _ = write!(&mut message1, "SP: [{:.6},{:.6},{:.6}], K: [{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}]\n", 
                                f32::from_be_bytes(SP[0].load(Ordering::Relaxed).to_be_bytes()),
                                f32::from_be_bytes(SP[1].load(Ordering::Relaxed).to_be_bytes()),
                                f32::from_be_bytes(SP[2].load(Ordering::Relaxed).to_be_bytes()),
                                f32::from_be_bytes(K[0].load(Ordering::Relaxed).to_be_bytes()),
                                f32::from_be_bytes(K[1].load(Ordering::Relaxed).to_be_bytes()),
                                f32::from_be_bytes(K[2].load(Ordering::Relaxed).to_be_bytes()),
                                f32::from_be_bytes(K[3].load(Ordering::Relaxed).to_be_bytes()),
                                f32::from_be_bytes(K[4].load(Ordering::Relaxed).to_be_bytes()),
                                f32::from_be_bytes(K[5].load(Ordering::Relaxed).to_be_bytes()),
                            );
                            let _ = serial.write(message1.as_bytes());

                        },
                        
                        _ => {},
                    }
                    //def fix_ang(x): return x+76 + 0.5*(cos(2*pi*(x+76)/4096)+1)*27
                    // fn angle_cope(top: i32, end: i32) -> (i32, i32){
                    //     let angle = (top + TOP_OFFSET.load(Ordering::Relaxed));
                    //     let final_angle = (angle as f32 + 13.5*(1.0+(PI*(angle as f32)/2048.0).cos())) * RADS_PER_TICK;
                    //     let angle2 = (end - 2206);
                    //     let final_angle2 = (angle2 as f32 + 3.5*(1.0+(PI*(angle2 as f32)/2048.0).cos())) * RADS_PER_TICK;
                    // }
                    let angle = TOP.load(Ordering::Relaxed) as f32;
                    let final_angle = - (angle + 19.0*(-1.0+(angle*RADS_PER_TICK).cos())) * RADS_PER_TICK;
                    let final_angle2 = END.load(Ordering::Relaxed) as f32 * RADS_PER_TICK;
                    // 2206 0
                    // 4247 pi
                    let mut message: String<100> = String::new();
                    let _ = write!(&mut message, "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}\n", 
                        CART_POS.load(Ordering::Relaxed) as f32 / ((10000*SPEED_MULT) as f32),
                        final_angle, 
                        final_angle2,
                        CART_VEL.load(Ordering::Relaxed) as f32 / ((10000*SPEED_MULT) as f32),
                        VT.load(Ordering::Relaxed) as f32 / 100_000.0, 
                        VE.load(Ordering::Relaxed) as f32 / 100_000.0,
                        CART_ACC.load(Ordering::Relaxed) as f32 / ((10000*SPEED_MULT) as f32),
                        // AVG_LOOP_TIME.load(Ordering::Relaxed),
                    );
                    let _ = serial.write(message.as_bytes());
                },
                Err(_e) => {} // do nothing, idk what to do
            }
        }
    }
}

#[interrupt]
unsafe fn IO_IRQ_BANK0() {
    if MODE.load(Ordering::Relaxed) != 2 { //if we're not homing
        CART_ACC.store(0, Ordering::Relaxed);
        CART_VEL.store(0, Ordering::Relaxed);
    }
    IN_RESET.store(true, Ordering::Relaxed);

    // USB_SERIAL.as_mut().unwrap().write(b"Limit Switch Triggered! Waiting for reset command.\n").unwrap();
    match RESET_PIN.as_mut() {
        Some(reset_pin) => {
            reset_pin.clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
        },
        None => {}
    }
}

#[interrupt]
unsafe fn TIMER_IRQ_0() {
    let alarm0 = ALARM.as_mut().unwrap();
    let _ = alarm0.schedule(STEPPER_VELOCITY_UPDATE_TIME.micros());

    let tx = DELAY_TX.as_mut().unwrap(); 
    let sm = STEP_SM.as_mut().unwrap();
    let mut cart_vel = CART_VEL.load(Ordering::Relaxed);
    let mut acc = CART_ACC.load(Ordering::Relaxed);
    CART_POS.store(CART_POS.load(Ordering::Relaxed) + (cart_vel/(1_000_000/STEPPER_VELOCITY_UPDATE_TIME as i32)), Ordering::Relaxed);

    // impose limits on acceleration (velocity is handled by .get_period())
    if acc > MAX_ACCELERATION*10*SPEED_MULT { acc = MAX_ACCELERATION*10*SPEED_MULT; }
    else if acc < -MAX_ACCELERATION*10*SPEED_MULT { acc = -MAX_ACCELERATION*10*SPEED_MULT; }
    if cart_vel > MAX_SPEED*10*SPEED_MULT { cart_vel = MAX_SPEED*10*SPEED_MULT; }
    else if cart_vel < -MAX_SPEED*10*SPEED_MULT { cart_vel = -MAX_SPEED*10*SPEED_MULT}

    CART_VEL.store(cart_vel + acc/100, Ordering::Relaxed); // the /100 bc time step is 10 ms

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

// #[panic_handler]
// unsafe fn panic(_info: &PanicInfo) -> ! {
//     let serial = USB_SERIAL.as_mut().unwrap();
//     let mut message: String<200> = String::new();
//     let _ = write!(&mut message, "{}", _info);
//     IN_RESET.store(true, Ordering::Relaxed);
//     CART_ACC.store(0, Ordering::Relaxed);
//     CART_VEL.store(0, Ordering::Relaxed);
//     loop {
//         let _ = serial.write(message.as_bytes());
//     }
// }