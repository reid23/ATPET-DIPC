//! # UART IRQ Echo Example
//!
//! This application demonstrates how to use the UART Driver to talk to a serial
//! connection. In this example, the IRQ owns the UART and you cannot do any UART
//! access from the main thread.
//!
//! The pinouts are:
//!
//! * GPIO 0 - UART TX (out of the RP2040)
//! * GPIO 1 - UART RX (in to the RP2040)
//! * GPIO 25 - An LED we can blink (active high)
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// These are the traits we need from Embedded HAL to treat our hardware
// objects as generic embedded devices.
use embedded_hal::{
    digital::v2::OutputPin,
    serial::{Read, Write},
};

// We also need this for the 'Delay' object to work.
use rp2040_hal::Clock;

// The macro for our start-up function
use rp_pico::entry;

// Time handling traits
use fugit::RateExtU32;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
// use rp_pico::hal;
use rp2040_hal as hal;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::{pac, uart::Parity};

// Our interrupt macro
use hal::pac::interrupt;

// Some short-cuts to useful types
use core::cell::RefCell;
use critical_section::Mutex;

/// Import the GPIO pins we use
use hal::gpio::bank0::{Gpio12, Gpio13};

// UART related types
use hal::uart::{DataBits, StopBits, UartConfig};


use usb_device::{class_prelude::*, prelude::*}; // USB device emulation
use usbd_serial::SerialPort; // more USB stuff

use hal::spi;
use hal::gpio;
// silly for tmc2208
// referenced from https://docs.rs/tmc2209/latest/src/tmc2209/lib.rs.html#415-428
fn crc(data: &[u8]) -> u8 {
    let mut crc = 0u8;
    for mut byte in data.iter().cloned() {
        for _ in 0..8 {
            if ((crc >> 7) ^ (byte & 0x01)) != 0 {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = crc << 1;
            }
            byte = byte >> 1;
        }
    }
    crc
}
// https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2202_TMC2208_TMC2224_datasheet_rev1.14.pdf
// see page 18 ^
fn getwritemsg(register: u8, data: &[u8; 4]) -> [u8; 8] {
    let nocrc = [0b00000101u8, 0u8, register|0b10000000u8, data[0], data[1], data[2], data[3]];
    let msg = [nocrc[0], nocrc[1], nocrc[2], nocrc[3], nocrc[4], nocrc[5], nocrc[6], crc(&nocrc)];
    msg
}
fn getreadmsg(register: u8) -> [u8; 4] {
    let nocrc = [0b00000101u8, 0u8, register];
    let msg = [nocrc[0], nocrc[1], nocrc[2], crc(&nocrc)];
    msg
}

/// Alias the type for our UART pins to make things clearer.
type UartPins = (
    hal::gpio::Pin<Gpio12, hal::gpio::FunctionUart, hal::gpio::PullNone>,
    hal::gpio::Pin<Gpio13, hal::gpio::FunctionUart, hal::gpio::PullNone>,
);

/// Alias the type for our UART to make things clearer.
type Uart = hal::uart::UartPeripheral<hal::uart::Enabled, pac::UART0, UartPins>;


static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

// /// This how we transfer the UART into the Interrupt Handler
// static GLOBAL_UART: Mutex<RefCell<Option<Uart>>> = Mutex::new(RefCell::new(None));

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then writes to the UART in
/// an infinite loop.
///
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
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

    // Lets us wait for fixed periods of time
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    
    let mut led_pin = pins.led.into_push_pull_output();
    
    // let sclk: gpio::Pin<_, gpio::FunctionSpi, gpio::PullNone> = pins.gpio2.reconfigure();
    // let mosi: gpio::Pin<_, gpio::FunctionSpi, gpio::PullNone> = pins.gpio3.reconfigure();
    // let miso: gpio::Pin<_, gpio::FunctionSpi, gpio::PullUp> = pins.gpio4.reconfigure();
    // let cs = pins.gpio5.into_push_pull_output();
    
    // // Create the SPI driver instance for the SPI0 device
    // let spi = spi::Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, miso, sclk));

    // // Exchange the uninitialised SPI driver for an initialised one
    // let spi = spi.init(
    //     &mut pac.RESETS,
    //     clocks.peripheral_clock.freq(),
    //     2.MHz(), // initialization happens at low baud rate
    //     embedded_hal::spi::MODE_0,
    // );
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    // Note (safety): This is safe as interrupts haven't been started yet
    unsafe { USB_BUS = Some(usb_bus); }
    
    // Grab a reference to the USB Bus allocator. We are promising to the
    // compiler not to take mutable access to this global variable whilst this
    // reference exists!
    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };
    // Set up the USB Communications Class Device driver thing (this is the thing we can actually write to)
    let mut serial = SerialPort::new(bus_ref);
    
    // make this emulate a usb device
    let mut usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Reid Dye")
        .product("ATPET Inverted Pendulum")
        .serial_number("0001")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();
    

    // unsafe { 
    //     USB_SERIAL = Some(serial); 
    //     USB_DEV = Some(usb_dev); 
    // }
    // unsafe { pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ); }

    let uart_pins = (
        // UART TX (characters sent from RP2040) on pin 11 (GPIO8)
        pins.gpio12.reconfigure(),
        // UART RX (characters received by RP2040) on pin 12 (GPIO9)
        pins.gpio13.reconfigure(),
    );

    // Make a UART on the given pins
    let mut uart: Uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(250.kHz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();


    // Write something to the UART on start-up so we can check the output pin
    // is wired correctly.
    // uart.write_full_blocking(b"uart_interrupt example started...\n");

    // Now we give away the entire UART peripheral, via the variable
    // `GLOBAL_UART`. We can no longer access the UART from this main thread.
    // critical_section::with(|cs| {
    //     GLOBAL_UART.borrow(cs).replace(Some(uart));
    // });

    // But we can blink an LED.

    //*setup gconf registers */
    uart.write_full_blocking(&getreadmsg(0u8));
    let mut buf = [0u8; 8];
    let mut buf2 = [0u8; 4];
    uart.read_full_blocking(&mut buf2);
    led_pin.set_high().unwrap();
    delay.delay_ms(500);
    led_pin.set_low().unwrap();
    // turn on spreadcycle, step pulses on INDEX, PDN_UART input disabled, usteps from MSTEP
    let newconf = [buf[3]|0b00100111, buf[4], buf[5], buf[6]];
    uart.write_full_blocking(&getwritemsg(0u8, &newconf));

    //*setup microsteps */
    uart.write_full_blocking(&getreadmsg(0x6Cu8)); //CHOPCONF
    uart.read_full_blocking(&mut buf);
    let newconf = [buf[3], buf[4], buf[5], buf[6] & 0b11110000];
    uart.write_full_blocking(&getwritemsg(0x6Cu8, &newconf));

    led_pin.set_high().unwrap();
    delay.delay_ms(500);
    led_pin.set_low().unwrap();
    loop {
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 8];
            serial.read(&mut buf);
            serial.write(&buf2);
            led_pin.set_low().unwrap();
            delay.delay_ms(50);
            // uart.write_full_blocking(&getwritemsg(0x22u8, &[buf[0], buf[1], buf[2], buf[3]]))
        }
        // The normal *Wait For Interrupts* (WFI) has a race-hazard - the
        // interrupt could occur between the CPU checking for interrupts and
        // the CPU going to sleep. We wait for events (and interrupts), and
        // then we set an event in every interrupt handler. This ensures we
        // always wake up correctly.
        // Light the LED to indicate we saw an interrupt.
        led_pin.set_high().unwrap();
        delay.delay_ms(10);
        // led_pin.set_low().unwrap();
    }
    
}

// #[interrupt]
// fn UART0_IRQ() {
//     // This variable is special. It gets mangled by the `#[interrupt]` macro
//     // into something that we can access without the `unsafe` keyword. It can
//     // do this because this function cannot be called re-entrantly. We know
//     // this because the function's 'real' name is unknown, and hence it cannot
//     // be called from the main thread. We also know that the NVIC will not
//     // re-entrantly call an interrupt.
//     static mut UART: Option<hal::uart::UartPeripheral<hal::uart::Enabled, pac::UART0, UartPins>> =
//         None;

//     // This is one-time lazy initialisation. We steal the variable given to us
//     // via `GLOBAL_UART`.
//     if UART.is_none() {
//         critical_section::with(|cs| {
//             *UART = GLOBAL_UART.borrow(cs).take();
//         });
//     }

//     // Check if we have a UART to work with
//     if let Some(uart) = UART {
//         // Echo the input back to the output until the FIFO is empty. Reading
//         // from the UART should also clear the UART interrupt flag.
//         while let Ok(byte) = uart.read() {
//             let _ = uart.write(byte);
//         }
//     }

//     // Set an event to ensure the main thread always wakes up, even if it's in
//     // the process of going to sleep.
//     cortex_m::asm::sev();
// }

// End of file