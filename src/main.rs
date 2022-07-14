//! # Nano Blinky Example
//!
//! Blinks the LED on a Arduino Nano Connect board.
//!
//! This will blink an LED attached to GP25, which is the pin the Nano uses for
//! the on-board LED.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

use core::convert::Infallible;

// The macro for our start-up function
use cortex_m_rt::entry;

// use embedded_hal::digital::v2::OutputPin;
// // Time handling traits
use embedded_time::rate::*;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

use arduino_nano_connect as bsp;

// Pull in any important traits
use bsp::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use bsp::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use bsp::hal;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
use usbd_serial::SerialPort;

use arrayvec::ArrayString;
use ufmt::uWrite;

const LINE_SIZE: usize = 128;

struct WritableLine {
    s: ArrayString<LINE_SIZE>,
}

impl WritableLine {
    pub fn new() -> Self {
        Self {
            s: ArrayString::new(),
        }
    }
}

impl uWrite for WritableLine {
    type Error = Infallible;

    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        self.s.push_str(s);
        Ok(())
    }
}

macro_rules! uprintln {
    // IMPORTANT use `tt` fragments instead of `expr` fragments (i.e. `$($exprs:expr),*`)
    ($writer:expr, $($tt:tt)*) => {{
        let mut line = WritableLine::new();
        ufmt::uwriteln!(&mut line, $($tt)*).ok();
        $writer.write(line.s.as_str().as_bytes()).ok();
        $writer.write(b"\r").ok()
    }}
}

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then blinks the LED in an
/// infinite loop.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        bsp::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set the LED to be an output
    // let mut led_pin = pins.sck0.into_push_pull_output();

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut said_hello = true;
    let mut count = 1;
    loop {
        //led_pin.set_high().unwrap();
        //delay.delay_ms(500);
        //led_pin.set_low().unwrap();
        //delay.delay_ms(500);

        //delay.delay_ms(1000);

        // A welcome message at the beginning
        if !said_hello && timer.get_counter() >= 2_000_000 {
            said_hello = true;
            uprintln!(serial, "Hello, world!");
        }

        uprintln!(serial, "blink {}", count);
        count += 1;

        // Consume new data
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            serial.read(&mut buf).ok();
        }
    }
}

// #[link_section = ".boot_loader"]
// #[used]
// pub static BOOT_LOADER: [u8; 256] = rp2040_boot2::BOOT_LOADER_AT25SF128A;
