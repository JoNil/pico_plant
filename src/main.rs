#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::rate::Extensions;
use hal::{i2c::I2C, pac, sio::Sio};
use panic_halt as _;
use rp2040_hal as hal;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();

    let sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut led_pin = pins.gpio25.into_push_pull_output();

    let mut i2c = I2C::i2c1(
        pac.I2C1,
        pins.gpio18,
        pins.gpio19,
        400.kHz(),
        &mut pac.RESETS,
    );

    loop {
        led_pin.set_high().unwrap();
        cortex_m::asm::delay(500_000);
        led_pin.set_low().unwrap();
        cortex_m::asm::delay(500_000);
    }
}
