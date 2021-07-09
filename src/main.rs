#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::rate::Extensions;
use hal::{gpio::FunctionI2C, i2c::I2C, pac, sio::Sio};
use panic_halt as _;
use rp2040_hal as hal;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER;

// https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf
// https://github.com/bitbank2/Pi_Pico_C_Projects/blob/master/ss_oled/ss_oled.c

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

    let sda_pin = pins.gpio18.into_mode::<FunctionI2C>();
    let scl_pin = pins.gpio19.into_mode::<FunctionI2C>();

    let mut i2c = I2C::i2c1(pac.I2C1, sda_pin, scl_pin, 400.kHz(), &mut pac.RESETS);

    loop {
        led_pin.set_high().unwrap();
        cortex_m::asm::delay(500_000);
        led_pin.set_low().unwrap();
        cortex_m::asm::delay(500_000);
    }
}
