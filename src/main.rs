#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_graphics::{
    image::{Image, ImageRaw},
    pixelcolor::BinaryColor,
    prelude::Point,
    Drawable,
};
use embedded_hal::digital::v2::OutputPin;
use embedded_time::rate::Extensions;
use hal::{gpio::FunctionI2C, i2c::I2C, pac, sio::Sio};
use panic_halt as _;
use rp2040_hal as hal;
use ssd1306::{
    mode::DisplayConfig, rotation::DisplayRotation, size::DisplaySize128x32, I2CDisplayInterface,
    Ssd1306,
};

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

    let sda_pin = pins.gpio14.into_mode::<FunctionI2C>();
    let scl_pin = pins.gpio19.into_mode::<FunctionI2C>();

    let i2c = I2C::i2c1(pac.I2C1, sda_pin, scl_pin, 400.kHz(), &mut pac.RESETS);

    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    display.init().unwrap();

    led_pin.set_high().unwrap();

    let raw: ImageRaw<BinaryColor> = ImageRaw::new(include_bytes!("./rust.raw"), 64);

    let im = Image::new(&raw, Point::new(32, 0));

    im.draw(&mut display).unwrap();

    display.flush().unwrap();

    loop {
        led_pin.set_high().unwrap();
        cortex_m::asm::delay(500_000);
        led_pin.set_low().unwrap();
        cortex_m::asm::delay(500_000);
    }
}
