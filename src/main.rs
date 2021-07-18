#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::{Point, Primitive},
    primitives::{Line, PrimitiveStyleBuilder},
    text::{Alignment, Text},
    Drawable,
};
use embedded_hal::{adc::OneShot, digital::v2::OutputPin};
use embedded_time::rate::Extensions;
use hal::{
    adc::Adc,
    clocks::ClocksManager,
    gpio::FunctionI2C,
    i2c::I2C,
    pac,
    pll::{
        common_configs::{PLL_SYS_125MHZ, PLL_USB_48MHZ},
        setup_pll_blocking,
    },
    sio::Sio,
    watchdog::Watchdog,
    xosc::setup_xosc_blocking,
};
use heapless::String;
use panic_halt as _;
use rp2040_hal as hal;
use ssd1306::{
    mode::DisplayConfig, rotation::DisplayRotation, size::DisplaySize128x32, I2CDisplayInterface,
    Ssd1306,
};
use ufmt::uwrite;
use ufmt_float::uFmt_f32;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER;

const XOSC_HZ: u32 = 12_000_000_u32;
const SYS_HZ: u32 = 125_000_000_u32;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let mut clocks = ClocksManager::new(pac.CLOCKS, &mut watchdog);

    let xosc = setup_xosc_blocking(pac.XOSC, XOSC_HZ.Hz()).ok().unwrap();

    let pll_sys = setup_pll_blocking(
        pac.PLL_SYS,
        XOSC_HZ.Hz().into(),
        PLL_SYS_125MHZ,
        &mut clocks,
        &mut pac.RESETS,
    )
    .ok()
    .unwrap();
    let pll_usb = setup_pll_blocking(
        pac.PLL_USB,
        XOSC_HZ.Hz().into(),
        PLL_USB_48MHZ,
        &mut clocks,
        &mut pac.RESETS,
    )
    .ok()
    .unwrap();

    clocks.init_default(&xosc, &pll_sys, &pll_usb);

    let sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut led_pin = pins.gpio25.into_push_pull_output();

    let mut temp_sens = Adc::new(pac.ADC, &mut pac.RESETS);
    let mut temp_sens_channel = temp_sens.enable_temp_sensor();

    let sda_pin = pins.gpio14.into_mode::<FunctionI2C>();
    let scl_pin = pins.gpio19.into_mode::<FunctionI2C>();

    let i2c = I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        SYS_HZ.Hz(),
    );

    led_pin.set_high().unwrap();

    let mut display = Ssd1306::new(
        I2CDisplayInterface::new(i2c),
        DisplaySize128x32,
        DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();

    display.init().unwrap();
    display.clear();
    display.flush().unwrap();

    let style = PrimitiveStyleBuilder::new()
        .stroke_width(1)
        .stroke_color(BinaryColor::On)
        .fill_color(BinaryColor::On)
        .build();

    let character_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    let data = [
        Point::new(0, 16),
        Point::new(16, 0),
        Point::new(32, 10),
        Point::new(48, 12),
        Point::new(64, 32),
        Point::new(80, 16),
        Point::new(96, 10),
        Point::new(112, 30),
        Point::new(128, 0),
    ];

    loop {
        led_pin.set_low().unwrap();

        display.clear();

        for (p1, p2) in data.iter().zip(data.iter().skip(1)) {
            Line::new(*p1, *p2)
                .into_styled(style)
                .draw(&mut display)
                .unwrap();
        }

        let temp: u16 = temp_sens.read(&mut temp_sens_channel).unwrap();
        let temp = temp as f32 * 3.3 / 4096.0;
        let temp = 27.0 - (temp - 0.706) / 0.001721;

        let mut text = String::<16>::new();
        uwrite!(&mut text, "T: {}", uFmt_f32::One(temp as f32)).unwrap();

        Text::with_alignment(&text, Point::new(0, 30), character_style, Alignment::Left)
            .draw(&mut display)
            .unwrap();

        led_pin.set_high().unwrap();

        display.flush().unwrap();
    }
}
