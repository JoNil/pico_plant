#![no_std]
#![no_main]

use core::str;
use cortex_m_rt::entry;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::{Point, Size},
    primitives::Rectangle,
    text::{Alignment, Text},
    Drawable,
};
use embedded_hal::{adc::OneShot, digital::v2::OutputPin};
use embedded_text::{
    alignment::HorizontalAlignment,
    style::{HeightMode, TextBoxStyleBuilder},
    TextBox,
};
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
use panic_persist::{self as _, get_panic_message_bytes};
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

    let mut display = Ssd1306::new(
        I2CDisplayInterface::new(i2c),
        DisplaySize128x32,
        DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();

    if let Some(msg) = get_panic_message_bytes() {
        let character_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
        let textbox_style = TextBoxStyleBuilder::new()
            .height_mode(HeightMode::FitToText)
            .alignment(HorizontalAlignment::Left)
            .build();

        let bounds = Rectangle::new(Point::zero(), Size::new(128, 32));

        display.clear();
        TextBox::with_textbox_style(
            unsafe { str::from_utf8_unchecked(msg) },
            bounds,
            character_style,
            textbox_style,
        )
        .draw(&mut display)
        .unwrap();
        display.flush().unwrap();

        #[allow(clippy::empty_loop)]
        loop {}
    }

    let mut led_pin = pins.gpio25.into_push_pull_output();

    let mut temp_sens = Adc::new(pac.ADC, &mut pac.RESETS);
    let mut temp_sens_channel = temp_sens.enable_temp_sensor();

    display.init().unwrap();
    display.clear();
    display.flush().unwrap();

    let character_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    let mut data: [f32; 128] = [0.0; 128];

    let mut measure_cycle = 0;

    loop {
        led_pin.set_low().unwrap();

        display.clear();

        let temp: u16 = temp_sens.read(&mut temp_sens_channel).unwrap();
        let temp = temp as f32 * 3.3 / 4096.0;
        let temp = 27.0 - (temp - 0.706) / 0.001721;

        measure_cycle += 1;

        if measure_cycle > 100 {
            data.rotate_left(1);
            *data.last_mut().unwrap() = temp;
            measure_cycle = 0;
        }

        for (x, t) in data.iter().enumerate() {
            let y = (32.0 * t / 30.0) as u32;
            display.set_pixel(x as u32, 32 - y, true);
        }

        let mut text = String::<16>::new();
        uwrite!(&mut text, "T: {}", uFmt_f32::One(temp as f32)).unwrap();

        Text::with_alignment(&text, Point::new(0, 30), character_style, Alignment::Left)
            .draw(&mut display)
            .unwrap();

        led_pin.set_high().unwrap();

        display.flush().unwrap();
    }
}
