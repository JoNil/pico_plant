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
    clocks::{init_clocks_and_plls, ClockSource},
    gpio::{FunctionI2C, Pins},
    i2c::I2C,
    pac::{self, interrupt},
    sio::Sio,
    usb::UsbBus,
    watchdog::Watchdog,
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
use usb_device::{
    class_prelude::UsbBusAllocator,
    device::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
};
use usbd_serial::SerialPort;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER;

const XOSC_HZ: u32 = 12_000_000_u32;

static mut USB_BUS: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_DEVICE: Option<UsbDevice<UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<UsbBus>> = None;

fn lerp(val1: f32, val2: f32, amount: f32) -> f32 {
    val1 + (val2 - val1) * amount
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = init_clocks_and_plls(
        XOSC_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
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
        clocks.system_clock.get_freq(),
    );

    let mut display = Ssd1306::new(
        I2CDisplayInterface::new(i2c),
        DisplaySize128x32,
        DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();

    let display_ok = if display.init().is_ok() {
        display.clear();
        display.flush().unwrap();
        true
    } else {
        false
    };

    let character_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    unsafe {
        USB_BUS = Some(UsbBusAllocator::new(UsbBus::new(
            pac.USBCTRL_REGS,
            pac.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut pac.RESETS,
        )));

        USB_SERIAL = Some(SerialPort::new(USB_BUS.as_ref().unwrap()));

        USB_DEVICE = Some(
            UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27dd))
                .manufacturer("JoNil")
                .product("Pico Plant")
                .serial_number("1")
                .max_packet_size_0(64)
                .device_class(2) // from: https://www.usb.org/defined-class-codes
                .build(),
        );

        USB_DEVICE.as_mut().unwrap().force_reset().ok();

        let p = pac::Peripherals::steal();
        // Enable interrupts for when a buffer is done, when the bus is reset,
        // and when a setup packet is received
        p.USBCTRL_REGS.inte.modify(|_, w| {
            w.buff_status()
                .set_bit()
                .bus_reset()
                .set_bit()
                .setup_req()
                .set_bit()
                .trans_complete()
                .set_bit()
        });

        // Enable the USB interrupt
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    }

    if let Some(msg) = get_panic_message_bytes() {
        let textbox_style = TextBoxStyleBuilder::new()
            .height_mode(HeightMode::FitToText)
            .alignment(HorizontalAlignment::Left)
            .build();

        let bounds = Rectangle::new(Point::zero(), Size::new(128, 32));

        let message = unsafe { str::from_utf8_unchecked(msg) };

        display.clear();
        TextBox::with_textbox_style(message, bounds, character_style, textbox_style)
            .draw(&mut display)
            .unwrap();

        if display_ok {
            display.flush().unwrap();
        }

        usb_write(message);

        #[allow(clippy::empty_loop)]
        loop {}
    }

    let mut led_pin = pins.gpio25.into_push_pull_output();

    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
    let mut temp_sens_channel = adc.enable_temp_sensor();

    let mut water_sensor_pin = pins.gpio27.into_floating_input();
    let mut input_voltage_sensor_pin = pins.gpio26.into_floating_input();

    let mut data: [f32; 128] = [0.0; 128];

    let mut measure_cycle = 0;

    let mut average_voltage = 0.0;
    let mut average_water = 0.0;

    loop {
        led_pin.set_low().unwrap();

        display.clear();

        let temp: u16 = adc.read(&mut temp_sens_channel).unwrap();
        let temp = temp as f32 * 3.3 / 4096.0;
        let temp = 27.0 - (temp - 0.706) / 0.001721;

        let voltage: u16 = adc.read(&mut input_voltage_sensor_pin).unwrap();
        let voltage = voltage as f32 * 5.0 / 4096.0;

        average_voltage = lerp(average_voltage, voltage, 0.05);

        let water: u16 = adc.read(&mut water_sensor_pin).unwrap();
        let water = f32::max(1.5 - ((5.0 - voltage) + water as f32 * 3.3 / 4096.0), 0.0);

        average_water = lerp(average_water, water, 0.01);

        measure_cycle += 1;

        if measure_cycle > 10 {
            data.rotate_left(1);
            *data.last_mut().unwrap() = average_water;
            measure_cycle = 0;

            {
                let mut text = String::<64>::new();
                uwrite!(&mut text, "T: {}\r\n", uFmt_f32::Three(temp)).unwrap();
                usb_write(&text);
            }
        }

        for (x, t) in data.iter().enumerate() {
            let y = (32.0 * t / 0.5) as u32;
            display.set_pixel(x as u32, 32 - y, true);
        }

        {
            let mut text = String::<48>::new();
            uwrite!(&mut text, "T: {}", uFmt_f32::One(temp)).unwrap();

            Text::with_alignment(&text, Point::new(0, 10), character_style, Alignment::Left)
                .draw(&mut display)
                .unwrap();
        }

        {
            let mut text = String::<48>::new();
            uwrite!(
                &mut text,
                "W: {}, V: {}",
                uFmt_f32::Three(average_water),
                uFmt_f32::Three(average_voltage)
            )
            .unwrap();

            Text::with_alignment(&text, Point::new(0, 30), character_style, Alignment::Left)
                .draw(&mut display)
                .unwrap();
        }

        led_pin.set_high().unwrap();

        if display_ok {
            display.flush().unwrap();
        }
    }
}

fn usb_write(str: &str) {
    cortex_m::interrupt::free(|_| unsafe {
        let serial = USB_SERIAL.as_mut().unwrap();
        serial.write(str.as_bytes()).ok();
    });
}

#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let serial = USB_SERIAL.as_mut().unwrap();

    usb_dev.poll(&mut [serial]);

    // Clear pending interrupt flags here.
    let p = pac::Peripherals::steal();
    let status = &p.USBCTRL_REGS.sie_status;
    if status.read().ack_rec().bit_is_set() {
        status.modify(|_r, w| w.ack_rec().set_bit());
    }
    if status.read().setup_rec().bit_is_set() {
        status.modify(|_r, w| w.setup_rec().set_bit());
    }
    if status.read().trans_complete().bit_is_set() {
        status.modify(|_r, w| w.trans_complete().set_bit());
    }
    if status.read().bus_reset().bit_is_set() {
        status.modify(|_r, w| w.bus_reset().set_bit());
    }
}
