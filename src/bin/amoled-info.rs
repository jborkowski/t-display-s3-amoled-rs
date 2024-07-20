#![no_std]
#![no_main]

extern crate alloc;

use alloc::string::String;
use embedded_hal_1::delay::DelayNs;
use esp_hal::analog::adc::{Adc, AdcConfig, Attenuation};
use esp_hal::delay::Delay;
use esp_hal::dma_buffers;
use esp_hal::rtc_cntl::Rtc;
use esp_hal::spi::master::Spi;
use esp_hal::system::SystemControl;
use esp_hal::timer::timg::TimerGroup;
use core::fmt::Write;
use core::mem::MaybeUninit;
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::mono_font::{MonoTextStyle, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::text::{Alignment, Text};
use esp_backtrace as _;
use esp_println::println;
use esp_hal::dma::{Dma, DmaPriority};
use esp_hal::gpio::{Io, Level, Output, NO_PIN};
use esp_hal::prelude::_fugit_RateExtU32;
use esp_hal::{
    clock::ClockControl, peripherals::Peripherals, prelude::*, 
};
use t_display_s3_amoled::rm67162::Orientation;
use esp_hal::spi::master::prelude::*;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {    
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

#[esp_hal::entry]
fn main() -> ! {
    init_heap();
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();


    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.LPWR, None);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        None
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        None
    );
    let mut wdt1 = timer_group1.wdt;
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();
    println!("Hello world!");

    // Set GPIO4 as an output, and set its state high initially.
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut led = Output::new(io.pins.gpio38, Level::High);
    //let user_btn = io.pins.gpio21.into_pull_down_input();
    //let boot0_btn = io.pins.gpio0.into_pull_up_input(); // default pull up

    led.set_high();

    println!("GPIO init OK");

    println!("init display");

    let sclk = io.pins.gpio47;
    let rst = io.pins.gpio17;
    let cs = io.pins.gpio6;

    let d0 = io.pins.gpio18;
    let d1 = io.pins.gpio7;
    let d2 = io.pins.gpio48;
    let d3 = io.pins.gpio5;

    let cs = Output::new(cs, Level::High);
    let mut rst = Output::new(rst, Level::Low);

    let mut delay = Delay::new(&clocks);

    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    // Descriptors should be sized as (BUFFERSIZE / 4092) * 3
    let (_tx_buffer, tx_descriptors, _rx_buffer, rx_descriptors) = dma_buffers!(32000);
    let spi = Spi::new_half_duplex(
        peripherals.SPI2, // use spi2 host
        75_u32.MHz(), // max 75MHz
        esp_hal::spi::SpiMode::Mode0,
        &clocks)
        .with_pins(Some(sclk),Some(d0),Some(d1),Some(d2),Some(d3),NO_PIN)
        .with_dma(dma_channel.configure(false,  DmaPriority::Priority0), tx_descriptors, rx_descriptors);

    let mut display = t_display_s3_amoled::rm67162::dma::RM67162Dma::new(spi, cs);
    display.reset(&mut rst, &mut delay).unwrap();
    display.init(&mut delay).unwrap();
    display
        .set_orientation(Orientation::LandscapeFlipped)
        .unwrap();

    display.clear(Rgb565::BLACK).unwrap();
    println!("screen init ok");

    // Create ADC instances
    let mut adc_config = AdcConfig::new();
    let mut vbat_pin =
        adc_config.enable_pin(io.pins.gpio12, Attenuation::Attenuation11dB);
    let mut adc1 = Adc::new(peripherals.ADC1, adc_config);

    println!("ADC init OK");

    let character_style = MonoTextStyle::new(&FONT_10X20, Rgb565::RED);
    Text::with_alignment(
        "Hello,\nRust World!",
        Point::new(300, 40),
        character_style,
        Alignment::Center,
    )
    .draw(&mut display)
    .unwrap();

    loop {
        // fps testing
        let mut s = String::new();

        let raw_val: u16 = nb::block!(adc1.read_oneshot(&mut vbat_pin)).unwrap();
        let vbat = (raw_val as f32 / 4095.0) * 3.3 * (100.0 + 100.0) / 100.0;
        core::write!(&mut s, "Vbat: {:.2}V\nRaw: {}", vbat, raw_val).unwrap();

        /*
            let elapsed = now_ms() - started;
            core::write!(
                &mut s,
                "Frames: {}\nFPS: {:.1}",
                cnt,
                if elapsed > 0 {
                    cnt as f32 / (elapsed as f32 / 1000.0)
                } else {
                    0.0
                }
            )
            .unwrap();
        */
        Text::with_alignment(
            &s,
            Point::new(100, 40),
            MonoTextStyleBuilder::new()
                .background_color(Rgb565::BLACK)
                .text_color(Rgb565::CSS_BISQUE)
                .font(&FONT_10X20)
                .build(),
            Alignment::Center,
        )
        .draw(&mut display)
        .unwrap();
        delay.delay_ms(1000u32);
    }
}


