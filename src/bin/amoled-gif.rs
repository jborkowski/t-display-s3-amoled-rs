#![no_std]
#![no_main]

extern crate alloc;

use core::mem::MaybeUninit;
use embedded_graphics::framebuffer::Framebuffer;
use esp_hal::delay::Delay;
use esp_hal::rtc_cntl::Rtc;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{dma_buffers, spi::master::Spi};
use esp_hal::system::SystemControl;

use embedded_graphics::pixelcolor::raw::BigEndian;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;

use esp_backtrace as _;
use esp_hal::dma::{Dma, DmaPriority};
use esp_hal::gpio::{Io, Level, Output, NO_PIN};
use esp_hal::prelude::_fugit_RateExtU32;
use esp_hal::spi::master::prelude::*;
use esp_hal::{clock::ClockControl, peripherals::Peripherals};
use esp_println::println;
use t_display_s3_amoled::rm67162::Orientation;

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
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks, None);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks, None);
    let mut wdt1 = timer_group1.wdt;
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();
    println!("Hello board!");

    // Set GPIO4 as an output, and set its state high initially.
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut led = Output::new(io.pins.gpio38, Level::Low);
    let _button = io.pins.gpio21;

    led.set_high();

    // Initialize the Delay peripheral, and use it to toggle the LED state in a
    // loop.
    let mut delay = Delay::new(&clocks);

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

    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0.configure(false, DmaPriority::Priority0);

    // Descriptors should be sized as (BUFFERSIZE / 4092) * 3
    let (_tx_buffer, tx_descriptors, _rx_buffer, rx_descriptors) = dma_buffers!(16368);
    let spi = Spi::new_half_duplex(
        peripherals.SPI2, // use spi2 host
        75_u32.MHz(),     // max 75MHz
        esp_hal::spi::SpiMode::Mode0,
        &clocks,
    )
    .with_pins(Some(sclk), Some(d0), Some(d1), Some(d2), Some(d3), NO_PIN)
    .with_dma(dma_channel, tx_descriptors, rx_descriptors);

    let mut display = t_display_s3_amoled::rm67162::dma::RM67162Dma::new(spi, cs);
    display.reset(&mut rst, &mut delay).unwrap();
    display.init(&mut delay).unwrap();
    display
        .set_orientation(Orientation::LandscapeFlipped)
        .unwrap();

    display.clear(Rgb565::WHITE).unwrap();
    println!("display init ok");

    let gif = tinygif::Gif::from_slice(include_bytes!("../../ferris.gif")).unwrap();

    let mut fb = Framebuffer::<
        Rgb565,
        _,
        BigEndian,
        536,
        240,
        { embedded_graphics::framebuffer::buffer_size::<Rgb565>(536, 240) },
    >::new();
    fb.clear(Rgb565::WHITE).unwrap();

    loop {
        for frame in gif.frames() {
            frame.draw(&mut fb.translated(Point::new(0, 30))).unwrap();
            // println!("draw frame {:?}", frame);
            unsafe {
                display.fill_with_framebuffer(fb.data()).unwrap();
            }
            led.toggle();
        }
    }
}
