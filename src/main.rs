#![no_std]
#![no_main]

extern crate alloc;
use alloc::boxed::Box;
use alloc::rc::Rc;
use rs_esp32s3_no_std_st7789_demo::dma::DmaBackend;
use core::cell::RefCell;
use core::mem::MaybeUninit;
use display_interface_spi::SPIInterfaceNoCS;
use embedded_graphics_core::prelude::{DrawTarget, Point, RgbColor, Size};
use embedded_graphics_core::{pixelcolor::raw::RawU16, primitives::Rectangle};
use esp_backtrace as _;
use esp_println::println;
use hal::spi::master::{Spi, dma};
use hal::{
    clock::{ClockControl, CpuClock},
    peripherals::Peripherals,
    prelude::*,
    spi::SpiMode,
    systimer::SystemTimer,
    timer::TimerGroup,
    Delay, Rtc, IO,
};
use mipidsi::Display;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 250 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

slint::include_modules!();
#[entry]
fn main() -> ! {
    init_heap();

    println!("Hello world!");
    slint::platform::set_platform(Box::new(EspBackend::default()))
        .expect("backend already initialized");

    let main_window = Recipe::new().unwrap();

    let strong = main_window.clone_strong();
    let timer = slint::Timer::default();
    timer.start(
        slint::TimerMode::Repeated,
        core::time::Duration::from_millis(1000),
        move || {
            if strong.get_counter() <= 0 {
                strong.set_counter(25);
            } else {
                strong.set_counter(0);
            }
        },
    );

    main_window.run().unwrap();

    panic!("The MCU demo should not quit");
}

#[derive(Default)]
pub struct EspBackend {
    window: RefCell<Option<Rc<slint::platform::software_renderer::MinimalSoftwareWindow>>>,
}

impl slint::platform::Platform for EspBackend {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(
            slint::platform::software_renderer::RepaintBufferType::ReusedBuffer,
        );
        self.window.replace(Some(window.clone()));
        Ok(window)
    }

    fn duration_since_start(&self) -> core::time::Duration {
        core::time::Duration::from_millis(
            SystemTimer::now() / (SystemTimer::TICKS_PER_SECOND / 1000),
        )
    }

    fn run_event_loop(&self) -> Result<(), slint::PlatformError> {
        let peripherals = Peripherals::take();
        let mut system = peripherals.SYSTEM.split();
        let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock240MHz).freeze();

        let mut rtc = Rtc::new(peripherals.RTC_CNTL);
        let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
        let mut wdt0 = timer_group0.wdt;
        let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
        let mut wdt1 = timer_group1.wdt;

        rtc.rwdt.disable();
        wdt0.disable();
        wdt1.disable();

        let mut delay = Delay::new(&clocks);
        let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

        let clk = io.pins.gpio18;
        let sdo = io.pins.gpio17;
        let cs = io.pins.gpio14;


        let spi = Spi::new_no_miso(
            peripherals.SPI2,
            clk,
            sdo,
            cs,
            60u32.MHz(),
            SpiMode::Mode0,
            &clocks,
        );
        println!("spi init.");

        let dc = io.pins.gpio15.into_push_pull_output();
        let rst = io.pins.gpio16.into_push_pull_output();

        let di = SPIInterfaceNoCS::new(spi, dc);
        let mut display = mipidsi::Builder::st7789(di)
            .with_display_size(240, 280)
            .with_window_offset_handler(|_| (0, 20))
            .with_framebuffer_size(240, 280)
            .with_invert_colors( mipidsi::ColorInversion::Inverted)
            .init(&mut delay, Some(rst))
            .unwrap();

        println!("display init.");
        let mut bl = io.pins.gpio13.into_push_pull_output();
        bl.set_high().unwrap();

        let size = slint::PhysicalSize::new(240, 280);

        self.window.borrow().as_ref().unwrap().set_size(size);

        let mut buffer_provider = DrawBuffer {
            display,
            buffer: &mut [slint::platform::software_renderer::Rgb565Pixel::default(); 240],
        };

        loop {
            slint::platform::update_timers_and_animations();

            if let Some(window) = self.window.borrow().clone() {
                window.draw_if_needed(|renderer| {
                    renderer.render_by_line(&mut buffer_provider);
                });
                if window.has_active_animations() {
                    continue;
                }
            }
        }
    }

    fn debug_log(&self, arguments: core::fmt::Arguments) {
        println!("{}", arguments);
    }
}

struct DrawBuffer<'a, Display> {
    display: Display,
    buffer: &'a mut [slint::platform::software_renderer::Rgb565Pixel],
}

impl<DI: display_interface::WriteOnlyDataCommand, RST: embedded_hal::digital::v2::OutputPin>
    slint::platform::software_renderer::LineBufferProvider
    for &mut DrawBuffer<'_, Display<DI, mipidsi::models::ST7789, RST>>
{
    type TargetPixel = slint::platform::software_renderer::Rgb565Pixel;

    fn process_line(
        &mut self,
        line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [slint::platform::software_renderer::Rgb565Pixel]),
    ) {
        let buffer = &mut self.buffer[range.clone()];

        render_fn(buffer);

        // We send empty data just to get the device in the right window
        self.display
            .set_pixels(
                range.start as u16,
                line as _,
                range.end as u16,
                line as u16,
                buffer
                    .iter()
                    .map(|x| embedded_graphics_core::pixelcolor::raw::RawU16::new(x.0).into()),
            )
            .unwrap();
        // self.display
        //     .fill_contiguous(
        //         &Rectangle::new(
        //             Point::new(range.start as _, line as _),
        //             Size::new(range.len() as _, 1),
        //         ),
        //         self.buffer[range.clone()]
        //             .iter()
        //             .map(|p| RawU16::new(p.0).into()),
        //     )
        //     .map_err(drop)
        //     .unwrap();
    }
}
