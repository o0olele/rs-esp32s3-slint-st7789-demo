extern crate alloc;
use alloc::rc::Rc;
use core::cell::RefCell;
use display_interface_spi::SPIInterfaceNoCS;
use embedded_graphics_core::pixelcolor::Rgb565;
use embedded_graphics_core::prelude::{DrawTarget, RgbColor};
use embedded_graphics_core::primitives::Rectangle;
use embedded_graphics_framebuf::FrameBuf;
use esp_println::println;
use esp_backtrace as _;
use hal::spi::master::prelude::_esp_hal_spi_master_dma_WithDmaSpi2;
use hal::spi::master::Spi;
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
#[derive(Default)]
pub struct DmaBackend {
    window: RefCell<Option<Rc<slint::platform::software_renderer::MinimalSoftwareWindow>>>,
}

// this is not avalable now.
impl slint::platform::Platform for DmaBackend {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(
            slint::platform::software_renderer::RepaintBufferType::ReusedBuffer,
        );
        window.set_size(slint::PhysicalSize::new(240, 280));
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

        let dma1 = hal::dma::gdma::Gdma::new(peripherals.DMA);
        let dma1_ch = dma1.channel3;

        let mut descriptors = [0u32; 8 * 3];
        let mut rx_descriptors = [0u32; 18 * 3];

        let spi = Spi::new_no_miso(
            peripherals.SPI2,
            clk,
            sdo,
            cs,
            60u32.MHz(),
            SpiMode::Mode0,
            &clocks,
        )
        .with_dma(dma1_ch.configure(
            false,
            &mut descriptors,
            &mut rx_descriptors,
            hal::dma::DmaPriority::Priority0,
        ));
        println!("spi init.");

        let dc = io.pins.gpio15.into_push_pull_output();
        let rst = io.pins.gpio16.into_push_pull_output();

        let di = SPIInterfaceNoCS::new(spi, dc);
        let mut display = mipidsi::Builder::st7789(di)
            .with_display_size(240, 280)
            .with_color_order(mipidsi::options::ColorOrder::Bgr)
            .with_window_offset_handler(|_| (0, 20))
            .with_framebuffer_size(240, 280)
            .init(&mut delay, Some(rst))
            .unwrap();

        let mut data = [embedded_graphics_core::pixelcolor::Rgb565::WHITE; 240];
        println!("display init.");
        let mut bl = io.pins.gpio13.into_push_pull_output();
        bl.set_high().unwrap();
        println!("bl init.");
        println!("window init.");
        let mut buffer_provider = DrawBuffer {
            display,
            line: &mut [slint::platform::software_renderer::Rgb565Pixel(0); 240],
            buffer: FrameBuf::new(&mut data, 240, 1),
        };

        println!("buffer init.");
        loop {
            println!("animate.");
            slint::platform::update_timers_and_animations();
            if let Some(window) = self.window.borrow().clone() {
                
                window.draw_if_needed(|renderer| {
                    renderer.render_by_line(&mut buffer_provider);
                   
                });
                // buffer_provider.flush();
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
    line: &'a mut [slint::platform::software_renderer::Rgb565Pixel],
    buffer: FrameBuf<Rgb565, &'a mut [Rgb565; 240]>,
}

impl<DI: display_interface::WriteOnlyDataCommand, RST: embedded_hal::digital::v2::OutputPin>
    DrawBuffer<'_, Display<DI, mipidsi::models::ST7789, RST>>
{
    pub fn flush(&mut self) {
        self.display.draw_iter(self.buffer.into_iter()).unwrap();
    }
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
        let buffer = &mut self.line[range.clone()];

        render_fn(buffer);

        // let _ = self.buffer.fill_contiguous(
        //     &Rectangle::new(
        //         embedded_graphics_core::prelude::Point::new(range.start as _, line as _),
        //         embedded_graphics_core::prelude::Size::new(range.len() as _, 1),
        //     ),
        //     self.line[range.clone()]
        //         .iter()
        //         .map(|p| embedded_graphics::pixelcolor::raw::RawU16::new(p.0).into()),
        // );
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
    }
}
