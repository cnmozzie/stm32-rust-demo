//! Demonstrate the use of a blocking `Delay` using the SYST (sysclock) timer.

#![deny(unsafe_code)]
#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

// Halt on panic
use panic_halt as _; // panic handler

use cortex_m_rt::entry;
use stm32f4xx_hal as hal;

use crate::hal::{pac, prelude::*};
use w5500::{bus::FourWire, MacAddress};
use embedded_nal::TcpClientStack;

#[entry]
fn main() -> ! {
    if let (Some(dp), Some(cp)) = (
        pac::Peripherals::take(),
        cortex_m::peripheral::Peripherals::take(),
    ) {
        // Set up the LED. On the Nucleo-446RE it's connected to pin PA5.
        let gpioa = dp.GPIOA.split();
        let gpiof = dp.GPIOF.split();
        let mut led0 = gpiof.pf9.into_push_pull_output();
        let mut led1 = gpiof.pf10.into_push_pull_output();

        // Set up the system clock. We want to run at 48MHz for this one.
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.MHz()).freeze();

        let spi = {
            let mode = hal::spi::Mode {
                polarity: hal::spi::Polarity::IdleLow,
                phase: hal::spi::Phase::CaptureOnFirstTransition,
            };

            hal::spi::Spi::new(
                dp.SPI1,
                (
                    gpioa.pa5.into_alternate(),
                    gpioa.pa6.into_alternate(),
                    gpioa.pa7.into_alternate(),
                ),
                mode,
                14.MHz(),
                &clocks,
            )
        };

        let cs = {
            let mut pin = gpioa.pa4.into_push_pull_output();
            pin.set_high();
            pin
        };

        let mut device = w5500::UninitializedDevice::new(FourWire::new(spi, cs))
            .initialize_manual(
                MacAddress::new(0, 1, 2, 3, 4, 5),
                embedded_nal::Ipv4Addr::new(192, 168, 50, 87),
                w5500::Mode::default(),
            )
            .unwrap();

        // Create a delay abstraction based on SysTick
        let mut delay = cp.SYST.delay(&clocks);

        loop {
            let mut socket = device.socket().unwrap();
            device
                .connect(
                    &mut socket,
                    embedded_nal::SocketAddr::new(
                        embedded_nal::IpAddr::V4(embedded_nal::Ipv4Addr::new(192, 168, 50, 28)),
                        5000,
                    ),
                )
                .unwrap();
            
            device
                .send(&mut socket, &[104, 101, 108, 108, 111, 10])
                .unwrap();
            
            device.close(socket).unwrap();
            // On for 1s, off for 1s.
            led0.toggle();
            delay.delay_ms(1000_u32);
            led1.toggle();
            delay.delay_ms(1000_u32);
        }
    }

    loop {}
}