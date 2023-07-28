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

use w5500::MacAddress;

use smoltcp_nal::smoltcp;
use smoltcp::iface::Interface;
use smoltcp::wire::{EthernetAddress, IpCidr, Ipv4Address, Ipv4Cidr};
use smoltcp::socket::dhcpv4;

pub type SpiCs = hal::gpio::gpioa::PA4<hal::gpio::Output<hal::gpio::PushPull>>;
pub type Spi = hal::spi::Spi<hal::pac::SPI1>;
pub type UsbBus = hal::otg_fs::UsbBus<hal::otg_fs::USB>;

/// The number of TCP sockets supported in the network stack.
const NUM_TCP_SOCKETS: usize = 4;
/// Containers for smoltcp-related network configurations
struct NetStorage {
    // Note: There is an additional socket set item required for the DHCP socket.
    pub sockets: [smoltcp::iface::SocketStorage<'static>; NUM_TCP_SOCKETS + 1],
    pub tcp_socket_storage: [TcpSocketStorage; NUM_TCP_SOCKETS],
}

impl NetStorage {
    const fn new() -> Self {
        NetStorage {
            sockets: [smoltcp::iface::SocketStorage::EMPTY; NUM_TCP_SOCKETS + 1],
            tcp_socket_storage: [TcpSocketStorage::new(); NUM_TCP_SOCKETS],
        }
    }
}

#[derive(Copy, Clone)]
struct TcpSocketStorage {
    rx_storage: [u8; 1024],

    // Note that TX storage is set to 4096 to ensure that it is sufficient to contain full
    // telemetry messages for all 8 RF channels.
    tx_storage: [u8; 4096],
}

impl TcpSocketStorage {
    const fn new() -> Self {
        Self {
            tx_storage: [0; 4096],
            rx_storage: [0; 1024],
        }
    }
}

pub enum Mac {
    W5500(w5500::raw_device::RawDevice<w5500::bus::FourWire<Spi, SpiCs>>),
}

impl smoltcp::phy::Device for Mac {
    type RxToken<'a> = RxToken where Self: 'a;
    type TxToken<'a> = TxToken<'a> where Self: 'a;

    fn capabilities(&self) -> smoltcp::phy::DeviceCapabilities {
        let mut caps = smoltcp::phy::DeviceCapabilities::default();
        caps.max_transmission_unit = 1500;
        caps.medium = smoltcp::phy::Medium::Ethernet;
        caps
    }

    fn receive(
        &mut self,
        _timestamp: smoltcp::time::Instant,
    ) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        let mut buffer = [0u8; 1500];
        let len = match self {
            Mac::W5500(w5500) => w5500.read_frame(&mut buffer[..]).unwrap(),
        };

        if len != 0 {
            Some((
                RxToken {
                    frame_buffer: buffer,
                    length: len,
                },
                TxToken { mac: self },
            ))
        } else {
            None
        }
    }

    fn transmit(&mut self, _timestamp: smoltcp::time::Instant) -> Option<Self::TxToken<'_>> {
        Some(TxToken { mac: self })
    }
}

pub struct RxToken {
    frame_buffer: [u8; 1500],
    length: usize,
}

impl smoltcp::phy::RxToken for RxToken {
    fn consume<R, F>(mut self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        f(&mut self.frame_buffer[..self.length])
    }
}

pub struct TxToken<'a> {
    mac: &'a mut Mac,
}

impl<'a> smoltcp::phy::TxToken for TxToken<'a> {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let mut buffer = [0u8; 1500];
        let result = f(&mut buffer[..len]);
        match self.mac {
            Mac::W5500(mac) => {
                mac.write_frame(&buffer[..len]).unwrap();
            }
        }

        result
    }
}

mod mock {
    use core::cell::Cell;
    use smoltcp_nal::smoltcp::time::{Duration, Instant};

    #[derive(Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct Clock(Cell<Instant>);

    impl Clock {
        pub fn new() -> Clock {
            Clock(Cell::new(Instant::from_millis(0)))
        }

        pub fn advance(&self, duration: Duration) {
            self.0.set(self.0.get() + duration)
        }

        pub fn elapsed(&self) -> Instant {
            self.0.get()
        }
    }
}


#[entry]
fn main() -> ! {
    let mock_clock = mock::Clock::new();
    if let (Some(dp), Some(_)) = (
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

        //let endpoint_memory = cortex_m::singleton!(: [u32; 1024] = [0; 1024]).unwrap();

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

        let w5500 = w5500::UninitializedDevice::new(w5500::bus::FourWire::new(spi, cs))
            .initialize_macraw(MacAddress::new(0, 1, 2, 3, 4, 5))
            .unwrap();
        
        let mut mac = Mac::W5500(w5500);
        let device = &mut mac;
        //let device = &mut mac;
        let mut config = smoltcp::iface::Config::default();
        config
            .hardware_addr
            .replace(smoltcp::wire::HardwareAddress::Ethernet(EthernetAddress([0, 1, 2, 3, 4, 5]).into()));
        
        let mut iface = smoltcp::iface::Interface::new(config, device);
        

        iface
            .routes_mut()
            .add_default_ipv4_route(Ipv4Address::new(0, 0, 0, 0))
            .unwrap();

        iface.update_ip_addrs(|addrs| addrs.push(IpCidr::new(smoltcp::wire::IpAddress::v4(0, 0, 0, 0), 0)).unwrap());

        // Create sockets
        let dhcp_socket = dhcpv4::Socket::new();
        let net_store = cortex_m::singleton!(: NetStorage = NetStorage::new()).unwrap();
        let mut sockets = smoltcp::iface::SocketSet::new(&mut net_store.sockets[..]);
        
        let tcp_socket = {
            let rx_buffer = smoltcp::socket::tcp::SocketBuffer::new(&mut net_store.tcp_socket_storage[0].rx_storage[..]);
            let tx_buffer = smoltcp::socket::tcp::SocketBuffer::new(&mut net_store.tcp_socket_storage[0].tx_storage[..]);

            smoltcp::socket::tcp::Socket::new(rx_buffer, tx_buffer)
        };
        let tcp_handle = sockets.add(tcp_socket);
        let dhcp_handle = sockets.add(dhcp_socket);

        loop {
            iface.poll(mock_clock.elapsed(), device, &mut sockets);
            mock_clock.advance(smoltcp::time::Duration::from_millis(1));
    
            let event = sockets.get_mut::<dhcpv4::Socket>(dhcp_handle).poll();
            match event {
                None => {}
                Some(dhcpv4::Event::Configured(config)) => {
                    //debug!("DHCP config acquired!");
    
                    //debug!("IP address:      {}", config.address);
                    set_ipv4_addr(&mut iface, config.address);
    
                    if let Some(router) = config.router {
                        //debug!("Default gateway: {}", router);
                        iface.routes_mut().add_default_ipv4_route(router).unwrap();
                    } else {
                        //debug!("Default gateway: None");
                        iface.routes_mut().remove_default_ipv4_route();
                    }
    
                    //for (i, s) in config.dns_servers.iter().enumerate() {
                        //debug!("DNS server {}:    {}", i, s);
                    //}
                    led1.set_high();
                    break;
                }
                Some(dhcpv4::Event::Deconfigured) => {
                    //debug!("DHCP lost config!");
                    led1.set_high();
                    set_ipv4_addr(&mut iface, Ipv4Cidr::new(Ipv4Address::UNSPECIFIED, 0));
                    iface.routes_mut().remove_default_ipv4_route();
                }
            }
        }

        let socket = sockets.get_mut::<smoltcp::socket::tcp::Socket>(tcp_handle);
        let cx = iface.context();
        if !socket.is_open() {
            socket
                .connect(cx, (smoltcp::wire::IpAddress::v4(192, 168, 50, 28), 5000), 65001)
                .unwrap();
        }

        loop {
            iface.poll(mock_clock.elapsed(), device, &mut sockets);
            mock_clock.advance(smoltcp::time::Duration::from_millis(1));
            let socket = sockets.get_mut::<smoltcp::socket::tcp::Socket>(tcp_handle);

            let mut data : [u8; 128] = [0; 128];
            let mut len = 0;
            
            match socket.recv_slice(&mut data) {
                Ok(size) => {len = size; led1.set_low();}
                Err(_) => {led1.set_high();}  // or we should handle the error
            }

            if socket.can_send() && len > 0 {
                socket.send_slice(&data[..len]).unwrap();
                led0.toggle();
            }
            
        }
    }

    loop {}
}

fn set_ipv4_addr(iface: &mut Interface, cidr: Ipv4Cidr) {
    iface.update_ip_addrs(|addrs| {
        let dest = addrs.iter_mut().next().unwrap();
        *dest = IpCidr::Ipv4(cidr);
    });
}