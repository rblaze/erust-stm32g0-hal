use crate::pac::usb::chepr::{STATRXR, STATTXR, UTYPE};
use crate::rcc::{Rcc, ResetEnable};

use super::buffers::{RxBuffer, TxBuffer};
use super::endpoint::Endpoint;

use critical_section::Mutex;
use rtt_target::debug_rprintln;
use usb_device::bus::{PollResult, UsbBus, UsbBusAllocator};
use usb_device::endpoint::EndpointType;
use usb_device::{Result, UsbDirection, UsbError};

/// Maximum number of endpoints per device.
/// It would better be placed in the trait implementation but I can't find the way to do this.
pub const MAX_ENDPOINTS: usize = 8;

pub trait UsbExt {
    /// Start address of packet buffer memory
    const MEMORY_BASE: *const ();
    /// Size of packet buffer memory in bytes
    const MEMORY_SIZE: usize;

    /// Constrain the peripheral.
    fn constrain(self, rcc: &Rcc) -> UsbBusAllocator<Bus<Self>>
    where
        Self: Sized,
        Bus<Self>: UsbBus;
}

impl UsbExt for crate::pac::USB {
    const MEMORY_BASE: *const () = 0x4000_9800 as _;
    const MEMORY_SIZE: usize = 2048;

    fn constrain(self, rcc: &Rcc) -> UsbBusAllocator<Bus<Self>>
    where
        Self: Sized,
        Bus<Self>: UsbBus,
    {
        // Enable USB clock (HSI48)
        rcc.enable_hsi48();

        // Enable USB device
        crate::pac::USB::enable(rcc);

        UsbBusAllocator::new(Bus(Mutex::new(BusData {
            usb: self,
            endpoints: Default::default(),
            allocator: super::buffers::Allocator::new(Self::MEMORY_SIZE),
        })))
    }
}

/// Constrained USB device
pub struct Bus<USB>(Mutex<BusData<USB>>);

impl Bus<crate::pac::USB> {
    /// Enable interrupts from USB device
    pub fn enable_interrupts(&self) {
        critical_section::with(|cs| {
            self.0.borrow(cs).usb.cntr().modify(|_, w| {
                w.ctrm()
                    .set_bit()
                    .wkupm()
                    .set_bit()
                    .suspm()
                    .set_bit()
                    .resetm()
                    .set_bit()
            });
        });
    }

    /// Disable interrupts from USB device
    pub fn disable_interrupts(&self) {
        critical_section::with(|cs| {
            self.0.borrow(cs).usb.cntr().modify(|_, w| {
                w.ctrm()
                    .clear_bit()
                    .wkupm()
                    .clear_bit()
                    .suspm()
                    .clear_bit()
                    .resetm()
                    .clear_bit()
            });
        });
    }
}

impl UsbBus for Bus<crate::pac::USB> {
    fn alloc_ep(
        &mut self,
        ep_dir: UsbDirection,
        ep_addr: Option<usb_device::endpoint::EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval: u8,
    ) -> Result<usb_device::endpoint::EndpointAddress> {
        self.0
            .get_mut()
            .alloc_ep(ep_dir, ep_addr, ep_type, max_packet_size, interval)
    }

    fn enable(&mut self) {
        self.0.get_mut().enable()
    }

    fn reset(&self) {
        critical_section::with(|cs| self.0.borrow(cs).reset())
    }

    fn set_device_address(&self, addr: u8) {
        critical_section::with(|cs| self.0.borrow(cs).set_device_address(addr))
    }

    fn write(&self, ep_addr: usb_device::endpoint::EndpointAddress, buf: &[u8]) -> Result<usize> {
        critical_section::with(|cs| self.0.borrow(cs).write(ep_addr, buf))
    }

    fn read(
        &self,
        ep_addr: usb_device::endpoint::EndpointAddress,
        buf: &mut [u8],
    ) -> Result<usize> {
        critical_section::with(|cs| self.0.borrow(cs).read(ep_addr, buf))
    }

    fn set_stalled(&self, ep_addr: usb_device::endpoint::EndpointAddress, stalled: bool) {
        critical_section::with(|cs| self.0.borrow(cs).set_stalled(ep_addr, stalled))
    }

    fn is_stalled(&self, ep_addr: usb_device::endpoint::EndpointAddress) -> bool {
        critical_section::with(|cs| self.0.borrow(cs).is_stalled(ep_addr))
    }

    fn suspend(&self) {
        critical_section::with(|cs| self.0.borrow(cs).suspend())
    }

    fn resume(&self) {
        critical_section::with(|cs| self.0.borrow(cs).resume())
    }

    fn poll(&self) -> PollResult {
        critical_section::with(|cs| self.0.borrow(cs).poll())
    }

    fn force_reset(&self) -> Result<()> {
        critical_section::with(|cs| self.0.borrow(cs).force_reset())
    }
}

struct BusData<USB> {
    usb: USB,
    endpoints: [Endpoint<USB>; MAX_ENDPOINTS],
    allocator: super::buffers::Allocator<USB>,
}

impl<USB> BusData<USB> {
    fn find_free_endpoint(&self) -> Result<usize> {
        // Do not allocate endpoint 0, it's reserved for control transfers.
        for index in 1..MAX_ENDPOINTS {
            if self.endpoints[index].ep_type.is_none() {
                return Ok(index);
            }
        }
        Err(usb_device::UsbError::EndpointOverflow)
    }
}

impl BusData<crate::pac::USB> {
    #[allow(unsafe_code)]
    fn set_statrx(&self, index: usize, state: STATRXR) {
        self.usb
            .chepr(index)
            .modify(|r, w| unsafe { w.statrx().bits(r.statrx().bits() ^ state as u8) });
    }

    #[allow(unsafe_code)]
    fn set_stattx(&self, index: usize, state: STATTXR) {
        self.usb
            .chepr(index)
            .modify(|r, w| unsafe { w.stattx().bits(r.stattx().bits() ^ state as u8) });
    }

    fn reload_endpoints(&self) {
        for index in 0..MAX_ENDPOINTS {
            let ep = &self.endpoints[index];
            match ep.ep_type {
                Some(ep_type) => {
                    debug_rprintln!(
                        "loading endpoint {} {:?} {} {}",
                        index,
                        ep_type,
                        ep.in_buffer.is_some(),
                        ep.out_buffer.is_some()
                    );
                    let utype = match ep_type {
                        EndpointType::Bulk => UTYPE::Bulk,
                        EndpointType::Interrupt => UTYPE::Interrupt,
                        EndpointType::Isochronous { .. } => UTYPE::Iso,
                        EndpointType::Control => UTYPE::Control,
                    };

                    #[allow(unsafe_code)]
                    self.usb
                        .chepr(index)
                        .write(|w| unsafe { w.ea().bits(index as u8).utype().variant(utype) });

                    let rxtarget = if ep.out_buffer.is_some() {
                        STATRXR::Valid
                    } else {
                        STATRXR::Disabled
                    };

                    let txtarget = if ep.in_buffer.is_some() {
                        STATTXR::Nak
                    } else {
                        STATTXR::Disabled
                    };

                    self.set_statrx(index, rxtarget);
                    self.set_stattx(index, txtarget);
                }
                None => {
                    // No endpoint
                    self.usb.chepr(index).reset();
                }
            }
        }
    }

    fn alloc_ep(
        &mut self,
        ep_dir: usb_device::UsbDirection,
        ep_addr: Option<usb_device::endpoint::EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
        _interval: u8,
    ) -> Result<usb_device::endpoint::EndpointAddress> {
        debug_rprintln!(
            "alloc_ep: {:?} {:?} ({:?} {:?}) {:?} {:?}",
            ep_dir,
            ep_addr,
            ep_addr.map(|a| a.direction()),
            ep_addr.map(|a| a.index()),
            ep_type,
            max_packet_size
        );
        // Find an endpoint with a given index or create a new one.
        let index = if let Some(addr) = ep_addr {
            // Adding second direction to the existing endpoint
            let index = addr.index();
            let ep = self.endpoints.get(index).ok_or(UsbError::InvalidEndpoint)?;

            // Endpoint must have the same type.
            if ep.ep_type.is_some_and(|t| t != ep_type) {
                return Err(UsbError::InvalidEndpoint);
            };

            // Endpoint can't be already initialized in this direction.
            let buffer_allocated = if ep_dir == UsbDirection::Out {
                ep.out_buffer.is_some()
            } else {
                ep.in_buffer.is_some()
            };
            if buffer_allocated {
                return Err(UsbError::InvalidEndpoint);
            }

            index
        } else {
            self.find_free_endpoint()?
        };

        if ep_dir == UsbDirection::Out {
            self.endpoints[index].out_buffer = Some(RxBuffer::new(
                &mut self.allocator,
                index,
                max_packet_size as usize,
                crate::pac::USB::MEMORY_BASE,
            )?);
        } else {
            self.endpoints[index].in_buffer = Some(TxBuffer::new(
                &mut self.allocator,
                index,
                max_packet_size as usize,
                crate::pac::USB::MEMORY_BASE,
            )?);
        }

        // All fallible operations done, update endpoint data.
        self.endpoints[index].ep_type = Some(ep_type);

        Ok(usb_device::endpoint::EndpointAddress::from_parts(
            index, ep_dir,
        ))
    }

    fn enable(&mut self) {
        debug_rprintln!("USB enable");
        // Power-on tranciever
        self.usb.cntr().modify(|_, w| w.pdwn().clear_bit());
        // TODO: RM0444 requires a delay here but datasheet doesn't specify for how long.
        // STM HAL doesn't seem to have any delay.
        cortex_m::asm::delay(500000);
        // Clear interrupts
        self.usb.istr().reset();
        self.usb.cntr().modify(|_, w| w.usbrst().clear_bit());
        // Enable pullup on DP, connect to host
        self.usb.bcdr().modify(|_, w| w.dppu_dpd().set_bit());
    }

    fn reset(&self) {
        debug_rprintln!("USB reset");
        // Reset device
        self.usb.daddr().reset();
        self.usb.cntr().modify(|_, w| w.usbrst().reset());
        self.usb.cntr().modify(|_, w| w.usbrst().clear_bit());

        self.reload_endpoints();

        // Clear interrupts
        #[allow(unsafe_code)]
        self.usb.istr().write(|w| unsafe { w.bits(0) });

        self.usb.daddr().write(|w| w.ef().set_bit());
    }

    #[allow(unsafe_code)]
    fn set_device_address(&self, addr: u8) {
        debug_rprintln!("set_device_address: {}", addr);
        unsafe {
            self.usb.daddr().modify(|_, w| w.add().bits(addr));
        }
    }

    fn write(&self, ep_addr: usb_device::endpoint::EndpointAddress, buf: &[u8]) -> Result<usize> {
        debug_rprintln!("USB write {:?} {} {:?}", ep_addr, buf.len(), buf);
        if !ep_addr.is_in() {
            return Err(UsbError::InvalidEndpoint);
        }

        let index = ep_addr.index();
        let status = self.usb.chepr(index).read();

        match status.stattx().variant() {
            STATTXR::Disabled => return Err(UsbError::InvalidState),
            STATTXR::Valid => return Err(UsbError::WouldBlock),
            _ => {}
        }

        self.endpoints[index].write(buf)?;
        // Start transfer
        self.usb.chepr(index).modify(|_, w| w.vttx().clear_bit());
        self.set_stattx(index, STATTXR::Valid);

        Ok(buf.len())
    }

    fn read(
        &self,
        ep_addr: usb_device::endpoint::EndpointAddress,
        buf: &mut [u8],
    ) -> Result<usize> {
        if !ep_addr.is_out() {
            return Err(UsbError::InvalidEndpoint);
        }

        let index = ep_addr.index();
        let status = self.usb.chepr(index).read();

        if status.statrx() == STATRXR::Disabled {
            return Err(UsbError::InvalidState);
        }

        if status.vtrx().bit_is_clear() {
            return Err(UsbError::WouldBlock);
        }

        let result = self.endpoints[index].read(buf);

        // Ready or not, prepare endpoint for the next transfer.
        self.usb.chepr(index).modify(|_, w| w.vtrx().clear());
        self.set_statrx(index, STATRXR::Valid);

        if let Ok(size) = result {
            debug_rprintln!("read data: req {}, got {:?}", buf.len(), &buf[..size]);
        } else {
            debug_rprintln!("read data: req {}, got {:?}", buf.len(), result);
        }
        result
    }

    fn set_stalled(&self, ep_addr: usb_device::endpoint::EndpointAddress, stalled: bool) {
        debug_rprintln!("set_stalled: {:?} {}", ep_addr, stalled);
        let index = ep_addr.index();

        match ep_addr.direction() {
            UsbDirection::Out => {
                if self.usb.chepr(index).read().statrx().is_stall() == stalled {
                    // No change in mode, do nothing.
                    return;
                }

                let target_mode = if stalled {
                    STATRXR::Stall
                } else {
                    STATRXR::Valid
                };

                self.set_statrx(index, target_mode);
            }
            UsbDirection::In => {
                if self.usb.chepr(index).read().stattx().is_stall() == stalled {
                    // No change in mode, do nothing.
                    return;
                }

                let target_mode = if stalled {
                    STATTXR::Stall
                } else {
                    STATTXR::Valid
                };

                self.set_stattx(index, target_mode);
            }
        }
    }

    fn is_stalled(&self, ep_addr: usb_device::endpoint::EndpointAddress) -> bool {
        let index = ep_addr.index();

        match ep_addr.direction() {
            UsbDirection::Out => self.usb.chepr(index).read().statrx().is_stall(),
            UsbDirection::In => self.usb.chepr(index).read().stattx().is_stall(),
        }
    }

    fn suspend(&self) {
        debug_rprintln!("USB suspend");
        self.usb.cntr().modify(|_, w| w.suspen().suspend());
        while self.usb.cntr().read().susprdy().bit_is_clear() {}
    }

    fn resume(&self) {
        debug_rprintln!("USB resume");
        // Shouldn't be necessary: per RM, this bit is cleared by hardware
        // simultaneous with the WAKEUP flag set.
        self.usb.cntr().modify(|_, w| w.suspen().clear_bit());
    }

    fn poll(&self) -> PollResult {
        let istr = self.usb.istr().read();

        if istr.wkup().is_wakeup() {
            self.usb.istr().write(|w| w.wkup().clear().susp().clear());
            debug_rprintln!("resume");
            PollResult::Resume
        } else if istr.rst_dcon().is_reset() {
            self.usb.istr().write(|w| w.rst_dcon().clear());
            debug_rprintln!("reset");
            PollResult::Reset
        } else if istr.susp().is_suspend() {
            self.usb.istr().write(|w| w.susp().clear());
            debug_rprintln!("suspend");
            PollResult::Suspend
        } else if istr.ctr().is_completed() {
            let mut ep_out = 0;
            let mut ep_in_complete = 0;
            let mut ep_setup = 0;

            for index in 0..MAX_ENDPOINTS {
                let chepr = self.usb.chepr(index).read();

                if chepr.vttx().bit_is_set() {
                    ep_in_complete |= 1 << index;
                    self.usb.chepr(index).modify(|_, w| w.vttx().clear());
                }

                if chepr.vtrx().bit_is_set() {
                    if chepr.setup().bit_is_set() {
                        ep_setup |= 1 << index;
                    } else {
                        ep_out |= 1 << index;
                    }
                }
            }

            debug_rprintln!(
                "data: rx {:b} tx {:b} setup {:b}",
                ep_out,
                ep_in_complete,
                ep_setup
            );
            PollResult::Data {
                ep_out,
                ep_in_complete,
                ep_setup,
            }
        } else {
            PollResult::None
        }
    }

    fn force_reset(&self) -> Result<()> {
        debug_rprintln!("force_reset");
        // Disconnect pull-up from DP
        self.usb.bcdr().modify(|_, w| w.dppu_dpd().clear_bit());
        // TODO: add sleep here?
        // Reconnect the pull-up resistor to DP
        self.usb.bcdr().modify(|_, w| w.dppu_dpd().set_bit());
        Ok(())
    }
}
