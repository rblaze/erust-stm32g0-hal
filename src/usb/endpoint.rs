use core::cell::Cell;

use super::buffers::{RxBuffer, TxBuffer};

use usb_device::endpoint::EndpointType;
use usb_device::{Result, UsbError};

#[derive(Debug)]
pub(super) struct Endpoint<USB> {
    /// Buffer for host-to-device transfers
    pub out_buffer: Option<RxBuffer<USB>>,
    /// Buffer for device-to-host transfers
    pub in_buffer: Option<TxBuffer<USB>>,
    /// Endpoint type for allocated endpoints, None for unallocated.
    pub ep_type: Option<EndpointType>,
    pub out_nack: Cell<bool>,
}

// Can't derive Default because it requires USB to be Default too.
impl<USB> Default for Endpoint<USB> {
    fn default() -> Self {
        Self {
            out_buffer: None,
            in_buffer: None,
            ep_type: None,
            out_nack: Cell::new(false),
        }
    }
}

impl<USB> Endpoint<USB> {
    pub fn read(&self, buf: &mut [u8]) -> Result<usize> {
        let buffer = self.out_buffer.as_ref().ok_or(UsbError::InvalidEndpoint)?;
        buffer.read(buf)
    }

    pub fn write(&self, buf: &[u8]) -> Result<usize> {
        let buffer = self.in_buffer.as_ref().ok_or(UsbError::InvalidEndpoint)?;
        buffer.write(buf)
    }
}
