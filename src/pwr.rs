pub trait PwrExt {
    fn constrain(self) -> Pwr;
}

impl PwrExt for crate::pac::PWR {
    fn constrain(self) -> Pwr {
        Pwr { pwr: self }
    }
}

/// Constrained PWR device
pub struct Pwr {
    #[allow(unused)]
    pwr: crate::pac::PWR,
}

impl Pwr {
    /// Enable USB power supply
    #[cfg(feature = "stm32g0b1")]
    pub fn enable_usb(&self) {
        self.pwr.cr2().modify(|_, w| w.usv().set_bit());
    }

    /// Disable USB power supply
    #[cfg(feature = "stm32g0b1")]
    pub fn disable_usb(&self) {
        self.pwr.cr2().modify(|_, w| w.usv().clear_bit());
    }
}
