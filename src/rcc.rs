pub mod config;
pub mod lptim;
mod reset_enable;

use self::config::Config;
use crate::pac::RCC;

use fugit::KilohertzU32;

/// HSI frequency
pub const HSI_FREQ: KilohertzU32 = KilohertzU32::MHz(16);
/// LSI frequency
pub const LSI_FREQ: KilohertzU32 = KilohertzU32::kHz(32);

/// Extension trait for RCC
pub trait RccExt {
    /// Constrain the peripheral and configure clocks.
    fn constrain(self, config: Config) -> Rcc;
}

impl RccExt for RCC {
    fn constrain(self, config: Config) -> Rcc {
        // HSI is a default clock source.
        // It is enabled and ready by default.

        let sysclk = match config.hsisys_prescaler {
            config::Prescaler::Div1 => HSI_FREQ,
            config::Prescaler::Div2 => HSI_FREQ / 2,
            config::Prescaler::Div4 => HSI_FREQ / 4,
            config::Prescaler::Div8 => HSI_FREQ / 8,
            config::Prescaler::Div16 => HSI_FREQ / 16,
            config::Prescaler::Div32 => HSI_FREQ / 32,
            config::Prescaler::Div64 => HSI_FREQ / 64,
            config::Prescaler::Div128 => HSI_FREQ / 128,
        };

        // Set HSI prescaler.
        self.cr()
            .modify(|_, w| w.hsidiv().variant(config.hsisys_prescaler));

        if config.lsi_enabled {
            // Enable LSI and wait for it to be ready.
            self.csr().modify(|_, w| w.lsion().set_bit());
            while self.csr().read().lsirdy().bit_is_clear() {}
        }

        Rcc { rcc: self, sysclk }
    }
}

/// Peripheral enable/disable/reset
pub trait ResetEnable {
    fn enable(rcc: &Rcc);
    fn disable(rcc: &Rcc);
    fn reset(rcc: &Rcc);
}

/// Constrained RCC peripheral
#[derive(Debug)]
pub struct Rcc {
    rcc: RCC,
    sysclk: KilohertzU32,
}

impl Rcc {
    /// Enable HSI48.
    /// It is only needed when USB is active.
    #[cfg(feature = "stm32g0b1")]
    pub fn enable_hsi48(&self) {
        self.rcc.cr().modify(|_, w| w.hsi48on().enabled());
        while self.rcc.cr().read().hsi48rdy().is_not_ready() {}
    }

    /// Disable HSI48.
    #[cfg(feature = "stm32g0b1")]
    pub fn disable_hsi48(&self) {
        self.rcc.cr().modify(|_, w| w.hsi48on().disabled());
    }

    /// Get system clock frequency.
    pub fn sysclk(&self) -> KilohertzU32 {
        self.sysclk
    }
}
