use super::Rcc;
use crate::pac::{LPTIM1, LPTIM2};

/// LPTIM clock selector
pub use crate::pac::rcc::ccipr::LPTIM1SEL as LptimClock;

pub trait LptimClockExt {
    fn set_clock(clock: LptimClock, rcc: &Rcc);
}

macro_rules! lptim_rcc {
    ($TIM:ident, $clock:ident) => {
        impl LptimClockExt for $TIM {
            fn set_clock(clock: LptimClock, rcc: &Rcc) {
                rcc.rcc.ccipr().modify(|_, w| w.$clock().variant(clock));
            }
        }
    };
}

lptim_rcc!(LPTIM1, lptim1sel);
lptim_rcc!(LPTIM2, lptim2sel);
