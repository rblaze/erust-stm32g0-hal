#![no_std]
#![deny(unsafe_code)]

pub mod adc;
pub mod exti;
pub mod gpio;
pub mod i2c;
pub mod rcc;
pub mod timer;

#[cfg(feature = "stm32g071")]
pub use stm32g0::stm32g071 as pac;

#[cfg(feature = "stm32g0b1")]
pub use stm32g0::stm32g0b1 as pac;
