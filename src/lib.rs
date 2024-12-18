#![no_std]
#![deny(unsafe_code)]

// TODO: cleanup HAL modules
// 1. use constrain() for creating peripherals via extension traits
// 2. except for GPIO, which is canonically split()
// 3. and except for timers, which can be turned into counter, pwm, etc

pub mod adc;
pub mod exti;
pub mod gpio;
pub mod i2c;
pub mod pwr;
pub mod rcc;
pub mod timer;
#[cfg(feature = "stm32g0b1")]
pub mod usb;

#[cfg(feature = "stm32g071")]
pub use stm32g0::stm32g071 as pac;

#[cfg(feature = "stm32g0b1")]
pub use stm32g0::stm32g0b1 as pac;
