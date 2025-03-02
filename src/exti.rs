#![allow(unsafe_code, unused)]

use crate::gpio::SignalEdge;
use crate::pac::EXTI;

/// EXTI trigger event
#[derive(Debug, Eq, PartialEq, PartialOrd, Clone, Copy)]
pub enum Event {
    Gpio0 = 0,
    Gpio1 = 1,
    Gpio2 = 2,
    Gpio3 = 3,
    Gpio4 = 4,
    Gpio5 = 5,
    Gpio6 = 6,
    Gpio7 = 7,
    Gpio8 = 8,
    Gpio9 = 9,
    Gpio10 = 10,
    Gpio11 = 11,
    Gpio12 = 12,
    Gpio13 = 13,
    Gpio14 = 14,
    Gpio15 = 15,
    Pvd = 16,
    Comp1 = 17,
    Comp2 = 18,
    Rtc = 19,
    #[cfg(feature = "stm32g0b1")]
    Comp3 = 20,
    Tamp = 21,
    #[cfg(feature = "stm32g0b1")]
    I2c2 = 22,
    I2c1 = 23,
    #[cfg(feature = "stm32g0b1")]
    Usart3 = 24,
    Usart1 = 25,
    Usart2 = 26,
    Cec = 27,
    LpUart1 = 28,
    LpTim1 = 29,
    LpTim2 = 30,
    LseCss = 31,
    Ucpd1 = 32,
    Ucpd2 = 33,
    #[cfg(feature = "stm32g0b1")]
    Vddio2 = 34,
    #[cfg(feature = "stm32g0b1")]
    LpUart2 = 35,
    #[cfg(feature = "stm32g0b1")]
    Usb = 36,
}

/// Extension trait for EXTI
pub trait ExtiExt {
    /// Interrupt enable
    fn listen(&self, ev: Event);
    /// Interrupt disable
    fn unlisten(&self, ev: Event);
    /// Indicate if the interrupt is currently pending
    fn is_pending(&self, ev: Event, edge: SignalEdge) -> bool;
    /// Clear interrupt pending flag
    fn unpend(&self, ev: Event);
}

impl ExtiExt for EXTI {
    fn listen(&self, ev: Event) {
        let line = ev as u8;

        unsafe {
            match line {
                0..=31 => self.imr1().modify(|r, w| w.bits(r.bits() | (1 << line))),
                32..=36 => self
                    .imr2()
                    .modify(|r, w| w.bits(r.bits() | (1 << (line - 32)))),
                _ => unreachable!(),
            };
        }
    }

    fn unlisten(&self, ev: Event) {
        self.unpend(ev);

        let line = ev as u8;
        unsafe {
            match line {
                0..=31 => self.imr1().modify(|r, w| w.bits(r.bits() & !(1 << line))),
                32..=36 => self
                    .imr2()
                    .modify(|r, w| w.bits(r.bits() & !(1 << (line - 32)))),
                _ => unreachable!(),
            };
        }
    }

    fn is_pending(&self, ev: Event, edge: SignalEdge) -> bool {
        let line = ev as u8;

        match line {
            0..=31 => match edge {
                SignalEdge::Rising => self.rpr1().read().bits() & (1 << line) != 0,
                SignalEdge::Falling => self.fpr1().read().bits() & (1 << line) != 0,
                SignalEdge::Both => {
                    self.rpr1().read().bits() & (1 << line) != 0
                        || self.fpr1().read().bits() & (1 << line) != 0
                }
            },
            #[cfg(feature = "stm32g0b1")]
            32..=36 => match edge {
                SignalEdge::Rising => self.rpr2().read().bits() & (1 << (line - 32)) != 0,
                SignalEdge::Falling => self.fpr2().read().bits() & (1 << (line - 32)) != 0,
                SignalEdge::Both => {
                    self.rpr2().read().bits() & (1 << (line - 32)) != 0
                        || self.fpr2().read().bits() & (1 << (line - 32)) != 0
                }
            },
            _ => false,
        }
    }

    fn unpend(&self, ev: Event) {
        let line = ev as u8;

        unsafe {
            match line {
                0..=18 | 20 => {
                    self.rpr1().modify(|_, w| w.bits(1 << line));
                    self.fpr1().modify(|_, w| w.bits(1 << line));
                }
                #[cfg(feature = "stm32g0b1")]
                34 => {
                    self.rpr2().modify(|_, w| w.bits(1 << (line - 32)));
                    self.fpr2().modify(|_, w| w.bits(1 << (line - 32)));
                }
                _ => unreachable!(),
            }
        }
    }
}
