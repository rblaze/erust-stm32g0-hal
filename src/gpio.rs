#![allow(unused)]
use core::marker::PhantomData;

use paste::paste;

use crate::pac::{EXTI, GPIOA, GPIOB, GPIOC, GPIOD};
use crate::rcc::{Rcc, ResetEnable};

#[cfg(feature = "stm32g0b1")]
use crate::pac::GPIOE;

/// Extension trait to split a GPIO peripheral in independent pins and registers
pub trait GpioExt {
    /// The parts to split the GPIO into
    type Parts;

    /// Splits the GPIO block into independent pins and registers
    fn split(self, rcc: &Rcc) -> Self::Parts;
}

/// Trigger edge
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum SignalEdge {
    Rising,
    Falling,
    Both,
}

/// Input mode
pub struct Input<MODE> {
    _mode: PhantomData<MODE>,
}

/// Floating input
pub struct Floating;

/// Pulled down input
pub struct PullDown;

/// Pulled up input
pub struct PullUp;

/// Output mode
pub struct Output<MODE> {
    _mode: PhantomData<MODE>,
}

/// Open drain input or output
pub struct OpenDrain;

/// Push pull output
pub struct PushPull;

/// Analog mode
pub struct Analog;

/// Alternate function mode
pub struct Alternate<const N: u8>;

/// Alternate function numbers
pub(crate) enum AltFunction {
    AF0 = 0,
    AF1 = 1,
    AF2 = 2,
    AF3 = 3,
    AF4 = 4,
    AF5 = 5,
    AF6 = 6,
    AF7 = 7,
    AF8 = 8,
    AF9 = 9,
    AF10 = 10,
}

mod marker {
    use super::Alternate;

    // Marker trait for allowed alternate function values
    pub trait AF {}

    impl AF for Alternate<0> {}
    impl AF for Alternate<1> {}
    impl AF for Alternate<2> {}
    impl AF for Alternate<3> {}
    impl AF for Alternate<4> {}
    impl AF for Alternate<5> {}
    impl AF for Alternate<6> {}
    impl AF for Alternate<7> {}
    impl AF for Alternate<8> {}
    impl AF for Alternate<9> {}
    impl AF for Alternate<10> {}

    // Marker trait for modes able to generate interrupts
    pub trait Interruptable {}

    impl<MODE> Interruptable for super::Input<MODE> {}
    impl<MODE> Interruptable for super::Output<MODE> {}
}

macro_rules! gpio_common {
    ($GPIO:ident, $port:ident, [$($PXi:ident: (
        $i:literal, $default_mode:ty, $afreg:ident),)+]) => {
    use paste::paste;
    paste! {
        use super::{GpioExt, ResetEnable, Rcc, EXTI, $GPIO};
        use super::{Output, PushPull, OpenDrain};
        use super::{Input, Floating, PullUp, PullDown};
        use super::{Analog, Alternate, AltFunction};
        use super::SignalEdge;
        use super::marker::{AF, Interruptable};

        use core::marker::PhantomData;

        impl GpioExt for $GPIO {
            type Parts = Parts;

            fn split(self, rcc: &Rcc) -> Self::Parts {
                $GPIO::enable(rcc);
                $GPIO::reset(rcc);

                Self::Parts{
                    $(
                        [<$PXi:lower>]: $PXi { _mode: PhantomData },
                    )+
                }
            }
        }

        pub struct Parts {
            $(
                pub [<$PXi:lower>]: $PXi<$default_mode>,
            )+
        }

        $(
            pub struct $PXi<MODE> {
                _mode: PhantomData<MODE>,
            }

            #[allow(unsafe_code)]
            impl<MODE> $PXi<MODE> {
                pub fn into_push_pull_output(self) -> $PXi<Output<PushPull>> {
                    let rb = unsafe { &(*$GPIO::ptr()) };
                    rb.pupdr().modify(|_, w| w.[<pupdr $i>]().floating());
                    rb.otyper().modify(|_, w| w.[<ot $i>]().push_pull());
                    rb.moder().modify(|_, w| w.[<moder $i>]().output());
                    $PXi { _mode: PhantomData }
                }

                pub fn into_open_drain_output(self) -> $PXi<Output<OpenDrain>> {
                    let rb = unsafe { &(*$GPIO::ptr()) };
                    rb.pupdr().modify(|_, w| w.[<pupdr $i>]().floating());
                    rb.otyper().modify(|_, w| w.[<ot $i>]().open_drain());
                    rb.moder().modify(|_, w| w.[<moder $i>]().output());
                    $PXi { _mode: PhantomData }
                }

                pub fn into_floating_input(self) -> $PXi<Input<Floating>> {
                    let rb = unsafe { &(*$GPIO::ptr()) };
                    rb.pupdr().modify(|_, w| w.[<pupdr $i>]().floating());
                    rb.moder().modify(|_, w| w.[<moder $i>]().input());
                    $PXi { _mode: PhantomData }
                }

                pub fn into_pulldown_input(self) -> $PXi<Input<PullDown>> {
                    let rb = unsafe { &(*$GPIO::ptr()) };
                    rb.pupdr().modify(|_, w| w.[<pupdr $i>]().pull_down());
                    rb.moder().modify(|_, w| w.[<moder $i>]().input());
                    $PXi { _mode: PhantomData }
                }

                pub fn into_pullup_input(self) -> $PXi<Input<PullUp>> {
                    let rb = unsafe { &(*$GPIO::ptr()) };
                    rb.pupdr().modify(|_, w| w.[<pupdr $i>]().pull_up());
                    rb.moder().modify(|_, w| w.[<moder $i>]().input());
                    $PXi { _mode: PhantomData }
                }

                pub fn into_analog(self) -> $PXi<Analog> {
                    let rb = unsafe { &(*$GPIO::ptr()) };
                    rb.pupdr().modify(|_, w| w.[<pupdr $i>]().floating());
                    rb.moder().modify(|_, w| w.[<moder $i>]().analog());
                    $PXi { _mode: PhantomData }
                }

                pub fn into_alternate_function<const N: u8>(self) -> $PXi<Alternate<N>>
                where Alternate<N> : AF {
                    let rb = unsafe { &(*$GPIO::ptr()) };
                    rb.$afreg().modify(|_, w| unsafe { w.[<afrel $i>]().bits(N) });
                    rb.moder().modify(|_, w| w.[<moder $i>]().alternate());
                    $PXi { _mode: PhantomData }
                }

                pub(crate) fn set_alternate_function_mode(&self, mode: AltFunction) {
                    let rb = unsafe { &(*$GPIO::ptr()) };
                    rb.$afreg().modify(|_, w| unsafe { w.[<afrel $i>]().bits(mode as u8) });
                    rb.moder().modify(|_, w| w.[<moder $i>]().alternate());
                }
            }

            impl<MODE> embedded_hal::digital::ErrorType for $PXi<MODE> {
                type Error = core::convert::Infallible;
            }

            impl<MODE> embedded_hal::digital::InputPin for $PXi<Input<MODE>> {
                #[allow(unsafe_code)]
                fn is_high(&mut self) -> Result<bool, Self::Error> {
                    let rb = unsafe { &(*$GPIO::ptr()) };
                    Ok(rb.idr().read().[<idr $i>]().is_high())
                }

                fn is_low(&mut self) -> Result<bool, Self::Error> {
                    self.is_high().map(|v| !v)
                }
            }

            #[allow(unsafe_code)]
            impl<MODE> embedded_hal::digital::OutputPin for $PXi<Output<MODE>> {
                fn set_high(&mut self) -> Result<(), Self::Error> {
                    unsafe {
                        let rb =  &(*$GPIO::ptr());
                        rb.bsrr().write(|w| w.[<bs $i>]().set_bit());
                        Ok(())
                    }
                }

                fn set_low(&mut self) -> Result<(), Self::Error> {
                    unsafe {
                        let rb =  &(*$GPIO::ptr());
                        rb.bsrr().write(|w| w.[<br $i>]().set_bit());
                        Ok(())
                    }
                }
            }

            impl<MODE> embedded_hal::digital::StatefulOutputPin for $PXi<Output<MODE>> {
                #[allow(unsafe_code)]
                fn is_set_high(&mut self) -> Result<bool, Self::Error> {
                    let rb = unsafe { &(*$GPIO::ptr()) };
                    Ok(rb.odr().read().[<odr $i>]().is_high())
                }

                fn is_set_low(&mut self) -> Result<bool, Self::Error> {
                    self.is_set_high().map(|v| !v)
                }
            }
        )+
    }
    };
}

macro_rules! gpio {
    ($GPIO:ident, $port:ident, [$($PXi:ident: (
            $i:literal, $default_mode:ty, $afreg:ident),)+]) => {
    paste! {
        pub mod [<$GPIO:lower>] {
            gpio_common!($GPIO, $port, [$($PXi: ($i, $default_mode, $afreg),)+]);
        }
    }
    };
    ($GPIO:ident, $port:ident, [$($PXi:ident: (
        $i:literal, $default_mode:ty, $afreg:ident,
        $muxreg:ident, $muxbits:ident),)+]) => {
    paste! {
        pub mod [<$GPIO:lower>] {
            gpio_common!($GPIO, $port, [$($PXi: ($i, $default_mode, $afreg),)+]);
            $(
                impl<MODE> $PXi<MODE> where MODE: Interruptable {
                    pub fn make_interrupt_source(&mut self, exti: &mut EXTI) {
                        exti.$muxreg().modify(|_, w| w.$muxbits().$port());
                    }

                    #[cfg(feature = "stm32g071")]
                    pub fn trigger_on_edge(&mut self, edge: SignalEdge, exti: &mut EXTI) {
                        match edge {
                            SignalEdge::Rising => {
                                exti.rtsr1().modify(|_, w| w.[<tr $i>]().enabled());
                            },
                            SignalEdge::Falling => {
                                exti.ftsr1().modify(|_, w| w.[<tr $i>]().enabled());
                            },
                            SignalEdge::Both => {
                                exti.rtsr1().modify(|_, w| w.[<tr $i>]().enabled());
                                exti.ftsr1().modify(|_, w| w.[<tr $i>]().enabled());
                            }
                        }
                    }

                    #[cfg(feature = "stm32g0b1")]
                    pub fn trigger_on_edge(&mut self, edge: SignalEdge, exti: &mut EXTI) {
                        match edge {
                            SignalEdge::Rising => {
                                exti.rtsr1().modify(|_, w| w.[<rt $i>]().enabled());
                            },
                            SignalEdge::Falling => {
                                exti.ftsr1().modify(|_, w| w.[<ft $i>]().enabled());
                            },
                            SignalEdge::Both => {
                                exti.rtsr1().modify(|_, w| w.[<rt $i>]().enabled());
                                exti.ftsr1().modify(|_, w| w.[<ft $i>]().enabled());
                            }
                        }
                    }
                }
            )+
        }
    }
    };
}

gpio!(GPIOA, pa, [
    // Pin: (pin, default_mode, bits...)
    PA0:  (0,  Analog, afrl, exticr1, exti0_7  ),
    PA1:  (1,  Analog, afrl, exticr1, exti8_15 ),
    PA2:  (2,  Analog, afrl, exticr1, exti16_23),
    PA3:  (3,  Analog, afrl, exticr1, exti24_31),
    PA4:  (4,  Analog, afrl, exticr2, exti0_7  ),
    PA5:  (5,  Analog, afrl, exticr2, exti8_15 ),
    PA6:  (6,  Analog, afrl, exticr2, exti16_23),
    PA7:  (7,  Analog, afrl, exticr2, exti24_31),
    PA8:  (8,  Analog, afrh, exticr3, exti0_7  ),
    PA9:  (9,  Analog, afrh, exticr3, exti8_15 ),
    PA10: (10, Analog, afrh, exticr3, exti16_23),
    PA11: (11, Analog, afrh, exticr3, exti24_31),
    PA12: (12, Analog, afrh, exticr4, exti0_7  ),
    PA13: (13, Alternate<0>, afrh, exticr4, exti8_15 ),
    PA14: (14, Alternate<0>, afrh, exticr4, exti16_23),
    PA15: (15, Analog, afrh, exticr4, exti24_31),
]);

gpio!(GPIOB, pb, [
    // Pin: (pin, default_mode, bits...)
    PB0:  (0,  Analog, afrl, exticr1, exti0_7  ),
    PB1:  (1,  Analog, afrl, exticr1, exti8_15 ),
    PB2:  (2,  Analog, afrl, exticr1, exti16_23),
    PB3:  (3,  Analog, afrl, exticr1, exti24_31),
    PB4:  (4,  Analog, afrl, exticr2, exti0_7  ),
    PB5:  (5,  Analog, afrl, exticr2, exti8_15 ),
    PB6:  (6,  Analog, afrl, exticr2, exti16_23),
    PB7:  (7,  Analog, afrl, exticr2, exti24_31),
    PB8:  (8,  Analog, afrh, exticr3, exti0_7  ),
    PB9:  (9,  Analog, afrh, exticr3, exti8_15 ),
    PB10: (10, Analog, afrh, exticr3, exti16_23),
    PB11: (11, Analog, afrh, exticr3, exti24_31),
    PB12: (12, Analog, afrh, exticr4, exti0_7  ),
    PB13: (13, Analog, afrh, exticr4, exti8_15 ),
    PB14: (14, Analog, afrh, exticr4, exti16_23),
    PB15: (15, Analog, afrh, exticr4, exti24_31),
]);

gpio!(GPIOC, pc, [
    // Pin: (pin, default_mode, bits...)
    PC0:  (0,  Analog, afrl, exticr1, exti0_7  ),
    PC1:  (1,  Analog, afrl, exticr1, exti8_15 ),
    PC2:  (2,  Analog, afrl, exticr1, exti16_23),
    PC3:  (3,  Analog, afrl, exticr1, exti24_31),
    PC4:  (4,  Analog, afrl, exticr2, exti0_7  ),
    PC5:  (5,  Analog, afrl, exticr2, exti8_15 ),
    PC6:  (6,  Analog, afrl, exticr2, exti16_23),
    PC7:  (7,  Analog, afrl, exticr2, exti24_31),
    PC8:  (8,  Analog, afrh, exticr3, exti0_7  ),
    PC9:  (9,  Analog, afrh, exticr3, exti8_15 ),
    PC10: (10, Analog, afrh, exticr3, exti16_23),
    PC11: (11, Analog, afrh, exticr3, exti24_31),
    PC12: (12, Analog, afrh, exticr4, exti0_7  ),
    PC13: (13, Analog, afrh, exticr4, exti8_15 ),
    PC14: (14, Analog, afrh, exticr4, exti16_23),
    PC15: (15, Analog, afrh, exticr4, exti24_31),
]);

gpio!(GPIOD, pd, [
    // Pin: (pin, default_mode, bits...)
    PD0:  (0,  Analog, afrl, exticr1, exti0_7  ),
    PD1:  (1,  Analog, afrl, exticr1, exti8_15 ),
    PD2:  (2,  Analog, afrl, exticr1, exti16_23),
    PD3:  (3,  Analog, afrl, exticr1, exti24_31),
    PD4:  (4,  Analog, afrl, exticr2, exti0_7  ),
    PD5:  (5,  Analog, afrl, exticr2, exti8_15 ),
    PD6:  (6,  Analog, afrl, exticr2, exti16_23),
    PD7:  (7,  Analog, afrl, exticr2, exti24_31),
    PD8:  (8,  Analog, afrh, exticr3, exti0_7  ),
    PD9:  (9,  Analog, afrh, exticr3, exti8_15 ),
    PD10: (10, Analog, afrh, exticr3, exti16_23),
    PD11: (11, Analog, afrh, exticr3, exti24_31),
    PD12: (12, Analog, afrh, exticr4, exti0_7  ),
    PD13: (13, Analog, afrh, exticr4, exti8_15 ),
    PD14: (14, Analog, afrh, exticr4, exti16_23),
    PD15: (15, Analog, afrh, exticr4, exti24_31),
]);

// GPIOE can't be used as EXTI interrupt source
#[cfg(feature = "stm32g0b1")]
gpio!(GPIOE, pe, [
    // Pin: (pin, default_mode, bits...)
    PE0:  (0,  Analog, afrl),
    PE1:  (1,  Analog, afrl),
    PE2:  (2,  Analog, afrl),
    PE3:  (3,  Analog, afrl),
    PE4:  (4,  Analog, afrl),
    PE5:  (5,  Analog, afrl),
    PE6:  (6,  Analog, afrl),
    PE7:  (7,  Analog, afrl),
    PE8:  (8,  Analog, afrh),
    PE9:  (9,  Analog, afrh),
    PE10: (10, Analog, afrh),
    PE11: (11, Analog, afrh),
    PE12: (12, Analog, afrh),
    PE13: (13, Analog, afrh),
    PE14: (14, Analog, afrh),
    PE15: (15, Analog, afrh),
]);
