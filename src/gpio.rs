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
    // AF8 = 8,
    // AF9 = 9,
    // AF10 = 10,
}

macro_rules! gpio_af {
    ($ENUM:ty) => {
        impl From<AltFunction> for $ENUM {
            fn from(af: AltFunction) -> Self {
                match af {
                    AltFunction::AF0 => <$ENUM>::Af0,
                    AltFunction::AF1 => <$ENUM>::Af1,
                    AltFunction::AF2 => <$ENUM>::Af2,
                    AltFunction::AF3 => <$ENUM>::Af3,
                    AltFunction::AF4 => <$ENUM>::Af4,
                    AltFunction::AF5 => <$ENUM>::Af5,
                    AltFunction::AF6 => <$ENUM>::Af6,
                    AltFunction::AF7 => <$ENUM>::Af7,
                    // AltFunction::AF8 => <$ENUM>::Af8,
                    // AltFunction::AF9 => <$ENUM>::Af9,
                    // AltFunction::AF10 => <$ENUM>::Af10,
                }
            }
        }
    };
}

gpio_af!(crate::pac::gpioa::afrl::AFSEL0);
gpio_af!(crate::pac::gpioa::afrh::AFSEL8);
gpio_af!(crate::pac::gpiob::afrl::AFSEL0);
gpio_af!(crate::pac::gpiob::afrh::AFSEL8);

mod marker {
    // Marker trait for allowed alternate function values
    pub trait AF {
        const AF_A_L: crate::pac::gpioa::afrl::AFSEL0;
        const AF_A_H: crate::pac::gpioa::afrh::AFSEL8;
        const AF_B_L: crate::pac::gpiob::afrl::AFSEL0;
        const AF_B_H: crate::pac::gpiob::afrh::AFSEL8;
    }

    macro_rules! impl_af {
        ($N:literal, $AF:ident) => {
            impl AF for super::Alternate<$N> {
                const AF_A_L: crate::pac::gpioa::afrl::AFSEL0 =
                    crate::pac::gpioa::afrl::AFSEL0::$AF;
                const AF_A_H: crate::pac::gpioa::afrh::AFSEL8 =
                    crate::pac::gpioa::afrh::AFSEL8::$AF;
                const AF_B_L: crate::pac::gpiob::afrl::AFSEL0 =
                    crate::pac::gpiob::afrl::AFSEL0::$AF;
                const AF_B_H: crate::pac::gpiob::afrh::AFSEL8 =
                    crate::pac::gpiob::afrh::AFSEL8::$AF;
            }
        };
    }

    impl_af!(0, Af0);
    impl_af!(1, Af1);
    impl_af!(2, Af2);
    impl_af!(3, Af3);
    impl_af!(4, Af4);
    impl_af!(5, Af5);
    impl_af!(6, Af6);
    impl_af!(7, Af7);

    // Marker trait for modes able to generate interrupts
    pub trait Interruptable {}

    impl<MODE> Interruptable for super::Input<MODE> {}
    impl<MODE> Interruptable for super::Output<MODE> {}
}

macro_rules! gpio_common {
    ($GPIO:ident, $port:ident, [$($PXi:ident: (
        $i:literal, $default_mode:ty,
        $afreg:ident, $afval:ident),)+]) => {
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
                    rb.$afreg().modify(|_, w| w.[<afrel $i>]().variant(<Alternate<N>>::$afval));
                    rb.moder().modify(|_, w| w.[<moder $i>]().alternate());
                    $PXi { _mode: PhantomData }
                }

                pub(crate) fn set_alternate_function_mode(&self, mode: AltFunction) {
                    let rb = unsafe { &(*$GPIO::ptr()) };
                    rb.$afreg().modify(|_, w| w.[<afrel $i>]().variant(mode.into()));
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
                        Ok(rb.bsrr().write(|w| w.[<bs $i>]().set_bit()))
                    }
                }

                fn set_low(&mut self) -> Result<(), Self::Error> {
                    unsafe {
                        let rb =  &(*$GPIO::ptr());
                        Ok(rb.bsrr().write(|w| w.[<br $i>]().set_bit()))
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
            $i:literal, $default_mode:ty,
            $afreg:ident, $afval:ident),)+]) => {
    paste! {
        pub mod [<$GPIO:lower>] {
            gpio_common!($GPIO, $port, [$($PXi: ($i, $default_mode,$afreg, $afval),)+]);
        }
    }
    };
    ($GPIO:ident, $port:ident, [$($PXi:ident: (
        $i:literal, $default_mode:ty,
        $afreg:ident, $afval:ident,
        $muxreg:ident, $muxbits:ident),)+]) => {
    paste! {
        pub mod [<$GPIO:lower>] {
            gpio_common!($GPIO, $port, [$($PXi: ($i, $default_mode,$afreg, $afval),)+]);
            $(
                impl<MODE> $PXi<MODE> where MODE: Interruptable {
                    pub fn make_interrupt_source(&mut self, exti: &mut EXTI) {
                        exti.$muxreg().modify(|_, w| w.$muxbits().$port());
                    }

                    #[cfg(feature = "stm32g071")]
                    pub fn trigger_on_edge(&mut self, edge: SignalEdge, exti: &mut EXTI) {
                        match edge {
                            SignalEdge::Rising => exti.rtsr1().modify(|_, w| w.[<tr $i>]().enabled()),
                            SignalEdge::Falling => exti.ftsr1().modify(|_, w| w.[<tr $i>]().enabled()),
                            SignalEdge::Both => {
                                exti.rtsr1().modify(|_, w| w.[<tr $i>]().enabled());
                                exti.ftsr1().modify(|_, w| w.[<tr $i>]().enabled());
                            }
                        }
                    }

                    #[cfg(feature = "stm32g0b1")]
                    pub fn trigger_on_edge(&mut self, edge: SignalEdge, exti: &mut EXTI) {
                        match edge {
                            SignalEdge::Rising => exti.rtsr1().modify(|_, w| w.[<rt $i>]().enabled()),
                            SignalEdge::Falling => exti.ftsr1().modify(|_, w| w.[<ft $i>]().enabled()),
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
    PA0:  (0,  Analog, afrl, AF_A_L, exticr1, exti0_7  ),
    PA1:  (1,  Analog, afrl, AF_A_L, exticr1, exti8_15 ),
    PA2:  (2,  Analog, afrl, AF_A_L, exticr1, exti16_23),
    PA3:  (3,  Analog, afrl, AF_A_L, exticr1, exti24_31),
    PA4:  (4,  Analog, afrl, AF_A_L, exticr2, exti0_7  ),
    PA5:  (5,  Analog, afrl, AF_A_L, exticr2, exti8_15 ),
    PA6:  (6,  Analog, afrl, AF_A_L, exticr2, exti16_23),
    PA7:  (7,  Analog, afrl, AF_A_L, exticr2, exti24_31),
    PA8:  (8,  Analog, afrh, AF_A_H, exticr3, exti0_7  ),
    PA9:  (9,  Analog, afrh, AF_A_H, exticr3, exti8_15 ),
    PA10: (10, Analog, afrh, AF_A_H, exticr3, exti16_23),
    PA11: (11, Analog, afrh, AF_A_H, exticr3, exti24_31),
    PA12: (12, Analog, afrh, AF_A_H, exticr4, exti0_7  ),
    PA13: (13, Alternate<0>, afrh, AF_A_H, exticr4, exti8_15 ),
    PA14: (14, Alternate<0>, afrh, AF_A_H, exticr4, exti16_23),
    PA15: (15, Analog, afrh, AF_A_H, exticr4, exti24_31),
]);

gpio!(GPIOB, pb, [
    // Pin: (pin, default_mode, bits...)
    PB0:  (0,  Analog, afrl, AF_B_L, exticr1, exti0_7  ),
    PB1:  (1,  Analog, afrl, AF_B_L, exticr1, exti8_15 ),
    PB2:  (2,  Analog, afrl, AF_B_L, exticr1, exti16_23),
    PB3:  (3,  Analog, afrl, AF_B_L, exticr1, exti24_31),
    PB4:  (4,  Analog, afrl, AF_B_L, exticr2, exti0_7  ),
    PB5:  (5,  Analog, afrl, AF_B_L, exticr2, exti8_15 ),
    PB6:  (6,  Analog, afrl, AF_B_L, exticr2, exti16_23),
    PB7:  (7,  Analog, afrl, AF_B_L, exticr2, exti24_31),
    PB8:  (8,  Analog, afrh, AF_B_H, exticr3, exti0_7  ),
    PB9:  (9,  Analog, afrh, AF_B_H, exticr3, exti8_15 ),
    PB10: (10, Analog, afrh, AF_B_H, exticr3, exti16_23),
    PB11: (11, Analog, afrh, AF_B_H, exticr3, exti24_31),
    PB12: (12, Analog, afrh, AF_B_H, exticr4, exti0_7  ),
    PB13: (13, Analog, afrh, AF_B_H, exticr4, exti8_15 ),
    PB14: (14, Analog, afrh, AF_B_H, exticr4, exti16_23),
    PB15: (15, Analog, afrh, AF_B_H, exticr4, exti24_31),
]);

gpio!(GPIOC, pc, [
    // Pin: (pin, default_mode, bits...)
    PC0:  (0,  Analog, afrl, AF_B_L, exticr1, exti0_7  ),
    PC1:  (1,  Analog, afrl, AF_B_L, exticr1, exti8_15 ),
    PC2:  (2,  Analog, afrl, AF_B_L, exticr1, exti16_23),
    PC3:  (3,  Analog, afrl, AF_B_L, exticr1, exti24_31),
    PC4:  (4,  Analog, afrl, AF_B_L, exticr2, exti0_7  ),
    PC5:  (5,  Analog, afrl, AF_B_L, exticr2, exti8_15 ),
    PC6:  (6,  Analog, afrl, AF_B_L, exticr2, exti16_23),
    PC7:  (7,  Analog, afrl, AF_B_L, exticr2, exti24_31),
    PC8:  (8,  Analog, afrh, AF_B_H, exticr3, exti0_7  ),
    PC9:  (9,  Analog, afrh, AF_B_H, exticr3, exti8_15 ),
    PC10: (10, Analog, afrh, AF_B_H, exticr3, exti16_23),
    PC11: (11, Analog, afrh, AF_B_H, exticr3, exti24_31),
    PC12: (12, Analog, afrh, AF_B_H, exticr4, exti0_7  ),
    PC13: (13, Analog, afrh, AF_B_H, exticr4, exti8_15 ),
    PC14: (14, Analog, afrh, AF_B_H, exticr4, exti16_23),
    PC15: (15, Analog, afrh, AF_B_H, exticr4, exti24_31),
]);

gpio!(GPIOD, pd, [
    // Pin: (pin, default_mode, bits...)
    PD0:  (0,  Analog, afrl, AF_B_L, exticr1, exti0_7  ),
    PD1:  (1,  Analog, afrl, AF_B_L, exticr1, exti8_15 ),
    PD2:  (2,  Analog, afrl, AF_B_L, exticr1, exti16_23),
    PD3:  (3,  Analog, afrl, AF_B_L, exticr1, exti24_31),
    PD4:  (4,  Analog, afrl, AF_B_L, exticr2, exti0_7  ),
    PD5:  (5,  Analog, afrl, AF_B_L, exticr2, exti8_15 ),
    PD6:  (6,  Analog, afrl, AF_B_L, exticr2, exti16_23),
    PD7:  (7,  Analog, afrl, AF_B_L, exticr2, exti24_31),
    PD8:  (8,  Analog, afrh, AF_B_H, exticr3, exti0_7  ),
    PD9:  (9,  Analog, afrh, AF_B_H, exticr3, exti8_15 ),
    PD10: (10, Analog, afrh, AF_B_H, exticr3, exti16_23),
    PD11: (11, Analog, afrh, AF_B_H, exticr3, exti24_31),
    PD12: (12, Analog, afrh, AF_B_H, exticr4, exti0_7  ),
    PD13: (13, Analog, afrh, AF_B_H, exticr4, exti8_15 ),
    PD14: (14, Analog, afrh, AF_B_H, exticr4, exti16_23),
    PD15: (15, Analog, afrh, AF_B_H, exticr4, exti24_31),
]);

// GPIOE can't be used as EXTI interrupt source
#[cfg(feature = "stm32g0b1")]
gpio!(GPIOE, pe, [
    // Pin: (pin, default_mode, bits...)
    PE0:  (0,  Analog, afrl, AF_B_L),
    PE1:  (1,  Analog, afrl, AF_B_L),
    PE2:  (2,  Analog, afrl, AF_B_L),
    PE3:  (3,  Analog, afrl, AF_B_L),
    PE4:  (4,  Analog, afrl, AF_B_L),
    PE5:  (5,  Analog, afrl, AF_B_L),
    PE6:  (6,  Analog, afrl, AF_B_L),
    PE7:  (7,  Analog, afrl, AF_B_L),
    PE8:  (8,  Analog, afrh, AF_B_H),
    PE9:  (9,  Analog, afrh, AF_B_H),
    PE10: (10, Analog, afrh, AF_B_H),
    PE11: (11, Analog, afrh, AF_B_H),
    PE12: (12, Analog, afrh, AF_B_H),
    PE13: (13, Analog, afrh, AF_B_H),
    PE14: (14, Analog, afrh, AF_B_H),
    PE15: (15, Analog, afrh, AF_B_H),
]);
