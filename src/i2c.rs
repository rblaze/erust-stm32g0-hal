use embedded_hal::i2c;

use crate::gpio::gpioa::*;
use crate::gpio::gpiob::*;
use crate::gpio::{AltFunction, OpenDrain, Output};
use crate::pac::{I2C1, I2C2};
use crate::rcc::{Rcc, ResetEnable};

#[cfg(feature = "stm32g0b1")]
use crate::gpio::gpioc::*;
#[cfg(feature = "stm32g0b1")]
use crate::pac::I2C3;

/// Trait for pins that can be used as I2C data
pub trait SdaPin<I2C> {
    fn setup(&self);
}

/// Trait for pins that can be used as I2C clock
pub trait SclPin<I2C> {
    fn setup(&self);
}

/// I2C bus configuration
#[derive(Debug, Copy, Clone)]
pub struct Config {
    analog_filter: bool,
    digital_filter: u8,
    prescaler: u8,
    scldel: u8,
    scadel: u8,
    scl_h: u8,
    scl_l: u8,
}

impl Config {
    /// Creates new I2C configuration from cubemx timings
    pub fn from_cubemx(enable_analog_filter: bool, digital_filter: u8, cubemx_bits: u32) -> Self {
        Self {
            analog_filter: enable_analog_filter,
            digital_filter,
            prescaler: ((cubemx_bits & 0xF000_0000) >> 28) as u8,
            scldel: ((cubemx_bits & 0x00F0_0000) >> 20) as u8,
            scadel: ((cubemx_bits & 0x000F_0000) >> 16) as u8,
            scl_h: ((cubemx_bits & 0x0000_FF00) >> 8) as u8,
            scl_l: (cubemx_bits & 0x0000_00FF) as u8,
        }
    }

    /// Programs peripheral
    fn write_timings<'a>(
        &self,
        w: &'a mut super::pac::i2c1::timingr::W,
    ) -> &'a mut super::pac::i2c1::timingr::W {
        w.presc()
            .set(self.prescaler)
            .sdadel()
            .set(self.scadel)
            .scldel()
            .set(self.scldel)
            .sclh()
            .set(self.scl_h)
            .scll()
            .set(self.scl_l)
    }
}

/// Extension trait to create I2C bus from a raw device
pub trait I2cExt<I2C> {
    fn constrain<SDA, SCL>(self, sda: SDA, scl: SCL, config: &Config, rcc: &Rcc) -> I2c<I2C>
    where
        SDA: SdaPin<I2C>,
        SCL: SclPin<I2C>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[non_exhaustive]
/// I2C errors
pub enum Error {
    ArbitrationLost,
    Bus,
    IncorrectFrameSize,
    Nack(i2c::NoAcknowledgeSource),
    Overrun,
}

impl i2c::Error for Error {
    fn kind(&self) -> i2c::ErrorKind {
        match self {
            Error::ArbitrationLost => i2c::ErrorKind::ArbitrationLoss,
            Error::Bus => i2c::ErrorKind::Bus,
            Error::IncorrectFrameSize => i2c::ErrorKind::Other,
            Error::Nack(source) => i2c::ErrorKind::NoAcknowledge(*source),
            Error::Overrun => i2c::ErrorKind::Overrun,
        }
    }
}

#[derive(Debug, PartialEq, Eq, PartialOrd, Ord)]
enum TransferState {
    InProgress,
    LastByte,
}

#[derive(Debug)]
/// I2C bus
pub struct I2c<I2C> {
    i2c: I2C,
}

impl<I2C> i2c::ErrorType for I2c<I2C> {
    type Error = Error;
}

macro_rules! i2c {
    ($I2C:ident, sda: [$(($PSDA:ty, $FSDA:ident),)+ ], scl: [$(($PSCL:ty, $FSCL:ident),)+ ]) => {
        $(
            impl SclPin<$I2C> for $PSCL {
                fn setup(&self) {
                    self.set_alternate_function_mode(AltFunction::$FSCL);
                }
            }
        )+

        $(
            impl SdaPin<$I2C> for $PSDA {
                fn setup(&self) {
                    self.set_alternate_function_mode(AltFunction::$FSDA);
                }
            }
        )+

        impl I2cExt<$I2C> for $I2C {
            fn constrain<SDA, SCL>(self, sda: SDA, scl: SCL, config: &Config, rcc: &Rcc) -> I2c<$I2C>
            where
                SDA: SdaPin<$I2C>,
                SCL: SclPin<$I2C>,
            {
                sda.setup();
                scl.setup();

                $I2C::enable(rcc);
                $I2C::reset(rcc);

                self.cr1().modify(|_, w| {
                    w.anfoff()
                        .bit(config.analog_filter)
                        .dnf()
                        .set(config.digital_filter)
                });

                self.timingr().modify(|_, w| config.write_timings(w));
                self.cr1().modify(|_, w| w.pe().enabled());

                I2c::<$I2C>::new(self)
            }
        }

        impl I2c<$I2C> {
            fn new(i2c: $I2C) -> Self {
                Self { i2c }
            }

            pub fn reset(&self) {
                self.i2c.cr1().modify(|_, w| w.pe().disabled());
                // When cleared, PE must be kept low for at least three APB clock cycles.
                cortex_m::asm::delay(100);
                self.i2c.cr1().modify(|_, w| w.pe().enabled());
            }

            // TODO: this code probably doesn't handle "reload" transfers
            // (Read/Read and Write/Write op pairs) correctly.
            fn check_errors(
                &self,
                transfer_kind: i2c::NoAcknowledgeSource,
            ) -> Result<(), Error> {
                let status = self.i2c.isr().read();
                if status.ovr().is_overrun() {
                    self.i2c.icr().write(|w| w.ovrcf().clear());
                    return Err(Error::Overrun);
                }
                if status.arlo().is_lost() {
                    self.i2c.icr().write(|w| w.ovrcf().clear());
                    return Err(Error::ArbitrationLost);
                }
                if status.berr().is_error() {
                    self.i2c.icr().write(|w| w.berrcf().clear());
                    return Err(Error::Bus);
                }
                if status.nackf().is_nack() {
                    self.i2c.icr().write(|w| w.nackcf().clear());
                    return Err(Error::Nack(transfer_kind));
                }

                Ok(())
            }

            fn wait_for_txis(
                &self,
                transfer_kind: i2c::NoAcknowledgeSource,
                state: TransferState,
            ) -> Result<(), Error> {
                loop {
                    match state {
                        TransferState::InProgress => {
                            if self.i2c.isr().read().txis().is_empty() {
                                break;
                            }
                        }
                        TransferState::LastByte => {
                            if self.i2c.isr().read().stopf().is_stop()
                                    || self.i2c.isr().read().tc().is_complete() {
                                break;
                            }
                        }
                    }
                    self.check_errors(transfer_kind)?;
                }

                Ok(())
            }

            fn send_buf(&self, wrbuf: &[u8]) -> Result<(), Error> {
                let mut source = i2c::NoAcknowledgeSource::Address;

                for byte in wrbuf {
                    self.wait_for_txis(source, TransferState::InProgress)?;
                    self.i2c.txdr().write(|w| w.txdata().set(*byte));

                    source = i2c::NoAcknowledgeSource::Data;
                }

                // Wait for final byte transfer
                self.wait_for_txis(source, TransferState::LastByte)?;

                Ok(())
            }

            fn wait_for_rxne(&self, transfer_kind: i2c::NoAcknowledgeSource) -> Result<(), Error> {
                loop {
                    if self.i2c.isr().read().rxne().is_not_empty() {
                        break;
                    }
                    self.check_errors(transfer_kind)?;

                    if self.i2c.isr().read().stopf().is_stop() {
                        // Recheck status register
                        if self.i2c.isr().read().rxne().is_not_empty() {
                            break;
                        }

                        self.i2c.icr().write(|w| w.stopcf().clear());
                        return Err(Error::IncorrectFrameSize);
                    }
                }

                Ok(())
            }

            fn read_buf(&self, rdbuf: &mut [u8]) -> Result<(), Error> {
                let mut source = i2c::NoAcknowledgeSource::Address;

                for byte in rdbuf {
                    self.wait_for_rxne(source)?;
                    *byte = self.i2c.rxdr().read().rxdata().bits();

                    source = i2c::NoAcknowledgeSource::Unknown;
                }

                Ok(())
            }
        }

        impl i2c::I2c<i2c::SevenBitAddress> for I2c<$I2C> {
            fn transaction(
                &mut self,
                address: i2c::SevenBitAddress,
                operations: &mut [i2c::Operation<'_>],
            ) -> Result<(), Self::Error> {
                // Wait for bus to become free.
                while self.i2c.isr().read().busy().is_busy() {}

                // Set address
                self.i2c.cr2().write(|w| {
                    w.add10()
                        .bit7()
                        .sadd()
                        .set((address as u16) << 1)
                        .autoend()
                        .software()
                });

                // Perform all operations but last.
                for i in 0..operations.len() - 1 {
                    let next_op_is_same = match (&operations[i], &operations[i + 1]) {
                        (i2c::Operation::Read(_), i2c::Operation::Read(_)) => true,
                        (i2c::Operation::Read(_), i2c::Operation::Write(_)) => false,
                        (i2c::Operation::Write(_), i2c::Operation::Read(_)) => false,
                        (i2c::Operation::Write(_), i2c::Operation::Write(_)) => true,
                    };

                    match &mut operations[i] {
                        i2c::Operation::Read(rdbuf) => {
                            // TODO: handle buffers longer than 255 bytes
                            assert!(rdbuf.len() < 256);
                            self.i2c.cr2().modify(|_, w| {
                                w.rd_wrn()
                                    .read()
                                    .reload()
                                    .bit(next_op_is_same)
                                    .nbytes()
                                    .set(rdbuf.len() as u8)
                                    .start()
                                    .set_bit()
                            });
                            self.read_buf(rdbuf)?;
                        }
                        i2c::Operation::Write(wrbuf) => {
                            // TODO: handle buffers longer than 255 bytes
                            assert!(wrbuf.len() < 256);
                            self.i2c.cr2().modify(|_, w| {
                                w.rd_wrn()
                                    .write()
                                    .reload()
                                    .bit(next_op_is_same)
                                    .nbytes()
                                    .set(wrbuf.len() as u8)
                                    .start()
                                    .set_bit()
                            });
                            self.send_buf(wrbuf)?;
                        }
                    }
                }

                match operations.last_mut() {
                    Some(i2c::Operation::Read(rdbuf)) => {
                        self.i2c.cr2().modify(|_, w| {
                            w.rd_wrn()
                                .read()
                                .reload()
                                .completed()
                                .autoend()
                                .automatic()
                                .nbytes()
                                .set(rdbuf.len() as u8)
                                .start()
                                .set_bit()
                        });
                        self.read_buf(rdbuf)?;
                    }
                    Some(i2c::Operation::Write(wrbuf)) => {
                        self.i2c.cr2().modify(|_, w| {
                            w.rd_wrn()
                                .write()
                                .reload()
                                .completed()
                                .autoend()
                                .automatic()
                                .nbytes()
                                .set(wrbuf.len() as u8)
                                .start()
                                .set_bit()
                        });
                        self.send_buf(wrbuf)?;
                    }
                    None => { /* empty operations list, do nothing */ }
                }

                while self.i2c.isr().read().stopf().is_no_stop() {}
                self.i2c.icr().write(|w| w.stopcf().clear());

                self.i2c.cr2().reset();

                Ok(())
            }
        }
    };
}

i2c!(I2C1,
    sda: [
        (PA10<Output<OpenDrain>>, AF6),
        (PB7<Output<OpenDrain>>, AF6),
        (PB9<Output<OpenDrain>>, AF6),
    ],
    scl: [
        (PA9<Output<OpenDrain>>, AF6),
        (PB6<Output<OpenDrain>>, AF6),
        (PB8<Output<OpenDrain>>, AF6),
    ]
);

#[cfg(feature = "stm32g071")]
i2c!(I2C2,
    sda: [
        (PA12<Output<OpenDrain>>, AF6),
        (PB11<Output<OpenDrain>>, AF6),
        (PB14<Output<OpenDrain>>, AF6),
    ],
    scl: [
        (PA11<Output<OpenDrain>>, AF6),
        (PB10<Output<OpenDrain>>, AF6),
        (PB13<Output<OpenDrain>>, AF6),
    ]
);

#[cfg(feature = "stm32g0b1")]
i2c!(I2C2,
    sda: [
        (PA6<Output<OpenDrain>>, AF8),
        (PA10<Output<OpenDrain>>, AF8),
        (PA12<Output<OpenDrain>>, AF6),
        (PB4<Output<OpenDrain>>, AF8),
        (PB11<Output<OpenDrain>>, AF6),
        (PB14<Output<OpenDrain>>, AF6),
    ],
    scl: [
        (PA7<Output<OpenDrain>>, AF8),
        (PA9<Output<OpenDrain>>, AF8),
        (PA11<Output<OpenDrain>>, AF6),
        (PB3<Output<OpenDrain>>, AF8),
        (PB10<Output<OpenDrain>>, AF6),
        (PB13<Output<OpenDrain>>, AF6),
    ]
);

#[cfg(feature = "stm32g0b1")]
i2c!(I2C3,
    sda: [
        (PA6<Output<OpenDrain>>, AF9),
        (PB4<Output<OpenDrain>>, AF6),
        (PC1<Output<OpenDrain>>, AF6),
    ],
    scl: [
        (PA7<Output<OpenDrain>>, AF9),
        (PB3<Output<OpenDrain>>, AF6),
        (PC0<Output<OpenDrain>>, AF6),
    ]
);
