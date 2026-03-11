use crate::pac::IWDG;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum IwdgPrescaler {
    Div4 = 0b000,
    Div8 = 0b001,
    Div16 = 0b010,
    Div32 = 0b011,
    Div64 = 0b100,
    Div128 = 0b101,
    Div256 = 0b110,
}

#[derive(Debug)]
pub struct Iwdg {
    iwdg: IWDG,
}

impl Iwdg {
    pub fn new(iwdg: IWDG) -> Self {
        Self { iwdg }
    }

    /// Starts the watchdog with given prescaler and reload values.
    /// reload is limited to 12 bits, max 0xFFF.
    pub fn start(&self, prescaler: IwdgPrescaler, reload: u16) {
        // Per RM0444 28.3.2
        // Enable IWDG
        self.iwdg.kr().write(|w| w.key().start());
        // Enable register access
        self.iwdg.kr().write(|w| w.key().unlock());

        // Write the prescaler
        while self.iwdg.sr().read().pvu().bit_is_set() {}
        self.iwdg.pr().write(|w| w.pr().set(prescaler as u8));

        // Write the reload register
        while self.iwdg.sr().read().rvu().bit_is_set() {}
        self.iwdg.rlr().write(|w| w.rl().set(reload));

        // Wait for registers to be updated
        while self.iwdg.sr().read().bits() != 0 {}

        // Refresh counter value
        self.feed();
    }

    /// Feeds the watchdog, resetting timer.
    pub fn feed(&self) {
        self.iwdg.kr().write(|w| w.key().feed());
    }
}
