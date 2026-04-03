//! LED driver for PC6
//!
//! Simple wrapper around Embassy GPIO output.

use embassy_stm32::gpio::{Level, Output, Speed, AnyPin};
use embassy_stm32::Peri;

/// LED driver wrapping a GPIO output pin
pub struct Led {
    pin: Output<'static>,
}

impl Led {
    /// Create new LED driver from a GPIO pin
    pub fn new(pin: Peri<'static, AnyPin>) -> Self {
        Self {
            pin: Output::new(pin, Level::Low, Speed::Low),
        }
    }

    /// Turn LED on (set high)
    pub fn on(&mut self) {
        self.pin.set_high();
    }

    /// Turn LED off (set low)
    pub fn off(&mut self) {
        self.pin.set_low();
    }

    /// Toggle LED state
    pub fn toggle(&mut self) {
        self.pin.toggle();
    }

    /// Check if LED is currently on
    pub fn is_on(&self) -> bool {
        self.pin.is_set_high()
    }
}