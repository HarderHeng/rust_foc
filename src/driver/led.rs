use embassy_stm32::gpio::{AnyPin, Level, Output, Speed};
use embassy_stm32::Peri;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

pub type LedHandle = Mutex<CriticalSectionRawMutex, Led>;

pub struct Led {
    pin: Output<'static>,
}

impl Led {
    pub fn new(pin: Peri<'static, AnyPin>) -> Self {
        Self {
            pin: Output::new(pin, Level::Low, Speed::Low),
        }
    }

    pub fn on(&mut self) {
        self.pin.set_high();
    }

    pub fn off(&mut self) {
        self.pin.set_low();
    }

    pub fn toggle(&mut self) {
        self.pin.toggle();
    }

    pub fn is_on(&self) -> bool {
        self.pin.is_set_high()
    }
}
