//! Last 2 KiB flash page: append-only calibration journal (128 records).
//!
//! Save only with PWM off (Idle), never from an ISR. Identical saves are no-ops;
//! full-page reclaim can lose calibration if power fails after erase.

use core::cell::RefCell;

use cortex_m::interrupt::Mutex;
use embassy_stm32::flash::{Blocking, Flash};
use embassy_stm32::interrupt::InterruptExt;
use embassy_stm32::peripherals::FLASH;
use embassy_stm32::Peri;

pub use ::foc::nvm::Record;
use ::foc::nvm::{self, SaveOutcome, Storage, PAGE_SIZE, SLOT_SIZE};

/// Offset from `0x0800_0000`. Last page of 128 KiB.
pub const PAGE_OFF: u32 = 0x1_F800;
pub const PAGE_END: u32 = PAGE_OFF + PAGE_SIZE as u32;
const ABS: u32 = 0x0800_0000 + PAGE_OFF;

static FLASH_SLOT: Mutex<RefCell<Option<Flash<'static, Blocking>>>> =
    Mutex::new(RefCell::new(None));

pub fn init(flash: Peri<'static, FLASH>) {
    cortex_m::interrupt::free(|cs| {
        let mut slot = FLASH_SLOT.borrow(cs).borrow_mut();
        assert!(slot.is_none(), "NVM already initialized");
        *slot = Some(Flash::new_blocking(flash));
    });
}

/// Own the peripheral outside a critical section; reentrant callers get None.
struct FlashLease(Option<Flash<'static, Blocking>>);

impl FlashLease {
    fn take() -> Option<Self> {
        cortex_m::interrupt::free(|cs| {
            FLASH_SLOT
                .borrow(cs)
                .borrow_mut()
                .take()
                .map(|flash| Self(Some(flash)))
        })
    }
}

impl Drop for FlashLease {
    fn drop(&mut self) {
        cortex_m::interrupt::free(|cs| {
            *FLASH_SLOT.borrow(cs).borrow_mut() = self.0.take();
        });
    }
}

/// Avoid the current ISR during flash writes without masking executor IRQs.
struct AdcIrqGuard(bool);

impl AdcIrqGuard {
    fn new() -> Self {
        let irq = embassy_stm32::interrupt::ADC1_2;
        let was_enabled = irq.is_enabled();
        irq.disable();
        Self(was_enabled)
    }
}

impl Drop for AdcIrqGuard {
    fn drop(&mut self) {
        if self.0 {
            unsafe { embassy_stm32::interrupt::ADC1_2.enable() };
        }
    }
}

impl Storage for FlashLease {
    fn read_slot(&mut self, index: usize) -> [u8; SLOT_SIZE] {
        assert!(index < nvm::SLOT_COUNT);
        let mut raw = [0; SLOT_SIZE];
        for (offset, byte) in raw.iter_mut().enumerate() {
            // Volatile reads ensure readback observes programmed flash bytes.
            *byte = unsafe {
                core::ptr::read_volatile((ABS as usize + index * SLOT_SIZE + offset) as *const u8)
            };
        }
        raw
    }

    fn erase(&mut self) -> bool {
        self.0
            .as_mut()
            .unwrap()
            .blocking_erase(PAGE_OFF, PAGE_END)
            .is_ok()
    }

    fn write_slot(&mut self, index: usize, raw: &[u8; SLOT_SIZE]) -> bool {
        assert!(index < nvm::SLOT_COUNT);
        self.0
            .as_mut()
            .unwrap()
            .blocking_write(PAGE_OFF + (index * SLOT_SIZE) as u32, raw)
            .is_ok()
    }
}

/// No FLASH unlock, and no ADC interrupt masking for read-only status queries.
pub fn load() -> Option<Record> {
    nvm::load(&mut FlashLease::take()?)
}

pub fn save(record: Record) -> bool {
    let Some(mut flash) = FlashLease::take() else {
        return false;
    };
    let _adc_irq = AdcIrqGuard::new();
    let result = nvm::save(&mut flash, record);
    match result {
        Ok(SaveOutcome::Unchanged) => defmt::info!("nvm unchanged"),
        Ok(_) => defmt::info!(
            "nvm saved off={} poles={}",
            record.theta_e_off_mrad,
            record.poles
        ),
        Err(_) => defmt::warn!("nvm save failed"),
    }
    result.is_ok()
}
