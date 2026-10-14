//! IWDG fallback for CPU/executor stalls, independent of the PWM/ADC IRQ.
//!
//! A nominal one-second timeout leaves an Idle maintenance allowance for 48
//! blocking ADC offset reads and one flash-page erase plus two doubleword writes.
//! LSI tolerance changes the real deadline; this allowance needs board acceptance.
//! No maintenance operation or interrupt feeds the watchdog directly.

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};

use cortex_m::interrupt::Mutex;
use embassy_stm32::peripherals::IWDG;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_stm32::Peri;

use crate::app::telemetry;
use crate::bsp::config::{IWDG_FEED_INTERVAL_MS, IWDG_TIMEOUT_MS};
use crate::foc::watchdog::FeedPolicy;

static ENCODER_SEQUENCE: AtomicU32 = AtomicU32::new(0);
static WATCHDOG: Mutex<RefCell<Option<SupervisedWatchdog>>> = Mutex::new(RefCell::new(None));

struct SupervisedWatchdog {
    hardware: IndependentWatchdog<'static, IWDG>,
    policy: FeedPolicy,
}

/// Called after peripheral bring-up, with PWM still off. IWDG cannot be stopped.
pub fn init(instance: Peri<'static, IWDG>) {
    let mut hardware = IndependentWatchdog::new(instance, IWDG_TIMEOUT_MS * 1_000);
    // Start before waiting: if LSI-domain synchronization stalls, reset must
    // remain available rather than hanging indefinitely with IWDG stopped.
    hardware.unleash();
    // Embassy writes PR/RLR but does not wait for their LSI-domain updates.
    // RCC's default LsConfig enables LSI before this point.
    loop {
        let status = embassy_stm32::pac::IWDG.sr().read();
        if !status.pvu() && !status.rvu() {
            break;
        }
    }
    let policy = FeedPolicy::new(
        telemetry::control_sequence(),
        ENCODER_SEQUENCE.load(Ordering::Relaxed),
        embassy_time::Instant::now().as_millis() as u32,
        IWDG_FEED_INTERVAL_MS,
    );
    cortex_m::interrupt::free(|cs| {
        let mut slot = WATCHDOG.borrow(cs).borrow_mut();
        assert!(slot.is_none(), "watchdog already initialized");
        hardware.pet();
        *slot = Some(SupervisedWatchdog { hardware, policy });
    });
}

/// Encoder task liveness, including completed error paths. Invalid sensor data
/// still trips the existing encoder interlock; it does not force Idle rebooting.
pub fn encoder_completed() {
    ENCODER_SEQUENCE.fetch_add(1, Ordering::Relaxed);
}

/// Only the analog supervisor calls this, after its protection and control work.
/// Feeding requires new ADC completions and encoder-task iterations. A running
/// ADC interrupt alone cannot pet IWDG when the cooperative executor is stuck.
pub fn supervise() {
    cortex_m::interrupt::free(|cs| {
        let mut slot = WATCHDOG.borrow(cs).borrow_mut();
        if let Some(watchdog) = slot.as_mut() {
            let now_ms = embassy_time::Instant::now().as_millis() as u32;
            if watchdog.policy.should_feed(
                telemetry::control_sequence(),
                ENCODER_SEQUENCE.load(Ordering::Relaxed),
                now_ms,
            ) {
                watchdog.hardware.pet();
            }
        }
    });
}
