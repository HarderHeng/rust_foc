//! 1 kHz speed PI. Writes `iq` for [`super::control::Mode::Speed`].

use core::sync::atomic::{AtomicPtr, Ordering};

use static_cell::StaticCell;

use crate::app::control;
use crate::bsp::config::{NOMINAL_CURRENT_A, SPEED_KI, SPEED_KP};
use crate::foc::SpeedLoop;

static SLOT: StaticCell<SpeedLoop> = StaticCell::new();
static PTR: AtomicPtr<SpeedLoop> = AtomicPtr::new(core::ptr::null_mut());

pub fn init() {
    let s = SLOT.init(SpeedLoop::new(SPEED_KP, SPEED_KI, NOMINAL_CURRENT_A));
    PTR.store(s as *mut SpeedLoop, Ordering::Release);
}

pub fn reset() {
    with_loop(|l| l.reset());
}

pub fn set_gains(kp: f32, ki: f32) {
    with_loop(|l| l.set_gains(kp, ki));
}

/// `omega_m` mechanical rad/s. Returns whether Iq was updated.
pub fn tick(omega_m: f32, valid: bool, dt: f32) -> bool {
    if control::mode() != control::Mode::Speed {
        return false;
    }
    if !valid {
        return false;
    }
    let rpm = omega_m * 60.0 / core::f32::consts::TAU;
    let Some(iq) = with_loop(|l| {
        l.set_iq_limit(NOMINAL_CURRENT_A);
        l.step(control::rpm_ref() as f32, rpm, dt.max(1e-4))
    }) else {
        return false;
    };
    control::write_iq_ma((iq * 1000.0) as i32);
    true
}

fn with_loop<R>(f: impl FnOnce(&mut SpeedLoop) -> R) -> Option<R> {
    cortex_m::interrupt::free(|_| {
        let p = PTR.load(Ordering::Acquire);
        if p.is_null() {
            None
        } else {
            Some(f(unsafe { &mut *p }))
        }
    })
}
