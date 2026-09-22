//! 1 kHz speed PI. Writes `iq` for [`super::control::Mode::Speed`].

use core::sync::atomic::{AtomicPtr, AtomicU32, Ordering};

use static_cell::StaticCell;

use crate::app::control;
use crate::bsp::config::{SPEED_IQ_MAX_A, SPEED_IQ_MIN_A, SPEED_KI, SPEED_KP, SPEED_LOOP_HZ};
use crate::foc::SpeedLoop;

static SLOT: StaticCell<SpeedLoop> = StaticCell::new();
static PTR: AtomicPtr<SpeedLoop> = AtomicPtr::new(core::ptr::null_mut());
static ACC_US: AtomicU32 = AtomicU32::new(0);

pub fn init() {
    let s = SLOT.init(SpeedLoop::new(SPEED_KP, SPEED_KI, SPEED_IQ_MAX_A));
    PTR.store(s as *mut SpeedLoop, Ordering::Release);
}

pub fn reset() {
    ACC_US.store(0, Ordering::Relaxed);
    crate::app::telemetry::reset_speed_filter();
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
    let us = ((dt.max(0.0) * 1_000_000.0) as u32).max(1);
    let acc = ACC_US.fetch_add(us, Ordering::Relaxed) + us;
    let period_us = 1_000_000 / SPEED_LOOP_HZ.max(1);
    if acc < period_us {
        return false;
    }
    ACC_US.store(acc - period_us, Ordering::Relaxed);
    let _ = omega_m;
    if !crate::app::telemetry::rpm_ready() {
        let kick = if control::rpm_ref() >= 0 {
            SPEED_IQ_MIN_A
        } else {
            -SPEED_IQ_MIN_A
        };
        control::write_iq_ma((kick * 1000.0) as i32);
        return true;
    }
    let rpm = crate::app::telemetry::rpm_meas() as f32;
    let tgt = control::rpm_ref() as f32;
    let Some(iq) = with_loop(|l| {
        l.set_iq_limit(SPEED_IQ_MAX_A);
        l.step(tgt, rpm, period_us as f32 / 1_000_000.0)
    }) else {
        return false;
    };
    // Same-sign only (no reverse shake). Allow 0 A when already too fast.
    let iq = if tgt > 0.0 {
        iq.clamp(0.0, SPEED_IQ_MAX_A)
    } else if tgt < 0.0 {
        iq.clamp(-SPEED_IQ_MAX_A, 0.0)
    } else {
        0.0
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
