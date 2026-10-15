//! Configurable-rate speed PI. Writes `iq` for [`super::control::Mode::Speed`].

use crate::app::control;
use crate::app::controller::{Controller, ControllerState};
use crate::bsp::config::{SPEED_IQ_MAX_A, SPEED_KI, SPEED_KP, SPEED_LOOP_HZ};
use crate::foc::SpeedLoop;

struct SpeedState {
    loop_: SpeedLoop,
    acc_us: u32,
}

impl ControllerState for SpeedState {
    fn reset(&mut self) {
        self.loop_.reset();
        self.acc_us = 0;
    }

    fn set_gains(&mut self, kp: f32, ki: f32) {
        self.loop_.set_gains(kp, ki);
    }
}

static LOOP: Controller<SpeedState> = Controller::new();

pub fn init() {
    LOOP.init(SpeedState {
        loop_: SpeedLoop::new(SPEED_KP, SPEED_KI, SPEED_IQ_MAX_A),
        acc_us: 0,
    });
}

pub fn reset() {
    crate::app::telemetry::reset_speed_filter();
    LOOP.reset();
}

pub fn set_gains(kp: f32, ki: f32) {
    LOOP.set_gains(kp, ki);
}

/// `omega_m` mechanical rad/s. Returns whether Iq was updated.
pub fn tick(_omega_m: f32, valid: bool, dt: f32) -> bool {
    let Some(token) = control::output_token() else {
        return false;
    };
    if token.mode() != control::Mode::Speed || !valid || !dt.is_finite() || dt <= 0.0 {
        return false;
    }
    let us = ((dt * 1_000_000.0) as u32).max(1);
    LOOP.compute(
        |state| {
            state.acc_us = state.acc_us.saturating_add(us);
            let period_us = 1_000_000 / SPEED_LOOP_HZ.max(1);
            if state.acc_us < period_us {
                return None;
            }
            let acc = core::mem::take(&mut state.acc_us);
            if !crate::app::telemetry::rpm_ready() {
                // Wait for real feedback, including for a zero/negative command.
                return Some(0);
            }
            let rpm = crate::app::telemetry::rpm_meas() as f32;
            let tgt = control::rpm_ref() as f32;
            let iq = state.loop_.step_same_direction(
                tgt,
                rpm,
                acc as f32 / 1_000_000.0,
                SPEED_IQ_MAX_A,
                control::iq_a(),
            );
            Some((iq * 1000.0) as i32)
        },
        |iq| {
            let Some(iq) = iq else { return false };
            control::commit_output(token, || control::write_iq_ma(iq)).is_some()
        },
    )
}
