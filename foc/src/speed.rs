//! Mechanical speed PI. Output is Iq (A). Run at ~1 kHz, not in the current ISR.

use super::pid::Pi;
use super::traits::Regulator;

pub struct SpeedLoop<R = Pi> {
    pub pi: R,
}

impl SpeedLoop {
    pub fn new(kp: f32, ki: f32, iq_lim: f32) -> Self {
        Self::from_regulator(Pi::new(kp, ki, -iq_lim, iq_lim))
    }
}

impl<R> SpeedLoop<R> {
    pub fn from_regulator(pi: R) -> Self {
        Self { pi }
    }
}

impl<R: Regulator> SpeedLoop<R> {
    pub fn reset(&mut self) {
        self.pi.reset();
    }

    pub fn set_gains(&mut self, kp: f32, ki: f32) {
        self.pi.set_gains(kp, ki);
    }

    pub fn set_iq_limit(&mut self, iq_lim: f32) {
        self.pi.set_limits(-iq_lim, iq_lim);
    }

    /// `rpm_ref` / `rpm_meas` are mechanical RPM. Returns Iq amps.
    pub fn step(&mut self, rpm_ref: f32, rpm_meas: f32, dt: f32) -> f32 {
        self.pi.step(rpm_ref - rpm_meas, dt)
    }
}
