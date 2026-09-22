//! Mechanical speed PI. Output is Iq (A); caller supplies the actual update interval.

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

    /// Non-regenerative policy: only torque in the requested direction.
    /// Feed the previous command's actual, slew-limited Iq back into anti-windup.
    pub fn step_same_direction(
        &mut self,
        rpm_ref: f32,
        rpm_meas: f32,
        dt: f32,
        iq_limit: f32,
        iq_applied: f32,
    ) -> f32 {
        let limit = iq_limit.max(0.0);
        let (lo, hi) = if rpm_ref > 0.0 {
            (0.0, limit)
        } else if rpm_ref < 0.0 {
            (-limit, 0.0)
        } else {
            (0.0, 0.0)
        };
        self.pi.set_limits(lo, hi);
        self.pi.track(iq_applied);
        self.step(rpm_ref, rpm_meas, dt)
    }

    /// `rpm_ref` / `rpm_meas` are mechanical RPM. Returns Iq amps.
    pub fn step(&mut self, rpm_ref: f32, rpm_meas: f32, dt: f32) -> f32 {
        self.pi.step(rpm_ref - rpm_meas, dt)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_command_never_kicks_the_motor() {
        let mut speed = SpeedLoop::new(0.002, 0.004, 0.12);
        for rpm in [-1000.0, 0.0, 1000.0] {
            assert_eq!(speed.step_same_direction(0.0, rpm, 0.1, 0.12, 0.02), 0.0);
        }
    }

    #[test]
    fn direction_limit_is_visible_to_the_pi() {
        let mut speed = SpeedLoop::new(0.002, 0.004, 0.12);
        let out = speed.step_same_direction(100.0, 200.0, 0.1, 0.12, 0.0);
        assert_eq!(out, 0.0);
        assert_eq!(speed.pi.last_output(), out);
        let out = speed.step_same_direction(-100.0, -200.0, 0.1, 0.12, 0.0);
        assert_eq!(out, 0.0);
        assert_eq!(speed.pi.last_output(), out);
    }

    #[test]
    fn applied_current_tracks_a_slow_ramp() {
        let mut speed = SpeedLoop::new(0.0, 1.0, 1.0);
        assert!((speed.step_same_direction(1.0, 0.0, 0.1, 1.0, 0.0) - 0.1).abs() < 1e-6);
        // Actual current reached only 0.02 A, not the requested 0.1 A.
        let out = speed.step_same_direction(1.0, 0.0, 0.1, 1.0, 0.02);
        assert!((out - 0.12).abs() < 1e-6);
    }
}
