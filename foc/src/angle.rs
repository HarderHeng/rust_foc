//! Mechanical unwrap, ω filter, and constant-speed prediction.

use crate::transforms::wrap_2pi;

const TWO_PI: f32 = core::f32::consts::TAU;
const COUNTS: i32 = 4096;

/// 12-bit rolling angle → unwrapped θm and a first-order ω.
#[derive(Clone, Copy)]
pub struct AngleTrack {
    last_raw: Option<u16>,
    unwrap: i32,
    last_theta: f32,
    omega_f: f32,
    omega_acc_th: f32,
    omega_acc_dt: f32,
    /// Blend after each [`Self::omega_window`]: `ω := α·Δθ/Δt + (1−α)·ω`.
    pub alpha: f32,
    /// Seconds of angle to accumulate before updating ω (AS5600 LSB is coarse).
    pub omega_window: f32,
}

impl Default for AngleTrack {
    fn default() -> Self {
        Self::new()
    }
}

impl AngleTrack {
    pub fn new() -> Self {
        Self {
            last_raw: None,
            unwrap: 0,
            last_theta: 0.0,
            omega_f: 0.0,
            omega_acc_th: 0.0,
            omega_acc_dt: 0.0,
            alpha: 0.35,
            omega_window: 0.01,
        }
    }

    pub fn last(&self) -> (f32, f32) {
        (self.last_theta, self.omega_f)
    }

    /// `raw` is 12-bit. First sample does not write ω (avoids a Δθ/dt spike).
    pub fn push(&mut self, raw: u16, dt: f32) -> (f32, f32) {
        let raw = raw & 0x0FFF;
        let had = self.last_raw.is_some();
        if let Some(prev) = self.last_raw {
            let mut d = raw as i32 - prev as i32;
            if d > COUNTS / 2 {
                d -= COUNTS;
            } else if d < -COUNTS / 2 {
                d += COUNTS;
            }
            self.unwrap = self.unwrap.wrapping_add(d);
        } else {
            self.unwrap = raw as i32;
        }
        self.last_raw = Some(raw);

        let theta = self.unwrap as f32 * (TWO_PI / COUNTS as f32);
        if had && dt > 0.0 {
            self.omega_acc_th += theta - self.last_theta;
            self.omega_acc_dt += dt;
            let win = self.omega_window.max(1e-4);
            if self.omega_acc_dt >= win {
                let raw_w = self.omega_acc_th / self.omega_acc_dt;
                let a = self.alpha.clamp(0.0, 1.0);
                self.omega_f = a * raw_w + (1.0 - a) * self.omega_f;
                self.omega_acc_th = 0.0;
                self.omega_acc_dt = 0.0;
            }
        }
        self.last_theta = theta;
        (theta, self.omega_f)
    }
}

/// Constant-ω hold: `θ + ω · age`. `age` is time since the last valid sample.
#[inline]
pub fn predict(theta: f32, omega: f32, age: f32) -> f32 {
    theta + omega * age
}

/// Electrical Park angle with an **electrical** offset (align latch).
#[inline]
pub fn park_theta(theta_m: f32, theta_e_off: f32, pole_pairs: u8) -> f32 {
    wrap_2pi(theta_m * f32::from(pole_pairs) - theta_e_off)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unwrap_crosses_zero() {
        let mut t = AngleTrack::new();
        t.push(4090, 0.001);
        let (th0, w0) = t.push(4090, 0.001);
        assert!(w0.abs() < 1e-3);
        let (th1, _) = t.push(5, 0.001);
        let d = th1 - th0;
        assert!(d > 0.0 && d < 0.03, "{d}");
    }

    #[test]
    fn first_sample_omega_is_zero() {
        let mut t = AngleTrack::new();
        let (_, w) = t.push(2048, 0.001);
        assert!(w.abs() < 1e-9);
    }

    #[test]
    fn windowed_omega_matches_constant_rate() {
        let mut t = AngleTrack::new();
        t.alpha = 1.0;
        t.omega_window = 0.01;
        let mut raw = 0u16;
        let mut w = 0.0;
        for _ in 0..20 {
            raw = (raw + 2) & 0x0FFF;
            (_, w) = t.push(raw, 0.001);
        }
        let expect = 2.0 * TWO_PI / COUNTS as f32 / 0.001;
        assert!((w - expect).abs() < 0.05, "{w} vs {expect}");
    }

    #[test]
    fn predict_uses_age() {
        let th = predict(1.0, 100.0, 0.002);
        assert!((th - 1.2).abs() < 1e-5);
    }

    #[test]
    fn park_theta_subtracts_electrical_offset() {
        let th = park_theta(0.5, 0.2, 2);
        assert!((th - 0.8).abs() < 1e-5);
    }
}
