//! Bounded mechanical angle, integer multi-turn position, and ω filtering.

use crate::transforms::wrap_2pi;

const TWO_PI: f32 = core::f32::consts::TAU;
const COUNTS: i32 = 4096;

/// 12-bit rolling angle → single-turn θm and a first-order ω.
/// Multi-turn position is kept separately in integer counts, never in the Park angle.
#[derive(Clone, Copy)]
pub struct AngleTrack {
    last_raw: Option<u16>,
    unwrap: i64,
    last_theta: f32,
    omega_f: f32,
    omega_acc_counts: i64,
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
            omega_acc_counts: 0,
            omega_acc_dt: 0.0,
            alpha: 0.35,
            omega_window: 0.01,
        }
    }

    pub fn last(&self) -> (f32, f32) {
        (self.last_theta, self.omega_f)
    }

    pub fn position_counts(&self) -> i64 {
        self.unwrap
    }

    /// `raw` is 12-bit. Returns θm in [0, 2π), independent of run duration.
    /// First sample does not write ω (avoids a Δθ/dt spike).
    pub fn push(&mut self, raw: u16, dt: f32) -> (f32, f32) {
        let raw = raw & 0x0FFF;
        let had = self.last_raw.is_some();
        let delta = self.last_raw.map(|prev| raw_delta(raw, prev)).unwrap_or(0);
        if had {
            self.unwrap = self.unwrap.saturating_add(i64::from(delta));
        } else {
            self.unwrap = i64::from(raw);
        }
        self.last_raw = Some(raw);

        let theta = raw as f32 * (TWO_PI / COUNTS as f32);
        if had && dt.is_finite() && dt > 0.0 {
            self.omega_acc_counts += i64::from(delta);
            self.omega_acc_dt += dt;
            let win = self.omega_window.max(1e-4);
            if self.omega_acc_dt >= win {
                let raw_w =
                    self.omega_acc_counts as f32 * (TWO_PI / COUNTS as f32) / self.omega_acc_dt;
                let a = self.alpha.clamp(0.0, 1.0);
                self.omega_f = a * raw_w + (1.0 - a) * self.omega_f;
                self.omega_acc_counts = 0;
                self.omega_acc_dt = 0.0;
            }
        }
        self.last_theta = theta;
        (theta, self.omega_f)
    }
}

/// Shortest signed displacement between two 12-bit samples.
pub fn raw_delta(raw: u16, previous: u16) -> i32 {
    let d = i32::from(raw & 0x0fff) - i32::from(previous & 0x0fff);
    if d > COUNTS / 2 {
        d - COUNTS
    } else if d < -COUNTS / 2 {
        d + COUNTS
    } else {
        d
    }
}

/// Electrical angle: reduce integer counts before converting to float.
pub fn park_theta_raw(raw: u16, theta_e_off: f32, pole_pairs: u8) -> f32 {
    let counts = (u32::from(raw & 0x0fff) * u32::from(pole_pairs)) % COUNTS as u32;
    wrap_2pi(counts as f32 * (TWO_PI / COUNTS as f32) - theta_e_off)
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
        let d = wrap_2pi(th1 - th0);
        assert!(d > 0.0 && d < 0.03, "{d}");
        assert_eq!(t.position_counts(), 4101);
        assert!(th1 < 0.01);
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
    fn long_run_does_not_change_angle_or_velocity() {
        let mut t = AngleTrack::new();
        t.push(4090, 0.001);
        // Simulate many years of accumulated position without a long test loop.
        t.unwrap = (1i64 << 45) + 4090;
        t.alpha = 1.0;
        t.omega_window = 0.001;
        let (theta, omega) = t.push(5, 0.001);
        assert!((theta - 5.0 * TWO_PI / 4096.0).abs() < 1e-6);
        assert!((omega - 11.0 * TWO_PI / 4096.0 / 0.001).abs() < 1e-4);
        assert_eq!(t.position_counts(), (1i64 << 45) + 4101);
    }

    #[test]
    fn reverse_wrap_keeps_signed_velocity() {
        let mut t = AngleTrack::new();
        t.alpha = 1.0;
        t.omega_window = 0.001;
        t.push(5, 0.001);
        let (_, omega) = t.push(4090, 0.001);
        assert!(omega < 0.0);
        assert_eq!(t.position_counts(), -6);
    }

    #[test]
    fn raw_park_angle_stays_bounded_for_all_pole_pairs() {
        for poles in [1, 7, 255] {
            for raw in 0..4096 {
                let theta = park_theta_raw(raw, 0.2, poles);
                assert!((0.0..TWO_PI).contains(&theta));
                let expected = park_theta(raw as f32 * TWO_PI / 4096.0, 0.2, poles);
                let error =
                    wrap_2pi(theta - expected + core::f32::consts::PI) - core::f32::consts::PI;
                assert!(error.abs() < 0.0003);
            }
        }
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
