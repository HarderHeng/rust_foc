//! Timestamped, wrap-aware encoder speed windows. Angle prediction remains a
//! separate board-level choice; this estimator never advances the Park angle.

use crate::angle::raw_delta;

#[derive(Clone, Copy)]
pub struct WindowSpeed {
    previous: Option<(u16, u32)>,
    counts: i64,
    elapsed_us: u64,
    rpm: Option<f32>,
    window_us: u32,
    max_gap_us: u32,
}

impl WindowSpeed {
    pub const fn new(window_us: u32, max_gap_us: u32) -> Self {
        Self {
            previous: None,
            counts: 0,
            elapsed_us: 0,
            rpm: None,
            window_us,
            max_gap_us,
        }
    }

    pub fn reset(&mut self) {
        self.previous = None;
        self.counts = 0;
        self.elapsed_us = 0;
        self.rpm = None;
    }

    pub fn rpm(&self) -> Option<f32> {
        self.rpm
    }

    /// Use acquisition timestamps, not task polling intervals. Equal timestamps
    /// do not consume displacement. Gaps invalidate readiness and rebase rather
    /// than interpreting arbitrarily many missed turns as a small signed delta.
    /// Configure max_gap below the half-turn interval at the maximum motor speed.
    pub fn push(&mut self, raw: u16, sampled_us: u32) {
        let raw = raw & 0x0fff;
        let Some((previous, at_us)) = self.previous else {
            self.previous = Some((raw, sampled_us));
            return;
        };
        let dt = sampled_us.wrapping_sub(at_us);
        if dt == 0 {
            return;
        }
        if self.window_us == 0 || dt >= self.max_gap_us || dt >= 1 << 31 {
            self.reset();
            self.previous = Some((raw, sampled_us));
            return;
        }
        self.previous = Some((raw, sampled_us));
        self.counts += i64::from(raw_delta(raw, previous));
        self.elapsed_us += u64::from(dt);
        if self.elapsed_us >= u64::from(self.window_us) {
            self.rpm = Some(self.counts as f32 * (60_000_000.0 / 4096.0) / self.elapsed_us as f32);
            self.counts = 0;
            self.elapsed_us = 0;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn acquisition_time_and_quantized_motion_determine_speed_with_jitter() {
        for sign in [-1.0, 1.0] {
            let mut speed = WindowSpeed::new(100_000, 10_000);
            let mut at = u32::MAX - 50_000;
            let mut elapsed = 0;
            for i in 0..2000 {
                let raw = ((sign * elapsed as f64 * 750.0 * 4096.0 / 60_000_000.0).round() as i64)
                    .rem_euclid(4096) as u16;
                speed.push(raw, at);
                if let Some(rpm) = speed.rpm() {
                    assert!((f64::from(rpm) - sign * 750.0).abs() < 0.3);
                }
                let dt = [250, 401, 599, 750][i % 4];
                elapsed += dt;
                at = at.wrapping_add(dt);
            }
            assert!(speed.rpm().is_some());
        }
    }

    #[test]
    fn duplicate_timestamp_does_not_lose_displacement() {
        let mut speed = WindowSpeed::new(1000, 10_000);
        speed.push(4090, 100);
        speed.push(4095, 100);
        speed.push(4, 1100);
        assert!((speed.rpm().unwrap() - 146.48438).abs() < 0.001);
    }

    #[test]
    fn silence_or_reversed_time_requires_a_new_complete_window() {
        for gap in [10_000, 20_000, u32::MAX] {
            let mut speed = WindowSpeed::new(1000, 10_000);
            speed.push(0, 0);
            speed.push(10, 1000);
            assert!(speed.rpm().is_some());
            speed.push(20, 1000u32.wrapping_add(gap));
            assert!(speed.rpm().is_none());
            speed.push(30, 2000u32.wrapping_add(gap));
            assert!(speed.rpm().is_some());
            speed.reset();
            assert!(speed.rpm().is_none());
        }
    }

    #[test]
    fn stationary_chatter_does_not_accumulate_absolute_distance() {
        let mut speed = WindowSpeed::new(100_000, 10_000);
        for i in 0..=1000 {
            speed.push(if i % 2 == 0 { 4095 } else { 0 }, i * 1000);
        }
        assert_eq!(speed.rpm(), Some(0.0));
    }
}
