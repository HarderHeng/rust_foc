//! Rate limiter. Use for current / speed / position references — not inside Pid.

/// Move `value` toward `target` by at most `rate * dt` (rate ≥ 0).
pub fn approach(value: f32, target: f32, rate: f32, dt: f32) -> f32 {
    let max = rate.max(0.0) * dt.max(0.0);
    let d = target - value;
    if d > max {
        value + max
    } else if d < -max {
        value - max
    } else {
        target
    }
}

/// Stateless integer helper. Repeated sub-unit steps can round to zero;
/// use [`SlewI32`] for a time-continuous mA / RPM reference.
pub fn approach_i32(value: i32, target: i32, rate: f32, dt: f32) -> i32 {
    let x = approach(value as f32, target as f32, rate, dt);
    if x >= 0.0 {
        (x + 0.5) as i32
    } else {
        (x - 0.5) as i32
    }
}

/// Stateful integer-output slew limiter with a fixed-point internal position.
///
/// Rates are rounded to 0.001 output units/s; `dt_ms` is elapsed milliseconds.
/// The internal resolution is 0.000001 output units. Only the published value
/// is rounded (nearest, ties away from zero), so sub-unit steps are retained.
/// Quantized output differs from the internal ramp by at most 0.5 unit.
#[derive(Clone, Copy, Debug)]
pub struct SlewI32 {
    position: i64,
}

impl SlewI32 {
    const SCALE: i64 = 1_000_000;

    pub const fn new(value: i32) -> Self {
        Self {
            position: value as i64 * Self::SCALE,
        }
    }

    pub fn value(&self) -> i32 {
        if self.position >= 0 {
            ((self.position + Self::SCALE / 2) / Self::SCALE) as i32
        } else {
            -(((-self.position) + Self::SCALE / 2) / Self::SCALE) as i32
        }
    }

    pub fn step(&mut self, target: i32, rate: f32, dt_ms: u32) -> i32 {
        if !rate.is_finite() || rate <= 0.0 || dt_ms == 0 {
            return self.value();
        }
        let distance = i64::from(target) * Self::SCALE - self.position;
        if distance == 0 {
            return target;
        }
        // milli-units/s × milliseconds = micro-units. Saturate before converting
        // back to signed: even extreme rates/intervals must not wrap or overshoot.
        let rate_milli = (rate * 1000.0 + 0.5) as u64;
        let step = rate_milli
            .saturating_mul(u64::from(dt_ms))
            .min(distance.unsigned_abs()) as i64;
        self.position += if distance >= 0 { step } else { -step };
        self.value()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reaches_in_expected_time() {
        let mut x = 0.0;
        for _ in 0..10 {
            x = approach(x, 10.0, 100.0, 0.01);
        }
        assert!((x - 10.0).abs() < 1e-4, "{x}");
    }

    #[test]
    fn respects_rate() {
        let x = approach(0.0, 100.0, 10.0, 0.1);
        assert!((x - 1.0).abs() < 1e-5, "{x}");
    }

    #[test]
    fn already_there() {
        assert!((approach(3.0, 3.0, 1.0, 0.01) - 3.0).abs() < 1e-6);
    }

    #[test]
    fn sub_unit_steps_do_not_stall_or_accelerate() {
        for (rate, expected) in [(0.1, 1), (1.0, 10), (100.0, 1000), (800.0, 8000)] {
            let mut ramp = SlewI32::new(0);
            for _ in 0..10_000 {
                ramp.step(20_000, rate, 1);
            }
            assert_eq!(ramp.value(), expected);
        }
    }

    #[test]
    fn negative_ramp_and_zero_crossing_are_symmetric() {
        let mut positive = SlewI32::new(-100);
        let mut negative = SlewI32::new(100);
        for _ in 0..2500 {
            assert_eq!(positive.step(100, 100.0, 1), -negative.step(-100, 100.0, 1));
        }
        assert_eq!(positive.value(), 100);
        assert_eq!(negative.value(), -100);
    }

    #[test]
    fn negative_half_unit_rounds_away_from_zero() {
        let mut negative = SlewI32::new(0);
        let mut positive = SlewI32::new(0);
        negative.position = -500_000;
        positive.position = 500_000;
        assert_eq!(negative.value(), -1);
        assert_eq!(positive.value(), 1);
    }

    #[test]
    fn elapsed_time_not_tick_partition_determines_position() {
        let mut regular = SlewI32::new(0);
        let mut jittered = SlewI32::new(0);
        for _ in 0..1000 {
            regular.step(10_000, 123.456, 1);
        }
        for _ in 0..100 {
            for dt in [1, 2, 7] {
                jittered.step(10_000, 123.456, dt);
            }
        }
        assert_eq!(regular.position, jittered.position);
        assert_eq!(regular.value(), 123);
    }

    #[test]
    fn reversal_and_target_clamp_do_not_bank_unused_motion() {
        let mut ramp = SlewI32::new(0);
        assert_eq!(ramp.step(100, 100.0, 4), 0); // +0.4 internally
        assert_eq!(ramp.step(-100, 100.0, 9), -1); // -0.5 internally
        assert_eq!(ramp.step(-1, 100.0, 1000), -1); // stop exactly at target
        assert_eq!(ramp.step(100, 100.0, 1), -1); // no saved budget from that 1 s
        let mut reset = SlewI32::new(0);
        assert_eq!(reset.step(100, 100.0, 4), 0);
    }

    #[test]
    fn invalid_rate_or_zero_time_holds_position() {
        let mut ramp = SlewI32::new(10);
        for rate in [0.0, -1.0, f32::NAN, f32::INFINITY] {
            assert_eq!(ramp.step(100, rate, 100), 10);
        }
        assert_eq!(ramp.step(100, 100.0, 0), 10);
    }

    #[test]
    fn full_integer_range_and_large_intervals_do_not_overflow() {
        let mut ramp = SlewI32::new(i32::MIN);
        assert_eq!(ramp.step(i32::MAX, f32::MAX, u32::MAX), i32::MAX);
        assert_eq!(ramp.step(i32::MIN, f32::MAX, u32::MAX), i32::MIN);
    }
}
