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

/// Integer helper for mA / RPM atomics.
pub fn approach_i32(value: i32, target: i32, rate: f32, dt: f32) -> i32 {
    let x = approach(value as f32, target as f32, rate, dt);
    if x >= 0.0 {
        (x + 0.5) as i32
    } else {
        (x - 0.5) as i32
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
}
