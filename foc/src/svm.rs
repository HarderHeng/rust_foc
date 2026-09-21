//! Inverse Clarke + midpoint SVPWM.

use super::types::{AlphaBeta, Duties};
use super::transforms::inv_clarke;

/// Map `v_alpha/v_beta` (volts) to three phase duties in 0..1.
pub fn svpwm(ab: AlphaBeta, vbus: f32) -> Duties {
    let vbus = vbus.max(1.0);
    let p = inv_clarke(ab);
    let vmax = p.a.max(p.b).max(p.c);
    let vmin = p.a.min(p.b).min(p.c);
    let voff = 0.5 * (vmax + vmin);
    Duties {
        a: 0.5 + (p.a - voff) / vbus,
        b: 0.5 + (p.b - voff) / vbus,
        c: 0.5 + (p.c - voff) / vbus,
    }
    .clamp01()
}

/// SVPWM inscribed-circle voltage limit (volts).
pub fn max_modulation(vbus: f32) -> f32 {
    vbus * 0.577_350_26
}

/// Shift each duty by `±shift` according to phase-current sign (inverter convention).
pub fn compensate_deadtime(d: Duties, ia: f32, ib: f32, ic: f32, shift: f32) -> Duties {
    let s = |i: f32| if i >= 0.0 { shift } else { -shift };
    Duties {
        a: d.a + s(ia),
        b: d.b + s(ib),
        c: d.c + s(ic),
    }
    .clamp01()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::AlphaBeta;

    #[test]
    fn duties_in_unit_interval() {
        let d = svpwm(AlphaBeta { alpha: 5.0, beta: 2.0 }, 12.0);
        for x in [d.a, d.b, d.c] {
            assert!((0.0..=1.0).contains(&x));
        }
    }

    #[test]
    fn zero_voltage_is_half() {
        let d = svpwm(AlphaBeta { alpha: 0.0, beta: 0.0 }, 12.0);
        assert!((d.a - 0.5).abs() < 1e-5);
        assert!((d.b - 0.5).abs() < 1e-5);
        assert!((d.c - 0.5).abs() < 1e-5);
    }

    #[test]
    fn deadtime_follows_current_sign() {
        let d = Duties {
            a: 0.5,
            b: 0.5,
            c: 0.5,
        };
        let c = compensate_deadtime(d, 1.0, -1.0, 0.0, 0.02);
        assert!((c.a - 0.52).abs() < 1e-5);
        assert!((c.b - 0.48).abs() < 1e-5);
        assert!((c.c - 0.52).abs() < 1e-5);
    }
}
