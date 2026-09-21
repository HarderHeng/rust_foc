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
    vbus * 0.57735026919
}
