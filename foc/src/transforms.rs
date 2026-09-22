//! Clarke / Park and inverses.

#[allow(unused_imports)]
use micromath::F32Ext;

use super::types::{AlphaBeta, Dq, PhaseAbc};

const SQRT3_INV: f32 = 0.577_350_26;
const SQRT3_OVER_2: f32 = 0.866_025_4;

pub fn wrap_2pi(theta: f32) -> f32 {
    const TWO_PI: f32 = core::f32::consts::TAU;
    let mut x = theta % TWO_PI;
    if x < 0.0 {
        x += TWO_PI;
    }
    x
}

/// Amplitude-invariant Clarke. `i.c` is unused (`ic = -ia - ib`).
pub fn clarke(i: PhaseAbc) -> AlphaBeta {
    AlphaBeta {
        alpha: i.a,
        beta: SQRT3_INV * i.a + 2.0 * SQRT3_INV * i.b,
    }
}

pub fn clarke_two_phase(ia: f32, ib: f32) -> AlphaBeta {
    clarke(PhaseAbc {
        a: ia,
        b: ib,
        c: -ia - ib,
    })
}

/// One electrical-angle sin/cos pair, shared by Park and inverse Park for a step.
#[derive(Clone, Copy)]
pub struct Rotation {
    sin: f32,
    cos: f32,
}

impl Rotation {
    #[inline]
    pub fn new(theta: f32) -> Self {
        let (sin, cos) = theta.sin_cos();
        Self { sin, cos }
    }

    #[inline]
    pub fn park(self, ab: AlphaBeta) -> Dq {
        Dq {
            d: ab.alpha * self.cos + ab.beta * self.sin,
            q: -ab.alpha * self.sin + ab.beta * self.cos,
        }
    }

    #[inline]
    pub fn inv_park(self, dq: Dq) -> AlphaBeta {
        AlphaBeta {
            alpha: dq.d * self.cos - dq.q * self.sin,
            beta: dq.d * self.sin + dq.q * self.cos,
        }
    }
}

pub fn park(ab: AlphaBeta, theta: f32) -> Dq {
    Rotation::new(theta).park(ab)
}

pub fn inv_park(dq: Dq, theta: f32) -> AlphaBeta {
    Rotation::new(theta).inv_park(dq)
}

pub fn inv_clarke(ab: AlphaBeta) -> PhaseAbc {
    PhaseAbc {
        a: ab.alpha,
        b: -0.5 * ab.alpha + SQRT3_OVER_2 * ab.beta,
        c: -0.5 * ab.alpha - SQRT3_OVER_2 * ab.beta,
    }
}

pub fn electrical_angle(theta_m: f32, offset_m: f32, pole_pairs: u8) -> f32 {
    wrap_2pi((theta_m - offset_m) * pole_pairs as f32)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::PhaseAbc;

    fn close(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "{a} vs {b}");
    }

    #[test]
    fn clarke_inv_roundtrip() {
        let i = PhaseAbc {
            a: 1.0,
            b: -0.5,
            c: -0.5,
        };
        let ab = clarke(i);
        let back = inv_clarke(ab);
        close(back.a, i.a);
        close(back.b, i.b);
        close(back.c, i.c);
    }

    #[test]
    fn park_inv_roundtrip() {
        let ab = AlphaBeta {
            alpha: 0.3,
            beta: -0.8,
        };
        let th = 1.2;
        let dq = park(ab, th);
        let back = inv_park(dq, th);
        close(back.alpha, ab.alpha);
        close(back.beta, ab.beta);
    }

    #[test]
    fn shared_rotation_matches_original_equations_over_a_revolution() {
        let ab = AlphaBeta {
            alpha: 3.2,
            beta: -1.7,
        };
        let voltage = Dq { d: -2.3, q: 5.1 };
        for i in -256..=256 {
            let theta = i as f32 * core::f32::consts::TAU / 256.0;
            let (s, c) = theta.sin_cos();
            let rotation = Rotation::new(theta);
            let dq = rotation.park(ab);
            close(dq.d, ab.alpha * c + ab.beta * s);
            close(dq.q, -ab.alpha * s + ab.beta * c);
            let out = rotation.inv_park(voltage);
            close(out.alpha, voltage.d * c - voltage.q * s);
            close(out.beta, voltage.d * s + voltage.q * c);
        }
    }

    #[test]
    fn wrap_negative() {
        let x = wrap_2pi(-0.1);
        assert!(x > 6.0 && x < 6.3);
    }
}
