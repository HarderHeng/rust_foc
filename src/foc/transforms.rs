//! Clarke / Park and inverses.

use micromath::F32Ext;

use super::types::{AlphaBeta, Dq, PhaseAbc};

const SQRT3_INV: f32 = 0.57735026919;
const SQRT3_OVER_2: f32 = 0.86602540378;

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

pub fn park(ab: AlphaBeta, theta: f32) -> Dq {
    let (s, c) = theta.sin_cos();
    Dq {
        d: ab.alpha * c + ab.beta * s,
        q: -ab.alpha * s + ab.beta * c,
    }
}

pub fn inv_park(dq: Dq, theta: f32) -> AlphaBeta {
    let (s, c) = theta.sin_cos();
    AlphaBeta {
        alpha: dq.d * c - dq.q * s,
        beta: dq.d * s + dq.q * c,
    }
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
