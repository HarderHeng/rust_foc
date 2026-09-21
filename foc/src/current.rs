//! One PWM-period current loop (122 FOC: PI → Vqdff → circle → SVPWM).

#[allow(unused_imports)]
use micromath::F32Ext;

use super::pid::Pi;
use super::svm::{max_modulation, svpwm};
use super::transforms::{clarke_two_phase, inv_park, park};
use super::types::{Dq, Duties};

/// 122-style d/q voltage feed-forward (`FF_VqdffComputation`).
///
/// `vd_ff = −ωe·Lq·iq`, `vq_ff = ωe·Ld·id + ωe·ψf` (uses **references**, SI volts).
#[derive(Clone, Copy)]
pub struct DqFf {
    pub ld: f32,
    pub lq: f32,
    /// Permanent-magnet flux (Wb). `0` disables BEMF term.
    pub flux: f32,
}

/// `MOTOR_VOLTAGE_CONSTANT` is Vrms line-line per krpm → ψf (Wb).
pub fn flux_from_ke_vrms_ll_krpm(ke: f32, pole_pairs: f32) -> f32 {
    let omega_e = pole_pairs * (1000.0 * core::f32::consts::TAU / 60.0);
    if omega_e < 1e-6 {
        0.0
    } else {
        ke * core::f32::consts::SQRT_2 / 3.0f32.sqrt() / omega_e
    }
}

pub fn dq_voltage_ff(iref: Dq, omega_e: f32, p: DqFf) -> Dq {
    Dq {
        d: -omega_e * p.lq * iref.q,
        q: omega_e * p.ld * iref.d + omega_e * p.flux,
    }
}

pub struct CurrentLoop {
    pub id: Pi,
    pub iq: Pi,
}

impl CurrentLoop {
    pub fn new(kp: f32, ki: f32, v_lim: f32) -> Self {
        Self {
            id: Pi::new(kp, ki, -v_lim, v_lim),
            iq: Pi::new(kp, ki, -v_lim, v_lim),
        }
    }

    pub fn reset(&mut self) {
        self.id.reset();
        self.iq.reset();
    }

    pub fn set_gains(&mut self, kp: f32, ki: f32) {
        self.id.kp = kp;
        self.id.ki = ki;
        self.iq.kp = kp;
        self.iq.ki = ki;
    }

    /// `ff` is added **after** the PIs (122 `FF_VqdConditioning`). Circle limit
    /// is applied here; each PI is told its remaining share via [`Pi::track`].
    pub fn step(&mut self, ia: f32, ib: f32, refs: Dq, theta_e: f32, vbus: f32, dt: f32, ff: Dq) -> (Dq, Duties) {
        let lim = max_modulation(vbus);
        self.id.set_limits(-lim, lim);
        self.iq.set_limits(-lim, lim);

        let meas = park(clarke_two_phase(ia, ib), theta_e);
        let vd_pi = self.id.step(refs.d - meas.d, dt);
        let vq_pi = self.iq.step(refs.q - meas.q, dt);

        let mut vd = vd_pi + ff.d;
        let mut vq = vq_pi + ff.q;

        let mag = (vd * vd + vq * vq).sqrt();
        if mag > lim && mag > 1e-6 {
            let s = lim / mag;
            vd *= s;
            vq *= s;
            self.id.track(vd - ff.d);
            self.iq.track(vq - ff.q);
        }

        let duties = svpwm(inv_park(Dq { d: vd, q: vq }, theta_e), vbus);
        (meas, duties)
    }
}

/// Open-loop voltage: fixed Vd/Vq, ramped electrical angle.
pub fn openloop_voltage(vd: f32, vq: f32, theta_e: f32, vbus: f32) -> Duties {
    svpwm(inv_park(Dq { d: vd, q: vq }, theta_e), vbus)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ff_is_minus_cross_coupling() {
        let ff = dq_voltage_ff(
            Dq { d: 0.0, q: 2.0 },
            100.0,
            DqFf {
                ld: 0.001,
                lq: 0.001,
                flux: 0.0,
            },
        );
        assert!((ff.d - (-0.2)).abs() < 1e-5);
        assert!(ff.q.abs() < 1e-5);
    }

    #[test]
    fn circle_track_does_not_touch_pid_when_inside() {
        let mut l = CurrentLoop::new(1.0, 0.0, 20.0);
        let refs = Dq { d: 0.1, q: 0.0 };
        let (_m, _) = l.step(0.0, 0.0, refs, 0.0, 24.0, 0.001, Dq::default());
        let i0 = l.id.integrator();
        let (_m, _) = l.step(0.0, 0.0, refs, 0.0, 24.0, 0.001, Dq::default());
        assert!((l.id.integrator() - i0).abs() < 1e-6);
    }
}
