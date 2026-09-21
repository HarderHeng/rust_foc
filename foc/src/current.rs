//! One PWM-period current loop (PI → feed-forward → limit → SVPWM).

#[allow(unused_imports)]
use micromath::F32Ext;

use super::pid::Pi;
use super::svm::{Svpwm, VdPriority};
use super::traits::{Modulator, PhaseCurrents, Regulator, VoltageFeedforward, VoltageLimiter};
use super::transforms::{clarke, inv_park, park};
use super::types::{Dq, Duties};

/// PMSM d/q voltage feed-forward (ST MCWB `FF_VqdffComputation`).
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

/// No decoupling / BEMF term.
#[derive(Clone, Copy, Default)]
pub struct FfOff;

impl VoltageFeedforward for FfOff {
    fn vdq(&self, _iref: Dq, _omega_e: f32) -> Dq {
        Dq::default()
    }
}

impl VoltageFeedforward for DqFf {
    fn vdq(&self, iref: Dq, omega_e: f32) -> Dq {
        dq_voltage_ff(iref, omega_e, *self)
    }
}

#[derive(Clone, Copy, Default)]
pub struct CurrentStep {
    pub meas: Dq,
    /// PI + feed-forward before final voltage limiting.
    pub voltage_ref: Dq,
    /// Final limited D/Q voltage applied to the modulator.
    pub voltage: Dq,
    pub duties: Duties,
}

/// Defaults: PI + SVPWM + Vd-priority (ST MCWB-style).
pub struct CurrentLoop<R = Pi, M = Svpwm, L = VdPriority> {
    pub id: R,
    pub iq: R,
    pub modulator: M,
    pub limiter: L,
}

impl CurrentLoop {
    pub fn new(kp: f32, ki: f32, v_lim: f32) -> Self {
        Self::new_with(
            Pi::new(kp, ki, -v_lim, v_lim),
            Pi::new(kp, ki, -v_lim, v_lim),
            Svpwm,
            VdPriority,
        )
    }
}

impl<R, M, L> CurrentLoop<R, M, L> {
    pub fn new_with(id: R, iq: R, modulator: M, limiter: L) -> Self {
        Self {
            id,
            iq,
            modulator,
            limiter,
        }
    }
}

impl<R: Regulator, M, L> CurrentLoop<R, M, L> {
    pub fn reset(&mut self) {
        self.id.reset();
        self.iq.reset();
    }

    pub fn set_gains(&mut self, kp: f32, ki: f32) {
        self.id.set_gains(kp, ki);
        self.iq.set_gains(kp, ki);
    }
}

impl<R: Regulator, M: Modulator, L: VoltageLimiter> CurrentLoop<R, M, L> {
    /// `ff` is added **after** the regulators (ST MCWB `FF_VqdConditioning`). Voltage limit
    /// is applied here; each PI is told its remaining share via [`Pi::track`].
    pub fn step(
        &mut self,
        i: impl PhaseCurrents,
        refs: Dq,
        theta_e: f32,
        vbus: f32,
        dt: f32,
        ff: Dq,
    ) -> (Dq, Duties) {
        let out = self.step_debug(i, refs, theta_e, vbus, dt, ff);
        (out.meas, out.duties)
    }

    pub fn step_debug(
        &mut self,
        i: impl PhaseCurrents,
        refs: Dq,
        theta_e: f32,
        vbus: f32,
        dt: f32,
        ff: Dq,
    ) -> CurrentStep {
        let lim = self.modulator.voltage_limit(vbus);
        Regulator::set_limits(&mut self.id, -lim, lim);
        Regulator::set_limits(&mut self.iq, -lim, lim);

        let meas = park(clarke(i.abc()), theta_e);
        let vd_pi = Regulator::step(&mut self.id, refs.d - meas.d, dt);
        let vq_pi = Regulator::step(&mut self.iq, refs.q - meas.q, dt);

        let voltage_ref = Dq {
            d: vd_pi + ff.d,
            q: vq_pi + ff.q,
        };
        let (vd, vq) = self.limiter.limit(voltage_ref.d, voltage_ref.q, lim);
        let voltage = Dq { d: vd, q: vq };

        if (voltage.d - voltage_ref.d).abs() > 1e-9 || (voltage.q - voltage_ref.q).abs() > 1e-9 {
            Regulator::track(&mut self.id, voltage.d - ff.d);
            Regulator::track(&mut self.iq, voltage.q - ff.q);
        }

        let duties = self.modulator.modulate(inv_park(voltage, theta_e), vbus);
        CurrentStep {
            meas,
            voltage_ref,
            voltage,
            duties,
        }
    }
}

/// Open-loop voltage: fixed Vd/Vq, ramped electrical angle.
pub fn openloop_voltage(vd: f32, vq: f32, theta_e: f32, vbus: f32) -> Duties {
    Svpwm.modulate(inv_park(Dq { d: vd, q: vq }, theta_e), vbus)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ff_is_minus_cross_coupling() {
        let p = DqFf {
            ld: 0.001,
            lq: 0.001,
            flux: 0.0,
        };
        let ff = p.vdq(Dq { d: 0.0, q: 2.0 }, 100.0);
        assert!((ff.d - (-0.2)).abs() < 1e-5);
        assert!(ff.q.abs() < 1e-5);
        let off = FfOff.vdq(Dq { d: 0.0, q: 2.0 }, 100.0);
        assert!(off.d.abs() < 1e-9 && off.q.abs() < 1e-9);
    }

    #[test]
    fn circle_track_does_not_touch_pid_when_inside() {
        let mut l = CurrentLoop::new(1.0, 0.0, 20.0);
        let refs = Dq { d: 0.1, q: 0.0 };
        let (_m, _) = l.step((0.0, 0.0), refs, 0.0, 24.0, 0.001, Dq::default());
        let i0 = l.id.integrator();
        let (_m, _) = l.step((0.0, 0.0), refs, 0.0, 24.0, 0.001, Dq::default());
        assert!((l.id.integrator() - i0).abs() < 1e-6);
    }

    #[test]
    fn saturation_keeps_vd_clips_vq() {
        let mut l = CurrentLoop::new(50.0, 0.0, 20.0);
        let refs = Dq { d: 0.1, q: 20.0 };
        let (_m, _) = l.step((0.0, 0.0), refs, 0.0, 24.0, 0.001, Dq::default());
        let vmax = Svpwm.voltage_limit(24.0);
        let vd = l.id.last_output();
        let vq = l.iq.last_output();
        assert!((vd - 5.0).abs() < 1e-3, "{vd}");
        let qmax = (vmax * vmax - vd * vd).sqrt();
        assert!(vq.abs() <= qmax + 1e-3, "{vq} > {qmax}");
        assert!((vd * vd + vq * vq).sqrt() <= vmax + 1e-3);
    }
}
