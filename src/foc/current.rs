//! One PWM-period current loop.

use micromath::F32Ext;

use super::pid::Pi;
use super::svm::{max_modulation, svpwm};
use super::transforms::{clarke_two_phase, inv_park, park};
use super::types::{Dq, Duties};

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

    pub fn step(&mut self, ia: f32, ib: f32, refs: Dq, theta_e: f32, vbus: f32, dt: f32) -> (Dq, Duties) {
        let lim = max_modulation(vbus);
        self.id.set_limits(-lim, lim);
        self.iq.set_limits(-lim, lim);

        let meas = park(clarke_two_phase(ia, ib), theta_e);
        let err_d = refs.d - meas.d;
        let err_q = refs.q - meas.q;
        let mut vd = self.id.step(err_d, dt);
        let mut vq = self.iq.step(err_q, dt);

        let mag = (vd * vd + vq * vq).sqrt();
        if mag > lim && mag > 1e-6 {
            let s = lim / mag;
            vd *= s;
            vq *= s;
            self.id.rewind_to(err_d, vd);
            self.iq.rewind_to(err_q, vq);
        }

        let duties = svpwm(inv_park(Dq { d: vd, q: vq }, theta_e), vbus);
        (meas, duties)
    }
}

/// Open-loop voltage: fixed Vd/Vq, ramped electrical angle.
pub fn openloop_voltage(vd: f32, vq: f32, theta_e: f32, vbus: f32) -> Duties {
    svpwm(inv_park(Dq { d: vd, q: vq }, theta_e), vbus)
}
