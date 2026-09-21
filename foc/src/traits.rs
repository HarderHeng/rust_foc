//! Static-dispatch seams. Firmware implements I/O; algorithms stay generic.

use crate::types::{AlphaBeta, Duties, PhaseAbc};

/// Parallel PI/PID (or a stand-in) used by current and speed loops.
pub trait Regulator {
    fn reset(&mut self);
    fn set_gains(&mut self, kp: f32, ki: f32);
    fn set_limits(&mut self, lo: f32, hi: f32);
    fn step(&mut self, error: f32, dt: f32) -> f32;
    fn track(&mut self, applied: f32);
}

/// Three-phase currents in amperes (A=U, B=V, C=W).
pub trait PhaseCurrents {
    fn abc(&self) -> PhaseAbc;
}

impl PhaseCurrents for PhaseAbc {
    fn abc(&self) -> PhaseAbc {
        *self
    }
}

impl PhaseCurrents for (f32, f32) {
    fn abc(&self) -> PhaseAbc {
        PhaseAbc {
            a: self.0,
            b: self.1,
            c: -self.0 - self.1,
        }
    }
}

impl PhaseCurrents for (f32, f32, f32) {
    fn abc(&self) -> PhaseAbc {
        PhaseAbc {
            a: self.0,
            b: self.1,
            c: self.2,
        }
    }
}

/// αβ volts → phase duties. Default is midpoint SVPWM.
pub trait Modulator {
    fn modulate(&self, ab: AlphaBeta, vbus: f32) -> Duties;
    fn voltage_limit(&self, vbus: f32) -> f32;
}

/// Clip `vd`/`vq` to a feasible set (circle, Vd-priority, …).
pub trait VoltageLimiter {
    fn limit(&self, vd: f32, vq: f32, vmax: f32) -> (f32, f32);
}

/// PWM / sample-pair scheduling. Implemented on the board drivers.
pub trait DutySink {
    fn apply(&mut self, duties: Duties);
}

impl<T: DutySink + ?Sized> DutySink for &mut T {
    fn apply(&mut self, duties: Duties) {
        (**self).apply(duties);
    }
}

impl<A: DutySink, B: DutySink> DutySink for (A, B) {
    fn apply(&mut self, duties: Duties) {
        self.0.apply(duties);
        self.1.apply(duties);
    }
}

/// Duty post-process (dead-time, etc.).
pub trait DutyMap {
    fn map(&self, duties: Duties, i: PhaseAbc) -> Duties;
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Rec(Duties);

    impl DutySink for Rec {
        fn apply(&mut self, duties: Duties) {
            self.0 = duties;
        }
    }

    #[test]
    fn tuple_sink_forwards() {
        let mut a = Rec(Duties::default());
        let mut b = Rec(Duties::default());
        let d = Duties {
            a: 0.2,
            b: 0.3,
            c: 0.4,
        };
        (&mut a, &mut b).apply(d);
        assert!((a.0.a - 0.2).abs() < 1e-6);
        assert!((b.0.c - 0.4).abs() < 1e-6);
    }
}
