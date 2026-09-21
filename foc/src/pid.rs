//! Portable parallel PID. No plant model and no FOC-specific limits.
//!
//! Matches 122 `PI_Controller`: feedback is computed and clamped here;
//! feed-forward and later limiters (voltage circle, SVM, current clip) stay
//! in the caller. After an outer clip, call [`Pid::track`] with the **PID
//! share** that was actually used (`u_applied − ff`), same idea as MATLAB
//! tracking mode / Åström back-calculation.
//!
//! Discrete form (`dt` in seconds):
//!
//! ```text
//! e  = sp − y
//! ep = β·sp − y          (error-only API: ep = e)
//! u* = Kp·ep + I + Kd·de/dt
//! u  = clamp(u*, umin, umax)
//! I  ← I + Kaw·(u − u*)  (default; Kaw = 1 is 122 wDischarge)
//! ```

/// How the integrator behaves when this block's output is clamped.
#[derive(Clone, Copy, PartialEq)]
pub enum AntiWindup {
    /// Integrate freely; clamp only the output.
    None,
    /// Rewind `I` toward the saturated output. `kaw = 1` is 122 `wDischarge`.
    BackCalc { kaw: f32 },
    /// Skip the `Ki·e·dt` update when `u` is on a rail and `e` drives further in.
    Conditional,
}

/// One sample. Plant feed-forward is **not** a field — add it after [`Pid::step`].
#[derive(Clone, Copy)]
pub struct PidIn {
    pub dt: f32,
    /// `None` = use `sp - meas` (requires `meas`).
    pub error: Option<f32>,
    pub sp: f32,
    pub meas: Option<f32>,
}

impl PidIn {
    pub fn error(error: f32, dt: f32) -> Self {
        Self {
            dt,
            error: Some(error),
            sp: 0.0,
            meas: None,
        }
    }

    pub fn meas(sp: f32, meas: f32, dt: f32) -> Self {
        Self {
            dt,
            error: None,
            sp,
            meas: Some(meas),
        }
    }
}

/// Parallel PID. Optional D / 2DOF β / anti-windup; no `Rs`, `L`, or circle limit.
#[derive(Clone, Copy)]
pub struct Pid {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub out_min: f32,
    pub out_max: f32,
    pub anti_windup: AntiWindup,
    /// Setpoint weight on P (`1` = full error). Used when `meas` is given.
    pub beta: f32,
    /// First-order D filter time constant (s). `0` = raw difference.
    pub d_tau: f32,
    /// Differentiate `−y` instead of `e` (no D kick on a setpoint step).
    pub d_on_meas: bool,
    integ: f32,
    d_state: f32,
    prev_err: f32,
    prev_meas: f32,
    last_out: f32,
    last_unsat: f32,
    primed: bool,
}

/// Current / speed loops stay PI (`Kd = 0`).
pub type Pi = Pid;

impl Pid {
    pub fn new(kp: f32, ki: f32, out_min: f32, out_max: f32) -> Self {
        Self::pi(kp, ki, out_min, out_max)
    }

    pub fn pi(kp: f32, ki: f32, out_min: f32, out_max: f32) -> Self {
        Self::pid(kp, ki, 0.0, out_min, out_max)
    }

    pub fn pid(kp: f32, ki: f32, kd: f32, out_min: f32, out_max: f32) -> Self {
        let (lo, hi) = ordered(out_min, out_max);
        Self {
            kp,
            ki,
            kd,
            out_min: lo,
            out_max: hi,
            anti_windup: AntiWindup::BackCalc { kaw: 1.0 },
            beta: 1.0,
            d_tau: 0.0,
            d_on_meas: false,
            integ: 0.0,
            d_state: 0.0,
            prev_err: 0.0,
            prev_meas: 0.0,
            last_out: 0.0,
            last_unsat: 0.0,
            primed: false,
        }
    }

    pub fn reset(&mut self) {
        self.integ = 0.0;
        self.d_state = 0.0;
        self.prev_err = 0.0;
        self.prev_meas = 0.0;
        self.last_out = 0.0;
        self.last_unsat = 0.0;
        self.primed = false;
    }

    pub fn set_limits(&mut self, out_min: f32, out_max: f32) {
        let (lo, hi) = ordered(out_min, out_max);
        self.out_min = lo;
        self.out_max = hi;
        self.integ = self.integ.clamp(lo, hi);
        self.last_out = self.last_out.clamp(lo, hi);
    }

    pub fn set_gains(&mut self, kp: f32, ki: f32, kd: f32) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }

    pub fn integrator(&self) -> f32 {
        self.integ
    }

    /// Last value returned by [`Self::step`] (this block's clamp), or last [`Self::track`].
    pub fn last_output(&self) -> f32 {
        self.last_out
    }

    /// `P + I + D` before this block's clamp.
    pub fn last_unsaturated(&self) -> f32 {
        self.last_unsat
    }

    /// Outer limiter used `applied` instead of [`Self::last_output`].
    ///
    /// `applied` is this regulator's share only (command minus any feed-forward
    /// the caller added). Voltage circle / SVM stay in the caller.
    pub fn track(&mut self, applied: f32) {
        if let AntiWindup::BackCalc { kaw } = self.anti_windup {
            self.integ += kaw * (applied - self.last_out);
        }
        self.last_out = applied;
    }

    pub fn step(&mut self, error: f32, dt: f32) -> f32 {
        self.step_in(PidIn::error(error, dt))
    }

    /// Setpoint / measurement form: applies `beta` and optional D-on-`y`.
    pub fn step_ref(&mut self, sp: f32, meas: f32, dt: f32) -> f32 {
        self.step_in(PidIn::meas(sp, meas, dt))
    }

    pub fn step_in(&mut self, input: PidIn) -> f32 {
        let dt = input.dt.max(0.0);
        let meas = input.meas;
        let err_i = match (input.error, meas) {
            (Some(e), _) => e,
            (None, Some(y)) => input.sp - y,
            (None, None) => 0.0,
        };
        let err_p = if meas.is_some() {
            self.beta * input.sp - meas.unwrap_or(0.0)
        } else {
            err_i
        };

        let p = self.kp * err_p;
        let d = self.derivative(err_i, meas, dt);

        if self.should_integrate(err_i) {
            self.integ += self.ki * err_i * dt;
        }

        let unsat = p + self.integ + d;
        let sat = unsat.clamp(self.out_min, self.out_max);

        if let AntiWindup::BackCalc { kaw } = self.anti_windup {
            self.integ += kaw * (sat - unsat);
        }

        self.prev_err = err_i;
        if let Some(y) = meas {
            self.prev_meas = y;
        }
        self.primed = true;
        self.last_unsat = unsat;
        self.last_out = sat;
        sat
    }

    fn should_integrate(&self, err_i: f32) -> bool {
        match self.anti_windup {
            AntiWindup::Conditional if self.primed => {
                let hi = self.last_out >= self.out_max && err_i > 0.0;
                let lo = self.last_out <= self.out_min && err_i < 0.0;
                !hi && !lo
            }
            _ => true,
        }
    }

    fn derivative(&mut self, err: f32, meas: Option<f32>, dt: f32) -> f32 {
        if self.kd == 0.0 || dt <= 0.0 {
            self.d_state = 0.0;
            return 0.0;
        }

        let raw = if self.d_on_meas {
            match meas {
                Some(y) if self.primed => -(y - self.prev_meas) / dt,
                _ => 0.0,
            }
        } else if self.primed {
            (err - self.prev_err) / dt
        } else {
            0.0
        };

        let d = if self.d_tau > 0.0 {
            let a = dt / (self.d_tau + dt);
            self.d_state + a * (raw - self.d_state)
        } else {
            raw
        };
        self.d_state = d;
        self.kd * d
    }
}

fn ordered(a: f32, b: f32) -> (f32, f32) {
    if a <= b {
        (a, b)
    } else {
        (b, a)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "{a} vs {b}");
    }

    #[test]
    fn p_only_proportional() {
        let mut p = Pid::pid(2.0, 0.0, 0.0, -10.0, 10.0);
        close(p.step(1.5, 0.001), 3.0);
    }

    #[test]
    fn output_clamp() {
        let mut p = Pid::pi(10.0, 0.0, -1.0, 1.0);
        close(p.step(1.0, 0.001), 1.0);
        close(p.step(-1.0, 0.001), -1.0);
    }

    #[test]
    fn backcalc_stops_integral_growth() {
        let mut p = Pid::pi(0.0, 100.0, -1.0, 1.0);
        for _ in 0..200 {
            let _ = p.step(1.0, 0.001);
        }
        assert!(p.integrator().abs() <= 1.0 + 1e-4, "{}", p.integrator());
        close(p.last_output(), 1.0);
    }

    #[test]
    fn no_antiwindup_integrator_runs_away() {
        let mut p = Pid::pi(0.0, 100.0, -1.0, 1.0);
        p.anti_windup = AntiWindup::None;
        for _ in 0..200 {
            let _ = p.step(1.0, 0.001);
        }
        assert!(p.integrator() > 10.0, "{}", p.integrator());
        close(p.last_output(), 1.0);
    }

    #[test]
    fn conditional_skips_pushing_into_rail() {
        let mut p = Pid::pi(0.0, 50.0, -1.0, 1.0);
        p.anti_windup = AntiWindup::Conditional;
        let _ = p.step(1.0, 0.1);
        let i0 = p.integrator();
        let _ = p.step(1.0, 0.1);
        close(p.integrator(), i0);
    }

    #[test]
    fn track_rewinds_after_outer_limit() {
        let mut p = Pid::pi(2.0, 0.0, -10.0, 10.0);
        close(p.step(0.5, 0.001), 1.0);
        let i0 = p.integrator();
        p.track(0.3);
        close(p.last_output(), 0.3);
        close(p.integrator(), i0 - 0.7);
    }

    #[test]
    fn d_on_error_reacts_to_step() {
        let mut p = Pid::pid(0.0, 0.0, 0.01, -10.0, 10.0);
        let _ = p.step(0.0, 0.001);
        let u = p.step(1.0, 0.001);
        assert!(u > 5.0, "{u}");
    }

    #[test]
    fn d_on_meas_ignores_setpoint_step() {
        let mut p = Pid::pid(0.0, 0.0, 0.01, -10.0, 10.0);
        p.d_on_meas = true;
        let _ = p.step_ref(0.0, 0.0, 0.001);
        close(p.step_ref(1.0, 0.0, 0.001), 0.0);
    }

    #[test]
    fn swapped_limits_are_ordered() {
        let p = Pid::pi(1.0, 0.0, 5.0, -5.0);
        close(p.out_min, -5.0);
        close(p.out_max, 5.0);
    }
}
