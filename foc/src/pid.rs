//! PI with back-calculation anti-windup.

#[derive(Clone, Copy)]
pub struct Pi {
    pub kp: f32,
    pub ki: f32,
    pub out_min: f32,
    pub out_max: f32,
    integ: f32,
}

impl Pi {
    pub fn new(kp: f32, ki: f32, out_min: f32, out_max: f32) -> Self {
        Self {
            kp,
            ki,
            out_min,
            out_max,
            integ: 0.0,
        }
    }

    pub fn reset(&mut self) {
        self.integ = 0.0;
    }

    pub fn set_limits(&mut self, out_min: f32, out_max: f32) {
        self.out_min = out_min;
        self.out_max = out_max;
        self.integ = self.integ.clamp(out_min, out_max);
    }

    /// After an external clamp (e.g. voltage circle), rewind the integrator.
    pub fn rewind_to(&mut self, error: f32, output: f32) {
        self.integ = output - self.kp * error;
    }

    pub fn step(&mut self, error: f32, dt: f32) -> f32 {
        self.integ += self.ki * error * dt;
        let unsat = self.kp * error + self.integ;
        let sat = unsat.clamp(self.out_min, self.out_max);
        self.integ += sat - unsat;
        sat
    }
}
