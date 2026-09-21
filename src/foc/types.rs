//! Shared FOC quantities (SI-ish f32).

#[derive(Clone, Copy, Default)]
pub struct AlphaBeta {
    pub alpha: f32,
    pub beta: f32,
}

#[derive(Clone, Copy, Default)]
pub struct Dq {
    pub d: f32,
    pub q: f32,
}

#[derive(Clone, Copy, Default)]
pub struct PhaseAbc {
    pub a: f32,
    pub b: f32,
    pub c: f32,
}

#[derive(Clone, Copy, Default)]
pub struct Duties {
    pub a: f32,
    pub b: f32,
    pub c: f32,
}

impl Duties {
    pub fn clamp01(self) -> Self {
        Self {
            a: self.a.clamp(0.0, 1.0),
            b: self.b.clamp(0.0, 1.0),
            c: self.c.clamp(0.0, 1.0),
        }
    }
}
