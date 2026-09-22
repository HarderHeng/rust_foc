//! Hardware-independent sample freshness checks. Times are wrapping milliseconds.

#[derive(Clone, Copy, Default)]
pub struct Freshness {
    updated_ms: Option<u32>,
}

impl Freshness {
    pub const fn new() -> Self {
        Self { updated_ms: None }
    }

    pub fn refresh(&mut self, now_ms: u32) {
        self.updated_ms = Some(now_ms);
    }

    pub fn invalidate(&mut self) {
        self.updated_ms = None;
    }

    /// Call regularly (well within one u32 clock wrap). Once expired, latch
    /// invalid until a new sample arrives; old samples cannot revive on wrap.
    pub fn is_fresh(&mut self, now_ms: u32, timeout_ms: u32) -> bool {
        match self.updated_ms {
            Some(at) if now_ms.wrapping_sub(at) < timeout_ms => true,
            _ => {
                self.invalidate();
                false
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn missing_expired_and_invalid_samples_are_rejected() {
        let mut age = Freshness::new();
        assert!(!age.is_fresh(0, 20));
        age.refresh(100);
        assert!(age.is_fresh(119, 20));
        assert!(!age.is_fresh(120, 20));
        assert!(!age.is_fresh(100, 20));
        age.refresh(130);
        assert!(age.is_fresh(130, 20));
        age.invalidate();
        assert!(!age.is_fresh(131, 20));
    }

    #[test]
    fn freshness_survives_millisecond_clock_wrap() {
        let mut age = Freshness::new();
        age.refresh(u32::MAX - 5);
        assert!(age.is_fresh(3, 20));
        assert!(!age.is_fresh(14, 20));
    }
}
