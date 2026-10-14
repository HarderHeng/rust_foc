//! Supervisor-side alignment qualification; no peripheral access.
//! A qualified hold needs measured current, fresh feedback, stable rotor position,
//! and regular observations. This is not a proof of mechanical alignment.

use crate::angle::raw_delta;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AlignResult {
    Pending,
    Complete,
    TimedOut,
}

#[derive(Clone, Copy)]
pub struct Alignment {
    started_ms: u32,
    hold: Option<(u32, u16)>,
    last_poll_ms: u32,
}

impl Alignment {
    pub const fn new(now_ms: u32) -> Self {
        Self {
            started_ms: now_ms,
            hold: None,
            last_poll_ms: now_ms,
        }
    }

    pub fn observe(
        &mut self,
        now_ms: u32,
        qualified: bool,
        raw: u16,
        hold_ms: u32,
        timeout_ms: u32,
        max_motion_counts: i32,
    ) -> AlignResult {
        if now_ms.wrapping_sub(self.started_ms) >= timeout_ms {
            return AlignResult::TimedOut;
        }
        // Do not count an unobserved task stall as a stable hold.
        let gap = now_ms.wrapping_sub(self.last_poll_ms);
        self.last_poll_ms = now_ms;
        if !qualified || gap > 5 {
            self.hold = None;
            return AlignResult::Pending;
        }
        match self.hold {
            Some((since, anchor)) if raw_delta(raw, anchor).abs() <= max_motion_counts => {
                if now_ms.wrapping_sub(since) >= hold_ms {
                    return AlignResult::Complete;
                }
            }
            _ => self.hold = Some((now_ms, raw)),
        }
        AlignResult::Pending
    }

    pub fn remaining_ms(&self, now_ms: u32, hold_ms: u32) -> u32 {
        self.hold
            .map(|(since, _)| hold_ms.saturating_sub(now_ms.wrapping_sub(since)))
            .unwrap_or(hold_ms)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_feedback_never_qualifies() {
        let mut a = Alignment::new(0);
        for ms in 0..1000 {
            assert_eq!(a.observe(ms, false, 0, 500, 1000, 8), AlignResult::Pending);
        }
        assert_eq!(
            a.observe(1000, false, 0, 500, 1000, 8),
            AlignResult::TimedOut
        );
    }

    #[test]
    fn stable_hold_handles_angle_and_clock_wrap() {
        let start = u32::MAX - 100;
        let mut a = Alignment::new(start);
        for elapsed in 0..500 {
            let raw = if elapsed % 2 == 0 { 4094 } else { 2 };
            assert_eq!(
                a.observe(start.wrapping_add(elapsed), true, raw, 500, 1000, 8),
                AlignResult::Pending
            );
        }
        assert_eq!(
            a.observe(start.wrapping_add(500), true, 0, 500, 1000, 8),
            AlignResult::Complete
        );
    }

    #[test]
    fn motion_bad_current_and_poll_stalls_restart_hold() {
        let mut a = Alignment::new(0);
        for ms in 0..400 {
            a.observe(ms, true, 100, 500, 2000, 8);
        }
        a.observe(400, true, 120, 500, 2000, 8);
        assert_eq!(a.remaining_ms(400, 500), 500);
        a.observe(401, false, 120, 500, 2000, 8);
        assert_eq!(a.remaining_ms(450, 500), 500);
        a.observe(402, true, 120, 500, 2000, 8);
        a.observe(902, true, 120, 500, 2000, 8);
        assert_eq!(a.remaining_ms(902, 500), 500);
    }
}
