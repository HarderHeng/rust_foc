//! Cycle-counter diagnostics, independent of the MCU and logging transport.

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CycleStats {
    pub last_cycles: u32,
    pub max_cycles: u32,
    pub calls: u32,
    /// Measured intervals at least as long as the supplied cycle budget.
    /// This is not a count of missed hardware triggers.
    pub overruns: u32,
}

impl CycleStats {
    pub const fn new() -> Self {
        Self {
            last_cycles: 0,
            max_cycles: 0,
            calls: 0,
            overruns: 0,
        }
    }

    /// One measured interval, shorter than one full u32 counter wrap.
    /// Counters saturate instead of silently resetting after a long run.
    pub fn record(&mut self, start: u32, end: u32, budget_cycles: u32) {
        let cycles = end.wrapping_sub(start);
        self.last_cycles = cycles;
        self.max_cycles = self.max_cycles.max(cycles);
        self.calls = self.calls.saturating_add(1);
        if cycles >= budget_cycles {
            self.overruns = self.overruns.saturating_add(1);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn budget_boundary_and_maximum_are_preserved() {
        let mut stats = CycleStats::new();
        stats.record(100, 200, 8500);
        stats.record(200, 8700, 8500);
        stats.record(8700, 17201, 8500);
        stats.record(17201, 17251, 8500);
        assert_eq!(
            stats,
            CycleStats {
                last_cycles: 50,
                max_cycles: 8501,
                calls: 4,
                overruns: 2,
            }
        );
    }

    #[test]
    fn dwt_wrap_is_not_an_overrun() {
        let mut stats = CycleStats::new();
        stats.record(u32::MAX - 9, 10, 8500);
        assert_eq!(stats.last_cycles, 20);
        assert_eq!(stats.overruns, 0);
    }

    #[test]
    fn long_run_counters_saturate_and_explicit_reset_clears_everything() {
        let mut stats = CycleStats {
            last_cycles: 9000,
            max_cycles: 9000,
            calls: u32::MAX,
            overruns: u32::MAX,
        };
        stats.record(0, 10_000, 8500);
        assert_eq!(stats.calls, u32::MAX);
        assert_eq!(stats.overruns, u32::MAX);
        assert_eq!(stats.max_cycles, 10_000);
        stats = CycleStats::new();
        stats.record(1, 2, 8500);
        assert_eq!(
            stats,
            CycleStats {
                last_cycles: 1,
                max_cycles: 1,
                calls: 1,
                overruns: 0
            }
        );
    }
}
