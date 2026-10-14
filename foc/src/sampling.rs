//! Two-shunt acquisition planning for center-aligned PWM1 and an up-count
//! PWM2 CH4 rising edge. All times are in timer ticks; `arr` is the actual
//! auto-reload register, whose relationship to a HAL duty range is HAL-specific.
//!
//! For phase compare `c`, its low side conducts between `c` on the up-count
//! and `2 * ARR - c` on the down-count. Acquisition starts only after the
//! turn-on settling budget and finishes before either measured low side opens.
//! This geometry does not validate real ADC latency or switching noise from
//! the unmeasured phase; those need board measurements.

/// Simultaneously measured phases; the third current is reconstructed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShuntPair {
    Uv,
    Uw,
    Vw,
}

/// Phase compares and the matching acquisition configuration for one PWM period.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SamplePlan {
    /// Actual CCR1..3, possibly reduced by an equal common-mode shift.
    pub ccr: [u32; 3],
    /// CCR4 on the up-count, strictly between zero and ARR.
    pub ccr4: u32,
    pub pair: ShuntPair,
}

/// Preserve line voltages while finding a valid two-shunt sampling window.
///
/// Input compares may be `0..=ARR + 1` (`ARR + 1` means continuously on).
/// `tw_after` includes low-side dead time and settling. `tw_before` is the
/// complete trigger-to-conversion-completion budget, including synchronization,
/// sampling, conversion, and any required margin; it is not only sample time.
/// Both ADCs must finish within this budget.
///
/// Prefer UV at `ARR - 1`. Otherwise measure the lowest two compares, as close
/// to the center as their window allows. If necessary, subtract the smallest
/// feasible common-mode shift from all three compares. Pairwise differences
/// remain exact. Return `None` if even this shift cannot provide a window;
/// callers must disable live outputs rather than apply an unmeasurable command.
pub fn plan_sampling(ccr: [u32; 3], arr: u32, tw_after: u32, tw_before: u32) -> Option<SamplePlan> {
    if arr < 2 || ccr.iter().any(|&c| u64::from(c) > u64::from(arr) + 1) {
        return None;
    }

    let center = arr - 1;
    if window(ccr, arr, tw_after, tw_before, ShuntPair::Uv) == Some(center) {
        return Some(SamplePlan {
            ccr,
            ccr4: center,
            pair: ShuntPair::Uv,
        });
    }

    let pair = lowest_pair(ccr);
    if let Some(ccr4) = window(ccr, arr, tw_after, tw_before, pair) {
        return Some(SamplePlan { ccr, ccr4, pair });
    }

    // For the later turn-on of the two measured phases, m, a trigger t exists
    // iff max(1, m + after) <= min(ARR - 1, 2*ARR - m - before).
    // These three integer upper bounds are precisely that intersection.
    let peak = u64::from(arr);
    let after = u64::from(tw_after);
    let before = u64::from(tw_before);
    let latest_m = (peak - 1)
        .checked_sub(after)?
        .min((2 * peak).checked_sub(after + before)? / 2)
        .min((2 * peak).checked_sub(before + 1)?);
    let [i, j] = pair_indices(pair);
    let m = u64::from(ccr[i].max(ccr[j]));
    let shift = m.checked_sub(latest_m)?;
    if shift > u64::from(*ccr.iter().min()?) {
        return None;
    }
    let shifted = ccr.map(|c| (u64::from(c) - shift) as u32);
    let ccr4 = window(shifted, arr, tw_after, tw_before, pair)?;
    Some(SamplePlan {
        ccr: shifted,
        ccr4,
        pair,
    })
}

fn pair_indices(pair: ShuntPair) -> [usize; 2] {
    match pair {
        ShuntPair::Uv => [0, 1],
        ShuntPair::Uw => [0, 2],
        ShuntPair::Vw => [1, 2],
    }
}

fn lowest_pair(ccr: [u32; 3]) -> ShuntPair {
    // Equal compares prefer UV, then UW, so sector boundaries are deterministic.
    if ccr[0] <= ccr[2] && ccr[1] <= ccr[2] {
        ShuntPair::Uv
    } else if ccr[0] <= ccr[1] && ccr[2] <= ccr[1] {
        ShuntPair::Uw
    } else {
        ShuntPair::Vw
    }
}

fn window(ccr: [u32; 3], arr: u32, tw_after: u32, tw_before: u32, pair: ShuntPair) -> Option<u32> {
    let [i, j] = pair_indices(pair);
    let m = u64::from(ccr[i].max(ccr[j]));
    let earliest = (m + u64::from(tw_after)).max(1);
    let latest = (2 * u64::from(arr))
        .checked_sub(m + u64::from(tw_before))?
        .min(u64::from(arr - 1));
    (earliest <= latest).then_some(latest as u32)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transforms::Rotation;
    use crate::types::Dq;
    use crate::{Modulator, Svpwm};

    const BOARD_ARR: u32 = 4250;
    const BOARD_AFTER: u32 = 297;
    const BOARD_BEFORE: u32 = 76;

    // Check each measured phase in elapsed time, independently of the planner's
    // maximum-compare bounds and pair-selection helpers.
    fn valid(plan: SamplePlan, arr: u32, after: u32, before: u32) -> bool {
        let selected = match plan.pair {
            ShuntPair::Uv => [true, true, false],
            ShuntPair::Uw => [true, false, true],
            ShuntPair::Vw => [false, true, true],
        };
        let t = u64::from(plan.ccr4);
        t > 0
            && t < u64::from(arr)
            && plan.ccr.iter().all(|&c| u64::from(c) <= u64::from(arr) + 1)
            && selected.iter().enumerate().all(|(i, &measured)| {
                if !measured {
                    return true;
                }
                let turn_on = u64::from(plan.ccr[i]);
                let turn_off = 2 * u64::from(arr) - turn_on;
                t >= turn_on + u64::from(after) && t + u64::from(before) <= turn_off
            })
    }

    fn unchanged_line_voltages(original: [u32; 3], applied: [u32; 3]) {
        for i in 0..3 {
            for j in 0..3 {
                assert_eq!(
                    i64::from(original[i]) - i64::from(original[j]),
                    i64::from(applied[i]) - i64::from(applied[j])
                );
            }
        }
    }

    #[test]
    fn center_prefers_uv_when_only_the_measured_phases_are_settled() {
        let plan = plan_sampling(
            [1000, 1000, BOARD_ARR + 1],
            BOARD_ARR,
            BOARD_AFTER,
            BOARD_BEFORE,
        )
        .unwrap();
        assert_eq!(plan.pair, ShuntPair::Uv);
        assert_eq!(plan.ccr4, BOARD_ARR - 1);
        assert_eq!(plan.ccr, [1000, 1000, BOARD_ARR + 1]);
        assert!(valid(plan, BOARD_ARR, BOARD_AFTER, BOARD_BEFORE));
    }

    #[test]
    fn two_high_duties_regression_handles_every_phase_permutation() {
        let arr = BOARD_ARR;
        let high = (0.933 * arr as f32) as u32;
        let low = (0.067 * arr as f32) as u32;
        // Two equal high duties require both the common shift and correct pair;
        // nearby unequal duties exercise all six orderings as well.
        for h2 in [high, high - 1] {
            for original in [
                [high, h2, low],
                [high, low, h2],
                [h2, high, low],
                [h2, low, high],
                [low, high, h2],
                [low, h2, high],
            ] {
                let plan = plan_sampling(original, arr, BOARD_AFTER, BOARD_BEFORE).unwrap();
                assert!(
                    valid(plan, arr, BOARD_AFTER, BOARD_BEFORE),
                    "{original:?}: {plan:?}"
                );
                assert!(original.iter().zip(plan.ccr).all(|(&a, b)| a > b));
                unchanged_line_voltages(original, plan.ccr);
            }
        }
    }

    #[test]
    fn lowest_pair_omits_the_largest_compare() {
        for (ccr, pair) in [
            ([BOARD_ARR + 1, 2000, 1000], ShuntPair::Vw),
            ([2000, BOARD_ARR + 1, 1000], ShuntPair::Uw),
            ([2000, 1000, BOARD_ARR + 1], ShuntPair::Uv),
        ] {
            let plan = plan_sampling(ccr, BOARD_ARR, BOARD_AFTER, BOARD_BEFORE).unwrap();
            assert_eq!(plan.pair, pair);
            assert_eq!(plan.ccr, ccr);
            assert!(valid(plan, BOARD_ARR, BOARD_AFTER, BOARD_BEFORE));
        }
    }

    #[test]
    fn equal_and_full_on_duties_preserve_zero_line_voltage() {
        for c in [0, BOARD_ARR / 2, BOARD_ARR, BOARD_ARR + 1] {
            let plan = plan_sampling([c; 3], BOARD_ARR, BOARD_AFTER, BOARD_BEFORE).unwrap();
            assert_eq!(plan.pair, ShuntPair::Uv);
            assert!(valid(plan, BOARD_ARR, BOARD_AFTER, BOARD_BEFORE));
            unchanged_line_voltages([c; 3], plan.ccr);
            if c >= BOARD_ARR {
                assert_eq!(plan.ccr, [3952; 3]);
            }
        }
    }

    #[test]
    fn insufficient_common_mode_headroom_is_rejected() {
        assert_eq!(
            plan_sampling(
                [0, BOARD_ARR + 1, BOARD_ARR + 1],
                BOARD_ARR,
                BOARD_AFTER,
                BOARD_BEFORE,
            ),
            None
        );
        assert_eq!(plan_sampling([0, 90, 101], 100, 10, 1), None);
    }

    #[test]
    fn settling_boundary_is_inclusive_but_the_next_tick_is_unavailable() {
        let plan = plan_sampling([0, 89, 101], 100, 10, 1).unwrap();
        assert_eq!(plan.ccr, [0, 89, 101]);
        assert_eq!(plan.ccr4, 99);
        assert!(valid(plan, 100, 10, 1));
        assert_eq!(plan_sampling([0, 90, 101], 100, 10, 1), None);
        let shifted = plan_sampling([1, 90, 101], 100, 10, 1).unwrap();
        assert_eq!(shifted.ccr, [0, 89, 100]);
        assert!(valid(shifted, 100, 10, 1));
    }

    #[test]
    fn complete_adc_budget_can_require_an_earlier_trigger() {
        // For c=87: low-side interval is [87, 113]. Settling ends at97 and
        // the complete 16-tick ADC operation ends exactly at the down edge113.
        let plan = plan_sampling([0, 87, 101], 100, 10, 16).unwrap();
        assert_eq!(plan.ccr4, 97);
        assert_eq!(plan.ccr, [0, 87, 101]);
        assert!(valid(plan, 100, 10, 16));
        assert_eq!(plan_sampling([0, 88, 101], 100, 10, 16), None);
        let shifted = plan_sampling([88; 3], 100, 10, 16).unwrap();
        assert_eq!(shifted.ccr, [87; 3]);
        assert_eq!(shifted.ccr4, 97);
    }

    #[test]
    fn long_adc_budget_uses_the_available_up_count_window() {
        let plan = plan_sampling([0, 0, 101], 100, 0, 199).unwrap();
        assert_eq!(plan.ccr4, 1);
        assert!(valid(plan, 100, 0, 199));
        assert_eq!(plan_sampling([0, 0, 101], 100, 0, 200), None);
    }

    #[test]
    fn invalid_geometry_and_full_integer_range_do_not_overflow() {
        for arr in [0, 1] {
            assert_eq!(plan_sampling([0; 3], arr, 0, 0), None);
        }
        assert_eq!(plan_sampling([102, 0, 0], 100, 0, 0), None);
        assert_eq!(plan_sampling([0; 3], 100, 100, 0), None);
        assert_eq!(plan_sampling([0; 3], 100, 0, u32::MAX), None);
        assert_eq!(plan_sampling([0; 3], 100, u32::MAX, 0), None);
        let plan = plan_sampling([u32::MAX; 3], u32::MAX, BOARD_AFTER, BOARD_BEFORE).unwrap();
        assert!(valid(plan, u32::MAX, BOARD_AFTER, BOARD_BEFORE));
        unchanged_line_voltages([u32::MAX; 3], plan.ccr);
    }

    // Brute-force physical time intervals, every pair, and every possible equal
    // shift. This independently establishes existence and the minimum shift.
    fn exhaustive_shift(ccr: [u32; 3], arr: u32, after: u32, before: u32) -> Option<u32> {
        for shift in 0..=*ccr.iter().min().unwrap() {
            let applied = ccr.map(|c| c - shift);
            for pair in [ShuntPair::Uv, ShuntPair::Uw, ShuntPair::Vw] {
                for ccr4 in 1..arr {
                    if valid(
                        SamplePlan {
                            ccr: applied,
                            ccr4,
                            pair,
                        },
                        arr,
                        after,
                        before,
                    ) {
                        return Some(shift);
                    }
                }
            }
        }
        None
    }

    #[test]
    fn small_timer_exhaustion_matches_independent_feasibility_and_minimum_shift() {
        for arr in 2..=7 {
            for after in 0..=9 {
                for before in 0..=9 {
                    for a in 0..=arr + 1 {
                        for b in 0..=arr + 1 {
                            for c in 0..=arr + 1 {
                                let input = [a, b, c];
                                let expected = exhaustive_shift(input, arr, after, before);
                                let actual = plan_sampling(input, arr, after, before);
                                assert_eq!(
                                    actual.map(|p| a - p.ccr[0]),
                                    expected,
                                    "ccr={input:?} arr={arr} after={after} before={before}"
                                );
                                if let Some(plan) = actual {
                                    assert!(valid(plan, arr, after, before));
                                    unchanged_line_voltages(input, plan.ccr);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn svpwm_revolution_preserves_line_voltages_with_valid_sampling() {
        let arr = BOARD_ARR;
        let vbus = 24.0;
        let limit = Svpwm.voltage_limit(vbus);
        let mut shifts = 0;
        for magnitude in [0.0, 0.1, 0.9, 1.0] {
            for i in 0..=1440 {
                let theta = i as f32 * core::f32::consts::TAU / 1440.0;
                let duties = Svpwm.modulate(
                    Rotation::new(theta).inv_park(Dq {
                        d: magnitude * limit,
                        q: 0.0,
                    }),
                    vbus,
                );
                let input = [duties.a, duties.b, duties.c].map(|d| (d * arr as f32) as u32);
                let plan = plan_sampling(input, arr, BOARD_AFTER, BOARD_BEFORE).unwrap();
                assert!(
                    valid(plan, arr, BOARD_AFTER, BOARD_BEFORE),
                    "theta={theta} {plan:?}"
                );
                unchanged_line_voltages(input, plan.ccr);
                shifts += usize::from(input != plan.ccr);
            }
        }
        assert!(shifts > 0, "sweep must exercise common-mode adjustment");
    }
}
