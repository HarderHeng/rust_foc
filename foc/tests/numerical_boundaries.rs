//! Exercise the non-test library. Invalid control quantities must remain
//! detectable without poisoning the state used by a later valid sample.

use foc::angle::AngleTrack;
use foc::transforms::wrap_2pi;
use foc::types::AlphaBeta;
use foc::{
    CircleLimit, CurrentLoop, Dq, Modulator, Pi, Pid, SpeedLoop, Svpwm, VdPriority, VoltageLimiter,
};

#[test]
fn invalid_pid_samples_leave_integral_and_derivative_history_unchanged() {
    let mut original = Pid::pid(2.0, 3.0, 0.1, -10.0, 10.0);
    original.d_tau = 0.01;
    original.step(0.3, 0.001);
    for (error, dt) in [
        (f32::NAN, 0.001),
        (f32::INFINITY, 0.001),
        (f32::NEG_INFINITY, 0.001),
        (0.5, f32::NAN),
        (0.5, f32::INFINITY),
        (0.5, f32::NEG_INFINITY),
        (0.5, -0.001),
        (f32::MAX, 1.0),
    ] {
        let mut tested = original;
        assert!(tested.step(error, dt).is_nan());
        assert_eq!(tested.integrator(), original.integrator());
        assert_eq!(tested.last_output(), original.last_output());
        let mut expected = original;
        assert_eq!(tested.step(0.4, 0.001), expected.step(0.4, 0.001));
    }
}

#[test]
fn invalid_limits_and_tracking_do_not_panic_or_prevent_recovery() {
    for limit in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        let mut p = Pi::new(1.0, 1.0, -limit, limit);
        assert!(p.step(1.0, 0.001).is_nan());
        p.set_limits(-1.0, 1.0);
        assert!(p.step(0.1, 0.001).is_finite());
        let before = p;
        p.track(limit);
        assert_eq!(p.integrator(), before.integrator());
        assert_eq!(p.last_output(), before.last_output());
        p.set_limits(-1.0, limit);
        assert!(p.step(0.1, 0.001).is_nan());
        p.set_limits(-1.0, 1.0);
        assert!(p.step(0.1, 0.001).is_finite());
    }
}

#[test]
fn voltage_limiters_handle_tiny_and_huge_finite_vectors() {
    for limiter in [&CircleLimit as &dyn VoltageLimiter, &VdPriority] {
        let (d, q) = limiter.limit(1e-25, -1e-25, 0.0);
        assert_eq!((d, q), (0.0, 0.0));
        for (d, q, max) in [
            (f32::MAX, -f32::MAX, 10.0),
            (8e29, 2e30, 1e30),
            (8e-26, -2e-25, 1e-25),
        ] {
            let (d, q) = limiter.limit(d, q, max);
            assert!(d.is_finite() && q.is_finite());
            let relative_magnitude =
                (f64::from(d) / f64::from(max)).hypot(f64::from(q) / f64::from(max));
            assert!(relative_magnitude <= 1.0 + 2e-7, "{d} {q} {max}");
        }
        for invalid in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            assert!(limiter.limit(invalid, 0.0, 10.0).0.is_nan());
            assert!(limiter.limit(0.0, invalid, 10.0).1.is_nan());
            assert!(limiter.limit(0.0, 0.0, invalid).0.is_nan());
        }
    }
    let (d, q) = CircleLimit.limit(f32::MAX, -f32::MAX, 10.0);
    assert!((f64::from(d) - 50.0_f64.sqrt()).abs() < 1e-6);
    assert_eq!(d, -q);
}

#[test]
fn modulator_uses_actual_subvolt_bus_and_keeps_invalid_bus_detectable() {
    let d = Svpwm.modulate(
        AlphaBeta {
            alpha: 0.1,
            beta: 0.0,
        },
        0.5,
    );
    assert!((d.a - 0.65).abs() < 1e-6);
    assert!((d.b - 0.35).abs() < 1e-6);
    assert_eq!(d.b, d.c);
    for bus in [0.0, -1.0, f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        let d = Svpwm.modulate(AlphaBeta::default(), bus);
        assert!(d.a.is_nan() && d.b.is_nan() && d.c.is_nan());
        assert!(Svpwm.voltage_limit(bus).is_nan());
    }
}

#[test]
fn invalid_current_loop_inputs_reject_the_command_and_allow_the_next_valid_sample() {
    for (refs, theta, bus, dt, ff) in [
        (
            Dq {
                d: f32::NAN,
                q: 0.0,
            },
            0.0,
            24.0,
            0.001,
            Dq::default(),
        ),
        (Dq::default(), f32::INFINITY, 24.0, 0.001, Dq::default()),
        (Dq::default(), 0.0, f32::NAN, 0.001, Dq::default()),
        (Dq::default(), 0.0, 24.0, f32::INFINITY, Dq::default()),
        (
            Dq::default(),
            0.0,
            24.0,
            0.001,
            Dq {
                d: 0.0,
                q: f32::NAN,
            },
        ),
        (
            Dq {
                d: f32::MAX,
                q: f32::MAX,
            },
            0.0,
            24.0,
            1.0,
            Dq::default(),
        ),
    ] {
        let mut loop_ = CurrentLoop::new(2.0, 3.0, 10.0);
        let bad = loop_.step_debug((0.0, 0.0), refs, theta, bus, dt, ff);
        assert!([bad.duties.a, bad.duties.b, bad.duties.c]
            .iter()
            .any(|x| !x.is_finite()));
        let good = loop_.step_debug(
            (0.0, 0.0),
            Dq { d: 0.1, q: 0.2 },
            0.0,
            24.0,
            0.001,
            Dq::default(),
        );
        assert!([good.duties.a, good.duties.b, good.duties.c]
            .iter()
            .all(|&x| (0.0..=1.0).contains(&x)));
    }
    let mut loop_ = CurrentLoop::new(2.0, 3.0, 10.0);
    for currents in [(f32::NAN, 0.0), (0.0, f32::INFINITY)] {
        let bad = loop_.step_debug(currents, Dq::default(), 0.0, 24.0, 0.001, Dq::default());
        assert!([bad.duties.a, bad.duties.b, bad.duties.c]
            .iter()
            .any(|x| !x.is_finite()));
    }
    let good = loop_.step_debug((0.0, 0.0), Dq::default(), 0.0, 24.0, 0.001, Dq::default());
    assert!([good.duties.a, good.duties.b, good.duties.c]
        .iter()
        .all(|x| x.is_finite()));
}

#[test]
fn angle_wrap_stays_strictly_below_tau_even_after_rounding() {
    for angle in [
        -f32::MIN_POSITIVE,
        -f32::EPSILON,
        -1e-30,
        -0.0,
        0.0,
        f32::MAX,
        -f32::MAX,
    ] {
        let wrapped = wrap_2pi(angle);
        assert!(
            (0.0..std::f32::consts::TAU).contains(&wrapped),
            "{angle} => {wrapped}"
        );
    }
    for angle in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        assert!(wrap_2pi(angle).is_nan());
    }
}

#[test]
fn invalid_velocity_filter_tuning_does_not_poison_retained_speed() {
    let mut tracker = AngleTrack::new();
    tracker.alpha = 1.0;
    tracker.omega_window = 0.001;
    tracker.push(0, 0.001);
    let (_, old_speed) = tracker.push(1, 0.001);
    tracker.alpha = f32::NAN;
    assert_eq!(tracker.push(2, 0.001).1, old_speed);
    tracker.alpha = 1.0;
    tracker.omega_window = f32::INFINITY;
    assert_eq!(tracker.push(3, 0.001).1, old_speed);
    tracker.omega_window = 0.001;
    let (_, speed) = tracker.push(5, 0.001);
    assert!(speed.is_finite());
    assert!((speed - old_speed * 2.0).abs() < 1e-6);
}

#[test]
fn invalid_speed_samples_cannot_change_tracking_state() {
    let mut speed = SpeedLoop::new(0.002, 0.004, 0.12);
    speed.step_same_direction(100.0, 10.0, 0.1, 0.12, 0.0);
    let before = speed.pi;
    assert!(speed
        .step_same_direction(100.0, f32::NAN, 0.1, 0.12, 0.02)
        .is_nan());
    assert_eq!(speed.pi.integrator(), before.integrator());
    assert_eq!(speed.pi.last_output(), before.last_output());
}
