//! Link the non-test no_std library, so test-prelude method resolution cannot
//! silently substitute std math in the production implementation.
use foc::transforms::Rotation;
use foc::types::AlphaBeta;
use foc::{CircleLimit, VdPriority, VoltageLimiter};

#[test]
fn production_voltage_limits_stay_on_circle() {
    let (d, q) = VdPriority.limit(8.0, 20.0, 10.0);
    assert!((q - 6.0).abs() < 1e-6, "regression: sqrt(36) must be 6");
    assert!((d * d + q * q).sqrt() <= 10.0 + 1e-6);
    for limit in [0.0, 1.0, 4.619, 10.0, 13.856, 16.165] {
        for di in -100..=100 {
            for qi in [-100, -25, 0, 25, 100] {
                let vd = di as f32 * 0.25;
                let vq = qi as f32 * 0.25;
                for limiter in [&VdPriority as &dyn VoltageLimiter, &CircleLimit] {
                    let (d, q) = limiter.limit(vd, vq, limit);
                    assert!(d.is_finite() && q.is_finite());
                    assert!((d * d + q * q).sqrt() <= limit + 3e-6);
                }
            }
        }
    }
}

#[test]
fn production_rotation_matches_independent_std_reference() {
    let ab = AlphaBeta {
        alpha: 3.2,
        beta: -1.7,
    };
    for i in -2048..=2048 {
        let theta = i as f32 * core::f32::consts::TAU / 1024.0;
        let (s, c) = theta.sin_cos();
        let rotation = Rotation::new(theta);
        let dq = rotation.park(ab);
        assert!((dq.d - (ab.alpha * c + ab.beta * s)).abs() < 2e-6);
        assert!((dq.q - (-ab.alpha * s + ab.beta * c)).abs() < 2e-6);
        let back = rotation.inv_park(dq);
        assert!((back.alpha - ab.alpha).abs() < 2e-6);
        assert!((back.beta - ab.beta).abs() < 2e-6);
    }
}
