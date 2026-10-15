//! Actual telemetry publications with a deterministic clock and hardware-free
//! critical sections. Sensor freshness includes acquisition/transport latency.
#![allow(dead_code)]

extern crate self as cortex_m;
extern crate self as embassy_time;
pub use foc;

use std::sync::atomic::{AtomicU64, Ordering::SeqCst};
static NOW_US: AtomicU64 = AtomicU64::new(0);
pub struct Instant(u64);
impl Instant {
    pub fn now() -> Self {
        Self(NOW_US.load(SeqCst))
    }
    pub fn as_micros(&self) -> u64 {
        self.0
    }
    pub fn as_millis(&self) -> u64 {
        self.0 / 1000
    }
}

pub mod interrupt {
    pub struct Mutex<T>(std::sync::Mutex<T>);
    impl<T> Mutex<T> {
        pub const fn new(value: T) -> Self {
            Self(std::sync::Mutex::new(value))
        }
        pub fn borrow<'a>(&'a self, _: &()) -> std::sync::MutexGuard<'a, T> {
            self.0.lock().unwrap()
        }
    }
    pub fn free<R>(f: impl FnOnce(&()) -> R) -> R {
        f(&())
    }
}

#[path = "../../src/bsp/config.rs"]
pub mod config;
#[path = "../../src/bsp/motor.rs"]
pub mod motor;
pub mod bsp {
    pub use crate::config;
}
pub mod driver {
    pub mod analog {
        #[derive(Clone, Copy, Default)]
        pub struct AnalogSample {
            pub iu_raw: u16,
            pub iv_raw: u16,
            pub iw_raw: u16,
            pub iu_a: f32,
            pub iv_a: f32,
            pub iw_a: f32,
            pub vbus_v: f32,
            pub temp_c: f32,
            pub bus_sampled_at_ms: u32,
        }
    }
}
#[path = "../../src/app/telemetry.rs"]
mod telemetry;

#[test]
fn acquisition_age_gaps_and_errors_cannot_publish_false_freshness_or_old_speed() {
    use telemetry as t;
    assert!(!t::enc_valid());
    for us in (0..=110_000).step_by(500) {
        NOW_US.store(us + 300, SeqCst);
        t::publish_angle_at(((us / 500) * 10) as u16, 0.5, 1.0, true, us);
    }
    assert!(t::rpm_ready());
    assert_eq!(t::rpm_meas(), 292);
    assert_eq!(t::enc_age_us(), Some(300));
    assert!(t::electrical_angle(0.0, 7).is_some());

    // Publishing now cannot hide an ANGLE transaction that started 20ms ago.
    NOW_US.store(130_000, SeqCst);
    t::publish_angle_at(10, 0.5, 1.0, true, 110_000);
    assert!(!t::enc_valid());
    assert!(!t::rpm_ready());
    assert_eq!(t::rpm_meas(), 0);
    assert_eq!(t::electrical_angle(0.0, 7), None);

    t::publish_angle_at(100, 0.5, 1.0, true, 130_000);
    assert!(t::enc_valid());
    assert!(!t::rpm_ready());
    NOW_US.store(131_000, SeqCst);
    t::publish_angle_at(100, 0.5, 1.0, true, 132_000); // future timestamp
    assert!(!t::enc_valid());
    t::publish_angle_at(100, f32::NAN, 1.0, true, 131_000);
    assert!(!t::enc_valid());

    // A valid sample can survive an ordinary microsecond clock wrap.
    let wrap = u64::from(u32::MAX) + 1;
    NOW_US.store(wrap - 200, SeqCst);
    t::publish_angle_at(4095, 0.5, 1.0, true, wrap - 500);
    NOW_US.store(wrap + 200, SeqCst);
    assert_eq!(t::enc_age_us(), Some(700));
    t::publish_angle_at(0, 0.0, 1.0, true, wrap);
    assert!(!t::rpm_ready());
    NOW_US.store(wrap + 20_000, SeqCst);
    assert!(!t::enc_valid());
    NOW_US.store(wrap, SeqCst);
    assert!(
        !t::enc_valid(),
        "expired samples cannot revive when the clock wraps"
    );

    NOW_US.store(100_000, SeqCst);
    t::publish_bus(driver::analog::AnalogSample {
        vbus_v: 24.0,
        temp_c: 25.0,
        bus_sampled_at_ms: 80,
        ..Default::default()
    });
    assert!(!t::bus_snapshot().2);
    t::publish_bus(driver::analog::AnalogSample {
        vbus_v: 24.0,
        temp_c: 25.0,
        bus_sampled_at_ms: 99,
        ..Default::default()
    });
    assert_eq!(t::bus_snapshot(), (24_000, 250, true));
    t::publish_bus(driver::analog::AnalogSample {
        vbus_v: f32::NAN,
        temp_c: 25.0,
        bus_sampled_at_ms: 100,
        ..Default::default()
    });
    assert!(!t::bus_snapshot().2);
}
