//! Host regression harness for the actual firmware control module.
//! Only hardware/IRQ/telemetry edges are mocked; state and command code is shared.
//! This checks software ordering, not peripheral timing or electrical safety.
#![allow(dead_code)]

extern crate self as cortex_m;
extern crate self as defmt;
pub use foc;

#[macro_export]
macro_rules! info {
    ($($arg:tt)*) => {};
}

#[path = "../../src/bsp/config.rs"]
pub mod config_impl;
#[path = "../../src/app/control.rs"]
pub mod control_impl;
#[path = "../../src/bsp/motor.rs"]
pub mod motor;

pub mod bsp {
    pub use crate::config_impl as config;
}

pub mod interrupt {
    use crate::control_impl::{self as control, FaultKind};
    use std::cell::Cell;
    thread_local! {
        static DEPTH: Cell<usize> = const { Cell::new(0) };
        static PENDING: Cell<Option<FaultKind>> = const { Cell::new(None) };
    }
    // Host analogue of the Cortex-M Cell mutex; no unsafe shared test state.
    pub struct Mutex<T>(std::sync::Mutex<T>);
    impl<T> Mutex<T> {
        pub const fn new(value: T) -> Self {
            Self(std::sync::Mutex::new(value))
        }
        pub fn borrow<'a>(&'a self, _: &()) -> std::sync::MutexGuard<'a, T> {
            assert!(DEPTH.get() > 0, "shared state requires a critical section");
            self.0.lock().unwrap()
        }
    }

    pub fn pend_fault(kind: FaultKind) {
        if DEPTH.get() == 0 {
            control::fault(kind);
        } else {
            PENDING.set(Some(kind));
        }
    }
    pub fn free<R>(f: impl FnOnce(&()) -> R) -> R {
        struct Guard;
        impl Drop for Guard {
            fn drop(&mut self) {
                DEPTH.set(DEPTH.get() - 1);
                if DEPTH.get() == 0 {
                    if let Some(kind) = PENDING.take() {
                        control::fault(kind);
                    }
                }
            }
        }
        DEPTH.set(DEPTH.get() + 1);
        let _guard = Guard;
        f(&())
    }
}

pub mod app {
    pub use crate::control_impl as control;
    pub mod foc_isr {
        use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering::SeqCst};
        pub static RESETS: AtomicUsize = AtomicUsize::new(0);
        pub static TRIP_ON_RESET: AtomicBool = AtomicBool::new(false);
        pub fn reset() {
            RESETS.fetch_add(1, SeqCst);
            if TRIP_ON_RESET.swap(false, SeqCst) {
                crate::interrupt::pend_fault(super::control::FaultKind::Overcurrent);
            }
        }
        pub fn reset_openloop() {}
        pub fn set_gains(_: f32, _: f32) {}
    }
    pub mod speed {
        use std::sync::atomic::{AtomicUsize, Ordering::SeqCst};
        pub static RESETS: AtomicUsize = AtomicUsize::new(0);
        pub fn reset() {
            RESETS.fetch_add(1, SeqCst);
        }
        pub fn set_gains(_: f32, _: f32) {}
    }
    pub mod telemetry {
        use std::sync::atomic::{AtomicBool, AtomicI32, Ordering::SeqCst};
        pub static BUS_VALID: AtomicBool = AtomicBool::new(false);
        pub static ENC_VALID: AtomicBool = AtomicBool::new(false);
        pub static VBUS: AtomicI32 = AtomicI32::new(24_000);
        pub static TEMP: AtomicI32 = AtomicI32::new(250);
        pub fn bus_snapshot() -> (u16, i16, bool) {
            (
                VBUS.load(SeqCst) as u16,
                TEMP.load(SeqCst) as i16,
                BUS_VALID.load(SeqCst),
            )
        }
        pub fn enc_valid() -> bool {
            ENC_VALID.load(SeqCst)
        }
        pub fn enc_raw() -> u16 {
            100
        }
        pub fn rpm_meas() -> i32 {
            100
        }
    }
}

pub mod driver {
    pub mod pwm {
        use crate::foc::Duties;
        use std::sync::Mutex;
        #[derive(Default)]
        pub struct MotorPwm {
            pub enabled: bool,
            pub break_pending: bool,
            pub trip_on_enable: bool,
            pub prepared: Option<Duties>,
        }
        pub static PWM: Mutex<MotorPwm> = Mutex::new(MotorPwm {
            enabled: false,
            break_pending: false,
            trip_on_enable: false,
            prepared: None,
        });
        impl MotorPwm {
            pub fn disable(&mut self) {
                self.enabled = false;
            }
            pub fn prepare_start(&mut self, duties: Duties) -> bool {
                self.disable();
                if self.break_pending {
                    return false;
                }
                self.prepared = Some(duties);
                self.enabled = true;
                if self.trip_on_enable {
                    self.trip_on_enable = false;
                    crate::interrupt::pend_fault(crate::control_impl::FaultKind::Brake);
                }
                true
            }
        }
        pub fn with_pwm<R>(f: impl FnOnce(&mut MotorPwm) -> R) -> Option<R> {
            crate::interrupt::free(|_| Some(f(&mut PWM.lock().unwrap())))
        }
    }
    pub mod analog {
        pub struct Analog;
        impl crate::foc::DutySink for Analog {
            fn apply(&mut self, _: crate::foc::Duties) {}
        }
        pub fn with_analog<R>(f: impl FnOnce(&mut Analog) -> R) -> Option<R> {
            Some(f(&mut Analog))
        }
        pub fn recalibrate() -> bool {
            true
        }
    }
    pub mod nvm {
        pub struct Record {
            pub theta_e_off_mrad: i32,
            pub poles: u8,
        }
        pub fn load() -> Option<Record> {
            None
        }
        pub fn save(_: Record) -> bool {
            true
        }
    }
}

// One test serializes access to the firmware's process-global state.
#[test]
fn control_safety_regressions() {
    use app::{control as c, foc_isr, speed, telemetry as t};
    use c::{FaultKind, Mode};
    use driver::pwm::PWM;
    use std::sync::atomic::Ordering::SeqCst;

    c::stop();
    assert!(!c::start(), "no bus/encoder sample must block start");
    assert!(!PWM.lock().unwrap().enabled);
    t::BUS_VALID.store(true, SeqCst);
    assert!(!c::start(), "closed loop requires encoder");
    t::ENC_VALID.store(true, SeqCst);
    t::VBUS.store(30_000, SeqCst);
    assert!(!c::start(), "overvoltage must block start");
    t::VBUS.store(24_000, SeqCst);
    t::TEMP.store(800, SeqCst);
    assert!(!c::start(), "overtemperature must block start");
    t::TEMP.store(250, SeqCst);

    c::set_id_ma(200);
    c::set_iq_ma(300);
    assert!(c::start());
    assert!(c::mode() == Mode::Run);
    let prepared = PWM.lock().unwrap().prepared.unwrap();
    assert_eq!((prepared.a, prepared.b, prepared.c), (0.5, 0.5, 0.5));
    assert!(!c::start_openloop(1000, 2), "live mode change needs stop");
    assert!(!c::set_poles(4));
    assert!(!c::set_theta_e_off_mrad(200));
    assert!(!c::capture_electrical_offset());
    c::stop();
    assert!(!PWM.lock().unwrap().enabled);
    assert_eq!(
        (c::id_target_ma(), c::iq_target_ma(), c::rpm_target()),
        (0, 0, 0)
    );

    // Fault arrives after the initial mode check, before PWM enable.
    foc_isr::TRIP_ON_RESET.store(true, SeqCst);
    c::start();
    assert!(
        c::mode() == Mode::Fault,
        "start must not overwrite the pending trip"
    );
    assert!(!PWM.lock().unwrap().enabled);
    assert!(c::last_fault() == FaultKind::Overcurrent);
    c::fault(FaultKind::Encoder);
    assert!(
        c::last_fault() == FaultKind::Overcurrent,
        "retain first fault"
    );
    assert!(!c::start_speed(200));

    c::stop();
    PWM.lock().unwrap().trip_on_enable = true;
    c::start();
    assert!(
        c::mode() == Mode::Fault,
        "trip at enable must not be lost either"
    );
    assert!(!PWM.lock().unwrap().enabled);

    c::stop();
    assert!(c::start_speed(200));
    let resets = (foc_isr::RESETS.load(SeqCst), speed::RESETS.load(SeqCst));
    let rpm_ref = c::rpm_ref();
    assert!(c::start_speed(300));
    assert_eq!(c::rpm_target(), 300);
    assert_eq!(c::rpm_ref(), rpm_ref);
    assert_eq!(
        resets,
        (foc_isr::RESETS.load(SeqCst), speed::RESETS.load(SeqCst))
    );
    // Exact elapsed-time ramps: 800 RPM/s and 800 mA/s, not one unit every ms.
    c::write_iq_ma(1500);
    c::tick(0);
    assert_eq!(c::iq_ma(), 0);
    assert_eq!(c::rpm_ref(), rpm_ref);
    for _ in 0..100 {
        c::tick(1);
    }
    assert_eq!(c::iq_ma(), 80);
    assert_eq!(c::rpm_ref(), rpm_ref + 80);
    c::tick(2); // Leave a fractional position before the fault/reset.
    c::fault(FaultKind::Encoder);
    c::write_iq_ma(500); // A preempted speed update must not restore a command.
    assert_eq!(c::iq_target_ma(), 0);
    assert_eq!(c::iq_ma(), 0);
    c::stop();
    assert!(c::start_speed(300));
    c::write_iq_ma(100);
    for _ in 0..5 {
        c::tick(1);
    }
    assert_eq!(
        c::iq_ma(),
        4,
        "old fractional reference must not survive a fault"
    );
    assert_eq!(c::rpm_ref(), 104);
    c::stop();
    assert!(c::start_speed(300));
    c::write_iq_ma(100);
    c::tick(5);
    assert_eq!(c::iq_ma(), 4, "stop resets the fractional reference too");
    assert_eq!(c::rpm_ref(), 104);

    c::stop();
    PWM.lock().unwrap().break_pending = true;
    assert!(!c::start());
    assert!(c::mode() == Mode::Fault);
    assert!(!PWM.lock().unwrap().enabled);
    PWM.lock().unwrap().break_pending = false;
    c::stop();
}
