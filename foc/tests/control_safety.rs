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
        pub static ID_MEAS: AtomicI32 = AtomicI32::new(0);
        pub static IQ_MEAS: AtomicI32 = AtomicI32::new(0);
        pub static RAW: AtomicI32 = AtomicI32::new(100);
        pub static SEQUENCE: AtomicI32 = AtomicI32::new(0);
        pub fn control_sequence() -> u32 {
            SEQUENCE.load(SeqCst) as u32
        }
        pub fn id_meas_ma() -> i16 {
            ID_MEAS.load(SeqCst) as i16
        }
        pub fn iq_meas_ma() -> i16 {
            IQ_MEAS.load(SeqCst) as i16
        }
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
            RAW.load(SeqCst) as u16
        }
        pub fn rpm_meas() -> i32 {
            100
        }
    }
}

pub mod driver {
    pub mod pwm {
        use crate::foc::sampling::{plan_sampling, SamplePlan};
        use crate::foc::Duties;
        use std::sync::Mutex;
        #[derive(Default)]
        pub struct MotorPwm {
            pub enabled: bool,
            pub break_pending: bool,
            pub trip_on_enable: bool,
            pub reject_plan: bool,
            pub prepared: Option<Duties>,
        }
        pub static PWM: Mutex<MotorPwm> = Mutex::new(MotorPwm {
            enabled: false,
            break_pending: false,
            trip_on_enable: false,
            reject_plan: false,
            prepared: None,
        });
        impl MotorPwm {
            pub fn disable(&mut self) {
                self.enabled = false;
            }
            pub fn plan(&self, d: Duties) -> Option<SamplePlan> {
                if self.reject_plan {
                    return None;
                }
                plan_sampling(
                    [d.a, d.b, d.c].map(|x| (x * 4250.0) as u32),
                    4250,
                    crate::bsp::config::tw_after_ticks(),
                    crate::bsp::config::tw_before_ticks(),
                )
            }
            pub fn prepare_start(
                &mut self,
                plan: &SamplePlan,
                analog: &mut super::analog::Analog,
            ) -> bool {
                self.disable();
                if self.break_pending {
                    return false;
                }
                analog.schedule_pair(plan.pair);
                self.prepared = Some(Duties {
                    a: plan.ccr[0] as f32 / 4250.0,
                    b: plan.ccr[1] as f32 / 4250.0,
                    c: plan.ccr[2] as f32 / 4250.0,
                });
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
        use std::sync::atomic::{AtomicBool, Ordering::SeqCst};
        pub static AVAILABLE: AtomicBool = AtomicBool::new(true);
        pub struct Analog;
        impl Analog {
            pub fn schedule_pair(&mut self, _: crate::foc::sampling::ShuntPair) {}
        }
        pub fn with_analog<R>(f: impl FnOnce(&mut Analog) -> R) -> Option<R> {
            AVAILABLE.load(SeqCst).then(|| f(&mut Analog))
        }
        pub fn recalibrate() -> bool {
            true
        }
    }
    pub mod nvm {
        use std::sync::atomic::{AtomicUsize, Ordering::SeqCst};
        pub static SAVES: AtomicUsize = AtomicUsize::new(0);
        pub struct Record {
            pub theta_e_off_mrad: i32,
            pub poles: u8,
        }
        pub fn load() -> Option<Record> {
            None
        }
        pub fn save(_: Record) -> bool {
            SAVES.fetch_add(1, SeqCst);
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

    assert!(!c::calibration_valid());
    assert!(!c::load_nvm());
    assert!(!c::start(), "missing calibration blocks Run");
    assert!(!c::start_speed(100), "missing calibration blocks Speed");
    assert!(!c::persist_nvm(), "cannot save an unqualified default");
    let default_offset = c::theta_e_off_mrad();
    assert!(c::capture_electrical_offset());
    assert_ne!(c::theta_e_off_mrad(), default_offset);
    assert!(!c::calibration_valid(), "position capture is unverified");
    assert!(!c::start(), "capturing a position cannot authorize Run");
    assert!(
        !c::start_speed(100),
        "capturing a position cannot authorize Speed"
    );
    assert!(!c::persist_nvm(), "cannot save an unverified capture");
    assert_eq!(driver::nvm::SAVES.load(SeqCst), 0);
    assert!(!PWM.lock().unwrap().enabled);

    let captured_offset = c::theta_e_off_mrad();
    t::ENC_VALID.store(false, SeqCst);
    t::RAW.store(150, SeqCst);
    assert!(!c::capture_electrical_offset());
    assert_eq!(c::theta_e_off_mrad(), captured_offset);
    assert!(!c::calibration_valid());
    t::ENC_VALID.store(true, SeqCst);
    assert!(
        c::start_openloop(500, 2),
        "Openloop does not need rotor zero"
    );
    c::stop();

    // Reference arrival alone cannot complete alignment, even at a tiny target.
    let old_offset = c::theta_e_off_mrad();
    assert!(c::request_align(Some(1)));
    for _ in 0..crate::bsp::config::ALIGN_TIMEOUT_MS {
        t::SEQUENCE.fetch_add(1, SeqCst);
        c::tick(1);
    }
    assert!(c::mode() == Mode::Fault);
    assert!(c::last_fault() == FaultKind::Alignment);
    assert!(!c::calibration_valid());
    assert_eq!(c::theta_e_off_mrad(), old_offset);
    assert!(!PWM.lock().unwrap().enabled);
    c::stop();

    assert!(c::request_align(None));
    t::ID_MEAS.store(500, SeqCst);
    for _ in 0..300 {
        t::SEQUENCE.fetch_add(1, SeqCst);
        c::tick(1);
    }
    // Motion and bad feedback each restart the hold.
    t::RAW.store(200, SeqCst);
    t::SEQUENCE.fetch_add(1, SeqCst);
    c::tick(1);
    assert_eq!(c::align_left_ms(), crate::bsp::config::ALIGN_MS);
    t::ID_MEAS.store(0, SeqCst);
    t::SEQUENCE.fetch_add(1, SeqCst);
    c::tick(1);
    t::ID_MEAS.store(500, SeqCst);
    for _ in 0..500 {
        t::SEQUENCE.fetch_add(1, SeqCst);
        c::tick(1);
    }
    assert!(c::mode() == Mode::Align, "full continuous hold required");
    t::SEQUENCE.fetch_add(1, SeqCst);
    c::tick(1);
    assert!(c::mode() == Mode::Idle);
    assert!(c::calibration_valid());
    assert!(!PWM.lock().unwrap().enabled);
    assert!(c::persist_nvm(), "qualified alignment can be saved");
    assert_eq!(driver::nvm::SAVES.load(SeqCst), 1);
    assert!(c::start(), "qualified alignment authorizes Run");
    c::stop();
    assert!(c::start_speed(100), "qualified alignment authorizes Speed");
    c::stop();

    // A fresh capture must revoke even a previously qualified calibration.
    let aligned_offset = c::theta_e_off_mrad();
    t::RAW.store(350, SeqCst);
    assert!(c::capture_electrical_offset());
    assert_ne!(c::theta_e_off_mrad(), aligned_offset);
    assert!(!c::calibration_valid());
    assert!(!c::start());
    assert!(!c::start_speed(100));
    assert!(!c::persist_nvm());
    assert_eq!(driver::nvm::SAVES.load(SeqCst), 1);

    // Explicit operator override remains the way to verify a known offset.
    assert!(c::set_theta_e_off_mrad(aligned_offset));
    let verified_offset = c::theta_e_off_mrad();
    t::ENC_VALID.store(false, SeqCst);
    assert!(!c::capture_electrical_offset());
    assert_eq!(c::theta_e_off_mrad(), verified_offset);
    assert!(c::calibration_valid());
    t::ENC_VALID.store(true, SeqCst);
    let poles = c::poles();
    assert!(!c::set_poles(0));
    assert!(c::set_poles(poles));
    assert!(c::calibration_valid());
    assert!(c::set_poles(poles + 1));
    assert!(!c::calibration_valid());
    assert!(!c::start());
    // Manual offset is an explicit operator override.
    assert!(c::set_theta_e_off_mrad(200));
    assert!(c::calibration_valid());
    assert!(c::persist_nvm(), "operator-verified offset can be saved");
    assert_eq!(driver::nvm::SAVES.load(SeqCst), 2);

    c::set_id_ma(200);
    c::set_iq_ma(300);
    assert!(c::start());
    assert!(c::mode() == Mode::Run);
    let prepared = PWM.lock().unwrap().prepared.unwrap();
    assert_eq!((prepared.a, prepared.b, prepared.c), (0.5, 0.5, 0.5));
    assert!(!c::start_openloop(1000, 2), "live mode change needs stop");
    assert!(!c::set_poles(4));
    assert!(!c::set_theta_e_off_mrad(200));
    let running_offset = c::theta_e_off_mrad();
    t::RAW.store(500, SeqCst);
    assert!(!c::capture_electrical_offset());
    assert_eq!(c::theta_e_off_mrad(), running_offset);
    assert!(c::calibration_valid());
    assert!(c::mode() == Mode::Run);
    assert!(PWM.lock().unwrap().enabled);
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

    // Reject sampling failures before enabling PWM, with an ADC fault rather
    // than misreporting a brake trip or silently running on an invalid pair.
    PWM.lock().unwrap().reject_plan = true;
    assert!(!c::start());
    assert!(c::mode() == Mode::Fault);
    assert!(c::last_fault() == FaultKind::Adc);
    assert!(!PWM.lock().unwrap().enabled);
    PWM.lock().unwrap().reject_plan = false;
    c::stop();
    driver::analog::AVAILABLE.store(false, SeqCst);
    assert!(!c::start());
    assert!(c::last_fault() == FaultKind::Adc);
    assert!(!PWM.lock().unwrap().enabled);
    driver::analog::AVAILABLE.store(true, SeqCst);
    c::stop();

    // All live modes, including Bench and Openloop, require ADC progress.
    for mode in [
        Mode::Run,
        Mode::Speed,
        Mode::Bench,
        Mode::Openloop,
        Mode::Align,
    ] {
        assert!(c::set_theta_e_off_mrad(200));
        assert!(match mode {
            Mode::Run => c::start(),
            Mode::Speed => c::start_speed(100),
            Mode::Bench => c::start_bench(),
            Mode::Openloop => c::start_openloop(500, 2),
            Mode::Align => c::request_align(None),
            _ => unreachable!(),
        });
        c::supervise_current(100);
        c::supervise_current(104);
        assert!(c::mode() == mode);
        t::SEQUENCE.fetch_add(1, SeqCst);
        c::supervise_current(105);
        c::supervise_current(109);
        assert!(c::mode() == mode);
        c::supervise_current(110);
        assert!(c::mode() == Mode::Fault);
        assert!(c::last_fault() == FaultKind::ControlStall);
        assert!(!PWM.lock().unwrap().enabled);
        assert_eq!(
            (c::id_target_ma(), c::iq_target_ma(), c::rpm_target()),
            (0, 0, 0)
        );
        c::stop();
    }
}
