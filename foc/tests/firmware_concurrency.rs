//! Host execution of the actual controller storage, FOC ISR, speed loop and control state machine.
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

#[path = "../../src/app/controller.rs"]
pub mod controller_impl;
#[path = "../../src/app/foc_isr.rs"]
pub mod foc_isr_impl;
#[path = "../../src/app/speed.rs"]
pub mod speed_impl;

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

    pub fn masked() -> bool {
        DEPTH.get() > 0
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
    pub use crate::controller_impl as controller;
    pub use crate::foc_isr_impl as foc_isr;
    pub use crate::speed_impl as speed;
    pub mod telemetry {
        use crate::foc::Dq;
        use std::cell::Cell;
        use std::sync::atomic::{AtomicBool, AtomicI32, Ordering::SeqCst};
        use std::sync::Mutex;
        pub static BUS_VALID: AtomicBool = AtomicBool::new(true);
        pub static ENC_VALID: AtomicBool = AtomicBool::new(true);
        pub static READY: AtomicBool = AtomicBool::new(true);
        pub static RPM: AtomicI32 = AtomicI32::new(0);
        pub static VDQ: Mutex<Dq> = Mutex::new(Dq { d: 0.0, q: 0.0 });
        thread_local! { pub static RPM_HOOK: Cell<Option<fn()>> = const { Cell::new(None) }; }
        pub fn control_sequence() -> u32 {
            0
        }
        pub fn id_meas_ma() -> i16 {
            0
        }
        pub fn iq_meas_ma() -> i16 {
            0
        }
        pub fn bus_snapshot() -> (u16, i16, bool) {
            (24_000, 250, BUS_VALID.load(SeqCst))
        }
        pub fn enc_valid() -> bool {
            ENC_VALID.load(SeqCst)
        }
        pub fn enc_raw() -> u16 {
            0
        }
        pub fn rpm_meas() -> i32 {
            if let Some(hook) = RPM_HOOK.take() {
                hook();
            }
            RPM.load(SeqCst)
        }
        pub fn rpm_ready() -> bool {
            READY.load(SeqCst)
        }
        pub fn reset_speed_filter() {
            READY.store(false, SeqCst);
        }
        pub fn vbus_mv() -> u16 {
            24_000
        }
        pub fn electrical_angle(_: f32, _: u8) -> Option<f32> {
            enc_valid().then_some(0.0)
        }
        pub fn omega_e_ff(_: u8) -> f32 {
            0.0
        }
        pub fn publish_currents(_: crate::driver::analog::AnalogSample) {}
        pub fn publish_dq(_: Dq) {
            assert!(crate::interrupt::masked());
        }
        pub fn publish_vdq(applied: Dq, _: Dq) {
            *VDQ.lock().unwrap() = applied;
        }
        pub fn publish_duties(_: crate::foc::Duties) {
            assert!(crate::interrupt::masked());
        }
    }
}

pub mod driver {
    pub mod pwm {
        use crate::foc::sampling::{plan_sampling, SamplePlan};
        use crate::foc::Duties;
        use std::cell::Cell;
        use std::sync::atomic::{AtomicBool, Ordering::SeqCst};
        use std::sync::Mutex;
        thread_local! { pub static PLAN_HOOK: Cell<Option<fn()>> = const { Cell::new(None) }; }
        pub static PLAN_FAIL: AtomicBool = AtomicBool::new(false);
        #[derive(Default)]
        pub struct MotorPwm {
            pub enabled: bool,
            pub commits: usize,
        }
        pub static PWM: Mutex<MotorPwm> = Mutex::new(MotorPwm {
            enabled: false,
            commits: 0,
        });
        fn make_plan(d: Duties) -> Option<SamplePlan> {
            plan_sampling(
                [d.a, d.b, d.c].map(|x| (x * 4249.0) as u32),
                4249,
                crate::bsp::config::tw_after_ticks(),
                crate::bsp::config::tw_before_ticks(),
            )
        }
        pub fn plan_duties(d: Duties) -> Option<SamplePlan> {
            assert!(
                !crate::interrupt::masked(),
                "sampling geometry must not mask interrupts"
            );
            if let Some(hook) = PLAN_HOOK.take() {
                hook();
            }
            if PLAN_FAIL.load(SeqCst) {
                None
            } else {
                make_plan(d)
            }
        }
        impl MotorPwm {
            pub fn disable(&mut self) {
                self.enabled = false;
            }
            pub fn is_enabled(&self) -> bool {
                self.enabled
            }
            pub fn plan(&self, d: Duties) -> Option<SamplePlan> {
                make_plan(d)
            }
            pub fn prepare_start(&mut self, _: &SamplePlan, _: &mut super::analog::Analog) -> bool {
                self.enabled = true;
                true
            }
            pub fn apply_plan(
                &mut self,
                plan: &SamplePlan,
                _: &mut super::analog::Analog,
            ) -> Option<Duties> {
                assert!(crate::interrupt::masked(), "hardware commit must be atomic");
                assert!(self.enabled, "never submit output after a break or stop");
                self.commits += 1;
                Some(Duties {
                    a: plan.ccr[0] as f32 / 4249.0,
                    b: plan.ccr[1] as f32 / 4249.0,
                    c: plan.ccr[2] as f32 / 4249.0,
                })
            }
        }
        pub fn with_pwm<R>(f: impl FnOnce(&mut MotorPwm) -> R) -> Option<R> {
            crate::interrupt::free(|_| Some(f(&mut PWM.lock().unwrap())))
        }
    }
    pub mod analog {
        use crate::foc::{PhaseAbc, PhaseCurrents};
        use std::cell::Cell;
        thread_local! { pub static SAMPLE_HOOK: Cell<Option<fn()>> = const { Cell::new(None) }; }
        #[derive(Clone, Copy, Default)]
        pub struct AnalogSample {
            pub iu_a: f32,
            pub iv_a: f32,
            pub iw_a: f32,
        }
        impl PhaseCurrents for AnalogSample {
            fn abc(&self) -> PhaseAbc {
                assert!(
                    !crate::interrupt::masked(),
                    "FOC computation must allow break preemption"
                );
                if let Some(hook) = SAMPLE_HOOK.take() {
                    hook();
                }
                PhaseAbc {
                    a: self.iu_a,
                    b: self.iv_a,
                    c: self.iw_a,
                }
            }
        }
        pub struct Analog;
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

fn run_current() {
    use app::{control as c, foc_isr as f};
    c::stop();
    assert!(c::set_theta_e_off_mrad(0));
    assert!(c::start());
    c::set_id_ma(500);
    c::tick(100);
    f::on_injected(driver::analog::AnalogSample::default());
}

fn commits() -> usize {
    driver::pwm::PWM.lock().unwrap().commits
}

fn break_during_compute() {
    assert!(!interrupt::masked());
    interrupt::pend_fault(app::control::FaultKind::Brake);
}

fn stop_restart_current() {
    assert!(!interrupt::masked());
    app::control::stop();
    assert!(app::control::start());
}

fn nested_current_and_requests() {
    use app::{control as c, foc_isr as f};
    let before = commits();
    f::on_injected(driver::analog::AnalogSample::default());
    assert_eq!(
        commits(),
        before,
        "checked-out controller rejects nested computation"
    );
    assert!(c::set_current_gains(3.0, 500.0));
    f::reset();
    assert!(c::set_current_gains(2.0, 0.0));
}

fn hardware_break_without_irq() {
    driver::pwm::PWM.lock().unwrap().enabled = false;
}

fn bus_expires_during_planning() {
    use std::sync::atomic::Ordering::SeqCst;
    app::telemetry::BUS_VALID.store(false, SeqCst);
}

fn speed_fault_during_compute() {
    assert!(
        !interrupt::masked(),
        "speed feedback/computation must allow break preemption"
    );
    interrupt::pend_fault(app::control::FaultKind::Brake);
}

fn speed_nested_requests() {
    use app::{control as c, speed};
    assert!(!interrupt::masked());
    assert!(
        !speed::tick(0.0, true, 0.1),
        "speed state must have a single owner"
    );
    assert!(c::set_speed_gains(0.5, 100.0));
    speed::reset();
    assert!(c::set_speed_gains(0.0, 1.0));
}

fn stop_restart_speed() {
    assert!(!interrupt::masked());
    app::control::stop();
    assert!(app::control::start_speed(100));
}

// Serialize the production modules' process-global firmware state.
#[test]
fn production_controllers_are_preemptible_and_never_commit_stale_outputs() {
    use app::{control as c, foc_isr as f, speed, telemetry as t};
    use c::{FaultKind, Mode};
    use driver::{
        analog::{AnalogSample, SAMPLE_HOOK},
        pwm::{PLAN_FAIL, PLAN_HOOK, PWM},
    };
    use std::sync::atomic::Ordering::SeqCst;

    // A gain request before initialization must survive initialization.
    assert!(c::set_current_gains(1.0, 0.0));
    assert!(c::set_speed_gains(0.0, 1.0));
    f::init();
    speed::init();
    run_current();
    assert!((t::VDQ.lock().unwrap().d - 0.5).abs() < 1e-6);

    // The actual fault path preempts production FOC math without an IRQ mask.
    let before = commits();
    SAMPLE_HOOK.set(Some(break_during_compute));
    f::on_injected(AnalogSample::default());
    assert!(c::mode() == Mode::Fault);
    assert!(c::last_fault() == FaultKind::Brake);
    assert_eq!(commits(), before);
    assert!(!PWM.lock().unwrap().enabled);

    run_current();
    let before = commits();
    SAMPLE_HOOK.set(Some(stop_restart_current));
    f::on_injected(AnalogSample::default());
    assert!(c::mode() == Mode::Run);
    assert_eq!(
        commits(),
        before,
        "a same-mode restart invalidates the old computation"
    );
    assert_eq!(c::id_target_ma(), 0);

    // Test the production epoch gate independently of the controller reset mailbox.
    let token = c::output_token().unwrap();
    c::stop();
    assert!(c::start());
    assert!(c::commit_output(token, || panic!("old epoch committed")).is_none());

    run_current();
    let before = commits();
    SAMPLE_HOOK.set(Some(nested_current_and_requests));
    f::on_injected(AnalogSample::default());
    assert_eq!(
        commits(),
        before,
        "gain/reset invalidates the computed result"
    );
    f::on_injected(AnalogSample::default());
    assert_eq!(commits(), before + 1);
    assert!(
        (t::VDQ.lock().unwrap().d - 1.0).abs() < 1e-6,
        "queued reset must clear the old integral; last gain pair must win"
    );

    let before = commits();
    SAMPLE_HOOK.set(Some(hardware_break_without_irq));
    f::on_injected(AnalogSample::default());
    assert_eq!(
        commits(),
        before,
        "hardware MOE clear prevents a software-only commit"
    );
    assert!(c::mode() == Mode::Fault);

    run_current();
    let before = commits();
    PLAN_HOOK.set(Some(bus_expires_during_planning));
    f::on_injected(AnalogSample::default());
    assert_eq!(
        commits(),
        before,
        "sensor interlocks are checked again at commit"
    );
    assert!(c::last_fault() == FaultKind::Adc);
    t::BUS_VALID.store(true, SeqCst);
    run_current();
    let before = commits();
    PLAN_FAIL.store(true, SeqCst);
    f::on_injected(AnalogSample::default());
    PLAN_FAIL.store(false, SeqCst);
    assert_eq!(commits(), before);
    assert!(c::last_fault() == FaultKind::Adc);
    assert!(!PWM.lock().unwrap().enabled);

    // Open-loop math uses the same ownership/epoch gate.
    c::stop();
    assert!(c::start_openloop(500, 2));
    let before = commits();
    SAMPLE_HOOK.set(Some(break_during_compute));
    f::on_injected(AnalogSample::default());
    assert_eq!(commits(), before);
    assert!(!PWM.lock().unwrap().enabled);

    // Every phase is validated even when open-loop voltage ignores feedback.
    for openloop in [false, true] {
        for invalid in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            for phase in 0..3 {
                c::stop();
                if openloop {
                    assert!(c::start_openloop(500, 2));
                } else {
                    assert!(c::start());
                }
                let mut currents = [0.0; 3];
                currents[phase] = invalid;
                let before = commits();
                f::on_injected(AnalogSample {
                    iu_a: currents[0],
                    iv_a: currents[1],
                    iw_a: currents[2],
                });
                assert!(c::mode() == Mode::Fault);
                assert!(c::last_fault() == FaultKind::Adc);
                assert_eq!(commits(), before);
                assert!(!PWM.lock().unwrap().enabled);
            }
        }
    }

    c::stop();
    assert!(c::start_speed(100));
    c::poll_refs(1_000);
    t::READY.store(true, SeqCst);
    t::RPM_HOOK.set(Some(speed_fault_during_compute));
    assert!(!speed::tick(0.0, true, 0.1));
    assert!(c::mode() == Mode::Fault);
    assert_eq!(c::iq_target_ma(), 0);

    c::stop();
    assert!(c::start_speed(100));
    c::poll_refs(1_000);
    t::READY.store(true, SeqCst);
    t::RPM_HOOK.set(Some(stop_restart_speed));
    assert!(!speed::tick(0.0, true, 0.1));
    assert!(c::mode() == Mode::Speed);
    assert_eq!(
        c::iq_target_ma(),
        0,
        "old speed computation must not target the new run"
    );

    c::stop();
    assert!(c::start_speed(1));
    c::poll_refs(1_000);
    t::READY.store(true, SeqCst);
    t::RPM_HOOK.set(Some(speed_nested_requests));
    assert!(!speed::tick(0.0, true, 0.1));
    assert_eq!(c::iq_target_ma(), 0);
    t::READY.store(true, SeqCst);
    assert!(
        !speed::tick(0.0, true, 0.05),
        "reset clears speed's elapsed-time accumulator"
    );
    assert!(speed::tick(0.0, true, 0.05));
    assert_eq!(
        c::iq_target_ma(),
        100,
        "latest queued speed gains are used after reset"
    );

    let current_gains = (c::current_kp(), c::current_ki());
    let speed_gains = (c::speed_kp(), c::speed_ki());
    for (kp, ki) in [
        (f32::NAN, 1.0),
        (1.0, f32::INFINITY),
        (-1.0, 1.0),
        (1.0, -1.0),
    ] {
        assert!(!c::set_current_gains(kp, ki));
        assert!(!c::set_speed_gains(kp, ki));
        assert_eq!((c::current_kp(), c::current_ki()), current_gains);
        assert_eq!((c::speed_kp(), c::speed_ki()), speed_gains);
    }
    c::stop();
}
