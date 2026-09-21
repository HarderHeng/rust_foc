//! Current-loop body. Called from ADC JEOS only.

use core::sync::atomic::{AtomicI32, AtomicPtr, Ordering};

use static_cell::StaticCell;

use crate::app::{control, telemetry};
use crate::bsp::config::{
    CURRENT_KI, CURRENT_KP, CURRENT_LOOP_TS, DEADTIME_DUTY, MOTOR_KE_VRMS_PER_KRPM, MOTOR_LS_H, NOMINAL_VBUS_V,
    SW_OCP_A,
};
use crate::driver::analog::AnalogSample;
use crate::driver::pwm::with_pwm;
use crate::foc::current::openloop_voltage;
use crate::foc::transforms::{clarke, park, wrap_2pi};
use crate::foc::{
    park_theta, CurrentLoop, DeadTime, Dq, DqFf, DutyMap, DutySink, FfOff, PhaseCurrents, VoltageFeedforward,
    flux_from_ke_vrms_ll_krpm,
};

static LOOP: StaticCell<CurrentLoop> = StaticCell::new();
static LOOP_PTR: AtomicPtr<CurrentLoop> = AtomicPtr::new(core::ptr::null_mut());
static OL_THETA_MRAD: AtomicI32 = AtomicI32::new(0);

pub fn init() {
    let slot = LOOP.init(CurrentLoop::new(CURRENT_KP, CURRENT_KI, NOMINAL_VBUS_V * 0.5));
    LOOP_PTR.store(slot as *mut CurrentLoop, Ordering::Release);
}

pub fn reset() {
    with_loop(|l| l.reset());
}

pub fn reset_openloop() {
    OL_THETA_MRAD.store(0, Ordering::Relaxed);
}

pub fn set_gains(kp: f32, ki: f32) {
    with_loop(|l| l.set_gains(kp, ki));
}

pub fn on_injected(s: AnalogSample) {
    telemetry::publish_currents(s);

    if s.iu_a.abs() > SW_OCP_A || s.iv_a.abs() > SW_OCP_A || s.iw_a.abs() > SW_OCP_A {
        control::fault(control::FaultKind::Overcurrent);
        return;
    }

    match control::mode() {
        control::Mode::Align | control::Mode::Run | control::Mode::Speed => step(s),
        control::Mode::Openloop => openloop_step(s),
        _ => {}
    }
}

fn vbus_v() -> f32 {
    let mv = telemetry::vbus_mv();
    if mv == 0 {
        NOMINAL_VBUS_V
    } else {
        mv as f32 / 1000.0
    }
}

fn openloop_step(s: AnalogSample) {
    let vbus = vbus_v();
    let dth = core::f32::consts::TAU * f32::from(control::ol_hz()) * CURRENT_LOOP_TS;
    let th = wrap_2pi(OL_THETA_MRAD.load(Ordering::Relaxed) as f32 / 1000.0 + dth);
    OL_THETA_MRAD.store((th * 1000.0) as i32, Ordering::Relaxed);

    let meas = park(clarke(s.abc()), th);
    telemetry::publish_dq(meas);

    let duties = DeadTime { shift: DEADTIME_DUTY }.map(openloop_voltage(0.0, control::ol_vq_v(), th, vbus), s.abc());
    apply_duties(duties);
}

fn step(s: AnalogSample) {
    if telemetry::vbus_mv() == 0 {
        return;
    }
    let vbus = vbus_v();

    let theta_e = if control::mode() == control::Mode::Align {
        // Forced D-axis: Park at 0 until `poll_align` latches the encoder offset.
        0.0
    } else {
        let (theta_m, valid) = telemetry::theta_m_predict(CURRENT_LOOP_TS);
        if !valid {
            // Hold last CCR. Encoder task trips after a grace window.
            return;
        }
        park_theta(theta_m, control::theta_e_off(), control::poles())
    };

    let refs = Dq {
        d: control::id_a(),
        q: if control::mode() == control::Mode::Align {
            0.0
        } else {
            control::iq_a()
        },
    };

    let omega_e = telemetry::enc_omega_mrad() as f32 / 1000.0 * f32::from(control::poles());
    let ff = if control::mode() == control::Mode::Align {
        FfOff.vdq(refs, omega_e)
    } else {
        DqFf {
            ld: MOTOR_LS_H,
            lq: MOTOR_LS_H,
            flux: flux_from_ke_vrms_ll_krpm(MOTOR_KE_VRMS_PER_KRPM, f32::from(control::poles())),
        }
        .vdq(refs, omega_e)
    };

    let Some(duties) = with_loop(|l| {
        let (meas, duties) = l.step(s, refs, theta_e, vbus, CURRENT_LOOP_TS, ff);
        telemetry::publish_dq(meas);
        duties
    }) else {
        return;
    };

    apply_duties(DeadTime { shift: DEADTIME_DUTY }.map(duties, s.abc()));
}

fn apply_duties(duties: crate::foc::Duties) {
    let _ = with_pwm(|p| crate::driver::analog::with_analog(|a| (p, a).apply(duties)));
}

fn with_loop<R>(f: impl FnOnce(&mut CurrentLoop) -> R) -> Option<R> {
    cortex_m::interrupt::free(|_| {
        let p = LOOP_PTR.load(Ordering::Acquire);
        if p.is_null() {
            None
        } else {
            Some(f(unsafe { &mut *p }))
        }
    })
}
