//! Current-loop body. Called from ADC JEOS only.

use core::sync::atomic::{AtomicPtr, AtomicU32, Ordering};

use static_cell::StaticCell;

use crate::app::{control, telemetry};
use crate::bsp::config::{
    CURRENT_KI, CURRENT_KP, CURRENT_LOOP_TS, MOTOR_FLUX_WB, MOTOR_LS_H, NOMINAL_VBUS_V, SW_OCP_A,
};
use crate::driver::analog::AnalogSample;
use crate::driver::pwm::with_pwm;
use crate::foc::transforms::{clarke, wrap_2pi, Rotation};
use crate::foc::{
    CurrentLoop, Dq, DqFf, DutySink, FfOff, Modulator, PhaseCurrents, Svpwm, VoltageFeedforward,
};

static LOOP: StaticCell<CurrentLoop> = StaticCell::new();
static LOOP_PTR: AtomicPtr<CurrentLoop> = AtomicPtr::new(core::ptr::null_mut());
/// Open-loop θe in radians (`f32` bits). Milliradian `i32` truncates a 2 Hz step to 0.
static OL_THETA_BITS: AtomicU32 = AtomicU32::new(0);

pub fn init() {
    enable_cyccnt();
    let slot = LOOP.init(CurrentLoop::new(
        CURRENT_KP,
        CURRENT_KI,
        NOMINAL_VBUS_V * 0.5,
    ));
    LOOP_PTR.store(slot as *mut CurrentLoop, Ordering::Release);
}

pub fn reset() {
    with_loop(|l| l.reset());
}

pub fn reset_openloop() {
    OL_THETA_BITS.store(0.0f32.to_bits(), Ordering::Relaxed);
}

pub fn set_gains(kp: f32, ki: f32) {
    with_loop(|l| l.set_gains(kp, ki));
}

/// Timing is owned by the IRQ wrapper so it also includes ADC reads and flag handling.
pub fn on_injected(s: AnalogSample) {
    telemetry::publish_currents(s);

    if control::outputs_live() {
        if let Some(kind) = control::sensor_fault(control::mode()) {
            control::fault(kind);
            return;
        }
    }

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
    // on_injected rejects missing, stale and out-of-range bus samples.
    telemetry::vbus_mv() as f32 / 1000.0
}

fn openloop_step(s: AnalogSample) {
    let vbus = vbus_v();
    let dth = core::f32::consts::TAU * f32::from(control::ol_hz()) * CURRENT_LOOP_TS;
    let th = wrap_2pi(f32::from_bits(OL_THETA_BITS.load(Ordering::Relaxed)) + dth);
    OL_THETA_BITS.store(th.to_bits(), Ordering::Relaxed);

    let rotation = Rotation::new(th);
    let meas = rotation.park(clarke(s.abc()));
    telemetry::publish_dq(meas);

    let voltage = Dq {
        d: 0.0,
        q: control::ol_vq_v(),
    };
    telemetry::publish_vdq(voltage, voltage);

    // Open-loop: do not use current-sign dead-time. Offset/noise flips the
    // sign every ISR and adds ~1.5% duty chatter (felt as strong cogging).
    let duties = Svpwm.modulate(rotation.inv_park(voltage), vbus);
    telemetry::publish_duties(duties);
    apply_duties(duties);
}

fn step(s: AnalogSample) {
    let vbus = vbus_v();

    let theta_e = if control::mode() == control::Mode::Align {
        // Forced D-axis: Park at 0 until `poll_align` latches the encoder offset.
        0.0
    } else {
        let Some(theta) = telemetry::electrical_angle(control::theta_e_off(), control::poles())
        else {
            control::fault(control::FaultKind::Encoder);
            return;
        };
        theta
    };

    let refs = Dq {
        d: control::id_a(),
        q: if control::mode() == control::Mode::Align {
            0.0
        } else {
            control::iq_a()
        },
    };

    let omega_e = telemetry::omega_e_ff(control::poles());
    let ff = if control::mode() == control::Mode::Align {
        FfOff.vdq(refs, omega_e)
    } else {
        DqFf {
            ld: MOTOR_LS_H,
            lq: MOTOR_LS_H,
            flux: MOTOR_FLUX_WB,
        }
        .vdq(refs, omega_e)
    };

    let Some(duties) = with_loop(|l| {
        let out = l.step_debug(s, refs, theta_e, vbus, CURRENT_LOOP_TS, ff);
        telemetry::publish_dq(out.meas);
        telemetry::publish_vdq(out.voltage, out.voltage_ref);
        out.duties
    }) else {
        return;
    };

    // Hardware TIM1 dead-time is already inserted. Current-sign software
    // compensation chatters while shunt offset is still being proven.
    telemetry::publish_duties(duties);
    apply_duties(duties);
}

fn apply_duties(duties: crate::foc::Duties) {
    let _ = with_pwm(|p| crate::driver::analog::with_analog(|a| (p, a).apply(duties)));
}

fn enable_cyccnt() {
    unsafe {
        let mut core = cortex_m::Peripherals::steal();
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();
    }
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
