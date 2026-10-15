//! Current-loop body. Called from ADC JEOS only.

use crate::app::controller::{Controller, ControllerState};
use crate::app::{control, telemetry};
use crate::bsp::config::{
    CURRENT_KI, CURRENT_KP, CURRENT_LOOP_TS, MOTOR_FLUX_WB, MOTOR_LS_H, NOMINAL_VBUS_V, SW_OCP_A,
};
use crate::driver::analog::AnalogSample;
use crate::driver::pwm::{plan_duties, with_pwm};
use crate::foc::transforms::{clarke, wrap_2pi, Rotation};
use crate::foc::{
    CurrentLoop, CurrentStep, Dq, DqFf, FfOff, Modulator, PhaseCurrents, Svpwm, VoltageFeedforward,
};

struct CurrentState {
    loop_: CurrentLoop,
    openloop_theta: f32,
}

impl ControllerState for CurrentState {
    fn reset(&mut self) {
        self.loop_.reset();
        self.openloop_theta = 0.0;
    }

    fn set_gains(&mut self, kp: f32, ki: f32) {
        self.loop_.set_gains(kp, ki);
    }
}

static LOOP: Controller<CurrentState> = Controller::new();

pub fn init() {
    enable_cyccnt();
    LOOP.init(CurrentState {
        loop_: CurrentLoop::new(CURRENT_KP, CURRENT_KI, NOMINAL_VBUS_V * 0.5),
        openloop_theta: 0.0,
    });
}

pub fn reset() {
    LOOP.reset();
}

pub fn set_gains(kp: f32, ki: f32) {
    LOOP.set_gains(kp, ki);
}

/// Timing is owned by the IRQ wrapper so it also includes ADC reads and flag handling.
pub fn on_injected(s: AnalogSample) {
    if !s.iu_a.is_finite() || !s.iv_a.is_finite() || !s.iw_a.is_finite() {
        // Open-loop voltage does not use measured current, so guard all modes
        // here before invalid feedback can be ignored or published as zero.
        control::fault(control::FaultKind::Adc);
        return;
    }
    telemetry::publish_currents(s);
    let token = control::output_token();

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
    let Some(token) = token else {
        return;
    };
    // Stop/restart or a trip during floating-point work invalidates this token
    // before any peripheral commit, even when the new run uses the same mode.
    let mode = token.mode();
    let poles = control::poles();
    let vbus = telemetry::vbus_mv() as f32 / 1000.0;
    let theta_e = if matches!(mode, control::Mode::Align | control::Mode::Openloop) {
        0.0
    } else {
        let Some(theta) = telemetry::electrical_angle(control::theta_e_off(), poles) else {
            control::fault(control::FaultKind::Encoder);
            return;
        };
        theta
    };
    let refs = Dq {
        d: control::id_a(),
        q: if mode == control::Mode::Align {
            0.0
        } else {
            control::iq_a()
        },
    };
    let omega_e = telemetry::omega_e_ff(poles);
    let ol_hz = control::ol_hz();
    let ol_vq = control::ol_vq_v();

    LOOP.compute(
        |state| {
            let out = if mode == control::Mode::Openloop {
                let dth = core::f32::consts::TAU * f32::from(ol_hz) * CURRENT_LOOP_TS;
                state.openloop_theta = wrap_2pi(state.openloop_theta + dth);
                let rotation = Rotation::new(state.openloop_theta);
                let voltage = Dq { d: 0.0, q: ol_vq };
                CurrentStep {
                    meas: rotation.park(clarke(s.abc())),
                    voltage_ref: voltage,
                    voltage,
                    duties: Svpwm.modulate(rotation.inv_park(voltage), vbus),
                }
            } else {
                let ff = if mode == control::Mode::Align {
                    FfOff.vdq(refs, omega_e)
                } else {
                    DqFf {
                        ld: MOTOR_LS_H,
                        lq: MOTOR_LS_H,
                        flux: MOTOR_FLUX_WB,
                    }
                    .vdq(refs, omega_e)
                };
                state
                    .loop_
                    .step_debug(s, refs, theta_e, vbus, CURRENT_LOOP_TS, ff)
            };
            // Geometry planning and FOC math execute outside the shared-state
            // critical section; only the prepared plan is committed.
            (out, plan_duties(out.duties))
        },
        |(out, plan)| {
            control::commit_output(token, || {
                let Some(plan) = plan else {
                    control::fault(control::FaultKind::Adc);
                    return false;
                };
                let applied = with_pwm(|p| {
                    if !p.is_enabled() {
                        return None;
                    }
                    crate::driver::analog::with_analog(|a| p.apply_plan(&plan, a)).flatten()
                })
                .flatten();
                if let Some(duties) = applied {
                    telemetry::publish_dq(out.meas);
                    telemetry::publish_vdq(out.voltage, out.voltage_ref);
                    telemetry::publish_duties(duties);
                    true
                } else {
                    control::fault(control::FaultKind::Adc);
                    false
                }
            })
            .unwrap_or(false)
        },
    );
}

fn enable_cyccnt() {
    #[cfg(target_arch = "arm")]
    unsafe {
        let mut core = cortex_m::Peripherals::steal();
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();
    }
}
