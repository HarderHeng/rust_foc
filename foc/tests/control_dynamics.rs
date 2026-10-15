//! Deterministic average-inverter/PMSM models, independent of production
//! transforms. These verify controller signs, response, and saturation recovery;
//! they do not model shunt timing, encoder latency, dead time, or board safety.

#[allow(dead_code)]
#[path = "../../src/bsp/motor.rs"]
mod motor;

use foc::{CurrentLoop, CurrentStep, Dq, DqFf, Duties, SpeedLoop, VoltageFeedforward};

const TS: f64 = 1.0 / 20_000.0;

#[derive(Clone, Copy)]
struct Machine {
    r: f64,
    ld: f64,
    lq: f64,
    flux: f64,
}

impl Machine {
    fn nominal() -> Self {
        Self {
            r: f64::from(motor::MOTOR_RS_OHM),
            ld: f64::from(motor::MOTOR_LS_H),
            lq: f64::from(motor::MOTOR_LS_H),
            flux: f64::from(motor::MOTOR_FLUX_WB),
        }
    }

    fn ff(self) -> DqFf {
        DqFf {
            ld: self.ld as f32,
            lq: self.lq as f32,
            flux: self.flux as f32,
        }
    }
}

struct Rig {
    machine: Machine,
    current: [f64; 2],
    theta: f64,
    delayed_duties: Duties,
    controller: CurrentLoop,
}

impl Rig {
    fn new(machine: Machine) -> Self {
        Self {
            machine,
            current: [0.0; 2],
            theta: 0.37,
            delayed_duties: Duties {
                a: 0.5,
                b: 0.5,
                c: 0.5,
            },
            controller: CurrentLoop::new(motor::CURRENT_KP, motor::CURRENT_KI, 24.0),
        }
    }

    fn step(&mut self, refs: Dq, omega: f64, vbus: f64, feedforward: bool) -> CurrentStep {
        let [id, iq] = self.current;
        let (s, c) = self.theta.sin_cos();
        let alpha = id * c - iq * s;
        let beta = id * s + iq * c;
        let currents = (
            alpha as f32,
            (-0.5 * alpha + 3.0_f64.sqrt() * 0.5 * beta) as f32,
        );
        let ff = if feedforward {
            self.machine.ff().vdq(refs, omega as f32)
        } else {
            Dq::default()
        };
        let out = self.controller.step_debug(
            currents,
            refs,
            self.theta as f32,
            vbus as f32,
            TS as f32,
            ff,
        );
        for duty in [out.duties.a, out.duties.b, out.duties.c] {
            assert!((0.0..=1.0).contains(&duty), "invalid duty {duty}");
        }
        let limit = vbus / 3.0_f64.sqrt();
        assert!(f64::from(out.voltage.d).hypot(f64::from(out.voltage.q)) <= limit + 3e-6);
        assert!(self.controller.id.integrator().is_finite());
        assert!(self.controller.iq.integrator().is_finite());

        // Reconstruct actual alpha/beta volts from the three bridge averages,
        // independently of the production SVPWM/inverse-Clarke equations.
        // Bus changes affect the bridge immediately, including the previous
        // duty command held by the modeled one-period preload delay.
        self.integrate(inverter_voltage(self.delayed_duties, vbus), omega);
        self.delayed_duties = out.duties;
        out
    }

    fn integrate(&mut self, voltage: [f64; 2], omega: f64) {
        // A full control-period voltage delay is deliberately included above.
        // RK4 substeps hold the physical alpha/beta voltage while the rotor turns.
        let h = TS / 4.0;
        for _ in 0..4 {
            let x = self.current;
            let t = self.theta;
            let k1 = self.derivative(x, t, voltage, omega);
            let k2 = self.derivative(add(x, k1, h * 0.5), t + omega * h * 0.5, voltage, omega);
            let k3 = self.derivative(add(x, k2, h * 0.5), t + omega * h * 0.5, voltage, omega);
            let k4 = self.derivative(add(x, k3, h), t + omega * h, voltage, omega);
            for i in 0..2 {
                self.current[i] += h * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) / 6.0;
                assert!(self.current[i].is_finite() && self.current[i].abs() < 100.0);
            }
            self.theta = (self.theta + omega * h).rem_euclid(std::f64::consts::TAU);
        }
    }

    fn derivative(&self, i: [f64; 2], theta: f64, v: [f64; 2], omega: f64) -> [f64; 2] {
        let (s, c) = theta.sin_cos();
        let vd = v[0] * c + v[1] * s;
        let vq = -v[0] * s + v[1] * c;
        let m = self.machine;
        [
            (vd - m.r * i[0] + omega * m.lq * i[1]) / m.ld,
            (vq - m.r * i[1] - omega * (m.ld * i[0] + m.flux)) / m.lq,
        ]
    }

    fn error(&self, refs: Dq) -> f64 {
        (self.current[0] - f64::from(refs.d)).hypot(self.current[1] - f64::from(refs.q))
    }
}

fn add(x: [f64; 2], dx: [f64; 2], dt: f64) -> [f64; 2] {
    [x[0] + dx[0] * dt, x[1] + dx[1] * dt]
}

fn inverter_voltage(d: Duties, vbus: f64) -> [f64; 2] {
    let u = f64::from(d.a) * vbus;
    let v = f64::from(d.b) * vbus;
    let w = f64::from(d.c) * vbus;
    [(2.0 * u - v - w) / 3.0, (v - w) / 3.0_f64.sqrt()]
}

#[test]
fn rl_axes_follow_positive_negative_and_reversing_steps_with_default_gains() {
    // Resistance/inductance variation is a model sweep, not retuning the PI.
    for (r_scale, l_scale) in [(1.0, 1.0), (0.7, 0.8), (1.3, 1.2)] {
        let mut m = Machine::nominal();
        m.r *= r_scale;
        m.ld *= l_scale;
        m.lq *= l_scale;
        let mut rig = Rig::new(m);
        for refs in [
            Dq { d: 0.6, q: 1.0 },
            Dq { d: -0.6, q: -1.0 },
            Dq { d: 0.0, q: 0.0 },
        ] {
            let mut peak_error = 0.0_f64;
            for tick in 0..800 {
                rig.step(refs, 0.0, 24.0, false);
                peak_error = peak_error.max(rig.error(refs));
                if tick >= 200 {
                    assert!(rig.error(refs) < 0.003, "RL settling: {:?}", rig.current);
                }
            }
            assert!(peak_error < 2.5, "unbounded reversal error {peak_error}");
        }
    }
}

#[test]
fn pmsm_tracks_both_directions_with_rotation_cross_coupling_and_bemf() {
    for omega in [-1000.0, -300.0, 300.0, 1000.0] {
        let mut rig = Rig::new(Machine::nominal());
        for refs in [Dq { d: 0.2, q: 0.7 }, Dq { d: -0.2, q: -0.7 }] {
            for tick in 0..1200 {
                rig.step(refs, omega, 24.0, true);
                if tick >= 400 {
                    assert!(rig.error(refs) < 0.008, "omega={omega}: {:?}", rig.current);
                }
            }
        }
    }
}

#[test]
fn feedforward_reduces_rotation_startup_error_without_changing_the_gains() {
    let mut with_ff = Rig::new(Machine::nominal());
    let mut without_ff = Rig::new(Machine::nominal());
    let refs = Dq { d: 0.0, q: 0.8 };
    let mut ff_error = 0.0;
    let mut plain_error = 0.0;
    for tick in 0..400 {
        with_ff.step(refs, 1200.0, 24.0, true);
        without_ff.step(refs, 1200.0, 24.0, false);
        if tick < 80 {
            ff_error += with_ff.error(refs) * TS;
            plain_error += without_ff.error(refs) * TS;
        }
    }
    assert!(
        ff_error < plain_error * 0.6,
        "FF error {ff_error}; plain {plain_error}"
    );
    assert!(with_ff.error(refs) < 0.008);
    assert!(without_ff.error(refs) < 0.008);
}

#[test]
fn voltage_saturation_recovers_after_reference_and_bus_changes() {
    for sign in [-1.0, 1.0] {
        let mut rig = Rig::new(Machine::nominal());
        // Exercise the math API beyond the firmware's current-command clamp;
        // this deliberately unreachable reference is a voltage-limiter stress.
        let unreachable = Dq {
            d: sign * 0.5,
            q: sign * 12.0,
        };
        let mut limited = 0;
        for _ in 0..2000 {
            let out = rig.step(unreachable, 0.0, 8.0, false);
            limited += usize::from((out.voltage.q - out.voltage_ref.q).abs() > 0.1);
        }
        assert!(limited > 1000, "test must spend time in voltage saturation");
        let refs = Dq {
            d: -sign * 0.1,
            q: -sign * 0.4,
        };
        for tick in 0..800 {
            rig.step(refs, 0.0, 24.0, false);
            if tick >= 200 {
                assert!(rig.error(refs) < 0.004, "recovery: {:?}", rig.current);
            }
        }
        // Feasible rotating load across a bus sag and restoration.
        for vbus in [24.0, 8.0, 16.0, 24.0] {
            for tick in 0..800 {
                rig.step(refs, 500.0, vbus, true);
                if tick >= 200 {
                    assert!(rig.error(refs) < 0.008, "bus={vbus}: {:?}", rig.current);
                }
            }
        }
    }
}

#[test]
fn ten_hz_speed_loop_has_model_dependent_tracking_and_correct_torque_direction() {
    // Explicit synthetic inertia/friction: no claim that these are measured
    // values for the bench motor. Real default gains, current and RPM limits,
    // slew rate, 10 Hz update, and a previous-window speed measurement are used.
    let friction = 0.000_03;
    let torque_per_amp =
        1.5 * f64::from(motor::DEFAULT_POLE_PAIRS) * f64::from(motor::MOTOR_FLUX_WB);
    // The light model has a bounded limit cycle with the unchanged 10 Hz gains;
    // this is deliberately not presented as close steady-state regulation.
    for (inertia, tail_error_bound) in [(0.000_05, 50.0), (0.000_2, 3.0), (0.000_5, 3.0)] {
        for direction in [-1.0, 1.0] {
            let mut speed = SpeedLoop::new(motor::SPEED_KP, motor::SPEED_KI, motor::SPEED_IQ_MAX_A);
            let mut rig = Rig::new(Machine::nominal());
            let mut omega_m = 0.0;
            let mut iq = 0.0_f32;
            let mut iq_target = 0.0_f32;
            let mut rpm_window = 0.0;
            let mut rpm_meas = 0.0;
            let target = direction * 300.0;
            let speed_ticks = (1.0 / f64::from(motor::SPEED_LOOP_HZ) / TS).round() as usize;
            let mut tail_error = 0.0_f64;
            for tick in 0..320_000 {
                if tick % speed_ticks == 0 {
                    iq_target = speed.step_same_direction(
                        target as f32,
                        rpm_meas as f32,
                        1.0 / motor::SPEED_LOOP_HZ as f32,
                        motor::SPEED_IQ_MAX_A,
                        iq,
                    );
                    assert!(direction * f64::from(iq_target) >= 0.0);
                    assert!(iq_target.abs() <= motor::SPEED_IQ_MAX_A);
                }
                iq = foc::approach(iq, iq_target, motor::SPEED_IQ_RAMP_A_S, TS as f32);
                rig.step(
                    Dq { d: 0.0, q: iq },
                    omega_m * f64::from(motor::DEFAULT_POLE_PAIRS),
                    24.0,
                    true,
                );
                let torque = torque_per_amp * rig.current[1];
                omega_m += (torque - friction * omega_m) / inertia * TS;
                rpm_window += omega_m * 60.0 / std::f64::consts::TAU;
                if (tick + 1) % speed_ticks == 0 {
                    rpm_meas = rpm_window / speed_ticks as f64;
                    rpm_window = 0.0;
                }
                if tick >= 280_000 {
                    let rpm = omega_m * 60.0 / std::f64::consts::TAU;
                    tail_error = tail_error.max((rpm - target).abs());
                }
            }
            let rpm = omega_m * 60.0 / std::f64::consts::TAU;
            assert!(
                tail_error < tail_error_bound,
                "J={inertia}, direction={direction}, tail error={tail_error}"
            );
            // Overspeed and a zero reference must command coast, not braking torque.
            assert_eq!(
                speed.step_same_direction(
                    target as f32,
                    (direction * 600.0) as f32,
                    0.1,
                    motor::SPEED_IQ_MAX_A,
                    iq,
                ),
                0.0
            );
            assert_eq!(
                speed.step_same_direction(0.0, rpm as f32, 0.1, motor::SPEED_IQ_MAX_A, 0.0),
                0.0
            );
        }
    }
}
