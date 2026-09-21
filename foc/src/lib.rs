//! FOC math. Call from the PWM/ADC ISR, not from Embassy tasks.

#![no_std]

pub mod angle;
pub mod current;
pub mod pid;
pub mod slew;
pub mod speed;
pub mod svm;
pub mod traits;
pub mod transforms;
pub mod types;

pub use angle::{park_theta, predict, AngleTrack};
pub use current::{CurrentLoop, DqFf, FfOff, dq_voltage_ff, flux_from_ke_vrms_ll_krpm, openloop_voltage};
pub use pid::{AntiWindup, Pi, Pid, PidIn};
pub use slew::{approach, approach_i32};
pub use speed::SpeedLoop;
pub use svm::{CircleLimit, DeadTime, Svpwm, VdPriority};
pub use traits::{DutyMap, DutySink, Modulator, PhaseCurrents, Regulator, VoltageFeedforward, VoltageLimiter};
pub use types::{Dq, Duties, PhaseAbc};
