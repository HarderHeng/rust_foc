//! FOC math. Call from the PWM/ADC ISR, not from Embassy tasks.

#![no_std]

pub mod current;
pub mod pid;
pub mod slew;
pub mod speed;
pub mod svm;
pub mod transforms;
pub mod types;

pub use current::{CurrentLoop, DqFf, dq_voltage_ff, flux_from_ke_vrms_ll_krpm, openloop_voltage};
pub use pid::{AntiWindup, Pi, Pid, PidIn};
pub use slew::{approach, approach_i32};
pub use speed::SpeedLoop;
pub use types::{Dq, Duties, PhaseAbc};
