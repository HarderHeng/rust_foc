//! FOC math. Call from the PWM/ADC ISR, not from Embassy tasks.

#![no_std]

pub mod current;
pub mod pid;
pub mod speed;
pub mod svm;
pub mod transforms;
pub mod types;

pub use current::{CurrentLoop, openloop_voltage};
pub use speed::SpeedLoop;
pub use types::{Dq, Duties, PhaseAbc};
