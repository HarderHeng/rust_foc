//! FOC math. Call from the PWM/ADC ISR, not from Embassy tasks.

pub mod current;
pub mod pid;
pub mod svm;
pub mod transforms;
pub mod types;

pub use current::{CurrentLoop, openloop_voltage};
pub use types::{Dq, Duties, PhaseAbc};
