//! Hardware-independent FOC math, acquisition planning and supervisor policies.

#![no_std]

pub mod alignment;
pub mod angle;
pub mod current;
pub mod feedback;
pub mod nvm;
pub mod pid;
pub mod safety;
pub mod sampling;
pub mod slew;
pub mod speed;
pub mod svm;
pub mod timing;
pub mod traits;
pub mod transforms;
pub mod types;
pub mod watchdog;

pub use angle::{park_theta, predict, AngleTrack};
pub use current::{
    dq_voltage_ff, flux_from_ke_vrms_ll_krpm, openloop_voltage, CurrentLoop, CurrentStep, DqFf,
    FfOff,
};
pub use pid::{AntiWindup, Pi, Pid, PidIn};
pub use sampling::{plan_sampling, SamplePlan, ShuntPair};
pub use slew::{approach, approach_i32, SlewI32};
pub use speed::SpeedLoop;
pub use svm::{CircleLimit, DeadTime, Svpwm, VdPriority};
pub use traits::{
    DutyMap, DutySink, Modulator, PhaseCurrents, Regulator, VoltageFeedforward, VoltageLimiter,
};
pub use types::{Dq, Duties, PhaseAbc};
