#![no_std]

pub mod app;
pub mod bsp;
pub mod driver;

/// Facade over the `foc` crate so existing `crate::foc::…` paths stay valid.
pub mod foc {
    pub use ::foc::*;
    pub use ::foc::{current, pid, slew, speed, svm, transforms, types};
}
