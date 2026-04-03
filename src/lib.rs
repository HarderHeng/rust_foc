//! STM32G431 FOC Motor Control Project
//!
//! Layered architecture:
//! - `bsp`: Board-level configuration (clocks, GPIO)
//! - `driver`: Peripheral drivers (LED, shell)
//! - `hal`: HAL traits (future FOC abstraction)

#![no_std]

pub mod bsp;
pub mod driver;
pub mod hal;