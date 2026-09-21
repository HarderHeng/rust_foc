//! STM32G431 FOC Motor Control Project
//!
//! Layered architecture:
//! - `bsp`: Board-level configuration (clocks, GPIO, FOC scales)
//! - `driver`: Peripheral drivers (LED, shell, PWM, AS5600)
//! - `foc`: Current-loop math (ISR-safe)
//! - `app`: Shared setpoints between shell and control
//! - `hal`: HAL traits (future)

#![no_std]

pub mod app;
pub mod bsp;
pub mod driver;
pub mod foc;
pub mod hal;