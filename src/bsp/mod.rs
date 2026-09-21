//! Board Support Package (BSP) for STM32G431
//!
//! Provides clock and power configuration.

pub mod board;
pub mod config;

pub use board::*;
pub use config::*;
