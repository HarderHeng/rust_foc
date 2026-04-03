//! Board-level clock and power configuration
//!
//! Configures STM32G431 for 170MHz from 8MHz HSE via PLL.

use embassy_stm32::rcc::*;
use embassy_stm32::time::Hertz;
use embassy_stm32::Config;

/// Initialize the board with 170MHz clock from 8MHz HSE
///
/// Returns Embassy peripherals for use by drivers.
pub fn init() -> embassy_stm32::Peripherals {
    let mut config = Config::default();

    // Configure HSE (8MHz external crystal)
    config.rcc.hse = Some(Hse {
        freq: Hertz::mhz(8),
        mode: HseMode::Oscillator,
    });

    // Configure PLL for 170MHz
    // HSE(8MHz) / DIV2 * MUL85 / DIV2 = 170MHz
    config.rcc.pll = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV2,
        mul: PllMul::MUL85,
        divp: None,
        divq: None,
        divr: Some(PllRDiv::DIV2),
    });

    // Select PLL as system clock
    config.rcc.sys = Sysclk::PLL1_R;

    // Configure APB prescalers (both at 170MHz for G4)
    config.rcc.apb1_pre = APBPrescaler::DIV1;
    config.rcc.apb2_pre = APBPrescaler::DIV1;

    embassy_stm32::init(config)
}