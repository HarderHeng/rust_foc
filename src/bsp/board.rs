use embassy_stm32::rcc::*;
use embassy_stm32::time::Hertz;
use embassy_stm32::Config;

/// 8 MHz HSE → PLL → 170 MHz SYSCLK, APB1/APB2 undivided.
pub fn init() -> embassy_stm32::Peripherals {
    let mut config = Config::default();

    config.rcc.hse = Some(Hse {
        freq: Hertz::mhz(8),
        mode: HseMode::Oscillator,
    });

    // HSE 8 MHz / 2 * 85 / 2 = 170 MHz
    config.rcc.pll = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV2,
        mul: PllMul::MUL85,
        divp: Some(PllPDiv::DIV6),
        divq: None,
        divr: Some(PllRDiv::DIV2),
    });

    config.rcc.sys = Sysclk::PLL1_R;
    config.rcc.apb1_pre = APBPrescaler::DIV1;
    config.rcc.apb2_pre = APBPrescaler::DIV1;
    config.rcc.mux.adc12sel = mux::Adcsel::SYS;
    config.rcc.boost = true;

    embassy_stm32::init(config)
}