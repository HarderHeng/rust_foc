//! B-G431B-ESC1 / UM2516 Table 4 and Cube `main.h` (project 122).

/// TIM1 complementary PWM.
pub mod pwm {
    /// PA8 TIM1_CH1
    pub const UH: &str = "PA8";
    /// PC13 TIM1_CH1N
    pub const UL: &str = "PC13";
    /// PA9 TIM1_CH2
    pub const VH: &str = "PA9";
    /// PA12 TIM1_CH2N
    pub const VL: &str = "PA12";
    /// PA10 TIM1_CH3
    pub const WH: &str = "PA10";
    /// PB15 TIM1_CH3N
    pub const WL: &str = "PB15";
}

/// Three-shunt OPAMP / ADC (122 `MX_OPAMP*` / `MX_ADC*`).
pub mod analog {
    /// PA1 OPAMP1_VINP / COMP1_INP
    pub const IU_P: &str = "PA1";
    /// PA3 OPAMP1_VINM (PGA bias)
    pub const IU_N: &str = "PA3";
    /// PA2 OPAMP1_VOUT / ADC1_IN3
    pub const IU_OUT: &str = "PA2";
    /// PA7 OPAMP2_VINP / COMP2_INP
    pub const IV_P: &str = "PA7";
    /// PA5 OPAMP2_VINM
    pub const IV_N: &str = "PA5";
    /// PA6 OPAMP2_VOUT / ADC2_IN3
    pub const IV_OUT: &str = "PA6";
    /// PB0 OPAMP3_VINP / COMP4_INP
    pub const IW_P: &str = "PB0";
    /// PB2 OPAMP3_VINM
    pub const IW_N: &str = "PB2";
    /// PB1 OPAMP3_VOUT / ADC1_IN12
    pub const IW_OUT: &str = "PB1";
    /// PA0 ADC1_IN1 VBUS divider
    pub const VBUS: &str = "PA0";
    /// PB14 ADC1_IN5 NTC
    pub const NTC: &str = "PB14";
}

pub mod io {
    pub const LED: &str = "PC6";
    pub const BUTTON: &str = "PC10";
    pub const UART_TX: &str = "PB3";
    pub const UART_RX: &str = "PB4";
    /// I2C1 SCL (Z+). Not PB6 — G431 I2C1 has no PB6.
    pub const ENC_SCL: &str = "PB8";
    pub const ENC_SDA: &str = "PB7";
}
