//! Board-specific configuration.
//!
//! Pin map matches ST B-G431B-ESC1 / UM2516 Table 4 unless noted.

/// LED pin configuration
pub const LED_PIN: usize = 6; // PC6

/// UART2 pin configuration
pub const UART_TX_PIN: usize = 3; // PB3
pub const UART_RX_PIN: usize = 4; // PB4

/// UART baud rate
pub const UART_BAUDRATE: u32 = 921600;

/// External crystal frequency (HSE)
pub const HSE_FREQ_HZ: u32 = 8_000_000;

/// System clock frequency (SYSCLK)
pub const SYSCLK_FREQ_HZ: u32 = 170_000_000;

/// TIM1 PWM frequency (center-aligned period).
pub const PWM_FREQ_HZ: u32 = 20_000;

/// Dead-time inserted between complementary edges.
pub const PWM_DEADTIME_NS: u32 = 800;

/// Dead-time in TIM1 ticks at SYSCLK (no timer prescaler).
pub const fn pwm_deadtime_ticks() -> u16 {
    let ticks = (SYSCLK_FREQ_HZ as u64 * PWM_DEADTIME_NS as u64) / 1_000_000_000;
    ticks as u16
}

/// Low-side shunt (official ESC-G431). Change if the clone differs.
pub const SHUNT_OHM: f32 = 0.003;

/// Combined current-sense gain on the official ESC-G431 network (not raw PGA=16).
/// Measure on your clone; NuttX/SimpleFOC quote ≈ −9.14 including invert.
pub const CURRENT_AMP_GAIN: f32 = 9.14;

/// Inverting current-sense network: measured volts decrease as current increases.
pub const CURRENT_SIGN: f32 = -1.0;

/// Amperes per volt at the OPAMP output (includes invert).
pub const CURRENT_A_PER_V: f32 = CURRENT_SIGN / (SHUNT_OHM * CURRENT_AMP_GAIN);

/// VBUS divider 18k / (18k + 169k).
pub const VBUS_DIV: f32 = 18_000.0 / (18_000.0 + 169_000.0);

pub const ADC_FULL_SCALE: f32 = 4095.0;
pub const ADC_VREF: f32 = 3.3;

/// Convert a raw ADC count (minus offset) to phase current in amperes.
pub fn adc_to_amps(counts: u16, offset: u16) -> f32 {
    let volts = (counts as f32 - offset as f32) * (ADC_VREF / ADC_FULL_SCALE);
    volts * CURRENT_A_PER_V
}

/// Convert a raw ADC count to DC bus voltage.
pub fn adc_to_vbus(counts: u16) -> f32 {
    (counts as f32 * (ADC_VREF / ADC_FULL_SCALE)) / VBUS_DIV
}

/// AS5600 7-bit address.
pub const AS5600_I2C_ADDR: u8 = 0x36;

/// I2C1 clock. Pins: PB8=SCL, PB7=SDA (official header Z+/B+).
/// PB6 (A+) is TIM4_CH1 only — it is not I2C1 on G431.
pub const AS5600_I2C_HZ: u32 = 400_000;

/// Change per motor. Default is a common hobby-motor value.
pub const DEFAULT_POLE_PAIRS: u8 = 7;

/// Current-loop sample time.
pub const CURRENT_LOOP_TS: f32 = 1.0 / PWM_FREQ_HZ as f32;

/// Clamp Id/Iq shell setpoints (milliamps) until hardware OC is in.
pub const MAX_CURRENT_MA: i32 = 8_000;
