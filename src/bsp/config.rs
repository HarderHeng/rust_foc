//! Electrical and motor constants from 122 (`power_stage_parameters.h`,
//! `drive_parameters.h`, `pmsm_motor_parameters.h`).

pub const UART_BAUDRATE: u32 = 921600;
pub const HSE_FREQ_HZ: u32 = 8_000_000;
pub const SYSCLK_FREQ_HZ: u32 = 170_000_000;

/// `PWM_FREQUENCY` in drive_parameters.h
pub const PWM_FREQ_HZ: u32 = 20_000;

/// `SW_DEADTIME_NS` (firmware). Hardware insert is 800 ns (`HW_DEAD_TIME_NS`).
pub const PWM_DEADTIME_NS: u32 = 750;

pub const fn pwm_deadtime_ticks() -> u16 {
    let ticks = (SYSCLK_FREQ_HZ as u64 * PWM_DEADTIME_NS as u64) / 1_000_000_000;
    ticks as u16
}

/// Dead-time as PWM duty (750 ns × 20 kHz ≈ 0.015).
pub const DEADTIME_DUTY: f32 = (PWM_DEADTIME_NS as f32 * PWM_FREQ_HZ as f32) / 1_000_000_000.0;

/// 122 `TNOISE_NS` — used with dead-time for the mid-PWM sample window.
pub const TNOISE_NS: u32 = 1_000;

pub const fn tw_after_ticks() -> u32 {
    ((SYSCLK_FREQ_HZ as u64) * (PWM_DEADTIME_NS + TNOISE_NS) as u64 / 1_000_000_000) as u32
}

/// 122 `TW_BEFORE`: a few ADC sample + trigger ticks.
pub const fn tw_before_ticks() -> u32 {
    16
}

/// Stay on UV / CCR4≈ARR while `(1 − max_duty) > this` (122 `ARR − maxCCR > Tafter`).
pub const SAMPLE_CENTER_MARGIN: f32 = {
    let arr = SYSCLK_FREQ_HZ / (2 * PWM_FREQ_HZ);
    tw_after_ticks() as f32 / arr as f32
};

/// `RSHUNT`
pub const SHUNT_OHM: f32 = 0.003;

/// `AMPLIFICATION_GAIN` (network, not raw PGA=16).
pub const CURRENT_AMP_GAIN: f32 = 9.14;

pub const CURRENT_SIGN: f32 = -1.0;

pub const CURRENT_A_PER_V: f32 = CURRENT_SIGN / (SHUNT_OHM * CURRENT_AMP_GAIN);

/// `VBUS_PARTITIONING_FACTOR` = 18k / (18k + 169k)
pub const VBUS_DIV: f32 = 0.096_255_65;

/// `NOMINAL_BUS_VOLTAGE_V`
pub const NOMINAL_VBUS_V: f32 = 24.0;

pub const ADC_FULL_SCALE: f32 = 4095.0;
pub const ADC_VREF: f32 = 3.3;

/// NTC linearized model from power_stage_parameters.h
pub const NTC_V0: f32 = 1.4;
pub const NTC_T0_C: f32 = 25.0;
pub const NTC_DV_DT: f32 = 0.019;
pub const NTC_T_MAX_C: f32 = 70.0;

/// `POLE_PAIR_NUM`
pub const DEFAULT_POLE_PAIRS: u8 = 4;

/// `RS` / `LS` (documentation / later observers)
pub const MOTOR_RS_OHM: f32 = 0.32;
pub const MOTOR_LS_H: f32 = 0.00047;
pub const MOTOR_KE_VRMS_PER_KRPM: f32 = 3.0;
pub const MOTOR_MAX_RPM: u16 = 6420;

/// `IQMAX_A` / `NOMINAL_CURRENT_A`
pub const NOMINAL_CURRENT_A: f32 = 5.0;

pub const CURRENT_LOOP_TS: f32 = 1.0 / PWM_FREQ_HZ as f32;

/// ~1/10 of the 20 kHz sample rate. `kp = Ls·ω`, `ki = Rs·ω`.
pub const CURRENT_BW_RAD: f32 = 2_000.0;
pub const CURRENT_KP: f32 = MOTOR_LS_H * CURRENT_BW_RAD;
pub const CURRENT_KI: f32 = MOTOR_RS_OHM * CURRENT_BW_RAD;

pub const SW_OCP_A: f32 = NOMINAL_CURRENT_A * 1.5;

/// 122 `OV_VOLTAGE_THRESHOLD_V` / `UD_VOLTAGE_THRESHOLD_V`
pub const VBUS_OV_MV: u16 = 28_000;
pub const VBUS_UV_MV: u16 = 8_000;

pub const OPENLOOP_VQ_MAX_MV: i32 = 3_000;
pub const OPENLOOP_HZ_MAX: u8 = 80;

/// Speed PI: A / RPM and A / (RPM·s). Conservative first tune.
pub const SPEED_LOOP_HZ: u32 = 1_000;
pub const SPEED_KP: f32 = 0.002;
pub const SPEED_KI: f32 = 0.01;
pub const SPEED_RPM_MAX: i32 = 8_000;

pub const MAX_CURRENT_MA: i32 = 5_000;

/// Run/Speed: trip if no `iq`/`rpm`/`start` command for this long.
pub const CMD_TIMEOUT_MS: u32 = 2_000;

/// 122 `DAC_OCP_Threshold` (12-bit DAC counts vs shunt/COMP).
pub const OCP_DAC_COUNTS: u16 = 2893;

pub const AS5600_I2C_ADDR: u8 = 0x36;
pub const AS5600_I2C_HZ: u32 = 400_000;

pub fn adc_to_amps(counts: u16, offset: u16) -> f32 {
    let volts = (counts as f32 - offset as f32) * (ADC_VREF / ADC_FULL_SCALE);
    volts * CURRENT_A_PER_V
}

pub fn adc_to_vbus(counts: u16) -> f32 {
    (counts as f32 * (ADC_VREF / ADC_FULL_SCALE)) / VBUS_DIV
}

pub fn adc_to_temp_c(counts: u16) -> f32 {
    let v = counts as f32 * (ADC_VREF / ADC_FULL_SCALE);
    NTC_T0_C + (v - NTC_V0) / NTC_DV_DT
}
