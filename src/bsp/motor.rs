//! Nameplate and first-tune for **one** machine.
//!
//! Swap this file (or edit the constants) when the motor changes. The power
//! stage, PWM, and ADC scale stay in [`super::config`]. Runtime overrides:
//! `foc poles`, `foc kp|ki|skp|ski`, `foc save` (offset + poles).

/// Printed on the stator / used by Park (`θe = n · θm − off`).
pub const DEFAULT_POLE_PAIRS: u8 = 7;

/// Phase resistance (Ω). Line-to-line winding ≈ 5.1 Ω on this bench motor.
pub const MOTOR_RS_OHM: f32 = 2.55;
/// Phase inductance (H) for the current PI and Dq feed-forward.
pub const MOTOR_LS_H: f32 = 0.000_86;
/// Line-to-line winding inductance (H), documentation only.
pub const MOTOR_LW_H: f32 = 0.002_8;
/// Permanent-magnet flux (Wb).
pub const MOTOR_FLUX_WB: f32 = 0.003_5;
/// Nameplate KV (mechanical rpm / V).
pub const MOTOR_KV: f32 = 220.0;
/// Optional Ke (Vrms line-line / krpm) if a datasheet gives that instead of ψf.
pub const MOTOR_KE_VRMS_PER_KRPM: f32 = 3.14;
/// Software speed clamp (rpm).
pub const MOTOR_MAX_RPM: u16 = 3000;

/// Software current limit (A) for this stator.
pub const NOMINAL_CURRENT_A: f32 = 1.5;
pub const MAX_CURRENT_MA: i32 = 1_500;
pub const SW_OCP_A: f32 = NOMINAL_CURRENT_A * 1.5;

/// Current-loop bandwidth (rad/s). `kp = Ls·ω`, `ki = Rs·ω`.
pub const CURRENT_BW_RAD: f32 = 2_000.0;
pub const CURRENT_KP: f32 = MOTOR_LS_H * CURRENT_BW_RAD;
pub const CURRENT_KI: f32 = MOTOR_RS_OHM * CURRENT_BW_RAD;

/// Forced-D align hold current (mA).
pub const ALIGN_ID_MA: i32 = 500;
pub const ALIGN_MS: u32 = 500;

/// Speed PI first tune (A/rpm, A/(rpm·s)). Empty-shaft gain is high on this KV.
pub const SPEED_LOOP_HZ: u32 = 10;
pub const SPEED_KP: f32 = 0.002;
pub const SPEED_KI: f32 = 0.004;
pub const SPEED_IQ_RAMP_A_S: f32 = 0.8;
pub const SPEED_RPM_WINDOW_S: f32 = 0.1;
pub const SPEED_IQ_MAX_A: f32 = 0.12;
pub const SPEED_IQ_MIN_A: f32 = 0.02;
/// Must be ≥ 500: `approach_i32` at 1 ms otherwise rounds a sub-rpm step to 0.
pub const SPEED_RAMP_RPM_S: f32 = 800.0;
pub const SPEED_RPM_MAX: i32 = 8_000;
pub const RPM_RAMP_RPM_S: f32 = MOTOR_MAX_RPM as f32;
