//! Application glue: setpoints shared by shell and (later) the FOC ISR.

use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU16, AtomicU8, Ordering};

use crate::bsp::config::{DEFAULT_POLE_PAIRS, MAX_CURRENT_MA};

pub static FOC_ENABLE: AtomicBool = AtomicBool::new(false);
pub static FOC_ALIGN: AtomicBool = AtomicBool::new(false);
pub static FOC_ID_MA: AtomicI32 = AtomicI32::new(0);
pub static FOC_IQ_MA: AtomicI32 = AtomicI32::new(0);
pub static FOC_POLES: AtomicU8 = AtomicU8::new(DEFAULT_POLE_PAIRS);
pub static PWM_DUTY_PCT: AtomicU8 = AtomicU8::new(50);
pub static ENC_RAW: AtomicU16 = AtomicU16::new(0);
pub static ENC_VALID: AtomicBool = AtomicBool::new(false);
pub static ENC_MDEG: AtomicI32 = AtomicI32::new(0);
/// Mechanical speed, millirad/s.
pub static ENC_OMEGA_MRAD: AtomicI32 = AtomicI32::new(0);

pub fn set_enable(on: bool) {
    FOC_ENABLE.store(on, Ordering::Relaxed);
}

pub fn enabled() -> bool {
    FOC_ENABLE.load(Ordering::Relaxed)
}

pub fn request_align() {
    FOC_ALIGN.store(true, Ordering::Relaxed);
}

pub fn take_align() -> bool {
    FOC_ALIGN.swap(false, Ordering::Relaxed)
}

pub fn set_id_ma(ma: i32) {
    FOC_ID_MA.store(ma.clamp(-MAX_CURRENT_MA, MAX_CURRENT_MA), Ordering::Relaxed);
}

pub fn set_iq_ma(ma: i32) {
    FOC_IQ_MA.store(ma.clamp(-MAX_CURRENT_MA, MAX_CURRENT_MA), Ordering::Relaxed);
}

pub fn id_ma() -> i32 {
    FOC_ID_MA.load(Ordering::Relaxed)
}

pub fn iq_ma() -> i32 {
    FOC_IQ_MA.load(Ordering::Relaxed)
}

pub fn id_a() -> f32 {
    id_ma() as f32 / 1000.0
}

pub fn iq_a() -> f32 {
    iq_ma() as f32 / 1000.0
}

pub fn set_poles(n: u8) {
    FOC_POLES.store(n.max(1), Ordering::Relaxed);
}

pub fn poles() -> u8 {
    FOC_POLES.load(Ordering::Relaxed)
}

pub fn set_pwm_pct(pct: u8) {
    PWM_DUTY_PCT.store(pct.min(100), Ordering::Relaxed);
}

pub fn pwm_pct() -> u8 {
    PWM_DUTY_PCT.load(Ordering::Relaxed)
}

pub fn publish_angle(raw: u16, theta_m: f32, omega_m: f32, valid: bool) {
    ENC_RAW.store(raw, Ordering::Relaxed);
    ENC_VALID.store(valid, Ordering::Relaxed);
    ENC_MDEG.store((theta_m * 180_000.0 / core::f32::consts::PI) as i32, Ordering::Relaxed);
    ENC_OMEGA_MRAD.store((omega_m * 1000.0) as i32, Ordering::Relaxed);
}

/// Interpolate mechanical angle using last sample + ω·dt (radians).
pub fn theta_m_interp(dt: f32) -> (f32, bool) {
    let valid = enc_valid();
    let theta = enc_mdeg() as f32 * (core::f32::consts::PI / 180_000.0);
    let omega = ENC_OMEGA_MRAD.load(Ordering::Relaxed) as f32 / 1000.0;
    (theta + omega * dt, valid)
}

pub fn enc_raw() -> u16 {
    ENC_RAW.load(Ordering::Relaxed)
}

pub fn enc_valid() -> bool {
    ENC_VALID.load(Ordering::Relaxed)
}

pub fn enc_mdeg() -> i32 {
    ENC_MDEG.load(Ordering::Relaxed)
}

pub fn enc_omega_mrad() -> i32 {
    ENC_OMEGA_MRAD.load(Ordering::Relaxed)
}
