//! Sensor snapshot shared with the shell and (later) the current ISR.

use core::sync::atomic::{AtomicBool, AtomicI16, AtomicI32, AtomicU16, AtomicU32, Ordering};

use crate::driver::analog::AnalogSample;
use crate::foc;

static ENC_RAW: AtomicU16 = AtomicU16::new(0);
static ENC_VALID: AtomicBool = AtomicBool::new(false);
static ENC_MDEG: AtomicI32 = AtomicI32::new(0);
static ENC_OMEGA_MRAD: AtomicI32 = AtomicI32::new(0);
/// PWM-period ticks since the last valid `publish_angle` (ISR increments).
static ENC_AGE_TICKS: AtomicU32 = AtomicU32::new(0);

pub fn publish_angle(raw: u16, theta_m: f32, omega_m: f32, valid: bool) {
    ENC_VALID.store(valid, Ordering::Relaxed);
    if !valid {
        return;
    }
    ENC_RAW.store(raw, Ordering::Relaxed);
    ENC_MDEG.store((theta_m * 180_000.0 / core::f32::consts::PI) as i32, Ordering::Relaxed);
    ENC_OMEGA_MRAD.store((omega_m * 1000.0) as i32, Ordering::Relaxed);
    ENC_AGE_TICKS.store(0, Ordering::Release);
}

fn theta_omega() -> (f32, f32) {
    let theta = enc_mdeg() as f32 * (core::f32::consts::PI / 180_000.0);
    let omega = ENC_OMEGA_MRAD.load(Ordering::Relaxed) as f32 / 1000.0;
    (theta, omega)
}

/// Last I2C sample, no prediction (align latch).
pub fn theta_m_sample() -> (f32, bool) {
    let (theta, _) = theta_omega();
    (theta, enc_valid())
}

/// ISR: `θ + ω · (N · dt)` where N is PWM periods since the last valid sample.
pub fn theta_m_predict(dt: f32) -> (f32, bool) {
    let ticks = ENC_AGE_TICKS.fetch_add(1, Ordering::Relaxed);
    let (theta, omega) = theta_omega();
    (foc::predict(theta, omega, ticks as f32 * dt), enc_valid())
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

static IU_MA: AtomicI16 = AtomicI16::new(0);
static IV_MA: AtomicI16 = AtomicI16::new(0);
static IW_MA: AtomicI16 = AtomicI16::new(0);
static VBUS_MV: AtomicU16 = AtomicU16::new(0);
static TEMP_C10: AtomicI16 = AtomicI16::new(0);
static IU_RAW: AtomicU16 = AtomicU16::new(0);
static IV_RAW: AtomicU16 = AtomicU16::new(0);
static IW_RAW: AtomicU16 = AtomicU16::new(0);
static ID_MEAS_MA: AtomicI16 = AtomicI16::new(0);
static IQ_MEAS_MA: AtomicI16 = AtomicI16::new(0);

pub fn publish_analog(s: AnalogSample) {
    publish_currents(s);
    publish_bus(s);
}

pub fn publish_currents(s: AnalogSample) {
    IU_RAW.store(s.iu_raw, Ordering::Relaxed);
    IV_RAW.store(s.iv_raw, Ordering::Relaxed);
    IW_RAW.store(s.iw_raw, Ordering::Relaxed);
    IU_MA.store((s.iu_a * 1000.0) as i16, Ordering::Relaxed);
    IV_MA.store((s.iv_a * 1000.0) as i16, Ordering::Relaxed);
    IW_MA.store((s.iw_a * 1000.0) as i16, Ordering::Relaxed);
}

pub fn publish_bus(s: AnalogSample) {
    VBUS_MV.store((s.vbus_v * 1000.0) as u16, Ordering::Relaxed);
    TEMP_C10.store((s.temp_c * 10.0) as i16, Ordering::Relaxed);
}

pub fn publish_dq(dq: crate::foc::Dq) {
    ID_MEAS_MA.store((dq.d * 1000.0) as i16, Ordering::Relaxed);
    IQ_MEAS_MA.store((dq.q * 1000.0) as i16, Ordering::Relaxed);
}

pub fn id_meas_ma() -> i16 {
    ID_MEAS_MA.load(Ordering::Relaxed)
}

pub fn iq_meas_ma() -> i16 {
    IQ_MEAS_MA.load(Ordering::Relaxed)
}

pub fn iu_ma() -> i16 {
    IU_MA.load(Ordering::Relaxed)
}
pub fn iv_ma() -> i16 {
    IV_MA.load(Ordering::Relaxed)
}
pub fn iw_ma() -> i16 {
    IW_MA.load(Ordering::Relaxed)
}
pub fn vbus_mv() -> u16 {
    VBUS_MV.load(Ordering::Relaxed)
}
pub fn temp_c10() -> i16 {
    TEMP_C10.load(Ordering::Relaxed)
}
pub fn iu_raw() -> u16 {
    IU_RAW.load(Ordering::Relaxed)
}
pub fn iv_raw() -> u16 {
    IV_RAW.load(Ordering::Relaxed)
}
pub fn iw_raw() -> u16 {
    IW_RAW.load(Ordering::Relaxed)
}

/// Mechanical RPM from the last encoder ω.
pub fn rpm_meas() -> i32 {
    let omega = ENC_OMEGA_MRAD.load(Ordering::Relaxed) as f32 / 1000.0;
    (omega * 60.0 / core::f32::consts::TAU) as i32
}
