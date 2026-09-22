//! Sensor snapshot shared with the shell and (later) the current ISR.

use core::sync::atomic::{AtomicBool, AtomicI16, AtomicI32, AtomicU16, AtomicU32, Ordering};

use crate::bsp::config::SYSCLK_FREQ_HZ;
use crate::driver::analog::AnalogSample;

static ENC_RAW: AtomicU16 = AtomicU16::new(0);
static ENC_VALID: AtomicBool = AtomicBool::new(false);
static ENC_MDEG: AtomicI32 = AtomicI32::new(0);
static ENC_THETA_BITS: AtomicU32 = AtomicU32::new(0);
static ENC_OMEGA_MRAD: AtomicI32 = AtomicI32::new(0);
/// PWM-period ticks since the last valid `publish_angle` (ISR increments).
static ENC_AGE_TICKS: AtomicU32 = AtomicU32::new(0);
static RPM_RAW_INIT: AtomicBool = AtomicBool::new(false);
static RPM_LAST_RAW: AtomicU16 = AtomicU16::new(0);
static RPM_ACC_COUNTS: AtomicI32 = AtomicI32::new(0);
static RPM_WIN_US: AtomicU32 = AtomicU32::new(0);
static RPM_F_BITS: AtomicU32 = AtomicU32::new(0.0f32.to_bits());
static RPM_READY: AtomicBool = AtomicBool::new(false);

pub fn publish_angle(raw: u16, theta_m: f32, omega_m: f32, valid: bool) {
    ENC_VALID.store(valid, Ordering::Relaxed);
    if !valid {
        return;
    }
    ENC_RAW.store(raw, Ordering::Relaxed);
    ENC_THETA_BITS.store(theta_m.to_bits(), Ordering::Relaxed);
    ENC_MDEG.store(
        (theta_m * 180_000.0 / core::f32::consts::PI) as i32,
        Ordering::Relaxed,
    );
    ENC_OMEGA_MRAD.store((omega_m * 1000.0) as i32, Ordering::Relaxed);
    ENC_AGE_TICKS.store(0, Ordering::Release);
}

/// Last I2C sample. Do not extrapolate with the 12-bit ω estimate — it chatters
/// at low speed and rotates the Park frame as if the rotor were shaking.
pub fn theta_m_sample() -> (f32, bool) {
    (
        f32::from_bits(ENC_THETA_BITS.load(Ordering::Relaxed)),
        enc_valid(),
    )
}

/// Age the sample counter; Park uses the last sample, not `θ + ω·age`.
pub fn theta_m_predict(_dt: f32) -> (f32, bool) {
    ENC_AGE_TICKS.fetch_add(1, Ordering::Relaxed);
    theta_m_sample()
}

/// Electrical speed for Dq feed-forward from the windowed RPM, not raw ω.
pub fn omega_e_ff(pole_pairs: u8) -> f32 {
    let rpm = f32::from_bits(RPM_F_BITS.load(Ordering::Relaxed));
    if !rpm.is_finite() {
        return 0.0;
    }
    rpm * (core::f32::consts::TAU / 60.0) * f32::from(pole_pairs)
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
static UD_MV: AtomicI16 = AtomicI16::new(0);
static UQ_MV: AtomicI16 = AtomicI16::new(0);
static UD_REF_MV: AtomicI16 = AtomicI16::new(0);
static UQ_REF_MV: AtomicI16 = AtomicI16::new(0);
static ISR_CYCLES: AtomicU32 = AtomicU32::new(0);
static ISR_CYCLES_MAX: AtomicU32 = AtomicU32::new(0);
static DA_PPT: AtomicU16 = AtomicU16::new(0);
static DB_PPT: AtomicU16 = AtomicU16::new(0);
static DC_PPT: AtomicU16 = AtomicU16::new(0);

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

pub fn publish_duties(d: crate::foc::Duties) {
    let ppt = |x: f32| ((x * 1000.0) as u16).min(1000);
    DA_PPT.store(ppt(d.a), Ordering::Relaxed);
    DB_PPT.store(ppt(d.b), Ordering::Relaxed);
    DC_PPT.store(ppt(d.c), Ordering::Relaxed);
}

pub fn da_ppt() -> u16 {
    DA_PPT.load(Ordering::Relaxed)
}
pub fn db_ppt() -> u16 {
    DB_PPT.load(Ordering::Relaxed)
}
pub fn dc_ppt() -> u16 {
    DC_PPT.load(Ordering::Relaxed)
}

pub fn publish_vdq(applied: crate::foc::Dq, reference: crate::foc::Dq) {
    UD_MV.store(volts_to_mv_i16(applied.d), Ordering::Relaxed);
    UQ_MV.store(volts_to_mv_i16(applied.q), Ordering::Relaxed);
    UD_REF_MV.store(volts_to_mv_i16(reference.d), Ordering::Relaxed);
    UQ_REF_MV.store(volts_to_mv_i16(reference.q), Ordering::Relaxed);
}

fn volts_to_mv_i16(v: f32) -> i16 {
    let mv = (v * 1000.0) as i32;
    mv.clamp(i16::MIN as i32, i16::MAX as i32) as i16
}

/// Last JEOS ISR duration in CPU cycles (no defmt / no RTT in that path).
pub fn publish_isr_cycles(cycles: u32) {
    ISR_CYCLES.store(cycles, Ordering::Relaxed);
    let _ = ISR_CYCLES_MAX.fetch_max(cycles, Ordering::Relaxed);
}

pub fn isr_cycles() -> u32 {
    ISR_CYCLES.load(Ordering::Relaxed)
}

pub fn isr_cycles_max() -> u32 {
    ISR_CYCLES_MAX.load(Ordering::Relaxed)
}

pub fn reset_isr_cycles() {
    ISR_CYCLES.store(0, Ordering::Relaxed);
    ISR_CYCLES_MAX.store(0, Ordering::Relaxed);
}

fn cycles_to_us(cycles: u32) -> u32 {
    cycles / (SYSCLK_FREQ_HZ / 1_000_000)
}

pub fn isr_us() -> u32 {
    cycles_to_us(isr_cycles())
}

pub fn isr_us_max() -> u32 {
    cycles_to_us(isr_cycles_max())
}

pub fn id_meas_ma() -> i16 {
    ID_MEAS_MA.load(Ordering::Relaxed)
}

pub fn iq_meas_ma() -> i16 {
    IQ_MEAS_MA.load(Ordering::Relaxed)
}

pub fn ud_mv() -> i16 {
    UD_MV.load(Ordering::Relaxed)
}

pub fn uq_mv() -> i16 {
    UQ_MV.load(Ordering::Relaxed)
}

pub fn ud_ref_mv() -> i16 {
    UD_REF_MV.load(Ordering::Relaxed)
}

pub fn uq_ref_mv() -> i16 {
    UQ_REF_MV.load(Ordering::Relaxed)
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

/// Mechanical RPM from net Δθ over [`crate::bsp::config::SPEED_RPM_WINDOW_S`].
pub fn rpm_meas() -> i32 {
    f32::from_bits(RPM_F_BITS.load(Ordering::Relaxed)) as i32
}

pub fn rpm_ready() -> bool {
    RPM_READY.load(Ordering::Relaxed)
}

pub fn reset_speed_filter() {
    RPM_RAW_INIT.store(false, Ordering::Relaxed);
    RPM_ACC_COUNTS.store(0, Ordering::Relaxed);
    RPM_WIN_US.store(0, Ordering::Relaxed);
    RPM_F_BITS.store(0.0f32.to_bits(), Ordering::Relaxed);
    RPM_READY.store(false, Ordering::Relaxed);
}

/// 12-bit wrap-aware Δraw over [`crate::bsp::config::SPEED_RPM_WINDOW_S`].
/// Unwrapped `θ` can accumulate encoder chatter into a fake high rpm.
pub fn push_speed_from_raw(raw: u16, dt: f32) {
    use crate::bsp::config::SPEED_RPM_WINDOW_S;
    let raw = raw & 0x0FFF;
    let dt = dt.max(1e-5);
    if !RPM_RAW_INIT.swap(true, Ordering::Relaxed) {
        RPM_LAST_RAW.store(raw, Ordering::Relaxed);
        RPM_ACC_COUNTS.store(0, Ordering::Relaxed);
        RPM_WIN_US.store(0, Ordering::Relaxed);
        return;
    }
    let prev = RPM_LAST_RAW.swap(raw, Ordering::Relaxed);
    let mut d = i32::from(raw) - i32::from(prev);
    if d > 2048 {
        d -= 4096;
    } else if d < -2048 {
        d += 4096;
    }
    RPM_ACC_COUNTS.fetch_add(d, Ordering::Relaxed);
    let us = (dt * 1_000_000.0) as u32;
    let acc = RPM_WIN_US.fetch_add(us, Ordering::Relaxed) + us;
    let win_us = (SPEED_RPM_WINDOW_S * 1_000_000.0) as u32;
    if acc < win_us {
        return;
    }
    RPM_WIN_US.store(0, Ordering::Relaxed);
    let counts = RPM_ACC_COUNTS.swap(0, Ordering::Relaxed);
    let t = acc as f32 / 1_000_000.0;
    if t < 1e-3 {
        return;
    }
    let rpm = counts as f32 / 4096.0 / t * 60.0;
    if rpm.is_finite() {
        RPM_F_BITS.store(rpm.to_bits(), Ordering::Relaxed);
        RPM_READY.store(true, Ordering::Relaxed);
    }
}
