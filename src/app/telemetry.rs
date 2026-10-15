//! Sensor snapshots and current-IRQ diagnostics shared with the shell.

use core::cell::{Cell, RefCell};
use core::sync::atomic::{AtomicBool, AtomicI16, AtomicI32, AtomicU16, AtomicU32, Ordering};
use cortex_m::interrupt::Mutex;
use embassy_time::Instant;

use crate::bsp::config::{
    BUS_FAULT_MS, ENC_FAULT_MS, ENC_SPEED_MAX_GAP_US, PWM_FREQ_HZ, SPEED_RPM_WINDOW_S,
    SYSCLK_FREQ_HZ,
};
use crate::driver::analog::AnalogSample;
use crate::foc::feedback::WindowSpeed;
use crate::foc::safety::Freshness;
use crate::foc::timing::CycleStats;

static ENC_RAW: AtomicU16 = AtomicU16::new(0);
static ENC_VALID: AtomicBool = AtomicBool::new(false);
static ENC_MDEG: AtomicI32 = AtomicI32::new(0);
static ENC_THETA_BITS: AtomicU32 = AtomicU32::new(0);
static ENC_OMEGA_MRAD: AtomicI32 = AtomicI32::new(0);
static ENC_SAMPLED_US: AtomicU32 = AtomicU32::new(0);
static ENC_AGE: Mutex<Cell<Freshness>> = Mutex::new(Cell::new(Freshness::new()));
static BUS_AGE: Mutex<Cell<Freshness>> = Mutex::new(Cell::new(Freshness::new()));

fn sample_fresh(age: &Mutex<Cell<Freshness>>, timeout_ms: u32) -> bool {
    cortex_m::interrupt::free(|cs| {
        let cell = age.borrow(cs);
        let mut stamp = cell.get();
        let fresh = stamp.is_fresh(Instant::now().as_millis() as u32, timeout_ms);
        cell.set(stamp);
        fresh
    })
}
static SPEED: Mutex<RefCell<WindowSpeed>> = Mutex::new(RefCell::new(WindowSpeed::new(
    (SPEED_RPM_WINDOW_S * 1_000_000.0) as u32,
    ENC_SPEED_MAX_GAP_US,
)));

pub fn publish_angle(raw: u16, theta_m: f32, omega_m: f32, valid: bool) {
    publish_angle_at(raw, theta_m, omega_m, valid, Instant::now().as_micros());
}

/// Keep acquisition time through I2C completion/publication. A slow transaction
/// cannot turn a stale sample into fresh control feedback.
pub fn publish_angle_at(raw: u16, theta_m: f32, omega_m: f32, valid: bool, sampled_at_us: u64) {
    let now_us = Instant::now().as_micros();
    let valid = valid
        && theta_m.is_finite()
        && omega_m.is_finite()
        && sampled_at_us <= now_us
        && now_us - sampled_at_us < u64::from(ENC_FAULT_MS) * 1000;
    cortex_m::interrupt::free(|cs| {
        ENC_VALID.store(valid, Ordering::Relaxed);
        if !valid {
            // Fail closed: do not keep applying the previous voltage on I2C errors.
            ENC_AGE.borrow(cs).set(Freshness::new());
            SPEED.borrow(cs).borrow_mut().reset();
            return;
        }
        ENC_RAW.store(raw & 0x0fff, Ordering::Relaxed);
        ENC_THETA_BITS.store(theta_m.to_bits(), Ordering::Relaxed);
        ENC_MDEG.store(
            (theta_m * 180_000.0 / core::f32::consts::PI) as i32,
            Ordering::Relaxed,
        );
        ENC_OMEGA_MRAD.store((omega_m * 1000.0) as i32, Ordering::Relaxed);
        ENC_SAMPLED_US.store(sampled_at_us as u32, Ordering::Relaxed);
        let mut age = Freshness::new();
        age.refresh((sampled_at_us / 1000) as u32);
        ENC_AGE.borrow(cs).set(age);
        SPEED
            .borrow(cs)
            .borrow_mut()
            .push(raw, sampled_at_us as u32);
    });
}

/// Last I2C sample. Do not extrapolate with the 12-bit ω estimate — it chatters
/// at low speed and rotates the Park frame as if the rotor were shaking.
pub fn theta_m_sample() -> (f32, bool) {
    cortex_m::interrupt::free(|_| {
        (
            f32::from_bits(ENC_THETA_BITS.load(Ordering::Relaxed)),
            enc_valid(),
        )
    })
}

/// Coherent raw-angle/validity snapshot; the clock advances even if the encoder task stalls.
pub fn electrical_angle(offset: f32, poles: u8) -> Option<f32> {
    let (raw, valid) = cortex_m::interrupt::free(|_| (enc_raw(), enc_valid()));
    valid.then(|| crate::foc::angle::park_theta_raw(raw, offset, poles))
}

/// Compatibility accessor: zero-order hold with a wall-clock freshness check.
pub fn theta_m_predict(_dt: f32) -> (f32, bool) {
    theta_m_sample()
}

/// Electrical speed for Dq feed-forward from the windowed RPM, not raw ω.
pub fn omega_e_ff(pole_pairs: u8) -> f32 {
    let rpm = cortex_m::interrupt::free(|cs| SPEED.borrow(cs).borrow().rpm()).unwrap_or(0.0);
    rpm * (core::f32::consts::TAU / 60.0) * f32::from(pole_pairs)
}

pub fn enc_raw() -> u16 {
    ENC_RAW.load(Ordering::Relaxed)
}

pub fn enc_valid() -> bool {
    cortex_m::interrupt::free(|_| {
        ENC_VALID.load(Ordering::Relaxed) && sample_fresh(&ENC_AGE, ENC_FAULT_MS)
    })
}

pub fn enc_mdeg() -> i32 {
    ENC_MDEG.load(Ordering::Relaxed)
}

pub fn enc_omega_mrad() -> i32 {
    ENC_OMEGA_MRAD.load(Ordering::Relaxed)
}

/// Conservative sample age for diagnostics; no extrapolation is enabled.
pub fn enc_age_us() -> Option<u32> {
    cortex_m::interrupt::free(|_| {
        enc_valid().then(|| {
            (Instant::now().as_micros() as u32).wrapping_sub(ENC_SAMPLED_US.load(Ordering::Relaxed))
        })
    })
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
static ISR_STATS: Mutex<Cell<CycleStats>> = Mutex::new(Cell::new(CycleStats::new()));
// Separate from resettable/saturating diagnostics: increments only after a sample is serviced.
static CONTROL_SEQUENCE: AtomicU32 = AtomicU32::new(0);

pub fn publish_control_completion() {
    CONTROL_SEQUENCE.fetch_add(1, Ordering::Relaxed);
}

pub fn control_sequence() -> u32 {
    CONTROL_SEQUENCE.load(Ordering::Relaxed)
}
/// One PWM period, not an allowance for exception entry/exit or scheduling latency.
pub const ISR_BUDGET_CYCLES: u32 = SYSCLK_FREQ_HZ / PWM_FREQ_HZ;
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
    cortex_m::interrupt::free(|cs| {
        VBUS_MV.store((s.vbus_v * 1000.0) as u16, Ordering::Relaxed);
        TEMP_C10.store((s.temp_c * 10.0) as i16, Ordering::Relaxed);
        let mut age = Freshness::new();
        if s.vbus_v.is_finite() && s.temp_c.is_finite() {
            age.refresh(s.bus_sampled_at_ms);
        }
        BUS_AGE.borrow(cs).set(age);
    });
}

pub fn invalidate_bus() {
    cortex_m::interrupt::free(|cs| BUS_AGE.borrow(cs).set(Freshness::new()));
}

/// VBUS, temperature, and freshness from the same publication.
pub fn bus_snapshot() -> (u16, i16, bool) {
    cortex_m::interrupt::free(|_| (vbus_mv(), temp_c10(), sample_fresh(&BUS_AGE, BUS_FAULT_MS)))
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

/// Record the JEOS handler body: flag handling + ADC reads + current control.
/// Excludes exception entry/exit, time waiting to enter the handler, and this
/// statistics publication. Includes any higher-priority IRQ that preempts it.
/// No logging or floating-point formatting in this path.
pub fn publish_isr_timing(start: u32, end: u32) {
    cortex_m::interrupt::free(|cs| {
        let cell = ISR_STATS.borrow(cs);
        let mut stats = cell.get();
        stats.record(start, end, ISR_BUDGET_CYCLES);
        cell.set(stats);
    });
}

pub fn isr_snapshot() -> CycleStats {
    cortex_m::interrupt::free(|cs| ISR_STATS.borrow(cs).get())
}

pub fn isr_cycles() -> u32 {
    isr_snapshot().last_cycles
}

pub fn isr_cycles_max() -> u32 {
    isr_snapshot().max_cycles
}

/// Clear the complete window atomically with respect to the ADC interrupt.
pub fn reset_isr_cycles() {
    cortex_m::interrupt::free(|cs| ISR_STATS.borrow(cs).set(CycleStats::new()));
}

pub fn cycles_to_us(cycles: u32) -> u32 {
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
    cortex_m::interrupt::free(|cs| SPEED.borrow(cs).borrow().rpm().unwrap_or(0.0) as i32)
}

pub fn rpm_ready() -> bool {
    cortex_m::interrupt::free(|cs| SPEED.borrow(cs).borrow().rpm().is_some() && enc_valid())
}

pub fn reset_speed_filter() {
    cortex_m::interrupt::free(|cs| SPEED.borrow(cs).borrow_mut().reset());
}
