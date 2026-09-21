//! Runtime mode and references. The current ISR and 1 kHz supervisor share this.
//! Time comes in as `dt_ms` from the analog task — not `embassy_time`.
//! Hardware enable/duty go through here only — not through the shell.

use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, AtomicU8, Ordering};

use crate::app::{foc_isr, telemetry};
use crate::app::speed as speed_loop;
use crate::bsp::config::{
    ALIGN_ID_MA, ALIGN_MS, CMD_TIMEOUT_MS, CURRENT_KI, CURRENT_KP, IDQ_RAMP_A_S, MAX_CURRENT_MA, MOTOR_MAX_RPM,
    RPM_RAMP_RPM_S, SPEED_KI, SPEED_KP, SPEED_RPM_MAX,
};
use crate::driver::analog;
use crate::driver::pwm::with_pwm;
use crate::foc::slew::approach_i32;
use crate::foc::transforms::wrap_2pi;

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Idle = 0,
    /// Equal-duty bring-up (`foc pwm`). Current loop does not write CCR.
    Bench = 1,
    Align = 2,
    Fault = 3,
    /// Id/Iq current loop owns TIM1 CCR.
    Run = 4,
    /// Ramped θe, fixed Vq (`foc openloop`).
    Openloop = 5,
    /// Speed PI writes Iq; current ISR still runs.
    Speed = 6,
}

impl Mode {
    fn from_u8(v: u8) -> Self {
        match v {
            1 => Self::Bench,
            2 => Self::Align,
            3 => Self::Fault,
            4 => Self::Run,
            5 => Self::Openloop,
            6 => Self::Speed,
            _ => Self::Idle,
        }
    }

    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Idle => "idle",
            Self::Bench => "bench",
            Self::Align => "align",
            Self::Fault => "fault",
            Self::Run => "run",
            Self::Openloop => "openloop",
            Self::Speed => "speed",
        }
    }
}

/// Latched trip cause. Survives `foc stop` so the shell can still print it.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum FaultKind {
    None = 0,
    Overcurrent = 1,
    Vbus = 2,
    Overtemp = 3,
    Encoder = 4,
    CmdTimeout = 5,
    Brake = 6,
}

impl FaultKind {
    fn from_u8(v: u8) -> Self {
        match v {
            1 => Self::Overcurrent,
            2 => Self::Vbus,
            3 => Self::Overtemp,
            4 => Self::Encoder,
            5 => Self::CmdTimeout,
            6 => Self::Brake,
            _ => Self::None,
        }
    }

    pub const fn as_str(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::Overcurrent => "ocp",
            Self::Vbus => "vbus",
            Self::Overtemp => "ntc",
            Self::Encoder => "enc",
            Self::CmdTimeout => "timeout",
            Self::Brake => "brk",
        }
    }
}

static MODE: AtomicU8 = AtomicU8::new(Mode::Idle as u8);
static ID_MA: AtomicI32 = AtomicI32::new(0);
static IQ_MA: AtomicI32 = AtomicI32::new(0);
static ID_TGT_MA: AtomicI32 = AtomicI32::new(0);
static IQ_TGT_MA: AtomicI32 = AtomicI32::new(0);
static RPM_TGT: AtomicI32 = AtomicI32::new(0);
static POLES: AtomicU8 = AtomicU8::new(crate::bsp::config::DEFAULT_POLE_PAIRS);
static PWM_DUTY_PCT: AtomicU8 = AtomicU8::new(50);
/// Electrical angle at D-axis lock (`poles * θm` when `θe = 0`).
static THETA_E_OFF_MRAD: AtomicI32 = AtomicI32::new(0);
static OL_VQ_MV: AtomicI32 = AtomicI32::new(0);
static OL_HZ: AtomicU8 = AtomicU8::new(0);
static RPM_REF: AtomicI32 = AtomicI32::new(0);
static CUR_KP: AtomicU32 = AtomicU32::new(0);
static CUR_KI: AtomicU32 = AtomicU32::new(0);
static SPD_KP: AtomicU32 = AtomicU32::new(0);
static SPD_KI: AtomicU32 = AtomicU32::new(0);
static LAST_FAULT: AtomicU8 = AtomicU8::new(FaultKind::None as u8);
static NOW_MS: AtomicU32 = AtomicU32::new(0);
static LAST_CMD_MS: AtomicU32 = AtomicU32::new(0);
static ALIGN_START_MS: AtomicU32 = AtomicU32::new(0);
static ALIGN_HOLDING: AtomicBool = AtomicBool::new(false);

fn now_ms() -> u32 {
    NOW_MS.load(Ordering::Relaxed)
}

#[derive(Clone, Copy)]
pub struct Snapshot {
    pub mode: Mode,
    pub id_ma: i32,
    pub iq_ma: i32,
    pub poles: u8,
    pub pwm_pct: u8,
}

pub fn snapshot() -> Snapshot {
    Snapshot {
        mode: mode(),
        id_ma: id_ma(),
        iq_ma: iq_ma(),
        poles: poles(),
        pwm_pct: pwm_pct(),
    }
}

pub fn mode() -> Mode {
    Mode::from_u8(MODE.load(Ordering::Relaxed))
}

pub fn outputs_live() -> bool {
    match mode() {
        Mode::Bench | Mode::Align | Mode::Run | Mode::Openloop | Mode::Speed => true,
        Mode::Idle | Mode::Fault => false,
    }
}

pub fn start() {
    if mode() == Mode::Fault {
        return;
    }
    LAST_FAULT.store(FaultKind::None as u8, Ordering::Relaxed);
    touch_cmd();
    foc_isr::reset();
    let _ = with_pwm(|p| p.enable());
    MODE.store(Mode::Run as u8, Ordering::Relaxed);
}

pub fn start_openloop(vq_mv: i32, hz: u8) {
    if mode() == Mode::Fault {
        return;
    }
    use crate::bsp::config::{OPENLOOP_HZ_MAX, OPENLOOP_VQ_MAX_MV};
    OL_VQ_MV.store(vq_mv.clamp(0, OPENLOOP_VQ_MAX_MV), Ordering::Relaxed);
    OL_HZ.store(hz.min(OPENLOOP_HZ_MAX), Ordering::Relaxed);
    LAST_FAULT.store(FaultKind::None as u8, Ordering::Relaxed);
    touch_cmd();
    foc_isr::reset_openloop();
    let _ = with_pwm(|p| p.enable());
    MODE.store(Mode::Openloop as u8, Ordering::Relaxed);
}

pub fn ol_vq_v() -> f32 {
    OL_VQ_MV.load(Ordering::Relaxed) as f32 / 1000.0
}

pub fn ol_vq_mv() -> i32 {
    OL_VQ_MV.load(Ordering::Relaxed)
}

pub fn ol_hz() -> u8 {
    OL_HZ.load(Ordering::Relaxed)
}

pub fn start_speed(rpm: i32) {
    if mode() == Mode::Fault {
        return;
    }
    let lim = i32::from(MOTOR_MAX_RPM).min(SPEED_RPM_MAX);
    let tgt = rpm.clamp(-lim, lim);
    RPM_TGT.store(tgt, Ordering::Relaxed);
    RPM_REF.store(telemetry::rpm_meas().clamp(-lim, lim), Ordering::Relaxed);
    ID_TGT_MA.store(0, Ordering::Relaxed);
    ID_MA.store(0, Ordering::Relaxed);
    LAST_FAULT.store(FaultKind::None as u8, Ordering::Relaxed);
    touch_cmd();
    speed_loop::reset();
    foc_isr::reset();
    let _ = with_pwm(|p| p.enable());
    MODE.store(Mode::Speed as u8, Ordering::Relaxed);
}

pub fn set_rpm_ref(rpm: i32) {
    let lim = i32::from(MOTOR_MAX_RPM).min(SPEED_RPM_MAX);
    RPM_TGT.store(rpm.clamp(-lim, lim), Ordering::Relaxed);
    touch_cmd();
}

pub fn rpm_ref() -> i32 {
    RPM_REF.load(Ordering::Relaxed)
}

pub fn rpm_target() -> i32 {
    RPM_TGT.load(Ordering::Relaxed)
}

pub fn start_bench() {
    if mode() == Mode::Fault {
        return;
    }
    apply_bench_duty();
    LAST_FAULT.store(FaultKind::None as u8, Ordering::Relaxed);
    touch_cmd();
    let _ = with_pwm(|p| p.enable());
    MODE.store(Mode::Bench as u8, Ordering::Relaxed);
}

pub fn stop() {
    let _ = with_pwm(|p| p.disable());
    speed_loop::reset();
    ID_MA.store(0, Ordering::Relaxed);
    IQ_MA.store(0, Ordering::Relaxed);
    MODE.store(Mode::Idle as u8, Ordering::Relaxed);
}

pub fn fault(kind: FaultKind) {
    let _ = with_pwm(|p| p.disable());
    speed_loop::reset();
    LAST_FAULT.store(kind as u8, Ordering::Relaxed);
    MODE.store(Mode::Fault as u8, Ordering::Relaxed);
}

pub fn last_fault() -> FaultKind {
    FaultKind::from_u8(LAST_FAULT.load(Ordering::Relaxed))
}

pub fn touch_cmd() {
    LAST_CMD_MS.store(now_ms(), Ordering::Relaxed);
}

/// Run with a non-zero Id/Iq target: trip if the operator goes silent.
/// Speed keeps `rpm` as a standing command (no timeout).
pub fn cmd_timed_out() -> bool {
    if mode() != Mode::Run || (id_target_ma() == 0 && iq_target_ma() == 0) {
        return false;
    }
    now_ms().wrapping_sub(LAST_CMD_MS.load(Ordering::Relaxed)) > CMD_TIMEOUT_MS
}

/// 1 kHz supervisor: advance the control clock, then slew refs and align.
/// `dt_ms` is wall time from the analog task (Embassy stays at that edge).
pub fn tick(dt_ms: u32) {
    let dt = dt_ms.max(1);
    NOW_MS.fetch_add(dt, Ordering::Relaxed);
    poll_refs(dt);
    poll_align();
}

/// Blocking regular-ADC offset. PWM must be off.
pub fn calibrate_offsets() -> bool {
    if outputs_live() || mode() == Mode::Fault {
        return false;
    }
    analog::recalibrate()
}

/// Hold D-axis current with θe=0 for [`ALIGN_MS`] *after Id arrives*, then latch offset.
pub fn request_align(id_ma: Option<i32>) {
    if mode() == Mode::Fault {
        return;
    }
    if outputs_live() && mode() != Mode::Align {
        stop();
    }
    let id = id_ma.unwrap_or(ALIGN_ID_MA).clamp(1, MAX_CURRENT_MA);
    ID_TGT_MA.store(id, Ordering::Relaxed);
    IQ_TGT_MA.store(0, Ordering::Relaxed);
    IQ_MA.store(0, Ordering::Relaxed);
    ALIGN_HOLDING.store(false, Ordering::Relaxed);
    LAST_FAULT.store(FaultKind::None as u8, Ordering::Relaxed);
    touch_cmd();
    foc_isr::reset();
    let _ = with_pwm(|p| p.enable());
    MODE.store(Mode::Align as u8, Ordering::Relaxed);
}

/// Call from a 1 ms task. Completes align after the hold window.
pub fn poll_align() {
    if mode() != Mode::Align {
        return;
    }
    if !ALIGN_HOLDING.load(Ordering::Relaxed) {
        if (id_ma() - ID_TGT_MA.load(Ordering::Relaxed)).unsigned_abs() <= 2 {
            ALIGN_HOLDING.store(true, Ordering::Relaxed);
            ALIGN_START_MS.store(now_ms(), Ordering::Relaxed);
        }
        return;
    }
    if align_left_ms() == 0 {
        finish_align();
    }
}

pub fn align_left_ms() -> u32 {
    if mode() != Mode::Align {
        return 0;
    }
    if !ALIGN_HOLDING.load(Ordering::Relaxed) {
        return ALIGN_MS;
    }
    let gone = now_ms().wrapping_sub(ALIGN_START_MS.load(Ordering::Relaxed));
    ALIGN_MS.saturating_sub(gone)
}

fn finish_align() {
    if !telemetry::enc_valid() {
        ID_TGT_MA.store(0, Ordering::Relaxed);
        ID_MA.store(0, Ordering::Relaxed);
        fault(FaultKind::Encoder);
        return;
    }
    capture_electrical_offset();
    ID_TGT_MA.store(0, Ordering::Relaxed);
    IQ_TGT_MA.store(0, Ordering::Relaxed);
    ID_MA.store(0, Ordering::Relaxed);
    IQ_MA.store(0, Ordering::Relaxed);
    foc_isr::reset();
    let _ = with_pwm(|p| p.disable());
    MODE.store(Mode::Idle as u8, Ordering::Relaxed);
}

pub fn set_id_ma(ma: i32) {
    ID_TGT_MA.store(ma.clamp(-MAX_CURRENT_MA, MAX_CURRENT_MA), Ordering::Relaxed);
    touch_cmd();
}

pub fn set_iq_ma(ma: i32) {
    IQ_TGT_MA.store(ma.clamp(-MAX_CURRENT_MA, MAX_CURRENT_MA), Ordering::Relaxed);
    touch_cmd();
}

/// Speed PI writes Iq without refreshing the command watchdog or the Iq slew.
pub fn write_iq_ma(ma: i32) {
    let v = ma.clamp(-MAX_CURRENT_MA, MAX_CURRENT_MA);
    IQ_MA.store(v, Ordering::Relaxed);
    IQ_TGT_MA.store(v, Ordering::Relaxed);
}

/// Slew Id/Iq (Run/Align) and rpm (Speed). ISR reads the slewed values.
pub fn poll_refs(dt_ms: u32) {
    let dt = dt_ms.max(1) as f32 / 1000.0;
    let ma_s = IDQ_RAMP_A_S * 1000.0;
    match mode() {
        Mode::Align | Mode::Run => {
            let id = approach_i32(id_ma(), ID_TGT_MA.load(Ordering::Relaxed), ma_s, dt);
            ID_MA.store(id, Ordering::Relaxed);
        }
        _ => {}
    }
    if mode() == Mode::Run {
        let iq = approach_i32(iq_ma(), IQ_TGT_MA.load(Ordering::Relaxed), ma_s, dt);
        IQ_MA.store(iq, Ordering::Relaxed);
    }
    if mode() == Mode::Speed {
        let rpm = approach_i32(rpm_ref(), RPM_TGT.load(Ordering::Relaxed), RPM_RAMP_RPM_S, dt);
        RPM_REF.store(rpm, Ordering::Relaxed);
    }
}

pub fn id_target_ma() -> i32 {
    ID_TGT_MA.load(Ordering::Relaxed)
}

pub fn iq_target_ma() -> i32 {
    IQ_TGT_MA.load(Ordering::Relaxed)
}

pub fn id_ma() -> i32 {
    ID_MA.load(Ordering::Relaxed)
}

pub fn iq_ma() -> i32 {
    IQ_MA.load(Ordering::Relaxed)
}

pub fn id_a() -> f32 {
    id_ma() as f32 / 1000.0
}

pub fn iq_a() -> f32 {
    iq_ma() as f32 / 1000.0
}

pub fn set_poles(n: u8) {
    POLES.store(n.max(1), Ordering::Relaxed);
}

pub fn poles() -> u8 {
    POLES.load(Ordering::Relaxed)
}

pub fn set_pwm_pct(pct: u8) {
    PWM_DUTY_PCT.store(pct.min(100), Ordering::Relaxed);
    if mode() != Mode::Fault {
        start_bench();
    }
}

pub fn pwm_pct() -> u8 {
    PWM_DUTY_PCT.load(Ordering::Relaxed)
}

pub fn capture_electrical_offset() {
    let (theta_m, _) = telemetry::theta_m_sample();
    let te = wrap_2pi(theta_m * f32::from(poles()));
    THETA_E_OFF_MRAD.store((te * 1000.0) as i32, Ordering::Relaxed);
}

pub fn set_theta_e_off_mrad(mrad: i32) {
    THETA_E_OFF_MRAD.store(mrad, Ordering::Relaxed);
}

pub fn theta_e_off() -> f32 {
    THETA_E_OFF_MRAD.load(Ordering::Relaxed) as f32 / 1000.0
}

pub fn theta_e_off_mrad() -> i32 {
    THETA_E_OFF_MRAD.load(Ordering::Relaxed)
}

pub fn init_gains() {
    set_current_gains(CURRENT_KP, CURRENT_KI);
    set_speed_gains(SPEED_KP, SPEED_KI);
    touch_cmd();
}

pub fn current_kp() -> f32 {
    load_f32(&CUR_KP)
}
pub fn current_ki() -> f32 {
    load_f32(&CUR_KI)
}
pub fn speed_kp() -> f32 {
    load_f32(&SPD_KP)
}
pub fn speed_ki() -> f32 {
    load_f32(&SPD_KI)
}

pub fn set_current_gains(kp: f32, ki: f32) {
    store_f32(&CUR_KP, kp.max(0.0));
    store_f32(&CUR_KI, ki.max(0.0));
    foc_isr::set_gains(current_kp(), current_ki());
}

pub fn set_speed_gains(kp: f32, ki: f32) {
    store_f32(&SPD_KP, kp.max(0.0));
    store_f32(&SPD_KI, ki.max(0.0));
    speed_loop::set_gains(speed_kp(), speed_ki());
}

fn store_f32(a: &AtomicU32, v: f32) {
    a.store(v.to_bits(), Ordering::Relaxed);
}

fn load_f32(a: &AtomicU32) -> f32 {
    f32::from_bits(a.load(Ordering::Relaxed))
}

fn apply_bench_duty() {
    let d = pwm_pct() as f32 / 100.0;
    let _ = with_pwm(|p| p.set_duty_all(d));
}
