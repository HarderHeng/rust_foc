//! Runtime mode and references. Embassy and the current ISR share this.
//! Hardware enable/duty go through here only — not through the shell.

use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, AtomicU8, Ordering};

use embassy_time::{Duration, Instant};

use crate::app::{foc_isr, telemetry};
use crate::app::speed as speed_loop;
use crate::bsp::config::{
    CMD_TIMEOUT_MS, CURRENT_KI, CURRENT_KP, MAX_CURRENT_MA, MOTOR_MAX_RPM, SPEED_KI, SPEED_KP, SPEED_RPM_MAX,
};
use crate::driver::analog;
use crate::driver::pwm::with_pwm;
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
static ALIGN_REQ: AtomicBool = AtomicBool::new(false);
static ID_MA: AtomicI32 = AtomicI32::new(0);
static IQ_MA: AtomicI32 = AtomicI32::new(0);
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
static LAST_CMD_TICKS: AtomicU32 = AtomicU32::new(0);

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
    RPM_REF.store(rpm.clamp(-lim, lim), Ordering::Relaxed);
    LAST_FAULT.store(FaultKind::None as u8, Ordering::Relaxed);
    touch_cmd();
    speed_loop::reset();
    foc_isr::reset();
    let _ = with_pwm(|p| p.enable());
    MODE.store(Mode::Speed as u8, Ordering::Relaxed);
}

pub fn set_rpm_ref(rpm: i32) {
    let lim = i32::from(MOTOR_MAX_RPM).min(SPEED_RPM_MAX);
    RPM_REF.store(rpm.clamp(-lim, lim), Ordering::Relaxed);
    touch_cmd();
}

pub fn rpm_ref() -> i32 {
    RPM_REF.load(Ordering::Relaxed)
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
    ALIGN_REQ.store(false, Ordering::Relaxed);
    speed_loop::reset();
    MODE.store(Mode::Idle as u8, Ordering::Relaxed);
}

pub fn fault(kind: FaultKind) {
    let _ = with_pwm(|p| p.disable());
    ALIGN_REQ.store(false, Ordering::Relaxed);
    speed_loop::reset();
    LAST_FAULT.store(kind as u8, Ordering::Relaxed);
    MODE.store(Mode::Fault as u8, Ordering::Relaxed);
}

pub fn last_fault() -> FaultKind {
    FaultKind::from_u8(LAST_FAULT.load(Ordering::Relaxed))
}

pub fn touch_cmd() {
    LAST_CMD_TICKS.store(Instant::now().as_ticks() as u32, Ordering::Relaxed);
}

/// Run / Speed only: no new command for [`CMD_TIMEOUT_MS`].
pub fn cmd_timed_out() -> bool {
    if !matches!(mode(), Mode::Run | Mode::Speed) {
        return false;
    }
    let last = LAST_CMD_TICKS.load(Ordering::Relaxed);
    Instant::from_ticks(u64::from(last)).elapsed() > Duration::from_millis(u64::from(CMD_TIMEOUT_MS))
}

/// Blocking regular-ADC offset. PWM must be off.
pub fn calibrate_offsets() -> bool {
    if outputs_live() || mode() == Mode::Fault {
        return false;
    }
    analog::recalibrate()
}

pub fn request_align() {
    if mode() == Mode::Fault {
        return;
    }
    ALIGN_REQ.store(true, Ordering::Relaxed);
    LAST_FAULT.store(FaultKind::None as u8, Ordering::Relaxed);
    touch_cmd();
    let _ = with_pwm(|p| p.enable());
    MODE.store(Mode::Align as u8, Ordering::Relaxed);
}

pub fn take_align() -> bool {
    ALIGN_REQ.swap(false, Ordering::Relaxed)
}

pub fn set_id_ma(ma: i32) {
    ID_MA.store(ma.clamp(-MAX_CURRENT_MA, MAX_CURRENT_MA), Ordering::Relaxed);
    touch_cmd();
}

pub fn set_iq_ma(ma: i32) {
    write_iq_ma(ma);
    touch_cmd();
}

/// Speed PI writes Iq without refreshing the command watchdog.
pub fn write_iq_ma(ma: i32) {
    IQ_MA.store(ma.clamp(-MAX_CURRENT_MA, MAX_CURRENT_MA), Ordering::Relaxed);
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
    let (theta_m, _) = telemetry::theta_m_interp(0.0);
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
