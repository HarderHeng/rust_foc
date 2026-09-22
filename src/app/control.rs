//! Runtime mode and references. The current ISR and 1 kHz supervisor share this.
//! Time comes in as `dt_ms` from the analog task — not `embassy_time`.
//! Hardware enable/duty go through here only — not through the shell.

use core::cell::Cell;
use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, AtomicU8, Ordering};
use cortex_m::interrupt::Mutex;

use crate::app::speed as speed_loop;
use crate::app::{foc_isr, telemetry};
use crate::bsp::config::{
    ALIGN_ID_MA, ALIGN_MS, CMD_TIMEOUT_MS, CURRENT_KI, CURRENT_KP, IDQ_RAMP_A_S, MAX_CURRENT_MA,
    MOTOR_MAX_RPM, SPEED_IQ_RAMP_A_S, SPEED_KI, SPEED_KP, SPEED_RAMP_RPM_S, SPEED_RPM_MAX,
};
use crate::driver::analog;
use crate::driver::nvm;
use crate::driver::pwm::with_pwm;
use crate::foc::slew::SlewI32;
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
    Adc = 7,
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
            7 => Self::Adc,
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
            Self::Adc => "adc",
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

#[derive(Clone, Copy)]
struct ReferenceRamps {
    id: SlewI32,
    iq: SlewI32,
    rpm: SlewI32,
}

impl ReferenceRamps {
    const fn new(id: i32, iq: i32, rpm: i32) -> Self {
        Self {
            id: SlewI32::new(id),
            iq: SlewI32::new(iq),
            rpm: SlewI32::new(rpm),
        }
    }
}

static RAMPS: Mutex<Cell<ReferenceRamps>> = Mutex::new(Cell::new(ReferenceRamps::new(0, 0, 0)));

fn reset_ramps(id: i32, iq: i32, rpm: i32) {
    cortex_m::interrupt::free(|cs| RAMPS.borrow(cs).set(ReferenceRamps::new(id, iq, rpm)));
}

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

/// Sensor interlocks shared by the start transaction, supervisor and current ISR.
pub fn sensor_fault(target: Mode) -> Option<FaultKind> {
    use crate::bsp::config::{NTC_T_MAX_C, VBUS_OV_MV, VBUS_UV_MV};
    let (mv, temp, fresh) = telemetry::bus_snapshot();
    if !fresh {
        Some(FaultKind::Adc)
    } else if !(VBUS_UV_MV..=VBUS_OV_MV).contains(&mv) {
        Some(FaultKind::Vbus)
    } else if temp > (NTC_T_MAX_C * 10.0) as i16 {
        Some(FaultKind::Overtemp)
    } else if matches!(target, Mode::Align | Mode::Run | Mode::Speed) && !telemetry::enc_valid() {
        Some(FaultKind::Encoder)
    } else {
        None
    }
}

/// Entire start commit is interrupt-atomic. No waits, flash or regular ADC reads here.
/// Changing between live modes requires an explicit stop.
fn enter_mode(target: Mode, prepare: impl FnOnce()) -> bool {
    cortex_m::interrupt::free(|_| {
        if mode() != Mode::Idle || sensor_fault(target).is_some() {
            return false;
        }
        prepare();
        foc_isr::reset();
        foc_isr::reset_openloop();
        ID_MA.store(0, Ordering::Relaxed);
        IQ_MA.store(0, Ordering::Relaxed);
        reset_ramps(0, 0, rpm_ref());
        let duty = if target == Mode::Bench {
            pwm_pct() as f32 / 100.0
        } else {
            0.5
        };
        let duties = crate::foc::Duties {
            a: duty,
            b: duty,
            c: duty,
        };
        let enabled = with_pwm(|p| {
            analog::with_analog(|a| {
                // Both current decoding and CCR4 must match the prepared duties.
                crate::foc::DutySink::apply(a, duties);
                p.prepare_start(duties)
            })
            .unwrap_or(false)
        })
        .unwrap_or(false);
        if !enabled {
            fault(FaultKind::Brake);
            return false;
        }
        LAST_FAULT.store(FaultKind::None as u8, Ordering::Relaxed);
        touch_cmd();
        MODE.store(target as u8, Ordering::Relaxed);
        true
    })
}

pub fn start() -> bool {
    cortex_m::interrupt::free(|_| {
        if mode() == Mode::Run {
            touch_cmd();
            return true;
        }
        enter_mode(Mode::Run, || {})
    })
}

pub fn start_openloop(vq_mv: i32, hz: u8) -> bool {
    use crate::bsp::config::{OPENLOOP_HZ_MAX, OPENLOOP_VQ_MAX_MV};
    enter_mode(Mode::Openloop, || {
        OL_VQ_MV.store(vq_mv.clamp(0, OPENLOOP_VQ_MAX_MV), Ordering::Relaxed);
        OL_HZ.store(hz.min(OPENLOOP_HZ_MAX), Ordering::Relaxed);
    })
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

pub fn start_speed(rpm: i32) -> bool {
    cortex_m::interrupt::free(|_| {
        if mode() == Mode::Speed {
            // A setpoint update is not a restart: retain PI and speed-filter state.
            set_rpm_ref(rpm);
            return true;
        }
        enter_mode(Mode::Speed, || {
            let lim = i32::from(MOTOR_MAX_RPM).min(SPEED_RPM_MAX);
            RPM_REF.store(telemetry::rpm_meas().clamp(-lim, lim), Ordering::Relaxed);
            set_rpm_ref(rpm);
            ID_TGT_MA.store(0, Ordering::Relaxed);
            IQ_TGT_MA.store(0, Ordering::Relaxed);
            speed_loop::reset();
        })
    })
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

pub fn start_bench() -> bool {
    enter_mode(Mode::Bench, || {})
}

fn clear_references() {
    ID_MA.store(0, Ordering::Relaxed);
    IQ_MA.store(0, Ordering::Relaxed);
    ID_TGT_MA.store(0, Ordering::Relaxed);
    IQ_TGT_MA.store(0, Ordering::Relaxed);
    RPM_TGT.store(0, Ordering::Relaxed);
    RPM_REF.store(0, Ordering::Relaxed);
    OL_VQ_MV.store(0, Ordering::Relaxed);
    OL_HZ.store(0, Ordering::Relaxed);
    ALIGN_HOLDING.store(false, Ordering::Relaxed);
    reset_ramps(0, 0, 0);
}

pub fn stop() {
    cortex_m::interrupt::free(|_| {
        let _ = with_pwm(|p| p.disable());
        speed_loop::reset();
        clear_references();
        MODE.store(Mode::Idle as u8, Ordering::Relaxed);
    });
}

pub fn fault(kind: FaultKind) {
    cortex_m::interrupt::free(|_| {
        let _ = with_pwm(|p| p.disable());
        if mode() != Mode::Fault {
            // Preserve the first trip cause, not a later consequence of it.
            LAST_FAULT.store(kind as u8, Ordering::Relaxed);
            MODE.store(Mode::Fault as u8, Ordering::Relaxed);
            speed_loop::reset();
            clear_references();
        }
    });
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
    if dt_ms == 0 {
        return;
    }
    NOW_MS.fetch_add(dt_ms, Ordering::Relaxed);
    cortex_m::interrupt::free(|_| {
        poll_refs(dt_ms);
        poll_align();
    });
}

/// Blocking regular-ADC offset. PWM must be off.
pub fn calibrate_offsets() -> bool {
    if outputs_live() || mode() == Mode::Fault {
        return false;
    }
    analog::recalibrate()
}

/// Hold D-axis current with θe=0 for [`ALIGN_MS`] *after Id arrives*, then latch offset.
pub fn request_align(id_ma: Option<i32>) -> bool {
    enter_mode(Mode::Align, || {
        let id = id_ma.unwrap_or(ALIGN_ID_MA).clamp(1, MAX_CURRENT_MA);
        ID_TGT_MA.store(id, Ordering::Relaxed);
        IQ_TGT_MA.store(0, Ordering::Relaxed);
        ALIGN_HOLDING.store(false, Ordering::Relaxed);
    })
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
    cortex_m::interrupt::free(|_| {
        if mode() != Mode::Align {
            return;
        }
        if let Some(kind) = sensor_fault(Mode::Align) {
            fault(kind);
            return;
        }
        latch_electrical_offset();
        let _ = with_pwm(|p| p.disable());
        clear_references();
        foc_isr::reset();
        MODE.store(Mode::Idle as u8, Ordering::Relaxed);
        // Flash writes remain explicit and outside any critical section.
    });
}

pub fn set_id_ma(ma: i32) {
    ID_TGT_MA.store(ma.clamp(-MAX_CURRENT_MA, MAX_CURRENT_MA), Ordering::Relaxed);
    touch_cmd();
}

pub fn set_iq_ma(ma: i32) {
    IQ_TGT_MA.store(ma.clamp(-MAX_CURRENT_MA, MAX_CURRENT_MA), Ordering::Relaxed);
    touch_cmd();
}

/// Speed PI writes the Iq **target**; `poll_refs` slews the value the ISR uses.
pub fn write_iq_ma(ma: i32) {
    cortex_m::interrupt::free(|_| {
        if mode() == Mode::Speed {
            IQ_TGT_MA.store(ma.clamp(-MAX_CURRENT_MA, MAX_CURRENT_MA), Ordering::Relaxed);
        }
    });
}

/// Slew Id/Iq (Run/Align) and rpm (Speed). ISR reads the slewed values.
pub fn poll_refs(dt_ms: u32) {
    if dt_ms == 0 {
        return;
    }
    // Keep fractional state and its published integer references in one transaction.
    // Fault/stop reset both, even if this function is called outside tick().
    cortex_m::interrupt::free(|cs| {
        let cell = RAMPS.borrow(cs);
        let mut ramps = cell.get();
        let mode = mode();
        let ma_s = IDQ_RAMP_A_S * 1000.0;
        if matches!(mode, Mode::Align | Mode::Run) {
            let id = ramps.id.step(id_target_ma(), ma_s, dt_ms);
            ID_MA.store(id, Ordering::Relaxed);
        }
        if mode == Mode::Run {
            let iq = ramps.iq.step(iq_target_ma(), ma_s, dt_ms);
            IQ_MA.store(iq, Ordering::Relaxed);
        } else if mode == Mode::Speed {
            let iq = ramps
                .iq
                .step(iq_target_ma(), SPEED_IQ_RAMP_A_S * 1000.0, dt_ms);
            IQ_MA.store(iq, Ordering::Relaxed);
            let rpm = ramps.rpm.step(rpm_target(), SPEED_RAMP_RPM_S, dt_ms);
            RPM_REF.store(rpm, Ordering::Relaxed);
        }
        cell.set(ramps);
    });
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

pub fn set_poles(n: u8) -> bool {
    cortex_m::interrupt::free(|_| {
        if mode() != Mode::Idle {
            return false;
        }
        POLES.store(n.max(1), Ordering::Relaxed);
        true
    })
}

pub fn poles() -> u8 {
    POLES.load(Ordering::Relaxed)
}

pub fn set_pwm_pct(pct: u8) -> bool {
    cortex_m::interrupt::free(|_| {
        if mode() != Mode::Idle {
            return false;
        }
        PWM_DUTY_PCT.store(pct.min(100), Ordering::Relaxed);
        start_bench()
    })
}

pub fn pwm_pct() -> u8 {
    PWM_DUTY_PCT.load(Ordering::Relaxed)
}

fn latch_electrical_offset() {
    let te = crate::foc::angle::park_theta_raw(telemetry::enc_raw(), 0.0, poles());
    THETA_E_OFF_MRAD.store((te * 1000.0) as i32, Ordering::Relaxed);
}

pub fn capture_electrical_offset() -> bool {
    cortex_m::interrupt::free(|_| {
        if mode() != Mode::Idle || !telemetry::enc_valid() {
            return false;
        }
        latch_electrical_offset();
        true
    })
}

pub fn set_theta_e_off_mrad(mrad: i32) -> bool {
    cortex_m::interrupt::free(|_| {
        if mode() != Mode::Idle {
            return false;
        }
        let bounded = wrap_2pi(mrad as f32 / 1000.0);
        THETA_E_OFF_MRAD.store((bounded * 1000.0) as i32, Ordering::Relaxed);
        true
    })
}

/// Apply last flash record at boot. Missing/corrupt page leaves defaults.
pub fn load_nvm() -> bool {
    match nvm::load() {
        Some(r) => {
            THETA_E_OFF_MRAD.store(r.theta_e_off_mrad, Ordering::Relaxed);
            POLES.store(r.poles.max(1), Ordering::Relaxed);
            defmt::info!("nvm load off={} poles={}", r.theta_e_off_mrad, r.poles);
            true
        }
        None => {
            defmt::info!("nvm empty");
            false
        }
    }
}

/// Erase+program the NVM page. Refuses while PWM is live.
pub fn persist_nvm() -> bool {
    if outputs_live() {
        return false;
    }
    nvm::save(nvm::Record {
        theta_e_off_mrad: theta_e_off_mrad(),
        poles: poles(),
    })
}

pub fn nvm_loaded() -> bool {
    nvm::load().is_some()
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
