//! 3-shunt OPAMP PGA + dual ADC (122 `MX_OPAMP*` / `MX_ADC*`).
//!
//! Phase currents: TIM1 CH4 / OC4REF rising, R3_2 two-phase pair (122
//! `ADCConfig1/2`). Third phase is reconstructed. VBUS / NTC stay regular.

use core::sync::atomic::{AtomicPtr, Ordering};

use embassy_stm32::adc::{Adc, AdcChannel, AdcConfig, AnyAdcChannel, SampleTime};
use embassy_stm32::opamp::{OpAmp, OpAmpGain, OpAmpOutput, OpAmpSpeed};
use embassy_stm32::pac::adc::vals::Exten;
use embassy_stm32::pac::{ADC1, ADC2};
use embassy_stm32::peripherals::{
    ADC1 as Adc1Peri, ADC2 as Adc2Peri, OPAMP1, OPAMP2, OPAMP3, PA0, PA1, PA2, PA3, PA5, PA6, PA7, PB0, PB1, PB2, PB14,
};
use embassy_stm32::Peri;
use static_cell::StaticCell;

use crate::bsp::config::{adc_to_amps, adc_to_temp_c, adc_to_vbus, SAMPLE_CENTER_MARGIN};
use crate::foc::{Duties, DutySink, PhaseAbc, PhaseCurrents};

const VBUS_SAMPLE: SampleTime = SampleTime::CYCLES247_5;
const NTC_SAMPLE: SampleTime = SampleTime::CYCLES47_5;
const INJ_SAMPLE: SampleTime = SampleTime::CYCLES2_5;
const OFFSET_SAMPLES: u8 = 16;

/// Cube `LL_ADC_INJ_TRIG_EXT_TIM1_CH4` = `ADC_JSQR_JEXTSEL_0` → field value 1.
const JEXTSEL_TIM1_CH4: u8 = 1;
const ADC1_CH_U: u8 = 3;
const ADC1_CH_W: u8 = 12;
const ADC2_CH_VOPAMP3: u8 = 18;
const ADC2_CH_V: u8 = 3;

static OP1: StaticCell<OpAmp<'static, OPAMP1>> = StaticCell::new();
static OP2: StaticCell<OpAmp<'static, OPAMP2>> = StaticCell::new();
static OP3: StaticCell<OpAmp<'static, OPAMP3>> = StaticCell::new();
static ANALOG: StaticCell<Analog> = StaticCell::new();
static ANALOG_PTR: AtomicPtr<Analog> = AtomicPtr::new(core::ptr::null_mut());

#[derive(Clone, Copy, Default)]
pub struct AnalogSample {
    pub iu_raw: u16,
    pub iv_raw: u16,
    pub iw_raw: u16,
    pub vbus_raw: u16,
    pub ntc_raw: u16,
    pub iu_a: f32,
    pub iv_a: f32,
    pub iw_a: f32,
    pub vbus_v: f32,
    pub temp_c: f32,
}

impl PhaseCurrents for AnalogSample {
    fn abc(&self) -> PhaseAbc {
        PhaseAbc {
            a: self.iu_a,
            b: self.iv_a,
            c: self.iw_a,
        }
    }
}

pub struct Analog {
    out_u: OpAmpOutput<'static, OPAMP1>,
    out_v: OpAmpOutput<'static, OPAMP2>,
    out_w: OpAmpOutput<'static, OPAMP3>,
    adc1: Adc<'static, Adc1Peri>,
    adc2: Adc<'static, Adc2Peri>,
    vbus: AnyAdcChannel<'static, Adc1Peri>,
    ntc: AnyAdcChannel<'static, Adc1Peri>,
    off_u: u16,
    off_v: u16,
    off_w: u16,
    last_pair: ShuntPair,
    next_pair: ShuntPair,
}

/// Simultaneous pair for the next TIM1 CH4 edge (122 sector tables).
#[derive(Clone, Copy, PartialEq, Eq)]
enum ShuntPair {
    /// ADC1 U + ADC2 V (ST sector 4/5, mid-PWM default)
    Uv,
    /// ADC1 U + ADC2 VOPAMP3 (sector 2/3)
    Uw,
    /// ADC1 W + ADC2 V (sector 6/1)
    Vw,
}

#[allow(clippy::too_many_arguments)]
pub fn init(
    opamp1: Peri<'static, OPAMP1>,
    opamp2: Peri<'static, OPAMP2>,
    opamp3: Peri<'static, OPAMP3>,
    adc1: Peri<'static, Adc1Peri>,
    adc2: Peri<'static, Adc2Peri>,
    iu_p: Peri<'static, PA1>,
    iu_n: Peri<'static, PA3>,
    iu_out: Peri<'static, PA2>,
    iv_p: Peri<'static, PA7>,
    iv_n: Peri<'static, PA5>,
    iv_out: Peri<'static, PA6>,
    iw_p: Peri<'static, PB0>,
    iw_n: Peri<'static, PB2>,
    iw_out: Peri<'static, PB1>,
    vbus: Peri<'static, PA0>,
    ntc: Peri<'static, PB14>,
) -> &'static mut Analog {
    let op1 = OP1.init(OpAmp::new(opamp1, OpAmpSpeed::Normal));
    let op2 = OP2.init(OpAmp::new(opamp2, OpAmpSpeed::Normal));
    let op3 = OP3.init(OpAmp::new(opamp3, OpAmpSpeed::Normal));

    let out_u = op1.pga_biased_ext(iu_p, iu_n, iu_out, OpAmpGain::Mul16);
    let out_v = op2.pga_biased_ext(iv_p, iv_n, iv_out, OpAmpGain::Mul16);
    let out_w = op3.pga_biased_ext(iw_p, iw_n, iw_out, OpAmpGain::Mul16);

    let slot = ANALOG.init(Analog {
        out_u,
        out_v,
        out_w,
        adc1: Adc::new(adc1, AdcConfig::default()),
        adc2: Adc::new(adc2, AdcConfig::default()),
        vbus: vbus.degrade_adc(),
        ntc: ntc.degrade_adc(),
        off_u: 0,
        off_v: 0,
        off_w: 0,
        last_pair: ShuntPair::Uv,
        next_pair: ShuntPair::Uv,
    });
    slot.calibrate_offsets();
    arm_injected();
    ANALOG_PTR.store(slot as *mut Analog, Ordering::Release);
    slot
}

/// Short ISR sections only. Do not block (no regular ADC) inside `f`.
pub fn with_analog<R>(f: impl FnOnce(&mut Analog) -> R) -> Option<R> {
    cortex_m::interrupt::free(|_| analog_mut().map(f))
}

/// VBUS/NTC: must not mask the current ISR for a 247-cycle conversion.
pub fn read_bus() -> Option<AnalogSample> {
    analog_mut().map(|a| a.read_bus())
}

/// Re-run shunt offset (PWM off). Regular ADC, not the injected ISR path.
pub fn recalibrate() -> bool {
    match analog_mut() {
        Some(a) => {
            a.calibrate_offsets();
            true
        }
        None => false,
    }
}

fn analog_mut() -> Option<&'static mut Analog> {
    let p = ANALOG_PTR.load(Ordering::Acquire);
    if p.is_null() {
        None
    } else {
        Some(unsafe { &mut *p })
    }
}

/// Latest injected raw counts: (Iu, Iv, Iw). ISR-safe; does not start a conversion.
pub fn latest_injected() -> (u16, u16, u16) {
    with_analog(|a| a.decode_jdr()).unwrap_or((0, 0, 0))
}

fn set_smpr(adc: embassy_stm32::pac::adc::Adc, ch: u8, st: SampleTime) {
    if ch <= 9 {
        adc.smpr().modify(|reg| reg.set_smp(ch as usize, st));
    } else {
        adc.smpr2().modify(|reg| reg.set_smp((ch - 10) as usize, st));
    }
}

fn arm_injected() {
    ADC1.cfgr().modify(|r| {
        r.set_jdiscen(false);
        r.set_jauto(false);
        r.set_jqdis(true);
    });
    set_smpr(ADC1, ADC1_CH_U, INJ_SAMPLE);
    set_smpr(ADC1, ADC1_CH_W, INJ_SAMPLE);
    ADC2.cfgr().modify(|r| {
        r.set_jdiscen(false);
        r.set_jauto(false);
        r.set_jqdis(true);
    });
    set_smpr(ADC2, ADC2_CH_VOPAMP3, INJ_SAMPLE);
    set_smpr(ADC2, ADC2_CH_V, INJ_SAMPLE);
    program_pair(ShuntPair::Uv);
    ADC1.cr().modify(|r| r.set_jadstart(true));
    ADC2.cr().modify(|r| r.set_jadstart(true));
    ADC1.ier().modify(|r| r.set_jeosie(true));
}

fn write_jsqr(adc: embassy_stm32::pac::adc::Adc, ch: u8) {
    adc.jsqr().write(|w| {
        w.set_jl(0);
        w.set_jextsel(JEXTSEL_TIM1_CH4);
        w.set_jexten(Exten::RISING_EDGE);
        w.set_jsq(0, ch);
    });
}

/// G4: `OPAINTOEN=1` routes OPAMP3 to ADC2 CH18 and disconnects PB1 (ADC1 IN12).
fn route_opamp3(pair: ShuntPair) {
    let internal = matches!(pair, ShuntPair::Uw);
    embassy_stm32::pac::OPAMP3.csr().modify(|w| w.set_opaintoen(internal));
}

fn program_pair(pair: ShuntPair) {
    route_opamp3(pair);
    match pair {
        ShuntPair::Uv => {
            write_jsqr(ADC1, ADC1_CH_U);
            write_jsqr(ADC2, ADC2_CH_V);
        }
        ShuntPair::Uw => {
            write_jsqr(ADC1, ADC1_CH_U);
            write_jsqr(ADC2, ADC2_CH_VOPAMP3);
        }
        ShuntPair::Vw => {
            write_jsqr(ADC1, ADC1_CH_W);
            write_jsqr(ADC2, ADC2_CH_V);
        }
    }
}

fn pair_from_duties(d: Duties) -> ShuntPair {
    // 122: keep sector 5 (UV) while ARR − maxCCR > Tafter.
    if 1.0 - d.a.max(d.b).max(d.c) > SAMPLE_CENTER_MARGIN {
        return ShuntPair::Uv;
    }
    if d.a >= d.b && d.a >= d.c {
        ShuntPair::Vw
    } else if d.b >= d.a && d.b >= d.c {
        ShuntPair::Uw
    } else {
        ShuntPair::Uv
    }
}

impl Analog {
    fn decode_jdr(&self) -> (u16, u16, u16) {
        let j1 = ADC1.jdr(0).read().jdata();
        let j2 = ADC2.jdr(0).read().jdata();
        match self.last_pair {
            ShuntPair::Uv => (j1, j2, 0),
            ShuntPair::Uw => (j1, 0, j2),
            ShuntPair::Vw => (0, j2, j1),
        }
    }

    /// After SVPWM: program JSQR for the next CH4 edge.
    /// `last_pair` is the pair the *next* JEOS must decode (conversion not yet taken).
    pub fn schedule_pair(&mut self, d: Duties) {
        self.apply(d);
    }
    pub fn calibrate_offsets(&mut self) {
        route_opamp3(ShuntPair::Uv);
        let mut su = 0u32;
        let mut sv = 0u32;
        let mut sw = 0u32;
        for _ in 0..OFFSET_SAMPLES {
            su += u32::from(self.adc1.blocking_read(&mut self.out_u, NTC_SAMPLE));
            sv += u32::from(self.adc2.blocking_read(&mut self.out_v, NTC_SAMPLE));
            sw += u32::from(self.adc1.blocking_read(&mut self.out_w, NTC_SAMPLE));
        }
        self.off_u = (su / u32::from(OFFSET_SAMPLES)) as u16;
        self.off_v = (sv / u32::from(OFFSET_SAMPLES)) as u16;
        self.off_w = (sw / u32::from(OFFSET_SAMPLES)) as u16;
    }

    pub fn read_currents(&mut self) -> AnalogSample {
        let (iu_raw, iv_raw, iw_raw) = self.decode_jdr();
        let mut iu_a = adc_to_amps(iu_raw, self.off_u);
        let mut iv_a = adc_to_amps(iv_raw, self.off_v);
        let mut iw_a = adc_to_amps(iw_raw, self.off_w);
        match self.last_pair {
            ShuntPair::Uv => iw_a = -iu_a - iv_a,
            ShuntPair::Uw => iv_a = -iu_a - iw_a,
            ShuntPair::Vw => iu_a = -iv_a - iw_a,
        }
        AnalogSample {
            iu_raw,
            iv_raw,
            iw_raw,
            iu_a,
            iv_a,
            iw_a,
            ..AnalogSample::default()
        }
    }

    pub fn read_bus(&mut self) -> AnalogSample {
        let vbus_raw = self.adc1.blocking_read(&mut self.vbus, VBUS_SAMPLE);
        let ntc_raw = self.adc1.blocking_read(&mut self.ntc, NTC_SAMPLE);
        AnalogSample {
            vbus_raw,
            ntc_raw,
            vbus_v: adc_to_vbus(vbus_raw),
            temp_c: adc_to_temp_c(ntc_raw),
            ..AnalogSample::default()
        }
    }

    pub fn read(&mut self) -> AnalogSample {
        let mut s = self.read_currents();
        let bus = self.read_bus();
        s.vbus_raw = bus.vbus_raw;
        s.ntc_raw = bus.ntc_raw;
        s.vbus_v = bus.vbus_v;
        s.temp_c = bus.temp_c;
        s
    }
}

impl DutySink for Analog {
    fn apply(&mut self, d: Duties) {
        self.next_pair = pair_from_duties(d);
        program_pair(self.next_pair);
        self.last_pair = self.next_pair;
    }
}
