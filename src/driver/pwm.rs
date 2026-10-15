//! TIM1 complementary PWM for B-G431B-ESC1 phase pins.

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

use embassy_stm32::gpio::{OutputType, Pull, Speed};
use embassy_stm32::pac::timer::vals::{Dir, Ocm};
use embassy_stm32::pac::TIM1 as TIM1_PAC;
use embassy_stm32::peripherals::{PA10, PA12, PA8, PA9, PB15, PC13, TIM1};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::complementary_pwm::{
    BreakComparatorPolarity, BreakInputPolarity, ComplementaryPwm, ComplementaryPwmPin,
    FilterValue, Mms2, Ossi, Ossr,
};
use embassy_stm32::timer::low_level::{CountingMode, MasterMode};
use embassy_stm32::timer::simple_pwm::{PwmPin, PwmPinConfig};
use embassy_stm32::timer::Channel;
use embassy_stm32::Peri;

use crate::bsp::config::{pwm_deadtime_ticks, tw_after_ticks, tw_before_ticks, PWM_FREQ_HZ};
use crate::driver::analog::Analog;
use crate::foc::sampling::{plan_sampling, SamplePlan};
use crate::foc::Duties;

static PWM: Mutex<RefCell<Option<MotorPwm>>> = Mutex::new(RefCell::new(None));

/// Plan without borrowing the output driver or masking interrupts. Center-aligned
/// TIM1 uses ARR as the HAL's full-scale compare value.
pub fn plan_duties(d: Duties) -> Option<SamplePlan> {
    if !d.a.is_finite() || !d.b.is_finite() || !d.c.is_finite() {
        return None;
    }
    let arr = u32::from(TIM1_PAC.arr().read().arr());
    let d = d.clamp01();
    let ccr = |x: f32| (x * arr as f32) as u32;
    plan_sampling(
        [ccr(d.a), ccr(d.b), ccr(d.c)],
        arr,
        tw_after_ticks(),
        tw_before_ticks(),
    )
}

fn pwm_pin_cfg() -> PwmPinConfig {
    PwmPinConfig {
        output_type: OutputType::PushPull,
        speed: Speed::VeryHigh,
        pull: Pull::Down,
    }
}

pub fn init_pwm(pwm: MotorPwm) {
    cortex_m::interrupt::free(|cs| {
        let mut slot = PWM.borrow(cs).borrow_mut();
        assert!(slot.is_none(), "PWM already initialized");
        *slot = Some(pwm);
    });
}

/// ISR-safe, checked exclusive borrow. Reentrant calls return None.
pub fn with_pwm<R>(f: impl FnOnce(&mut MotorPwm) -> R) -> Option<R> {
    cortex_m::interrupt::free(|cs| {
        let mut slot = PWM.borrow(cs).try_borrow_mut().ok()?;
        slot.as_mut().map(f)
    })
}

/// Terminal panic/HardFault path only: no shared borrow, allocation or logging.
/// Interrupts remain disabled; normal firmware execution must never resume.
pub fn emergency_shutdown() -> ! {
    cortex_m::interrupt::disable();
    TIM1_PAC.bdtr().modify(|w| w.set_moe(false));
    // Also disconnect all six gate outputs, independently of the MOE idle setup.
    TIM1_PAC.ccer().modify(|w| {
        for ch in 0..3 {
            w.set_cce(ch, false);
            w.set_ccne(ch, false);
        }
    });
    cortex_m::asm::dsb();
    loop {
        cortex_m::asm::nop();
    }
}

/// Six-switch TIM1 driver. Outputs stay idle-low until [`MotorPwm::prepare_start`].
pub struct MotorPwm {
    inner: ComplementaryPwm<'static, TIM1>,
    max_duty: u32,
}

impl MotorPwm {
    pub fn new(
        tim: Peri<'static, TIM1>,
        uh: Peri<'static, PA8>,
        ul: Peri<'static, PC13>,
        vh: Peri<'static, PA9>,
        vl: Peri<'static, PA12>,
        wh: Peri<'static, PA10>,
        wl: Peri<'static, PB15>,
    ) -> Self {
        let cfg = pwm_pin_cfg();
        let mut inner = ComplementaryPwm::new(
            tim,
            Some(PwmPin::new_with_config(uh, cfg)),
            Some(ComplementaryPwmPin::new_with_config(ul, cfg)),
            Some(PwmPin::new_with_config(vh, cfg)),
            Some(ComplementaryPwmPin::new_with_config(vl, cfg)),
            Some(PwmPin::new_with_config(wh, cfg)),
            Some(ComplementaryPwmPin::new_with_config(wl, cfg)),
            None,
            None,
            Hertz::hz(PWM_FREQ_HZ),
            CountingMode::CenterAlignedUpInterrupts,
        );

        // `new()` enables MOE; drop it before CCER is touched.
        inner.set_master_output_enable(false);

        inner.set_dead_time(pwm_deadtime_ticks());
        inner.set_off_state_selection_idle(Ossi::IDLE_LEVEL);
        inner.set_off_state_selection_run(Ossr::IDLE_LEVEL);
        // Both H and L idle-low (122 OCIdle / OCNIdle = LOW).
        TIM1_PAC.cr2().modify(|w| {
            w.set_ois(0, false);
            w.set_oisn(0, false);
            w.set_ois(1, false);
            w.set_oisn(1, false);
            w.set_ois(2, false);
            w.set_oisn(2, false);
        });

        inner.enable(Channel::Ch1);
        inner.enable(Channel::Ch2);
        inner.enable(Channel::Ch3);

        let max_duty = inner.get_max_duty();
        // 122: CH4 is PWM2, TRGO = OC4REF. Rising edge near the counter peak.
        TIM1_PAC
            .ccmr_output(1)
            .modify(|w| w.set_ocm(1, Ocm::PWM_MODE2));
        inner.set_duty(Channel::Ch4, max_duty.saturating_sub(1));
        // CC4E only (no CH4N pin). 122 R3_2 enables CH4 so TIM1_CH4 can trigger injected ADC.
        TIM1_PAC.ccer().modify(|w| w.set_cce(3, true));
        TIM1_PAC
            .cr2()
            .modify(|w| w.set_mms(MasterMode::COMPARE_OC4));
        inner.set_mms2(Mms2::RESET);

        let mid = max_duty / 2;
        inner.set_duty(Channel::Ch1, mid);
        inner.set_duty(Channel::Ch2, mid);
        inner.set_duty(Channel::Ch3, mid);
        inner.set_master_output_enable(false);

        Self { inner, max_duty }
    }

    /// 122 `MX_TIM1`: BKCOMP1/2/4 → BRK, filter FDIV2_N6, AOE off.
    /// Call after DAC/COMP are live so a floating comparator cannot trip first.
    pub fn enable_comp_break(&mut self) {
        self.inner.set_break_input_pin_enable(false);
        self.inner
            .set_break_comparator_polarity(0, BreakComparatorPolarity::NOT_INVERTED);
        self.inner
            .set_break_comparator_polarity(1, BreakComparatorPolarity::NOT_INVERTED);
        self.inner
            .set_break_comparator_polarity(3, BreakComparatorPolarity::NOT_INVERTED);
        self.inner.set_break_comparator_enable(0, true);
        self.inner.set_break_comparator_enable(1, true);
        self.inner.set_break_comparator_enable(3, true);
        self.inner
            .set_break_polarity(BreakInputPolarity::ACTIVE_HIGH);
        self.inner.set_break_filter(FilterValue::FDTS_DIV2_N6);
        self.inner.set_automatic_output_enable(false);
        self.inner.set_break_enable(true);
        TIM1_PAC.dier().modify(|w| w.set_bie(true));
    }

    pub fn max_duty(&self) -> u32 {
        self.max_duty
    }

    /// Live TIM1 CCR1..4 (counts). For shell/bring-up, not used in the ISR.
    pub fn read_ccr(&self) -> [u32; 4] {
        [
            u32::from(TIM1_PAC.ccr(0).read().ccr()),
            u32::from(TIM1_PAC.ccr(1).read().ccr()),
            u32::from(TIM1_PAC.ccr(2).read().ccr()),
            u32::from(TIM1_PAC.ccr(3).read().ccr()),
        ]
    }

    /// Arm only after safe CCR values have been transferred to the active registers.
    /// A pending hardware break must be serviced, never cleared by a start request.
    pub fn prepare_start(&mut self, plan: &SamplePlan, analog: &mut Analog) -> bool {
        self.disable();
        if TIM1_PAC.sr().read().bif(0) {
            return false;
        }
        // A pending JDR plus one in-flight old conversion may precede the first
        // new trigger. Neither is valid feedback for this start configuration.
        analog.discard_startup_samples();
        self.write_plan(plan, analog);
        TIM1_PAC.egr().write(|r| r.set_ug(true));
        self.inner.set_master_output_enable(true);
        if TIM1_PAC.sr().read().bif(0) {
            self.disable();
            return false;
        }
        self.is_enabled()
    }

    pub fn disable(&mut self) {
        self.inner.set_master_output_enable(false);
    }

    pub fn is_enabled(&self) -> bool {
        self.inner.get_master_output_enable()
    }

    /// Use the actual timer ARR and the same integer quantization as CCR writes.
    /// Invalid voltages must never turn into an arbitrary duty through float casts.
    pub fn plan(&self, d: Duties) -> Option<SamplePlan> {
        plan_duties(d)
    }

    /// Commit all preloads and ADC routing from one plan. UDIS prevents a timer
    /// update from transferring only a prefix of CCR1..4 while these are written.
    /// This transaction must finish before the next CH4 edge (board timing check).
    pub fn apply_plan(&mut self, plan: &SamplePlan, analog: &mut Analog) -> Option<Duties> {
        let updates_disabled = TIM1_PAC.cr1().read().udis();
        TIM1_PAC.cr1().modify(|w| w.set_udis(true));
        let before = TIM1_PAC.cnt().read().cnt();
        let arr = TIM1_PAC.arr().read().arr();
        // Center sampling completes on the down-count. Plans must reach all
        // preloads before the valley, where the next period becomes active.
        // A new JEOS here means this handler has already missed another sample.
        // After a whole-cycle delay, DIR can already be DOWN again while the
        // next conversion has not yet raised JEOS. Exclude that peak-side gap.
        if TIM1_PAC.cr1().read().dir() != Dir::DOWN
            || u32::from(before) > u32::from(arr).saturating_sub(tw_before_ticks())
            || embassy_stm32::pac::ADC1.isr().read().jeos()
        {
            self.disable();
            TIM1_PAC.cr1().modify(|w| w.set_udis(updates_disabled));
            return None;
        }
        self.write_plan_registers(plan, analog);
        // Restore updates before the final check: crossing the valley while
        // UDIS was set must be detected even if it happens during restoration.
        TIM1_PAC.cr1().modify(|w| w.set_udis(updates_disabled));
        let after = TIM1_PAC.cnt().read().cnt();
        let timely = TIM1_PAC.cr1().read().dir() == Dir::DOWN
            && after <= before
            && !embassy_stm32::pac::ADC1.isr().read().jeos();
        if !timely {
            self.disable();
        }
        timely.then(|| self.applied_duties(plan))
    }

    /// Startup has MOE off and explicitly transfers preloads with UG, so it
    /// does not need the live down-count deadline guard.
    fn write_plan(&mut self, plan: &SamplePlan, analog: &mut Analog) {
        let updates_disabled = TIM1_PAC.cr1().read().udis();
        TIM1_PAC.cr1().modify(|w| w.set_udis(true));
        self.write_plan_registers(plan, analog);
        TIM1_PAC.cr1().modify(|w| w.set_udis(updates_disabled));
    }

    fn write_plan_registers(&mut self, plan: &SamplePlan, analog: &mut Analog) {
        self.inner.set_duty(Channel::Ch1, plan.ccr[0]);
        self.inner.set_duty(Channel::Ch2, plan.ccr[1]);
        self.inner.set_duty(Channel::Ch3, plan.ccr[2]);
        self.inner.set_duty(Channel::Ch4, plan.ccr4);
        analog.schedule_pair(plan.pair);
    }

    fn applied_duties(&self, plan: &SamplePlan) -> Duties {
        let max = self.max_duty as f32;
        Duties {
            a: plan.ccr[0] as f32 / max,
            b: plan.ccr[1] as f32 / max,
            c: plan.ccr[2] as f32 / max,
        }
    }
}
