//! TIM1 complementary PWM for B-G431B-ESC1 phase pins.

use core::sync::atomic::{AtomicPtr, Ordering};

use embassy_stm32::Peri;
use embassy_stm32::gpio::OutputType;
use embassy_stm32::peripherals::{PA8, PA9, PA10, PA12, PB15, PC13, TIM1};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::Channel;
use embassy_stm32::timer::complementary_pwm::{
    BreakComparatorPolarity, BreakInputPolarity, ComplementaryPwm, ComplementaryPwmPin, FilterValue, Mms2,
};
use embassy_stm32::timer::low_level::{CountingMode, MasterMode};
use embassy_stm32::timer::simple_pwm::PwmPin;
use static_cell::StaticCell;

use crate::bsp::config::{PWM_FREQ_HZ, pwm_deadtime_ticks};
use crate::foc::Duties;

static PWM: StaticCell<MotorPwm> = StaticCell::new();
static PWM_PTR: AtomicPtr<MotorPwm> = AtomicPtr::new(core::ptr::null_mut());

pub fn init_pwm(pwm: MotorPwm) -> &'static mut MotorPwm {
    let slot = PWM.init(pwm);
    PWM_PTR.store(slot as *mut MotorPwm, Ordering::Release);
    slot
}

/// ISR-safe: short critical section, no Embassy mutex.
pub fn with_pwm<R>(f: impl FnOnce(&mut MotorPwm) -> R) -> Option<R> {
    cortex_m::interrupt::free(|_| {
        let p = PWM_PTR.load(Ordering::Acquire);
        if p.is_null() {
            None
        } else {
            Some(f(unsafe { &mut *p }))
        }
    })
}

/// Six-switch TIM1 driver. Outputs stay Hi-Z until [`MotorPwm::enable`].
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
        let mut inner = ComplementaryPwm::new(
            tim,
            Some(PwmPin::new(uh, OutputType::PushPull)),
            Some(ComplementaryPwmPin::new(ul, OutputType::PushPull)),
            Some(PwmPin::new(vh, OutputType::PushPull)),
            Some(ComplementaryPwmPin::new(vl, OutputType::PushPull)),
            Some(PwmPin::new(wh, OutputType::PushPull)),
            Some(ComplementaryPwmPin::new(wl, OutputType::PushPull)),
            None,
            None,
            Hertz::hz(PWM_FREQ_HZ),
            CountingMode::CenterAlignedUpInterrupts,
        );

        // `new()` enables MOE; drop it before CCER is touched.
        inner.set_master_output_enable(false);

        inner.set_dead_time(pwm_deadtime_ticks());
        inner.enable(Channel::Ch1);
        inner.enable(Channel::Ch2);
        inner.enable(Channel::Ch3);

        let max_duty = inner.get_max_duty();
        // CH4 unused on pins: CCR=ARR → OC4REF at counter peak (low-side ON).
        // Cube 122: TRGO = OC4REF, TRGO2 = RESET; ADC injected uses TIM1_CH4.
        inner.set_duty(Channel::Ch4, max_duty);
        embassy_stm32::pac::TIM1.cr2().modify(|w| w.set_mms(MasterMode::COMPARE_OC4));
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
        self.inner.set_break_polarity(BreakInputPolarity::ACTIVE_HIGH);
        self.inner.set_break_filter(FilterValue::FDTS_DIV2_N6);
        self.inner.set_automatic_output_enable(false);
        self.inner.set_break_enable(true);
        embassy_stm32::pac::TIM1.dier().modify(|w| w.set_bie(true));
    }

    pub fn max_duty(&self) -> u32 {
        self.max_duty
    }

    pub fn enable(&mut self) {
        self.inner.set_master_output_enable(true);
    }

    pub fn disable(&mut self) {
        self.inner.set_master_output_enable(false);
    }

    pub fn is_enabled(&self) -> bool {
        self.inner.get_master_output_enable()
    }

    pub fn set_duties(&mut self, d: Duties) {
        let d = d.clamp01();
        let max = self.max_duty as f32;
        let ccr = |x: f32| (x * max).clamp(0.0, max) as u32;
        self.inner.set_duty(Channel::Ch1, ccr(d.a));
        self.inner.set_duty(Channel::Ch2, ccr(d.b));
        self.inner.set_duty(Channel::Ch3, ccr(d.c));
    }

    /// Same duty on all phases (PWM bring-up, motor disconnected).
    pub fn set_duty_all(&mut self, duty_01: f32) {
        self.set_duties(Duties {
            a: duty_01,
            b: duty_01,
            c: duty_01,
        });
    }
}
