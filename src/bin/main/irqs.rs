//! Embassy bindings plus motor-path NVIC (122 `MX_NVIC_Init`).

use embassy_stm32::interrupt::typelevel::Handler;
use embassy_stm32::interrupt::{self, InterruptExt};
use embassy_stm32::pac::{ADC1, ADC2, TIM1};
use embassy_stm32::{bind_interrupts, dma, exti, i2c, peripherals, usart};
use stm32g431_foc::app::{control, foc_isr};
use stm32g431_foc::driver::analog;

pub struct Tim1Brk;
impl Handler<interrupt::typelevel::TIM1_BRK_TIM15> for Tim1Brk {
    unsafe fn on_interrupt() {
        TIM1.sr().modify(|w| w.set_bif(0, false));
        control::fault();
    }
}

pub struct AdcJeos;
impl Handler<interrupt::typelevel::ADC1_2> for AdcJeos {
    unsafe fn on_interrupt() {
        if ADC1.isr().read().jeos() {
            ADC1.isr().modify(|r| r.set_jeos(true));
            ADC2.isr().modify(|r| r.set_jeos(true));
            if let Some(s) = analog::with_analog(|a| a.read_currents()) {
                foc_isr::on_injected(s);
            }
        }
    }
}

bind_interrupts!(pub struct Irqs {
    USART2 => usart::InterruptHandler<peripherals::USART2>;
    DMA1_CHANNEL1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
    DMA1_CHANNEL2 => dma::InterruptHandler<peripherals::DMA1_CH2>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    DMA1_CHANNEL3 => dma::InterruptHandler<peripherals::DMA1_CH3>;
    DMA1_CHANNEL4 => dma::InterruptHandler<peripherals::DMA1_CH4>;
    EXTI15_10 => exti::InterruptHandler<interrupt::typelevel::EXTI15_10>;
    TIM1_BRK_TIM15 => Tim1Brk;
    ADC1_2 => AdcJeos;
});

pub fn enable_motor() {
    interrupt::TIM1_BRK_TIM15.set_priority(interrupt::Priority::P1);
    interrupt::ADC1_2.set_priority(interrupt::Priority::P2);
    unsafe {
        interrupt::TIM1_BRK_TIM15.enable();
        interrupt::ADC1_2.enable();
    }
}
