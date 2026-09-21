//! Hardware overcurrent: DAC3 threshold + COMP1/2/4 (122 `MX_DAC3` / `MX_COMP*`).
//!
//! COMP INP shares the shunt+ pins with the OPAMPs (already analog). TIM1 BRK
//! is armed from [`crate::driver::pwm::MotorPwm::enable_comp_break`].

use embassy_stm32::dac::{Dac, Value};
use embassy_stm32::mode::Blocking;
use embassy_stm32::pac::comp::vals::Inm;
use embassy_stm32::pac::{COMP1, COMP2, COMP4};
use embassy_stm32::peripherals::DAC3;
use embassy_stm32::Peri;
use static_cell::StaticCell;

use crate::bsp::config::OCP_DAC_COUNTS;

static DAC: StaticCell<Dac<'static, Blocking>> = StaticCell::new();

pub fn init(dac3: Peri<'static, DAC3>) {
    let mut dac = <Dac<'static, Blocking>>::new_internal(dac3);
    let v = Value::Bit12Right(OCP_DAC_COUNTS);
    dac.ch1().set(v);
    dac.ch2().set(v);
    DAC.init(dac);

    // INP = IO1 (PA1 / PA7 / PB0). INM = DAC3 CH1 (COMP1) or CH2 (COMP2/4).
    configure(COMP1, Inm::DACA);
    configure(COMP2, Inm::DACB);
    configure(COMP4, Inm::DACB);
}

fn configure(comp: embassy_stm32::pac::comp::Comp, inm: Inm) {
    comp.csr().modify(|w| {
        w.set_inpsel(false);
        w.set_inmsel(inm);
        w.set_hyst(embassy_stm32::pac::comp::vals::Hysteresis::NONE);
        w.set_polarity(embassy_stm32::pac::comp::vals::Polarity::NOT_INVERTED);
        w.set_en(true);
    });
}
