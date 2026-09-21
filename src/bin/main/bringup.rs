use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Pull;
use embassy_stm32::i2c::I2c;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{Config as UartConfig, RingBufferedUartRx, Uart, UartTx};
use embassy_stm32::Peripherals;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use static_cell::StaticCell;
use stm32g431_foc::app::foc_isr;
use stm32g431_foc::app::speed;
use stm32g431_foc::bsp::config::UART_BAUDRATE;
use stm32g431_foc::driver::analog;
use stm32g431_foc::driver::as5600::{i2c_config, As5600};
use stm32g431_foc::driver::led::{Led, LedHandle};
use stm32g431_foc::driver::ocp;
use stm32g431_foc::driver::pwm::{init_pwm, with_pwm, MotorPwm};

use crate::irqs::Irqs;

static LED: StaticCell<Mutex<CriticalSectionRawMutex, Led>> = StaticCell::new();
static UART_RX_DMA: StaticCell<[u8; 128]> = StaticCell::new();

pub struct Board {
    pub led: &'static LedHandle,
    pub uart_rx: RingBufferedUartRx<'static>,
    pub uart_tx: UartTx<'static, Async>,
    pub encoder: As5600,
    pub button: ExtiInput<'static, Async>,
}

pub fn start(p: Peripherals) -> Board {
    let led = LED.init(Mutex::new(Led::new(p.PC6.into())));

    init_pwm(MotorPwm::new(
        p.TIM1, p.PA8, p.PC13, p.PA9, p.PA12, p.PA10, p.PB15,
    ));

    foc_isr::init();
    speed::init();
    stm32g431_foc::app::control::init_gains();
    analog::init(
        p.OPAMP1, p.OPAMP2, p.OPAMP3, p.ADC1, p.ADC2, p.PA1, p.PA3, p.PA2, p.PA7, p.PA5, p.PA6, p.PB0,
        p.PB2, p.PB1, p.PA0, p.PB14,
    );
    ocp::init(p.DAC3);
    let _ = with_pwm(|p| p.enable_comp_break());
    crate::irqs::enable_motor();

    let button = ExtiInput::new(p.PC10, p.EXTI10, Pull::None, Irqs);

    let i2c = I2c::new(p.I2C1, p.PB8, p.PB7, p.DMA1_CH3, p.DMA1_CH4, Irqs, i2c_config());

    let mut uart_cfg = UartConfig::default();
    uart_cfg.baudrate = UART_BAUDRATE;
    let uart = Uart::new(
        p.USART2,
        p.PB4,
        p.PB3,
        p.DMA1_CH1,
        p.DMA1_CH2,
        Irqs,
        uart_cfg,
    )
    .expect("UART init failed");
    let (uart_tx, rx) = uart.split();
    let uart_rx = rx.into_ring_buffered(UART_RX_DMA.init([0u8; 128]));

    Board {
        led,
        uart_rx,
        uart_tx,
        encoder: As5600::new(i2c),
        button,
    }
}
