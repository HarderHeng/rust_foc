//! Bring up board peripherals after clocks are running.

use embassy_stm32::i2c::I2c;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{Config as UartConfig, RingBufferedUartRx, Uart, UartTx};
use embassy_stm32::Peripherals;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use static_cell::StaticCell;
use stm32g431_foc::bsp::config::UART_BAUDRATE;
use stm32g431_foc::driver::pwm::init_pwm;
use stm32g431_foc::driver::shell::LedHandle;
use stm32g431_foc::driver::{As5600, Led, MotorPwm, i2c_config};

use crate::irqs::Irqs;

static LED: StaticCell<Mutex<CriticalSectionRawMutex, Led>> = StaticCell::new();
static UART_RX_DMA: StaticCell<[u8; 128]> = StaticCell::new();

pub struct Board {
    pub led: &'static LedHandle,
    pub uart_rx: RingBufferedUartRx<'static>,
    pub uart_tx: UartTx<'static, Async>,
    pub encoder: As5600,
}

pub fn start(p: Peripherals) -> Board {
    let led = LED.init(Mutex::new(Led::new(p.PC6.into())));
    defmt::info!("LED initialized on PC6");

    init_pwm(MotorPwm::new(
        p.TIM1, p.PA8, p.PC13, p.PA9, p.PA12, p.PA10, p.PB15,
    ));
    defmt::info!("TIM1 PWM 20kHz center-aligned, MOE off");

    let i2c = I2c::new(p.I2C1, p.PB8, p.PB7, p.DMA1_CH3, p.DMA1_CH4, Irqs, i2c_config());
    defmt::info!("I2C1 AS5600 async on PB8/PB7");

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
    defmt::info!("UART2 initialized at {} baud", UART_BAUDRATE);

    Board {
        led,
        uart_rx,
        uart_tx,
        encoder: As5600::new(i2c),
    }
}
