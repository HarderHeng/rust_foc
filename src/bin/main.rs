//! STM32G431 FOC Main Application
//!
//! Embassy async runtime with shell and LED tasks.

#![no_std]
#![no_main]

use defmt::unwrap;
use embassy_executor::Spawner;
use embassy_stm32::{bind_interrupts, dma, i2c, peripherals, usart};
use embassy_stm32::usart::{Uart, UartRx, Config};
use embassy_stm32::mode::Async;
use embassy_stm32::i2c::I2c;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Instant;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use stm32g431_foc::app;
use stm32g431_foc::bsp::init;
use stm32g431_foc::bsp::config::UART_BAUDRATE;
use stm32g431_foc::driver::{As5600, Led, MotorPwm, Shell, init_pwm, i2c_config};

bind_interrupts!(struct Irqs {
    USART2 => usart::InterruptHandler<peripherals::USART2>;
    DMA1_CHANNEL1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
    DMA1_CHANNEL2 => dma::InterruptHandler<peripherals::DMA1_CH2>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    DMA1_CHANNEL3 => dma::InterruptHandler<peripherals::DMA1_CH3>;
    DMA1_CHANNEL4 => dma::InterruptHandler<peripherals::DMA1_CH4>;
});

/// Global LED instance wrapped in Mutex for shared mutable access
static LED: StaticCell<Mutex<CriticalSectionRawMutex, Led>> = StaticCell::new();

#[embassy_executor::task]
async fn shell_task(
    mut rx: UartRx<'static, Async>,
    tx: embassy_stm32::usart::UartTx<'static, Async>,
    led: &'static Mutex<CriticalSectionRawMutex, Led>,
) {
    Shell::new(tx).run(&mut rx, led).await;
}

/// Heartbeat task - blinks LED every 1 second
#[embassy_executor::task]
async fn heartbeat_task(led: &'static Mutex<CriticalSectionRawMutex, Led>) {
    let mut count = 0u32;
    loop {
        embassy_time::Timer::after_secs(1).await;
        count += 1;
        defmt::info!("heartbeat {}", count);

        // Toggle LED for visual heartbeat
        let mut led = led.lock().await;
        led.toggle();
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize board (170MHz clock)
    let p = init();
    defmt::info!("STM32G431 initialized - SYSCLK: 170MHz, HSE: 8MHz");

    // Initialize LED on PC6
    let led = LED.init(Mutex::new(Led::new(p.PC6.into())));
    defmt::info!("LED initialized on PC6");

    let pwm = MotorPwm::new(p.TIM1, p.PA8, p.PC13, p.PA9, p.PA12, p.PA10, p.PB15);
    init_pwm(pwm);
    defmt::info!("TIM1 PWM 20kHz center-aligned, MOE off");

    // I2C1: PB8=SCL (Z+/H3), PB7=SDA (B+/H2). PB6 is not I2C1 on G431.
    let i2c = I2c::new(
        p.I2C1,
        p.PB8,
        p.PB7,
        p.DMA1_CH3,
        p.DMA1_CH4,
        Irqs,
        i2c_config(),
    );
    defmt::info!("I2C1 AS5600 async on PB8/PB7");

    // Initialize UART2 on PB3/PB4
    let mut uart_cfg = Config::default();
    uart_cfg.baudrate = UART_BAUDRATE;

    let uart = Uart::new(
        p.USART2,
        p.PB4,  // RX
        p.PB3,  // TX
        p.DMA1_CH1,  // TX DMA
        p.DMA1_CH2,  // RX DMA
        Irqs,
        uart_cfg,
    ).expect("UART init failed");

    let (tx, rx) = uart.split();
    defmt::info!("UART2 initialized at {} baud", UART_BAUDRATE);

    spawner.spawn(unwrap!(shell_task(rx, tx, led)));
    spawner.spawn(unwrap!(heartbeat_task(led)));
    spawner.spawn(unwrap!(encoder_task(As5600::new(i2c))));

    defmt::info!("All tasks started");
}

#[embassy_executor::task]
async fn encoder_task(mut enc: As5600) {
    let mut last = Instant::now();
    loop {
        let now = Instant::now();
        let dt = now.duration_since(last).as_micros() as f32 / 1_000_000.0;
        last = now;
        let s = enc.read(dt.max(1e-4)).await;
        app::publish_angle(s.raw, s.theta_m, s.omega_m, s.valid);
        embassy_time::Timer::after_millis(1).await;
    }
}