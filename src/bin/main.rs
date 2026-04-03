//! STM32G431 FOC Main Application
//!
//! Embassy async runtime with shell and LED tasks.

#![no_std]
#![no_main]

use defmt::unwrap;
use embassy_executor::Spawner;
use embassy_stm32::{bind_interrupts, dma, peripherals, usart};
use embassy_stm32::usart::{Uart, UartRx, Config};
use embassy_stm32::mode::Async;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use stm32g431_foc::bsp::init;
use stm32g431_foc::driver::{Led, Shell, ShellWriter, init_shell_tx, get_shell_tx};
use stm32g431_foc::bsp::config::UART_BAUDRATE;

bind_interrupts!(struct Irqs {
    USART2 => usart::InterruptHandler<peripherals::USART2>;
    DMA1_CHANNEL1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
    DMA1_CHANNEL2 => dma::InterruptHandler<peripherals::DMA1_CH2>;
});

/// Global LED instance wrapped in Mutex for shared mutable access
static LED: StaticCell<Mutex<CriticalSectionRawMutex, Led>> = StaticCell::new();

/// Shell task - processes UART RX bytes
#[embassy_executor::task]
async fn shell_task(mut rx: UartRx<'static, Async>) {
    let writer = ShellWriter::new(get_shell_tx());
    let mut shell = Shell::new(writer);

    shell.print_welcome();

    let mut buf = [0u8; 1];
    loop {
        match rx.read(&mut buf).await {
            Ok(()) => {
                shell.process(buf[0]);

                // Handle LED commands (integration with actual LED)
                // Note: This is a simplified integration
                // Real implementation would need command parsing
            }
            Err(e) => {
                defmt::error!("UART RX error: {:?}", e);
            }
        }
    }
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
    init_shell_tx(tx);

    defmt::info!("UART2 initialized at {} baud", UART_BAUDRATE);

    // Spawn tasks
    spawner.spawn(unwrap!(shell_task(rx)));
    spawner.spawn(unwrap!(heartbeat_task(led)));

    defmt::info!("All tasks started");
}