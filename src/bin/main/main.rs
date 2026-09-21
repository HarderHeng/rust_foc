//! STM32G431 FOC entry: clocks, bring-up, spawn tasks.

#![no_std]
#![no_main]

use defmt::unwrap;
use embassy_executor::Spawner;
use {defmt_rtt as _, panic_probe as _};

use stm32g431_foc::bsp;

mod bringup;
mod irqs;
mod tasks;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = bsp::init();
    defmt::info!("STM32G431 initialized - SYSCLK: 170MHz, HSE: 8MHz");

    let board = bringup::start(p);

    spawner.spawn(unwrap!(tasks::shell_task(board.uart_rx, board.uart_tx, board.led)));
    spawner.spawn(unwrap!(tasks::heartbeat_task(board.led)));
    spawner.spawn(unwrap!(tasks::encoder_task(board.encoder)));
    spawner.spawn(unwrap!(tasks::analog_task()));
    spawner.spawn(unwrap!(tasks::button_task(board.button)));

    defmt::info!("All tasks started");
}
