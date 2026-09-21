//! Embassy tasks. Keep FOC ISR out of here.

use embassy_stm32::mode::Async;
use embassy_stm32::usart::{RingBufferedUartRx, UartTx};
use embassy_time::{Instant, Timer};
use stm32g431_foc::app;
use stm32g431_foc::driver::as5600::As5600;
use stm32g431_foc::driver::shell::{LedHandle, Shell};

#[embassy_executor::task]
pub async fn shell_task(
    mut rx: RingBufferedUartRx<'static>,
    tx: UartTx<'static, Async>,
    led: &'static LedHandle,
) {
    Shell::new(tx).run(&mut rx, led).await;
}

#[embassy_executor::task]
pub async fn heartbeat_task(led: &'static LedHandle) {
    let mut count = 0u32;
    loop {
        Timer::after_secs(1).await;
        count += 1;
        defmt::info!("heartbeat {}", count);
        led.lock().await.toggle();
    }
}

#[embassy_executor::task]
pub async fn encoder_task(mut enc: As5600) {
    let mut last = Instant::now();
    loop {
        let now = Instant::now();
        let dt = now.duration_since(last).as_micros() as f32 / 1_000_000.0;
        last = now;
        let s = enc.read(dt.max(1e-4)).await;
        app::publish_angle(s.raw, s.theta_m, s.omega_m, s.valid);
        Timer::after_millis(1).await;
    }
}
