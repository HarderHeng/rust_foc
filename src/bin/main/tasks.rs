use embassy_stm32::exti::ExtiInput;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{RingBufferedUartRx, UartTx};
use embassy_time::{Duration, Instant, Timer};
use stm32g431_foc::app::control;
use stm32g431_foc::app::shell::Shell;
use stm32g431_foc::app::telemetry;
use stm32g431_foc::app::speed;
use stm32g431_foc::bsp::config::{AS5600_PERIOD_US, ENC_FAULT_MS, NTC_T_MAX_C, VBUS_OV_MV, VBUS_UV_MV};
use stm32g431_foc::driver::analog;
use stm32g431_foc::driver::as5600::As5600;
use stm32g431_foc::driver::led::LedHandle;

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
        if control::mode() == control::Mode::Fault {
            Timer::after_millis(100).await;
        } else {
            Timer::after_secs(1).await;
        }
        count += 1;
        defmt::info!("heartbeat {}", count);
        led.lock().await.toggle();
    }
}

#[embassy_executor::task]
pub async fn encoder_task(mut enc: As5600) {
    let mut last = Instant::now();
    let mut last_ok = Instant::now();
    let period = Duration::from_micros(AS5600_PERIOD_US as u64);
    loop {
        let start = Instant::now();
        let dt = start.duration_since(last).as_micros() as f32 / 1_000_000.0;
        last = start;
        let s = enc.read(dt.max(1e-5)).await;
        telemetry::publish_angle(s.raw, s.theta_m, s.omega_m, s.valid);
        if s.valid {
            last_ok = start;
        } else if start.duration_since(last_ok) >= Duration::from_millis(ENC_FAULT_MS as u64)
            && matches!(control::mode(), control::Mode::Run | control::Mode::Speed)
        {
            control::fault(control::FaultKind::Encoder);
        }
        let _ = speed::tick(s.omega_m, s.valid, dt.max(1e-5));
        let used = Instant::now().duration_since(start);
        if used < period {
            Timer::after(period - used).await;
        }
    }
}

#[embassy_executor::task]
pub async fn analog_task() {
    let mut last = Instant::now();
    loop {
        let now = Instant::now();
        let dt_ms = now.duration_since(last).as_millis().max(1) as u32;
        last = now;
        control::tick(dt_ms);

        if let Some(s) = analog::read_bus() {
            telemetry::publish_bus(s);
            let mv = telemetry::vbus_mv();
            let t10 = telemetry::temp_c10();
            if control::outputs_live() && mv > 0 && !(VBUS_UV_MV..=VBUS_OV_MV).contains(&mv) {
                control::fault(control::FaultKind::Vbus);
            } else if control::outputs_live() && t10 > (NTC_T_MAX_C * 10.0) as i16 {
                control::fault(control::FaultKind::Overtemp);
            } else if control::cmd_timed_out() {
                control::fault(control::FaultKind::CmdTimeout);
            }
        } else if control::cmd_timed_out() {
            control::fault(control::FaultKind::CmdTimeout);
        }
        Timer::after_millis(1).await;
    }
}

#[embassy_executor::task]
pub async fn button_task(mut button: ExtiInput<'static, Async>) {
    loop {
        button.wait_for_falling_edge().await;
        Timer::after_millis(30).await;
        if control::mode() == control::Mode::Fault || control::outputs_live() {
            control::stop();
        } else {
            control::start();
        }
        while button.is_low() {
            Timer::after_millis(10).await;
        }
    }
}
