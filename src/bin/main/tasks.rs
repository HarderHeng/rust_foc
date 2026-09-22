use embassy_stm32::exti::ExtiInput;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{RingBufferedUartRx, UartTx};
use embassy_time::{Duration, Instant, Timer};
use stm32g431_foc::app::control;
use stm32g431_foc::app::shell::Shell;
use stm32g431_foc::app::speed;
use stm32g431_foc::app::telemetry;
use stm32g431_foc::bsp::config::AS5600_PERIOD_US;
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
    loop {
        if control::mode() == control::Mode::Fault {
            Timer::after_millis(100).await;
        } else {
            Timer::after_secs(1).await;
        }
        led.lock().await.toggle();
    }
}

#[embassy_executor::task]
pub async fn encoder_task(mut enc: As5600) {
    let mut last = Instant::now();
    let mut last_valid = Instant::now();
    let period = Duration::from_micros(AS5600_PERIOD_US as u64);
    loop {
        let start = Instant::now();
        let dt = start.duration_since(last).as_micros() as f32 / 1_000_000.0;
        last = start;
        // Δraw spans valid samples, including any failed transactions in between.
        let valid_dt = start.duration_since(last_valid).as_micros() as f32 / 1_000_000.0;
        let s = enc.read(valid_dt.max(1e-5)).await;
        telemetry::publish_angle(s.raw, s.theta_m, s.omega_m, s.valid);
        if s.valid {
            telemetry::push_speed_from_raw(s.raw, valid_dt.max(1e-5));
            last_valid = start;
        } else if matches!(
            control::mode(),
            control::Mode::Align | control::Mode::Run | control::Mode::Speed
        ) {
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
    let mut last_ms = Instant::now().as_millis();
    loop {
        let now_ms = Instant::now().as_millis();
        // Difference of absolute millisecond stamps retains sub-ms time across
        // polls; flooring each individual interval would systematically lose it.
        let dt_ms = now_ms.saturating_sub(last_ms).min(u64::from(u32::MAX)) as u32;
        last_ms = now_ms;
        // Nonblocking regular conversions coexist with the injected current loop.
        if let Some(s) = analog::read_bus() {
            telemetry::publish_bus(s);
        }
        if control::outputs_live() {
            if let Some(kind) = control::sensor_fault(control::mode()) {
                control::fault(kind);
            } else if control::cmd_timed_out() {
                control::fault(control::FaultKind::CmdTimeout);
            }
        }
        control::tick(dt_ms);
        Timer::after_millis(1).await;
    }
}

#[embassy_executor::task]
pub async fn foc_debug_task() {
    loop {
        Timer::after_millis(100).await;
        let isr = telemetry::isr_snapshot();
        defmt::info!(
            "foc mode={} ia={} ib={} ic={} id={} iq={} id_ref={} iq_ref={} ud={} uq={} da={} db={} dc={} pos={} rpm={} vbus={} isr={}/{} over={} calls={} fault={}",
            control::mode().as_str(),
            telemetry::iu_ma(),
            telemetry::iv_ma(),
            telemetry::iw_ma(),
            telemetry::id_meas_ma(),
            telemetry::iq_meas_ma(),
            control::id_target_ma(),
            control::iq_target_ma(),
            telemetry::ud_mv(),
            telemetry::uq_mv(),
            telemetry::da_ppt(),
            telemetry::db_ppt(),
            telemetry::dc_ppt(),
            telemetry::enc_mdeg(),
            telemetry::rpm_meas(),
            telemetry::vbus_mv(),
            telemetry::cycles_to_us(isr.last_cycles),
            telemetry::cycles_to_us(isr.max_cycles),
            isr.overruns,
            isr.calls,
            control::last_fault().as_str(),
        );
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
