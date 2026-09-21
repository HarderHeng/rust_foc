# stm32g431-foc

Embassy Rust FOC for **STM32G431CB** boards electrically compatible with ST B-G431B-ESC1. Pin map and electrical constants follow Cube project `122`.

- Current loop at **20 kHz** (ADC JEOS ISR): Clarke/Park → PI → 122-style Vqd feed-forward → voltage circle → SVPWM.
- Speed PI at **~1 kHz**; AS5600 on I2C1 (PB8/PB7).
- Math lives in the `foc/` crate (`cargo htest`). Firmware re-exports it as `crate::foc`.

**Design:** [current-loop spec](docs/superpowers/specs/2026-09-21-stm32g431-foc-current-loop.md) · [plan](docs/superpowers/plans/2026-09-21-stm32g431-foc-current-loop.md)  
**Bring-up (clock/LED/UART):** [2026-04-03 spec](docs/superpowers/specs/2026-04-03-stm32g431-foc-design.md)

## Build

```bash
cargo check --lib --bins          # thumbv7em-none-eabihf
cargo htest                       # host unit tests for foc/
probe-rs run --chip STM32G431CB   # default cargo runner
```

USART2 is **921600** 8N1 (PB3 TX / PB4 RX). `DEFMT_LOG=info` is set in `.cargo/config.toml`.

## First spin (shell)

```text
foc stop
cal current
foc align          # or: foc align 500
foc status         # off=… mrad, fault=none
foc start
foc iq 200         # mA, slewed at 10 A/s
```

`foc stop` coasts (MOE off). `foc status` keeps the last `fault=` after stop. Align needs a valid AS5600 magnet.

## Modes

Idle · Bench (`foc pwm`) · Align · Run · Openloop · Speed (`foc rpm`) · Fault
