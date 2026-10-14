# stm32g431-foc

Embassy Rust FOC for **STM32G431CB** boards electrically compatible with ST B-G431B-ESC1. Pin map and electrical constants follow Cube project `122`.

- Current loop at **20 kHz** (ADC JEOS ISR): Clarke/Park → PI → 122-style Vqd feed-forward → **Vd-priority** voltage limit → SVPWM.
- Speed PI (~10 Hz with a 100 ms encoder window); AS5600 on I2C1 PB8/PB7 (J8 Z+/B+).
- **Motor nameplate** is one file: [`src/bsp/motor.rs`](src/bsp/motor.rs). Swap or edit it for another machine. Runtime: `foc poles`, `foc kp|ki|skp|ski`, `foc motor`, `foc save`.
- Math lives in the `foc/` crate (`cargo htest`). Firmware re-exports it as `crate::foc`.

**Design:** [current-loop spec](docs/specs/2026-09-21-stm32g431-foc-current-loop.md) · [plan](docs/plans/2026-09-21-stm32g431-foc-current-loop.md)  
**Bring-up (clock/LED/UART):** [2026-04-03 spec](docs/specs/2026-04-03-stm32g431-foc-design.md)

## Build

```bash
scripts/check.py --offline        # fmt, host tests, check, clippy; 120 s deadline per command
scripts/check.py --offline --build  # also links release firmware (requires flip-link)
probe-rs run --chip STM32G431CB   # default cargo runner; board acceptance only
```

USART2 is **921600** 8N1 (PB3 TX / PB4 RX). Tune and watch the loop on this UART (`foc status`, `foc isr`). Do not halt the core with a probe-rs/SWD breakpoint while PWM is live.

`DEFMT_LOG=info` is in `.cargo/config.toml` for boot/NVM/UART errors and the 100 ms RTT FOC debug line. RTT is best-effort (`drop-on-contention` + non-blocking): lines may be dropped rather than stalling control. The 20 kHz ISR does not call `defmt`. Unset `DEFMT_LOG` later if you want those macros compiled out.

## First spin (shell)

```text
foc stop
cal current
foc align          # or: foc align 500; qualified hold, RAM offset
foc status         # cal=ok required for Run/Speed; fault=none
foc save           # optional: persist the qualified offset
foc start
foc iq 200         # mA, slewed at 10 A/s
```

`foc stop` coasts (MOE off) and clears all current/speed/open-loop targets. `foc status` keeps the last `fault=` after stop. Starts require fresh, in-range VBUS/NTC; Align/Run/Speed also require a valid AS5600 magnet. Change live modes only after `foc stop`. Repeated `foc rpm` commands update the target without restarting the controllers.

Run/Speed require `cal=ok`: a qualified alignment, a valid version-2 NVM record, or an explicitly entered verified offset (`foc offset <mrad>`). Firmware with NVM version 2 intentionally ignores older records; re-align and save once after upgrading. Changing pole pairs invalidates calibration.

Alignment completes only after continuous fresh current-loop feedback with measured Id near target, Iq near zero, a valid encoder, and rotor motion within the configured limit. Otherwise it trips `fault=align` after 2 s and keeps the previous offset. Live outputs trip `fault=stall` if the supervisor sees no current-control completion for 5 ms. Panic and HardFault disable TIM1 outputs before halting. These are software protections; a hardware watchdog is still required.

VBUS/NTC are sampled while running. Missing bus data trips `fault=adc`; encoder errors trip immediately, and stale samples time out after 20 ms. Pole pairs and offset can only be edited in Idle. Encoder position is single-turn; accumulated turns no longer reduce Park-angle precision.

Id/Iq/RPM ramps retain sub-unit steps instead of rounding every tick. `foc isr` reports the JEOS handler-body last/max cycles, call count and budget overruns; `foc isr reset` clears the measurement window. It includes ADC reads but excludes exception entry/exit and scheduling latency. Park and inverse Park share one sin/cos pair per control step.

See [safety changes and board acceptance checklist](docs/safety-bringup.md) before loaded operation. Host tests cover algorithms and the firmware control state machine with mocked hardware; ADC/PWM timing still requires on-board verification.

## Modes

Idle · Bench (`foc pwm`) · Align · Run · Openloop · Speed (`foc rpm`) · Fault
