# stm32g431-foc

Embassy Rust FOC for **STM32G431CB** boards electrically compatible with ST B-G431B-ESC1. Pin map and electrical constants follow Cube project `122`.

- Current loop at **20 kHz** (ADC JEOS ISR): Clarke/Park → PI → 122-style Vqd feed-forward → **Vd-priority** voltage limit → SVPWM.
- Speed PI (~10 Hz with a 100 ms encoder window); AS5600 on I2C1 PB8/PB7 (J8 Z+/B+).
- **Motor nameplate** is one file: [`src/bsp/motor.rs`](src/bsp/motor.rs). Swap or edit it for another machine. Runtime: `foc poles`, `foc kp|ki|skp|ski`, `foc motor`, `foc save`.
- Math lives in the `foc/` crate (`cargo htest`). Firmware re-exports it as `crate::foc`.

**Design:** [current-loop spec](docs/specs/2026-09-21-stm32g431-foc-current-loop.md) · [plan](docs/plans/2026-09-21-stm32g431-foc-current-loop.md)  
**Bring-up (clock/LED/UART):** [2026-04-03 spec](docs/specs/2026-04-03-stm32g431-foc-design.md)

## Build

CI pins Rust 1.98.1 and flip-link 0.1.12. Local builds need the
`thumbv7em-none-eabihf` target, rustfmt, Clippy, and `flip-link` on PATH.

```bash
scripts/check.py --offline        # host tests, fmt/check/clippy; 120 s deadline per command
scripts/check.py --offline --build  # link, validate ELF, write manifest (requires flip-link)
probe-rs run --chip STM32G431CB   # default cargo runner; board acceptance only
```

The build validates FLASH/RAM placement, the reset vector, flip-link stack placement,
and the reserved NVM page. `target/thumbv7em-none-eabihf/release/manifest.json` records
the ELF SHA-256, source/build identity, toolchain and memory use; CI uploads both files.
The stack budget is a link-time bound, not a measured peak stack usage.

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

Run/Speed require `cal=ok`: a qualified alignment, a valid version-3 NVM record, or an
explicitly entered verified offset (`foc offset <mrad>`). Version-1 and version-2
records are ignored: earlier firmware could save unverified offsets, and these records
cannot identify their origin. After upgrading, complete a qualified `foc align` and
`foc save`, or enter an independently verified offset with `foc offset <mrad>` (which
automatically saves it). Changing pole pairs invalidates calibration.

`foc zero` requires Idle and a fresh encoder sample. It captures the present position
as an **unverified** electrical offset, invalidates calibration, and does not save it.
Run/Speed and `foc save` remain blocked until a qualified `foc align` or an explicit
`foc offset <mrad>` using an independently verified value. The latter is an operator
override; the command does not verify rotor alignment.

`foc align` without a current uses the default. Invalid text, integer overflow, or
extra arguments report `usage` without starting alignment. Valid integer currents
retain the controller's existing clamping behavior.

Alignment completes only after continuous fresh current-loop feedback with measured Id near target, Iq near zero, a valid encoder, and rotor motion within the configured limit. Otherwise it trips `fault=align` after 2 s and keeps the previous offset. Live outputs trip `fault=stall` if the supervisor sees no current-control completion for 5 ms. Panic and HardFault disable TIM1 outputs before halting. An independent LSI-clocked watchdog starts after bring-up, with a nominal 1 s reset deadline. Only the supervisor feeds it, at least 50 ms apart and after both ADC and encoder-task progress. Actual timeout and reset output levels require board acceptance.

VBUS/NTC are sampled while running. Missing bus data trips `fault=adc`; encoder errors trip immediately, and stale samples time out after 20 ms. Pole pairs and offset can only be edited in Idle. Encoder position is single-turn; accumulated turns no longer reduce Park-angle precision.

Encoder freshness and windowed RPM use the start of each ANGLE I2C transaction,
including its transport delay. `enc` reports `age_us`. Invalid feedback and long
sampling gaps discard RPM readiness until a new complete window is available.
The Park angle remains a held sample; AS5600 internal delay and high-speed phase
lag require measurement.

Id/Iq/RPM ramps retain sub-unit steps instead of rounding every tick. `foc isr` reports the JEOS handler-body last/max cycles, call count and budget overruns; `foc isr reset` clears the measurement window. It includes ADC reads but excludes exception entry/exit and scheduling latency. Park and inverse Park share one sin/cos pair per control step.

PWM phase compares, CH4 trigger and ADC pair now come from one integer sampling plan. When two high duties leave no settled window, the planner subtracts an equal common-mode shift from all phases, preserving quantized line voltages. An impossible plan or a live commit that misses the down-count window disables outputs with `fault=adc`. Startup discards two potentially old injected results. `foc pwm` may therefore apply a lower common duty than requested near 100%; check the CCR readback. Sampling geometry is host-tested; switching noise and real ADC latency still need scope verification.

Current/speed controllers own their state while computing with interrupts enabled.
Short critical sections transfer ownership and commit results. Stop, fault, restart,
reset or gain changes invalidate in-flight results; output commits recheck sensor
freshness. Both injected ADCs must complete without queue overflow before current
decoding. Nonfinite current samples trip `fault=adc`; gains must be finite and
nonnegative. Shell gain values accept scientific notation, such as `foc kp 2e-3`.

Calibration saves append version-3 records to the existing 128 16-byte slots in the
2 KiB NVM page. Old-version slots remain consumed; new records append after them.
Identical saves do no flash work; torn appends retain an earlier valid record, and
writes are read back. A full page requires erase: power loss during reclaim can
lose calibration, requiring alignment again. Real flash/ECC recovery needs board tests.

See [safety changes and board acceptance checklist](docs/safety-bringup.md) before loaded operation. Host tests cover algorithms and the firmware control state machine with mocked hardware; ADC/PWM timing still requires on-board verification.

The [software verification and remaining work](docs/next-steps.md) records independent
RL/PMSM model coverage and its limits, including speed oscillation in a synthetic
light-inertia model. Default PI gains, PWM frequency and dead time are unchanged.

## Modes

Idle · Bench (`foc pwm`) · Align · Run · Openloop · Speed (`foc rpm`) · Fault
