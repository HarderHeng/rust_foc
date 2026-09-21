# STM32G431 FOC Current-Loop Design

**Date**: 2026-09-21 (updated 2026-09-22)  
**Target**: STM32G431CB, board electrically compatible with ST B-G431B-ESC1  
**Position**: AS5600 (I2C)  
**Closed loops**: Id / Iq current (20 kHz ISR); optional speed PI (~1 kHz)

This document is the FOC design after the existing infrastructure (clock, PC6 LED, USART2 shell). It does not replace `2026-04-03-stm32g431-foc-design.md`. Hardware constants and R3_2 sampling follow Cube project `122`.

## Overview

Field-Oriented Control on the Embassy firmware crate `stm32g431-foc`. PWM, current sampling, and the Id/Iq PI run in the **ADC JEOS ISR**. Embassy stays for shell, telemetry, AS5600 I2C, align timing, and reference ramps.

Clarke / Park / PI / SVPWM / slew live in the host-testable `foc/` crate (`cargo htest`). The firmware re-exports it as `crate::foc`.

## Hardware Baseline (B-G431B-ESC1)

Pin map follows ST UM2516 Table 4. Existing BSP already matches LED and UART.

| Function | Pin | Peripheral |
|----------|-----|------------|
| UH / UL | PA8 / PC13 | TIM1_CH1 / CH1N |
| VH / VL | PA9 / PA12 | TIM1_CH2 / CH2N |
| WH / WL | PA10 / PB15 | TIM1_CH3 / CH3N |
| Curr U+ / U- | PA1 / PA3 | OPAMP1 VINP / VINM |
| OPAMP1 out | PA2 | ADC1_IN3 |
| Curr V+ / V- | PA7 / PA5 | OPAMP2 VINP / VINM |
| OPAMP2 out | PA6 | ADC2_IN3 |
| Curr W+ / W- | PB0 / PB2 | OPAMP3 VINP / VINM |
| OPAMP3 out | internal | ADC2 (internal channel) |
| VBUS | PA0 | ADC1_IN1 |
| NTC | PB14 | ADC |
| Status LED | PC6 | GPIO (existing) |
| USART2 TX/RX | PB3 / PB4 | 921600 (existing) |
| AS5600 SCL/SDA | PB8 / PB7 | I2C1 (official Z+/B+; PB6 is not I2C1) |
| AS5600 DIR | tie GND or 3V3 | direction |
| User button | PC10 | optional enable |

**Assumption:** AS5600 uses I2C1 on the hall/encoder header: `PB8=SCL`, `PB7=SDA`, 3V3, GND. G431 does not map I2C1 onto PB6. If you wired SCL to PB6, either move the wire to PB8 or say so and we add a bit-bang bus.

Official analog constants (verify on the clone if shunts differ):

| Parameter | Typical ESC-G431 | Notes |
|-----------|------------------|--------|
| Shunt | 3 mΩ | low-side 3-shunt |
| OPAMP / PGA | gain ≈ 16, inverting | current sign is **negative** |
| Iphase scale | ≈ −36.47 A/V | `1 / (Rshunt × gain)` |
| VBUS divider | 18k / (18k+169k) ≈ 0.0963 | max ~25 V class |
| Gate driver | L6387-class | complementary + dead-time required |

## Control Architecture

```
                    1 kHz: slew id*/iq*/rpm*     (foc::slew, not inside Pid)
AS5600 (I2C, ~1 kHz) ──interp θe──┐
                                  │
Ia,Ib (,Ic) ─Clarke─Park(θe)─► Id,Iq
                                  │
              id, iq ── PI ── Vπ ─ + Vff ─ Vd-priority ─ invPark ─ SVPWM ─ TIM1
                                  ▲
                     VBUS; PI.track(V − Vff) after circle
```

- **PWM**: TIM1 center-aligned, 20 kHz, complementary, firmware dead-time 750 ns (`SW_DEADTIME_NS`). Pins idle-low, OSSI/OSSR on. CH4 is PWM2; `CCR4` follows 122 `Tafter` / `Tbefore`.
- **Current sense**: R3_2 two-phase pair each PWM. OPAMP3 `OPAINTOEN` on for UW (ADC2 CH18), off for UV/VW (PB1 / ADC1 IN12).
- **Current loop**: 20 kHz, ADC1 JEOS (not an Embassy task).
- **Pid**: portable parallel PI/PID; clamps its own output; `track(applied)` for outer limits. No motor model, no voltage circle inside Pid.
- **Vqd feed-forward** (122 `FF_VqdffComputation`, after PI): `vd_ff = −ωe·Lq·iq*`, `vq_ff = ωe·Ld·id* + ωe·ψf` (`Ld=Lq=LS`, `ψf` from `Ke`).
- **Angle**: AS5600 at ~1 kHz; ISR interpolates `θm + ωm·dt`. Invalid encoder: hold last CCR; trip after grace in Run/Speed.
- **Align**: `foc align [mA]` — hold `id` (default 500 mA), `iq=0`, `θe=0` for 500 ms, latch `θ_offset`, coast to Idle. Encoder invalid → `fault=enc`.
- **Ramps** (~1 kHz): Id/Iq 10 A/s in Align/Run; rpm 6420 rpm/s in Speed, starting from measured rpm.

## Timing

```
TIM1 (center-aligned)
        ┌──── period 50 µs ────┐
cnt  __/‾‾‾‾‾‾‾\______/‾‾‾‾
           ▲
           └── TIM1 CH4 / OC4REF (PWM2): low-side ON window
               ADC1+ADC2 injected; JEOS → current-loop ISR
```

- Dual ADC: pair UV / UW / VW per 122 sector rule; reconstruct the third (`ia+ib+ic=0`).
- Offset calibration: PWM off, regular ADC (`cal current`).

## Software Layers

```
foc/                         # no_std math; `cargo htest`
├── pid.rs                   # Pid + track; Pi alias
├── slew.rs                  # reference rate limit
├── current.rs               # PI → Vff → circle → SVPWM
├── speed.rs / svm / transforms / types
src/
├── bin/main/                # Embassy: shell, encoder, analog, button
├── bsp/                     # clocks, 122 electrical constants
├── driver/                  # pwm, analog (R3_2), as5600, ocp, led
└── app/
    ├── control.rs           # mode, refs, align, ramps, faults
    ├── foc_isr.rs           # JEOS current step
    └── speed.rs / shell / telemetry
```

**ISR budget (170 MHz, ~50 µs period):** keep the current step under ~15 µs. Use `f32`; G431 has FPU. Use CORDIC for `sin/cos` if the ISR is tight; otherwise `libm`/`micromath` is acceptable for the first bring-up.

Embassy tasks must not take TIM1, ADC1/2, OPAMP1/2/3, or the ADC DMA channels used by the loop.

## Algorithm (one PWM period)

1. Read injected JDR counts → amps (invert + shunt offset).
2. Reconstruct missing phase if needed.
3. Clarke → `Iα, Iβ`.
4. `θe = pole_pairs * θm_interp` (wrap 0..2π).
5. Park → `Id, Iq` (`θe = 0` in Align).
6. `Vπ = PI(e)` (Pid clamp + back-calc). `V* = Vπ + Vff`. Circle-limit `√(Vd²+Vq²) ≤ Vbus/√3`. `PI.track(V − Vff)` if scaled.
7. inv Park → `Vα, Vβ` → SVM → CCR1/2/3 and next-pair JSQR / OPAMP3 route.
8. Telemetry atomics for shell (`foc status`).

**Align / boot:**

1. `foc stop`, `cal current` (PWM off).
2. `foc align` [optional mA]: ramp `id`, `iq=0`, `θe=0` for 500 ms; latch encoder as `θ_offset`; Idle.
3. `foc start`, then `foc iq <mA>` (10 A/s slew).

## Modes

| Mode | Who writes TIM1 | Notes |
|------|-----------------|-------|
| Idle / Fault | none (MOE off) | Fault latches `FaultKind` |
| Bench | equal duty (`foc pwm`) | Current loop does not write CCR |
| Align | current ISR, `θe=0` | Then latch offset, return Idle |
| Run | current ISR | Slewed `id`/`iq` |
| Speed | current ISR | 1 kHz speed PI writes Iq (no Iq slew) |
| Openloop | ISR, fixed Vq | Ramped electrical angle |

## Bring-up (lab)

1. PWM / `cal current`, motor disconnected.
2. `foc openloop <vq_mV> <Hz>`, unloaded.
3. `foc align` → check `off=` in `foc status`.
4. `foc start` → small `foc iq`, Id ≈ 0.
5. Optional `foc rpm <n>`.

## Shell (USART2, 921600)

| Command | Action |
|---------|--------|
| `foc status` | mode, slewed refs, meas Id/Iq, offset, rpm, `fault=`, Vbus; `align_left` while aligning |
| `foc start` / `foc stop` | current loop / coast (MOE off) |
| `foc id <mA>` / `foc iq <mA>` | targets; slewed at 10 A/s in Run |
| `foc align [mA]` | forced-D hold then latch offset (default 500 mA / 500 ms) |
| `foc rpm <n>` | speed mode; rpm slewed from measured speed |
| `foc openloop <vq_mV> <Hz>` | fixed Vq, ramped θe |
| `foc poles <n>` / `foc offset` / `foc zero` | pole pairs / electrical offset |
| `foc kp\|ki\|skp\|ski` | current / speed gains |
| `cal current` | shunt offset (PWM must be off) |

## Safety (minimum for first spin)

- Phase overcurrent (amps vs `SW_OCP_A`); COMP1/2/4 + DAC3 → TIM1 BRK (`fault=brk`)
- VBUS undervoltage / overvoltage; NTC overtemp
- AS5600 invalid → hold Park; trip after grace in Run/Speed (`fault=enc`)
- Command timeout 2 s on Run/Speed (`fault=timeout`); speed-loop Iq does not pet the watchdog
- Fault → TIM1 MOE off (idle-low), ~5 Hz LED, `foc status` keeps `fault=` after `foc stop`

## Out of Scope (later)

- Position loop
- Sensorless observer (122 STO)
- Field weakening (Id < 0 at high speed); voltage limit is already 122 Vd-priority
- CAN / ST MCSDK interoperability
- High-rate analog angle path

## Success Criteria

1. 20 kHz center-aligned complementary PWM visible on UH/VH/WH with dead-time.
2. With motor disconnected, offset cal is stable; reconstructed `ia+ib+ic` residual is small at 50% duty.
3. Open-loop voltage (fixed `Vq`, `θe` ramp) turns the rotor; current waveform is sinusoidal-ish.
4. After align, ramp `iq_ref` and Id/Iq track without trip; `id` stays near 0.
5. Fault injection (overcurrent clamp / unplug AS5600) disables PWM.
6. Existing LED / UART shell still work.

## Build

- Firmware: `cargo check --lib --bins` (default `thumbv7em-none-eabihf`); flash via `probe-rs` (`STM32G431CB`).
- Host math tests: `cargo htest` (`foc` crate only; thumb has no libtest).
- `DEFMT_LOG=info` in `.cargo/config.toml`.

`micromath` is used inside `foc/`. Embassy I2C for AS5600. Custom line editor on USART2 (not `embedded-cli`). No extra RTOS.
