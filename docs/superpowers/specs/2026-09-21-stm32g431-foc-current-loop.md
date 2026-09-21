# STM32G431 FOC Current-Loop Design

**Date**: 2026-09-21  
**Target**: STM32G431CB, board electrically compatible with ST B-G431B-ESC1  
**Position**: AS5600 (I2C)  
**First closed loop**: Id / Iq current loop

This document is the FOC design after the existing infrastructure (clock, PC6 LED, USART2 shell). It does not replace `2026-04-03-stm32g431-foc-design.md`.

## Overview

Implement Field-Oriented Control on the existing Embassy Rust crate. PWM, current sampling, and the Id/Iq PI run in a **hard real-time interrupt path**. Embassy stays for shell, telemetry, and AS5600 I2C.

First milestone is a stable current loop: given `id_ref` / `iq_ref` and electrical angle from AS5600, regulate measured Id/Iq and drive TIM1 SVPWM.

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
AS5600 (I2C, ~1 kHz) ──interp θe──┐
                                  │
Ia,Ib (,Ic) ─Clarke─Park(θe)─► Id,Iq
                                  │
id_ref, iq_ref ── PI ── Vd,Vq ─invPark─ SVPWM ─ TIM1
                                  ▲
                     VBUS for modulation limit
```

- **PWM**: TIM1 center-aligned, 20 kHz, complementary, dead-time ~800 ns (tune to FET/driver).
- **Current loop**: 20 kHz, same as PWM period, **ADC EOC / DMA TC ISR** (not an Embassy task).
- **Angle**: I2C cannot run at 20 kHz. Read AS5600 in a 1 kHz task; unwrap 12-bit angle; interpolate `θe` in the ISR with last electrical speed.
- **Optional later**: AS5600 analog OUT onto an ADC channel for per-PWM angle (PB12 pot pin is the natural spare).

## Timing

```
TIM1 (center-aligned)
        ┌──── period 50 µs ────┐
cnt  __/‾‾‾‾‾‾‾\______/‾‾‾‾
           ▲
           └── TRGO2 at peak: low-side FETs ON → 3-shunt valid
               ADC1+ADC2 simultaneous sample
               DMA complete → current-loop ISR
```

- Dual ADC: U on ADC1, V on ADC2 in one trigger; W on the next slot or same sequence via OPAMP3 internal.
- First implementation: sample **two phases** every PWM, reconstruct the third (`ia+ib+ic=0`), keep the third OPAMP for `ia+ib+ic` residual check.
- Offset calibration: PWM 50% / outputs disabled or zero voltage, average N samples at startup.

## Software Layers

```
src/
├── bin/main.rs          # Embassy: shell, I2C poll, telemetry
├── bsp/                 # clocks, pin/scale constants
├── driver/
│   ├── pwm.rs           # TIM1 complementary + brake
│   ├── analog.rs        # OPAMP + dual ADC + DMA
│   └── as5600.rs        # I2C angle, status, unwrap
├── foc/
│   ├── types.rs         # SI-ish f32 state
│   ├── transforms.rs    # Clarke / Park / inv Park
│   ├── svm.rs           # inverse Clarke + SVPWM
│   ├── pid.rs           # PI + anti-windup
│   └── current.rs       # one-step current loop
└── app/
    └── foc_isr.rs       # ISR glue, fault flags
```

**ISR budget (170 MHz, ~50 µs period):** keep the current step under ~15 µs. Use `f32`; G431 has FPU. Use CORDIC for `sin/cos` if the ISR is tight; otherwise `libm`/`micromath` is acceptable for the first bring-up.

Embassy tasks must not take TIM1, ADC1/2, OPAMP1/2/3, or the ADC DMA channels used by the loop.

## Algorithm (one PWM period)

1. Read DMA current counts → volts → amps (apply invert + offset).
2. Reconstruct missing phase if needed.
3. Clarke → `Iα, Iβ`.
4. `θe = pole_pairs * θm_interp` (wrap 0..2π).
5. Park → `Id, Iq`.
6. PI: `Vd, Vq` with anti-windup; circle-limit `√(Vd²+Vq²) ≤ Vbus/√3` (SVPWM).
7. inv Park → `Vα, Vβ` → SVM → three compare values.
8. Write TIM1 CCR1/2/3. Update debug snapshot (atomics / lock-free slot).

**Align / boot:**

1. Brake / PWM off, ADC offset cal.
2. Optional forced align: `id_ref > 0`, `iq_ref = 0` for N ms, store AS5600 as `θ_offset` (or use mechanical zero).
3. Enable current loop with `id_ref = 0`, small `iq_ref`.

## Shell (extend existing CLI)

| Command | Action |
|---------|--------|
| `foc status` | state, Id/Iq meas/ref, θe, Vbus, faults |
| `foc start` / `foc stop` | enable / coast or brake |
| `foc iq <A>` / `foc id <A>` | set references (clamped) |
| `foc align` | forced-d align, save offset |
| `foc poles <n>` | pole pairs |
| `cal current` | re-run shunt offset |

## Safety (minimum for first spin)

- Phase overcurrent (raw ADC or amps)
- VBUS undervoltage / overvoltage
- AS5600 MAG invalid / I2C timeout → trip after grace, do not free-run Park
- Command timeout (no new `iq` / heartbeat)
- Fault → TIM1 MOE off (outputs inactive), LED pattern, shell reports latch

## Out of Scope (later)

- Speed / position outer loops
- Sensorless observer
- Field weakening
- CAN / ST MCSDK interoperability
- High-rate analog angle path

## Success Criteria

1. 20 kHz center-aligned complementary PWM visible on UH/VH/WH with dead-time.
2. With motor disconnected, offset cal is stable; reconstructed `ia+ib+ic` residual is small at 50% duty.
3. Open-loop voltage (fixed `Vq`, `θe` ramp) turns the rotor; current waveform is sinusoidal-ish.
4. After align, step `iq_ref` and Id/Iq track without trip; `id` stays near 0.
5. Fault injection (overcurrent clamp / unplug AS5600) disables PWM.
6. Existing LED / UART shell still work.

## Dependencies (add as needed)

- `micromath` or `libm` for `sin/cos/sqrt` if CORDIC HAL is not used yet
- Embassy I2C (`embassy-stm32` I2C) for AS5600
- No extra RTOS; ISR + Embassy only
