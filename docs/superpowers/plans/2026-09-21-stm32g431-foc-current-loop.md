# STM32G431 FOC Current-Loop Implementation Plan

> **For agentic workers:** implement task-by-task. Check boxes as you go. Do not skip the open-loop PWM/current bring-up before closing Id/Iq.

**Goal:** Close an Id/Iq current loop on a B-G431B-ESC1-compatible board, using TIM1 SVPWM, 3-shunt OPAMP/ADC, and AS5600 electrical angle.

**Architecture:** Hard ISR for sample → Clarke/Park → PI → SVPWM. Embassy for shell, AS5600 I2C, telemetry. Constants live in BSP.

**Tech stack:** Existing Embassy crate (`stm32g431-foc`), `thumbv7em-none-eabihf`, probe-rs.

**Spec:** `docs/superpowers/specs/2026-09-21-stm32g431-foc-current-loop.md`

---

## File Structure (additions)

```
src/
├── bsp/config.rs              # extend: PWM pins, ADC/OPAMP, I2C, scales
├── driver/
│   ├── pwm.rs                 # TIM1 complementary PWM
│   ├── analog.rs              # OPAMP + dual ADC + DMA
│   └── as5600.rs              # I2C angle
├── foc/
│   ├── mod.rs
│   ├── types.rs
│   ├── transforms.rs
│   ├── svm.rs
│   ├── pid.rs
│   └── current.rs
└── app/
    ├── mod.rs
    └── foc_isr.rs
```

Do not move the current-loop math into Embassy tasks.

---

### Task 1: Board constants and pin map

**Files:** `src/bsp/config.rs`, `src/bsp/mod.rs`

- [x] **Step 1:** Add named constants (do not magic-number them in drivers):

```rust
pub const PWM_FREQ_HZ: u32 = 20_000;
pub const PWM_DEADTIME_NS: u32 = 800;
pub const SHUNT_OHM: f32 = 0.003;
pub const CURRENT_AMP_GAIN: f32 = 16.0;
pub const CURRENT_SIGN: f32 = -1.0; // inverting OPAMP path
pub const VBUS_DIV: f32 = 18_000.0 / (18_000.0 + 169_000.0);
pub const AS5600_I2C_ADDR: u8 = 0x36;
pub const DEFAULT_POLE_PAIRS: u8 = 7; // change per motor
```

- [x] **Step 2:** Document pin table in comments (PA8/PC13, PA9/PA12, PA10/PB15, OPAMP1/2/3, I2C1 PB8/PB7).

- [ ] **Step 3:** If the clone board uses different shunts, put the measured values here only.

---

### Task 2: TIM1 complementary PWM

**Files:** Create `src/driver/pwm.rs`

- [x] **Step 1:** Configure TIM1 center-aligned mode 1, 20 kHz from 170 MHz, 3× complementary channels, dead-time, MOE, idle-off.

- [x] **Step 2:** Expose `set_duty(u, v, w)` in 0..1 or CCR counts; `enable()` / `disable()` via MOE or outputs-idle.

- [x] **Step 3:** Enable TRGO2 at **counter peak** (low-side on) for ADC.

- [ ] **Step 4:** Bench check with no motor: UH/VH/WH 20 kHz, complementary, dead-time present, 50% duty default.

**Stop if:** any high/low of a phase overlap, or TRGO2 is not at the peak.

---

### Task 3: OPAMP + dual ADC + DMA

**Files:** Create `src/driver/analog.rs`

- [ ] **Step 1:** OPAMP1/2 PGA (or follower if the board already sets external gain) matching ESC-G431; OPAMP3 internal output.

- [ ] **Step 2:** ADC1+ADC2 simultaneous, TIM1 TRGO2 trigger, DMA into a double buffer.

- [ ] **Step 3:** Convert counts → amps / volts using `config.rs`. Sign must match the inverting network.

- [ ] **Step 4:** `calibrate_offsets()` with PWM safe (MOE off or 50% and motor disconnected). Store Ia/Ib/(Ic) offsets.

- [ ] **Step 5:** Optional: reconstruct `ic = -ia-ib`; log `ia+ib+ic` residual.

**Stop if:** samples are not synchronized to PWM mid, or offset jumps more than a few tens of counts between cals.

---

### Task 4: Open-loop voltage (no current PI yet)

**Files:** `src/foc/svm.rs`, `src/foc/transforms.rs`, `src/app/foc_isr.rs`

- [ ] **Step 1:** Implement inverse Clarke + SVPWM (`Vα,Vβ,Vbus` → three duties).

- [ ] **Step 2:** ISR or a test mode: ramp `θe`, fixed `Vd=0`, small `Vq`, write TIM1.

- [ ] **Step 3:** Motor on a stand: rotor turns smoothly; current traces look like phase-shifted sinusoids.

- [ ] **Step 4:** Shell: `foc openloop <vq> <hz>` / `foc stop`.

**Stop if:** cogging/jumps, or one phase current is dead (PWM or OPAMP pin wrong).

---

### Task 5: AS5600 angle

**Files:** Create `src/driver/as5600.rs`

- [x] **Step 1:** Embassy I2C1 on PB8/PB7, 400 kHz, addr `0x36`.

- [x] **Step 2:** Read raw angle (regs 0x0E/0x0F), status (magnet). Unwrap to continuous mechanical angle.

- [x] **Step 3:** 1 kHz Embassy task publishes `{theta_m, omega_m, valid}` to a lock-free slot.

- [ ] **Step 4:** ISR interpolates `theta_m + omega_m * dt`, then `theta_e = wrap(pole_pairs * (theta_m - offset))`.

- [ ] **Step 5:** Shell: `enc` prints deg, status, MAG.

**Stop if:** MAG not OK, or angle does not increase monotonically when turning the rotor by hand.

---

### Task 6: Current-loop math

**Files:** `src/foc/pid.rs`, `src/foc/current.rs`, `src/foc/transforms.rs`

- [x] **Step 1:** Clarke, Park, inv Park (f32).

- [x] **Step 2:** Two PI controllers (Id, Iq), Ts = 1/20e3, output clamp + integrator anti-windup.

- [x] **Step 3:** `current::step(meas, refs, theta_e, vbus) -> duties`.

- [x] **Step 4:** Voltage circle limit using VBUS.

- [ ] **Step 5:** Host-side unit tests are optional (std test crate later); at minimum, check Clarke/Park invertibility with a few vectors in comments or a small `#[cfg(test)]` if you add a lib test feature.

---

### Task 7: Align + close the loop

**Files:** `src/app/foc_isr.rs`, `src/bin/main.rs`, `src/driver/shell.rs`

- [ ] **Step 1:** State machine: `Idle → Cal → Align → Run → Fault`.

- [ ] **Step 2:** Align: `id_ref` small, `iq_ref=0`, 0.5–1 s, capture AS5600 as `theta_offset`.

- [ ] **Step 3:** Run: `id_ref=0`, operator sets `iq_ref`. ISR uses interpolated `θe`.

- [ ] **Step 4:** Shell: `foc start|stop|align|id|iq|poles|status`.

- [ ] **Step 5:** Tune Kp/Ki on a stand (start conservative). Id should stay near 0; Iq should step-track.

**Stop if:** Park frame is wrong (Iq/Id swap or sign) — flip current sign or `θ_offset` by π, do not “tune around” a 180° error.

---

### Task 8: Faults and keep-alive of existing features

**Files:** `src/app/foc_isr.rs`, `src/driver/led.rs`, shell

- [ ] **Step 1:** Latch: overcurrent, VBUS UV/OV, AS5600 invalid/timeout, command timeout.

- [ ] **Step 2:** On fault: TIM1 outputs off, LED distinct from heartbeat, `foc status` shows latch + `foc stop` to clear.

- [ ] **Step 3:** Confirm USART2 shell and PC6 heartbeat still run while FOC is idle.

---

## Suggested bring-up order (lab)

1. PWM only, no motor.
2. ADC offsets, no motor.
3. Open-loop voltage, motor unloaded.
4. AS5600 by hand.
5. Align + small Iq, unloaded.
6. Raise Iq slowly; then add load.

## Success Criteria

Match the spec: PWM+dead-time, residual current check, open-loop spin, Id/Iq track, fault disables PWM, shell/LED intact.

## Notes for implementers

- Embassy 0.6 PWM/ADC APIs on G4 may need PAC fallback for TRGO2 + dual simultaneous + OPAMP3 internal. Prefer Embassy; use `embassy_stm32::pac` only in `driver/analog.rs` / `pwm.rs` if the HAL cannot express the trigger.
- Do not allocate in the ISR.
- Keep `DEFAULT_POLE_PAIRS` and current scale wrong-by-default comments visible — they are the first things that break a new motor.
