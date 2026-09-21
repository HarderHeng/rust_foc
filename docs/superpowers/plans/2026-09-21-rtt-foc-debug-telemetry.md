# RTT FOC Debug Telemetry Implementation Plan

> **For agentic workers:** implement in small steps. Keep RTT logging out of the ISR. Prefer dropped logs over blocking motor control.

**Goal:** Add 100 ms `defmt-rtt` FOC debug telemetry with drop/non-blocking RTT behavior.

**Spec:** `docs/superpowers/specs/2026-09-21-rtt-foc-debug-telemetry.md`

---

## Files expected to change

```text
Cargo.toml
Cargo.lock
src/app/telemetry.rs
src/app/foc_isr.rs
src/app/mod.rs
src/bin/main/tasks.rs
src/bin/main/main.rs
README.md                         # short debug note
```

Do not move control-loop math into Embassy tasks.

---

## Task 1: Make RTT best-effort

**Files:** `Cargo.toml`, `Cargo.lock`

- [x] Change `defmt-rtt` dependency to:

```toml
defmt-rtt = { version = "1.3.0", features = ["drop-on-contention", "disable-blocking-mode"] }
```

- [x] Run `cargo update -p defmt-rtt` if Cargo does not update the lockfile automatically.
- [x] Confirm the project still targets ARM for firmware checks. `drop-on-contention` is ARM-only and should not be pulled into host `foc` tests.

**Stop if:** host tests start depending on the root firmware crate or `defmt-rtt` is compiled for x86 host.

---

## Task 2: Publish voltage debug values from ISR

**Files:** `src/app/telemetry.rs`, `src/app/mod.rs`, `src/app/foc_isr.rs`

- [x] Add atomics for final applied D/Q voltage:
  - `UD_MV: AtomicI16`
  - `UQ_MV: AtomicI16`
- [x] Add atomics for pre-limit/reference D/Q voltage:
  - `UD_REF_MV: AtomicI16`
  - `UQ_REF_MV: AtomicI16`
- [x] Add telemetry API:
  - `publish_vdq(applied: Dq, reference: Dq)` or equivalent
  - `ud_mv()`, `uq_mv()`, `ud_ref_mv()`, `uq_ref_mv()`
- [x] Re-export the getters from `src/app/mod.rs`.
- [x] In `foc_isr`, publish voltage values after PI/feed-forward/limiting are known.

Implementation note:

- Current `CurrentLoop::step()` returns only `(meas, duties)`. To expose voltages cleanly, either:
  1. extend it to return a small debug struct, or
  2. add a `step_debug` variant used by firmware while preserving existing host tests.
- Keep the FOC math crate host-testable.
- If changing `CurrentLoop::step()` signature, update all host tests and open-loop callers deliberately.

Open-loop note:

- For `foc openloop`, publish `ud/uq` as the commanded open-loop voltage (`vd=0`, `vq=ol_vq_v`) and use the same values for refs unless a separate pre-limit value exists.

**Stop if:** publishing voltage debug introduces `defmt`, locks, allocation, or large structs in the ISR.

---

## Task 3: Add the 100 ms debug task

**Files:** `src/bin/main/tasks.rs`, `src/bin/main/main.rs`

- [x] Add an Embassy task:

```rust
#[embassy_executor::task]
pub async fn foc_debug_task() {
    loop {
        Timer::after_millis(100).await;
        defmt::info!(
            "foc mode={} ia={} ib={} ic={} id={} iq={} id_ref={} iq_ref={} ud={} uq={} ud_ref={} uq_ref={} pos={} rpm={} vbus={} isr={}/{} fault={}",
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
            telemetry::ud_ref_mv(),
            telemetry::uq_ref_mv(),
            telemetry::enc_mdeg(),
            telemetry::rpm_meas(),
            telemetry::vbus_mv(),
            telemetry::isr_us(),
            telemetry::isr_us_max(),
            control::last_fault().as_str(),
        );
    }
}
```

- [x] Spawn it from `main.rs` with the other tasks.
- [x] Consider removing or reducing `heartbeat_task` `defmt::info!` if the 10 Hz FOC line is noisy enough.

**Stop if:** the debug task accesses PWM/ADC/FLASH peripherals directly or blocks on shell/UART.

---

## Task 4: Keep the shell and docs consistent

**Files:** `README.md`, optionally current-loop spec

- [x] Document that RTT debug is best-effort and can drop frames.
- [x] Document that USART shell remains the safe interactive debug/control path.
- [x] Mention that `DEFMT_LOG=info` enables the 100 ms RTT line.

---

## Task 5: Verify

- [x] `cargo check --lib --bins`
- [x] `cargo htest`
- [x] `git diff --check`
- [x] Flash/run with RTT attached; confirm a 10 Hz `foc ...` line.
- [ ] Disconnect/stop RTT drain while firmware runs; motor control and heartbeat/shell should continue.
- [ ] Use USART `foc isr` before/after enabling logs; verify max ISR time does not regress materially.

---

## Rollback plan

If RTT still affects control timing:

- [ ] Keep `defmt-rtt` best-effort features enabled.
- [ ] Gate `foc_debug_task` behind a Cargo feature, e.g. `rtt-foc-debug`.
- [ ] Reduce period to 500 ms or split detailed fields into on-demand shell output.
- [ ] Remove voltage debug first; keep only currents/position/ISR budget if necessary.
