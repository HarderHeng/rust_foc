# RTT FOC Debug Telemetry Spec

**Goal:** Add a low-priority Embassy debug task that prints a compact FOC snapshot over `defmt-rtt` every 100 ms, without adding logging to the TIM1/ADC current ISR and without allowing RTT backpressure to stall motor control.

**Non-goals:**

- No `defmt::*` calls in the injected ADC / FOC ISR.
- No locks, heap allocation, formatting strings, or blocking peripheral I/O in the current-loop path.
- No replacement for the USART shell telemetry; RTT is debug-only and may drop frames.

## Runtime model

- Current-loop ISR remains hard real-time and only publishes scalar telemetry through atomics.
- Embassy thread-mode task wakes every 100 ms and reads atomics from `app::telemetry` / `app::control`.
- RTT frames are allowed to be dropped under contention or full-buffer conditions.
- Project uses a single thread-mode Embassy executor. Do not add an interrupt-mode executor that logs through RTT unless the `defmt-rtt` contention mode is re-evaluated.

## RTT behavior

Use `defmt-rtt` with:

```toml
defmt-rtt = { version = "1.3.0", features = ["drop-on-contention", "disable-blocking-mode"] }
```

Rationale:

- `drop-on-contention`: if an interrupt preempts a thread-mode RTT writer, the contending frame is dropped instead of waiting with interrupts masked.
- `disable-blocking-mode`: if the host is slow/disconnected or the RTT ring is full, frames are dropped/truncated instead of busy-waiting forever.

Trade-off: logs are best-effort. Missing lines are acceptable; FOC timing is not allowed to depend on the debugger.

## Telemetry fields

Print a single compact line at 10 Hz. Units are integer-friendly to keep encoded frames small.

Required fields:

| Field | Source | Unit |
|-------|--------|------|
| `mode` | `control::mode().as_str()` | enum string |
| `ia ib ic` | `app::iu_ma/iv_ma/iw_ma()` | mA |
| `id iq` | `app::id_meas_ma/iq_meas_ma()` | mA |
| `id_ref iq_ref` | `control::id_target_ma/iq_target_ma()` or slewed refs if needed | mA |
| `pos` | `app::enc_mdeg()` | mdeg mechanical |
| `rpm` | `app::rpm_meas()` | rpm mechanical |
| `vbus` | `app::vbus_mv()` | mV |
| `isr` | `app::isr_us()/isr_us_max()` | us |
| `fault` | `control::last_fault().as_str()` | enum string |

Voltage-loop debug fields to add before logging them:

| Field | Meaning | Unit |
|-------|---------|------|
| `ud uq` | final limited D/Q voltage applied by ISR | mV |
| `ud_ref uq_ref` | PI + feed-forward voltage before final voltage limiting, or applied voltage target for open-loop | mV |

The voltage fields must be published from the ISR as atomics after the values are computed. The debug task must not recompute PI or feed-forward state.

## Log format

Example:

```text
foc mode=run ia=12 ib=-30 ic=18 id=4 iq=198 id_ref=0 iq_ref=200 ud=-20 uq=530 pos=123456 rpm=42 vbus=23800 isr=7/10 fault=none
```

Keep the line short enough to fit one RTT frame comfortably. If additional fields are needed, add a second slower debug line or gate them behind a compile-time/runtime flag.

## Control

Initial implementation may log unconditionally every 100 ms when `DEFMT_LOG` enables `info`.

Optional later controls:

- `foc dbg on|off` shell toggle.
- `foc dbg <ms>` period control.
- Log only when outputs are live.

## Safety constraints

- Do not call `defmt` from `foc_isr::on_injected` or helper functions called exclusively by it.
- Do not read mutable current-loop internals from the debug task.
- Keep all ISR-published values copy-sized atomics (`AtomicI16/I32/U32`).
- Do not call `defmt::flush()` in periodic tasks.
- After enabling RTT debug, measure `foc isr` last/max before and after a run; no systematic current-loop budget regression is acceptable.

## Validation

- `cargo check --lib --bins`
- `cargo htest`
- Run firmware with probe-rs attached and detached; firmware must continue running if RTT is not drained.
- Verify `foc isr` max remains within the current-loop timing budget with RTT telemetry enabled.
