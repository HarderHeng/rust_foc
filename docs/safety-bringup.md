# Safety changes: host verification and board acceptance

## Implemented behavior

- VBUS/NTC use a nonblocking, single-rank regular ADC1 sequence, alternating channels.
  The supervisor polls at approximately 1 ms, so a complete pair normally arrives every
  2 ms; the first poll starts VBUS, the next completed poll starts NTC, and the following
  completed poll publishes the pair and immediately starts the next VBUS conversion.
  Injected current conversions retain priority. Regular sampling does not toggle ADEN,
  rewrite injected sample times, or re-arm JSQR each poll.
- ADC ISR status is cleared with W1C writes, preserving regular EOS/EOC flags.
- ADC ownership is protected by a short `Mutex<RefCell<Option<Analog>>>` borrow.
  Offset calibration removes the object from the shared slot, disables JEOS, stops both
  conversion groups, and performs blocking calibration with interrupts enabled and PWM
  off. If the bounded ADC stop wait fails, calibration returns false and latches `fault=adc`.
- Bus freshness is measured from acquisition start. Missing/expired bus samples trip
  `fault=adc`; out-of-range voltage and temperature trip `vbus` / `ntc`.
- All start paths require a fresh, in-range bus/temperature sample. Align/Run/Speed also
  require a valid encoder. Explicit I2C/magnet errors trip immediately; silent encoder
  publication expires after `ENC_FAULT_MS` (20 ms). The current ISR independently
  checks freshness using wall time, not encoder-task progress.
- Start validation, safe CCR preparation, MOE enable and mode commit form one short
  interrupt-masked transaction. Pending hardware break flags are not cleared by start.
  Mode changes while live require `foc stop`; repeated Run start is a keepalive and
  repeated Speed commands update the setpoint without resetting either PI.
- Stop and fault clear current/speed/open-loop references. The first fault cause is
  retained until a successful restart. `foc stop` acknowledges Fault but does not
  bypass sensor or hardware-break interlocks on the next start.
- Pole pairs and electrical offset can only be changed in Idle. After changing pole
  pairs, perform alignment again before running; automatic alignment-validity tracking
  is not yet implemented.
- Encoder control angle is single-turn. Multi-turn position remains integer counts;
  speed is derived from signed count differences. `enc mdeg` now reports single-turn
  position rather than an ever-growing floating-point angle.
- Speed PI includes direction limits and tracking of the slew-limited Iq reference.
  No startup torque is injected before the RPM filter is ready, including at zero RPM.
- Id/Iq/RPM slew limiters retain fixed-point fractional position. Rates have a resolution
  of 0.001 output units/s; published integer references are rounded within 0.5 unit.
  Stop/fault/start reset fractional state; repeated setpoint updates do not. The
  supervisor uses differences of absolute millisecond timestamps rather than losing
  the fractional part of each polling interval. Zero elapsed time does not advance it.
- Park and inverse Park share one sin/cos pair per current or open-loop step.
  ADC timing, regulator gains and release optimization level are unchanged.

## IRQ timing diagnostics

`foc isr` reports one coherent snapshot:

```text
isr handler last=<us> us max=<us> us cyc=<last>/<max> budget_cyc=8500 calls=<n> over=<n>
```

The cycle budget is derived from `SYSCLK_FREQ_HZ / PWM_FREQ_HZ`: currently 8500 cycles
(50 microseconds). `over` counts measured durations **greater than or equal to** that
budget, not missed ADC triggers. Counts saturate at `u32::MAX`; `foc isr reset` clears
last/max/calls/over atomically. RTT also reports last/max, calls and over.

Measurement now starts in the JEOS handler before the flag check and ends after ADC
reads, current reconstruction, protection, control and duty application. Early-return
fault/idle paths are included. It excludes exception entry/exit, interrupt scheduling
latency, and the final statistics publication. Preemption by a higher-priority IRQ is
included. Thus zero overruns is **not** proof of meeting the hardware deadline; leave
margin and use on-board timing measurements for acceptance. The diagnostic does not
trip the motor automatically. Do not compare these maxima directly with the previous
control-body-only measurement.

After safe unloaded startup, clear the window with `foc isr reset`, exercise UART/RTT
traffic and regular sampling, then collect `foc isr` plus oscilloscope timing. No speedup
is claimed from host tests or the shared-rotation refactor alone.

## Host checks

```sh
cargo htest --locked
cargo check --release --locked
cargo clippy --release --locked -- -D warnings
cargo clippy -p foc --target x86_64-unknown-linux-gnu --all-targets --locked -- -D warnings
cargo fmt --all -- --check
cargo build --release --locked  # requires flip-link
```

`foc/tests/control_safety.rs` imports the actual firmware control implementation and
mocks the hardware edges. Its interrupt model queues faults until the outermost
critical section exits, exercising both a fault during reset and a fault at PWM enable.
It also checks sensor interlocks, cleared stop targets, first-fault retention, rejected
live parameter changes, repeated RPM commands retaining controller state, exact elapsed-time
reference ramps, and fractional-state clearing across stop/fault. Pure tests cover slow
and negative ramps, reversal, jittered intervals, integer limits, DWT wrap, diagnostic
counter saturation, and rotation equations.
These tests do **not** validate ADC/TIM1 electrical timing or real IRQ latency.

## Board acceptance required before loaded operation

Use a current-limited supply, initially with the motor disconnected. Do not halt the
core with SWD while PWM is enabled. Do not treat a successful host test as board approval.

1. **Idle sampling:** verify `adc` shows plausible VBUS/NTC and that VBUS responds to
   safe supply changes. Re-run `cal current` and verify bus updates resume afterwards.
2. **Live sampling:** in a safe unloaded setup, verify bus readings continue changing
   with PWM enabled. Scope the injected ADC trigger/current-sampling window and confirm
   that regular sampling has not altered JSQR, shunt sample times, or current-loop cadence.
3. **Freshness trips:** use a dedicated test build to suppress bus or encoder publication
   without stopping the CPU. Check MOE clears within the configured age limit plus one
   service interval; verify the latched fault and zero references. Explicit encoder
   communication/magnet failures should trip immediately in closed-loop modes.
4. **Start and restart:** verify sensor-unready start is rejected, old asymmetric CCR
   values are not re-applied, a pending BRK cannot be overridden, and stop/start does not
   restore old Iq/RPM commands. Confirm all live-mode transitions require stop.
5. **Speed updates:** after unloaded alignment, update RPM several times; ensure filter
   readiness and PI state persist. Check zero RPM, overspeed/coast behavior and reversal.
6. **Timing:** measure worst-case full JEOS ISR latency (including ADC reads and entry/
   exit), particularly with telemetry and regular sampling active. The PWM budget is
   50 microseconds at 20 kHz. `foc isr` covers the handler body including ADC reads,
   but excludes entry/exit, scheduling latency and final statistics publication.
7. **Duration:** run an unloaded endurance test through repeated mechanical revolutions;
   electrical-angle resolution must not degrade with accumulated position.

Remaining work includes board-level end-to-end IRQ timing measurements, minimizing critical
sections after measurement, alignment-quality validation, and hardware watchdog integration.
NVM writes still require PWM off and can stall flash execution.
