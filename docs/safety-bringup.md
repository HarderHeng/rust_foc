# Safety changes: host verification and board acceptance

## Implemented behavior

- VBUS/NTC use a nonblocking, single-rank regular ADC1 sequence, alternating channels.
  The supervisor polls at approximately 1 ms, so a complete pair normally arrives every
  2 ms; the first poll starts VBUS, the next completed poll starts NTC, and the following
  completed poll publishes the pair and immediately starts the next VBUS conversion.
  Injected current conversions retain priority. Regular sampling does not toggle ADEN,
  rewrite injected sample times, or re-arm JSQR each poll.
- ADC ISR status is cleared with W1C writes, preserving regular EOS/EOC flags.
  Current decoding requires both ADC1 and ADC2 JEOS with neither JQOVF set.
  ADC1 completion allows ADC2 bounded skew: a 76-cycle deadline and a hard 32-poll
  cap, whichever stops the wait first (plus the final poll's execution time).
  A missing partner or queue overflow trips
  `fault=adc` without publishing a control completion. The poll bound also applies
  when DWT is stopped. The actual ADC skew remains a board acceptance item.
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
  Encoder age starts immediately before the ANGLE I2C transaction, so completion
  cannot renew an old acquisition. Future/expired/nonfinite samples are rejected;
  `enc` exposes this conservative age as `age_us`. This does not measure AS5600
  internal filter latency. The control angle remains zero-order hold.
- Start validation, safe CCR preparation, MOE enable and mode commit form one short
  interrupt-masked transaction. Pending hardware break flags are not cleared by start.
  Mode changes while live require `foc stop`; repeated Run start is a keepalive and
  repeated Speed commands update the setpoint without resetting either PI.
- Stop and fault clear current/speed/open-loop references. The first fault cause is
  retained until a successful restart. `foc stop` acknowledges Fault but does not
  bypass sensor or hardware-break interlocks on the next start.
- Pole pairs and electrical offset can only be changed in Idle. Run/Speed require valid
  calibration (`cal=ok`): a qualified alignment, a valid NVM version-3 record, or an
  explicit operator-entered, independently verified offset (`foc offset <mrad>`).
  This offset command is an operator override; firmware does not verify the entered
  value. The command automatically saves it. Changing pole pairs invalidates calibration;
  setting the same value does not. NVM versions 1 and 2 are ignored on upgrade because
  earlier firmware could save unverified offsets and the records do not identify
  their origin. Upgrade requires a qualified `foc align` followed by `foc save`, or
  an independently verified `foc offset <mrad>`. Uncalibrated state cannot be saved.
  Openloop and Bench do not require rotor calibration.
- `foc zero` requires Idle and a fresh encoder sample. It captures the current position
  as an unverified electrical offset, invalidates calibration, and does not save it.
  It cannot qualify Run/Speed or `foc save`; use a qualified alignment or explicitly
  enter an independently verified offset before saving.
- Alignment is qualified on the supervisor using fresh current-loop completions,
  measured Id within `min(100 mA, target/5)` of target, measured |Iq| within the same
  tolerance, a valid encoder, and rotor motion of at most 8 counts throughout a
  continuous 500 ms hold. A loss of qualification or a poll gap above 5 ms restarts the
  hold. Timeout after 2 s trips `fault=align`, disables PWM, invalidates calibration,
  and keeps the previous offset. The thresholds require unloaded board review.
  `foc align` without a current uses the default. Invalid text, integer overflow, or
  extra arguments return `usage` without starting alignment; valid integer currents
  retain the controller's existing clamp.
- The supervisor trips `fault=stall` if live outputs observe no current-control
  completion for 5 ms. The completion sequence is separate from `foc isr` diagnostics,
  so a statistics reset cannot pet it.
- Independent IWDG starts after peripheral bring-up and boot offset calibration.
  Its LSI-clocked reset timeout is nominally 1 s; actual LSI tolerance must be measured.
  Only the analog supervisor feeds it, after protection/control work, at least 50 ms
  apart and only when both ADC completions and encoder-task iterations have advanced.
  Neither interrupt progress alone nor an Idle maintenance operation bypasses this
  policy. Encoder error paths count as task liveness while the sensor interlocks still
  disable closed-loop outputs. Runtime offset calibration (48 blocking ADC reads)
  and a flash save (two doubleword writes, plus page erase on reclaim) must fit the timeout
  allowance; their worst-case duration has not yet been established on the board.
- Panic and HardFault disable TIM1 MOE and CH1-3/CH1N-3, without logging or shared
  borrows, then halt with interrupts disabled. PWM access uses a checked
  `Mutex<RefCell<Option<_>>>` rather than an escaping mutable reference.
- AS5600 samples are invalid when the magnet is missing, too weak, or too strong.
- Production math explicitly uses `libm` (`sqrtf`, `sincosf`) on host tests and firmware.
  Integration tests link the non-test library and verify voltage-limit magnitudes and
  rotation accuracy. Timing impact must be measured on the board.
- Encoder control angle is single-turn. Multi-turn position remains integer counts;
  speed is derived from signed count differences. `enc mdeg` now reports single-turn
  position rather than an ever-growing floating-point angle.
- Speed PI includes direction limits and tracking of the slew-limited Iq reference.
  No startup torque is injected before the RPM filter is ready, including at zero RPM.
  RPM uses signed count differences and integer acquisition-time intervals. Encoder
  errors or gaps at least `ENC_SPEED_MAX_GAP_US` invalidate speed readiness and rebase
  the window. This gap is capped at 10 ms and at the half-turn interval at maximum
  configured RPM; a fresh complete 100 ms window is then required. Duplicate stamps
  do not consume displacement. Sampling aliasing above the configured maximum speed
  cannot be ruled out by this absolute-encoder estimator.
- Id/Iq/RPM slew limiters retain fixed-point fractional position. Rates have a resolution
  of 0.001 output units/s; published integer references are rounded within 0.5 unit.
  Stop/fault/start reset fractional state; repeated setpoint updates do not. The
  supervisor uses differences of absolute millisecond timestamps rather than losing
  the fractional part of each polling interval. Zero elapsed time does not advance it.
- Park and inverse Park share one sin/cos pair per current or open-loop step.
  ADC timing, regulator gains and release optimization level are unchanged.
- PWM phase CCRs, CH4 trigger and ADC pair share one integer sampling planner.
  Each measured phase must settle for `Tafter` after its up-count switching edge and
  remain conducting throughout the complete ADC budget. The current budget is
  76 TIM1 ticks: 2.5 sampling + 12.5 conversion + 4 synchronization/margin cycles at
  the HAL-selected SYSCLK/4 ADC clock. Prefer UV at ARR-1; otherwise select the lowest
  two compares. If needed, subtract the smallest equal common-mode shift from all
  three CCRs, preserving every quantized line voltage. An impossible window trips
  `fault=adc`. Host tests include 0.933/0.933/0.067 and all its permutations, timer
  boundary exhaustion, full duty, complete ADC budgets and an SVPWM revolution sweep.
  This fixes the known geometric window failure; ADC latency, regular/injected ADC
  arbitration and noise from the unmeasured phase remain scope acceptance items.
- Live commits mask timer update transfers with UDIS while writing all four CCRs
  and ADC routing. They must start and finish on the down-count before the valley,
  with no new pending JEOS; a missed window disables MOE and trips `fault=adc`.
  Startup uses explicit UG with MOE off and discards two injected results to cover
  a pending old JDR and an in-flight old single-rank conversion. These skips are not
  used to conceal runtime deadline failures. Near 100%, Bench common duty can be
  shifted down as well; use CCR readback to see the applied duty.
- Current/speed controller state is transferred out of its shared slot under a short
  critical section. PI, transforms and PWM geometry execute with interrupts enabled;
  reentrant computation cannot borrow the checked-out state. Reset and gain requests
  enter a mailbox and invalidate the pending result. Run epochs reject results from
  a stopped/faulted/restarted run, including a restart into the same mode. Commit
  rechecks sensor freshness and the PWM path checks hardware MOE before applying.
  This establishes ownership and interrupt ordering, not a measured WCET improvement.
- Nonfinite phase-current samples trip `fault=adc` in closed and open loop. Invalid
  PID input, limits or overflowing arithmetic return an invalid output without
  corrupting retained history; the driver rejects nonfinite duties. Runtime gain
  updates reject an entire pair if either value is nonfinite or negative. Shell
  parsing accepts finite scientific notation and long decimal fractions.
- NVM uses version 3 while keeping the existing 16-byte layout in an append journal
  of 128 slots within the reserved 2 KiB page. Version-1/version-2 slots are ignored
  for loading but remain consumed, so new version-3 records can append after them.
  Identical saves are no-ops; invalid/torn slots are
  consumed rather than overwritten, and later appends leave earlier valid records
  available. Reclaim occurs only when full, verifies erased bytes, then programs
  and reads back the replacement. PWM must be off. This single-page design can lose
  calibration if power fails after reclaim erase, leaving Run/Speed interlocked on
  next boot until recalibration. Host byte-failure tests do not establish actual
  STM32 flash ECC or brownout recovery behavior.

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

The live register commit has a stricter phase deadline: after a near-center CH4
sample, all preloads and ADC routing must be committed before the following valley
(approximately half a PWM period, less conversion time). The runtime guard trips
`fault=adc` on a missed commit even when the 50 us diagnostic overrun count is zero.

After safe unloaded startup, clear the window with `foc isr reset`, exercise UART/RTT
traffic and regular sampling, then collect `foc isr` plus oscilloscope timing. No speedup
is claimed from host tests or the shared-rotation refactor alone.

## Host checks

```sh
scripts/check.py --offline
scripts/check.py --offline --build  # requires flip-link
```

Each command has a 120 s deadline and its process group is killed on timeout.

`foc/tests/control_safety.rs` imports the actual firmware control implementation and
mocks the hardware edges. Its interrupt model queues faults until the outermost
critical section exits, exercising both a fault during reset and a fault at PWM enable.
It also checks sensor interlocks, cleared stop targets, first-fault retention, rejected
live parameter changes, repeated RPM commands retaining controller state, exact elapsed-time
reference ramps, fractional-state clearing across stop/fault, calibration interlocks,
alignment failure/restart/success, pole-change invalidation, and the current-control
stall trip in every live mode. Pure tests cover slow
and negative ramps, reversal, jittered intervals, integer limits, DWT wrap, diagnostic
counter saturation, and rotation equations.
Sampling tests independently exhaust small timer geometries and sweep the SVPWM
circle; watchdog tests exercise worker stalls and maintenance recovery. The UART
completion/formatting helpers are also compiled and tested from their production
module. `firmware_concurrency.rs` imports the production controller/current/speed
modules and injects faults, stop/restart, reset/gain updates, sensor expiry and
hardware MOE loss during computation. It asserts that calculation and planning
execute outside the mocked critical section. `telemetry_feedback.rs` imports the
production telemetry module to check acquisition age, errors and timestamp wrap.

Independent f64 average-inverter/RL/PMSM models reconstruct applied voltage from
actual SVPWM duties, include a full-period command delay, and check current steps,
rotation, feed-forward and saturation recovery. They omit shunt timing, encoder
latency, dead time and physical noise. The synthetic mechanical model demonstrates
load sensitivity: unchanged 10 Hz/100 ms speed control oscillates with observed
tail instantaneous error of about 20 RPM at inertia 5e-5 kg m². Peak-to-peak amplitude
and period were not measured. Larger modeled inertias meet a 3 RPM tail-error bound; the light
case only asserts a 50 RPM bound, not precise settling. See `docs/next-steps.md`.

The 2026-10-15 baseline is 113 passing Rust tests plus 13 Python artifact-validation tests.
CI pins Rust 1.98.1 and flip-link 0.1.12 and runs the full script including release
linking. The artifact validator checks FLASH/RAM/NVM placement, vector/entry and
flip-link stack placement, and writes ELF/tool/input hashes and size budgets to
`manifest.json`. These link-time budgets do not prove runtime stack peaks.
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
8. **New trips:** verify `fault=stall` by suppressing JEOS in a dedicated test build;
   verify panic and HardFault disable all six outputs within an acceptable time.
9. **Alignment:** verify success, tolerance, rotor-motion detection and timeout with a
   current-limited unloaded motor; check that a failed alignment keeps the old offset.
   Confirm `foc zero` invalidates calibration, does not save, and blocks Run/Speed and
   `foc save`. Check malformed `foc align` commands report usage without enabling PWM.
10. **Sampling planner:** with controlled PWM test patterns, scope both measured
    low-side windows and the ADC trigger/conversion. Include two equal high duties,
    all phase permutations, the complete modulation sweep and regular ADC activity.
    Check that common-mode adjustment preserves line voltages and that delayed
    commits trip `fault=adc`. Verify startup ignores the two old results and remains
    at safe prepared duty until new feedback is accepted.
    Suppress one ADC's injected completion and separately force queue overflow in a
    dedicated build; ensure a stale JDR is never decoded and `fault=adc` is latched.
11. **IWDG:** in dedicated test builds with outputs initially off, stall the executor
    while ADC interrupts continue, stop the ADC IRQ while tasks continue, and halt
    all CPU work. Measure reset delay and all six gate outputs through reset/reboot.
    Verify reboot stays in Idle. Repeatedly run Idle calibration and flash save at
    supply/temperature limits; confirm their duration plus feed interval fits the
    shortest measured watchdog timeout, including LSI variation.
12. **NVM:** verify v1/v2-only records are rejected and keep Run/Speed interlocked;
    confirm v3 calibration loads and new v3 records append after consumed old-version
    slots. Verify repeated identical saves avoid programming, changed records append,
    and full-page reclaim succeeds. Test
    interrupted writes and reclaim with controlled power loss; record actual ECC/
    reset behavior and verify absent calibration prevents Run/Speed after reboot.
13. **Encoder timing and speed:** measure ANGLE acquisition-to-PWM delay and AS5600
    internal lag across RPM. Record `enc age_us`, window readiness after gaps, and
    low-inertia oscillation under limited current before choosing PI or filter changes.

Remaining work includes board-level validation of the sampling planner and IWDG,
end-to-end IRQ timing/stack measurements, and speed tuning against measured mechanics.
NVM writes still require PWM off and can stall flash execution.
