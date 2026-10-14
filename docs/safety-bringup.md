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
- Pole pairs and electrical offset can only be changed in Idle. Run/Speed require valid
  calibration (`cal=ok`): a qualified alignment, a valid NVM version-2 record, or an
  explicit operator-entered offset. Changing pole pairs invalidates calibration;
  setting the same value does not. NVM version 1 records are ignored on upgrade because
  they may have been written by the earlier unqualified alignment. Uncalibrated state
  cannot be saved. Openloop and Bench do not require rotor calibration.
- Alignment is qualified on the supervisor using fresh current-loop completions,
  measured Id within `min(100 mA, target/5)` of target, measured |Iq| within the same
  tolerance, a valid encoder, and rotor motion of at most 8 counts throughout a
  continuous 500 ms hold. A loss of qualification or a poll gap above 5 ms restarts the
  hold. Timeout after 2 s trips `fault=align`, disables PWM, invalidates calibration,
  and keeps the previous offset. The thresholds require unloaded board review.
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
  and a flash save (one page erase and two doubleword writes) must fit the timeout
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
module. The current baseline is 83 passing tests. CI pins Rust 1.98.1 and flip-link
0.1.12 and runs the full script including release linking.
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
10. **Sampling planner:** with controlled PWM test patterns, scope both measured
    low-side windows and the ADC trigger/conversion. Include two equal high duties,
    all phase permutations, the complete modulation sweep and regular ADC activity.
    Check that common-mode adjustment preserves line voltages and that delayed
    commits trip `fault=adc`. Verify startup ignores the two old results and remains
    at safe prepared duty until new feedback is accepted.
11. **IWDG:** in dedicated test builds with outputs initially off, stall the executor
    while ADC interrupts continue, stop the ADC IRQ while tasks continue, and halt
    all CPU work. Measure reset delay and all six gate outputs through reset/reboot.
    Verify reboot stays in Idle. Repeatedly run Idle calibration and flash save at
    supply/temperature limits; confirm their duration plus feed interval fits the
    shortest measured watchdog timeout, including LSI variation.

Remaining work includes board-level validation of the sampling planner and IWDG,
end-to-end IRQ timing measurements, and minimizing critical sections after measurement.
NVM writes still require PWM off and can stall flash execution.
