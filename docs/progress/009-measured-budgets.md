# Phase 9: the timing budgets measured, and the figures that explain them

Date: 2026-09-03

This phase put the Phase-8 firmware on the board for the first time and measured the two timing
budgets that had never been anything but desk estimates. Both were wrong, and both were wrong in
the direction that quietly degrades results rather than announcing itself. `CAL STARTLAT` was
20 µs while arming a train costs 19–42 µs, so a trigger-started train's first latch was late and
a trigger routed to both engines put the second one ~33 µs behind the first. `CAL SETTLE` was
4 µs while the analog path plus the ADC aperture needs 8–9 µs, so every in-train reading was 8–9 %
short of the value the output actually reached. Two new benchmarks make both measurable without
an oscilloscope — `BENCHARM` times `Engine::startTrain`, `BENCHSETTLE` reads the same step back at
increasing delays after its latch — the defaults were corrected from what they measured, and the
engine now reports by name when an arm does not fit its budget. Five explanatory figures were
added: two showing the timing budgets to scale, three showing which field of an `S`, `L` or `W`
command sets which part of the waveform. The README became a quick start for `stimjimAWG`, and
`docs/bench-wiring.md` describes from scratch the wiring and procedure for everything a scope is
still needed for. All 100 serial smoke checks, all four host test suites and all four build
configurations pass; the scope acceptance run passes for the first time since Phase 6.

## What the bench found

### 1. `STARTLAT` did not cover the arm, and Phase 8 is what exposed it

Phase 8 moved the start-latency anchor to the trigger edge and moved `Measure::armPlan` to before
`t0` is taken. Both were right. Together they moved the whole cost of `Engine::startTrain` inside
`STARTLAT` instead of after it, and nobody checked that 20 µs was enough to hold it.

The first scope run of the phase measured an engine-to-engine skew of **−32.7 µs**, reproducible
to ±0.1 µs, where Phase 8 predicted roughly zero. The firmware's own counters named the cause
without a scope: `# engine: 2 event(s) were already due when the player reached them, worst by
27333 ns`. A new benchmark, `BENCHARM,<slot>,<n>`, times the arm directly (bus-locked, and the
train is stopped again inside the same lock, so nothing plays):

| Slot | arm, min / avg / max |
|---|---|
| undriven, 0 stages | 16.20 / 16.29 / 16.58 µs |
| `S`, one channel, one stage, measurement off | 18.20 / 18.25 / 18.58 µs |
| `S`, two channels, one stage, measurement off | 19.20 / 19.24 / 19.51 µs |
| `S`, two channels, one stage, default `MEAS` | 23.41 / 23.51 / 23.83 µs |
| `S`, two channels, ten stages, measurement off | 28.24 / 28.27 / 28.74 µs |
| `L`, two channels, one stage, measurement off | 22.83 / 22.92 / 23.24 µs |
| `W`, two channels, measurement off | 37.49 / 37.51 / 37.83 µs |

So a measurement plan costs an arm ~4.3 µs, a stage ~1.0 µs, and the sine setup ~18 µs — but the
dominant term is the **16.6 µs an arm costs before it has done anything train-specific at all**.
That is a target for a later phase, not this one; see *Remaining*.

`STARTLAT` must be at least the arm plus `PRELOAD + DACPROG2 + SJ_MIN_SCHEDULE_US` = arm + 12 µs.
The default is now **60 µs** on the register build and **120 µs** on the portable one, which
covers every single-engine train type including a measured sine, and covers a two-engine
independent route for `S` and `L` slots. Two sine trains on one edge need about 100 µs and are
left to say so rather than being paid for by every other user of the trigger path.

Saying so is the second half of the fix. `Engine::startTrain` now compares the instant it finishes
against the first latch's preload window and, when it has already opened, records the `STARTLAT`
that would have covered *this* arm. The train still runs — its first latch is simply late — and
the completion carries

```
WARN engine: arming this train took longer than CAL STARTLAT (60 us), so its first latch could
not be on time — set CAL STARTLAT >= <n> us (BENCHARM,<slot> measures the arm; a TRIG independent
route arms twice)
```

This is deliberately a per-train report rather than a refusal: a late first latch is a defect in
the *calibration*, not in the waveform, and refusing to run would be a worse answer for someone
mid-experiment than running with a named, correctable fault.

After the fix the engine-to-engine offset is **−0.25 µs**, against −7.5 µs under the Phase-7
per-engine anchor. Phase 8's claim is delivered; it just could not be delivered at a 20 µs budget.

### 2. `SETTLE` was 4 µs and the instrument needs 9

`CAL SETTLE` is how long after a latch a reading means anything. It had never been measured — the
comment in `Config.h` said "RECALIBRATE with a scope", and a scope was never available with enough
vertical resolution to do it. The board can measure it on itself, which is what `BENCHSETTLE`
does: park, latch the same step, spin `d` microseconds, read, repeat, for `d` from 0 upwards.

```
BENCHSETTLE,0,8000,20,64
delay_us   0    1    2     3     4     5     6     7     8     9    10
raw      259  390  732  1030  1146  1219  1242  1249  1253  1254  1255
```

The same shape appears at −8000 codes, at +2000 codes and on channel 1: the reading reaches its
final value at **8–9 µs**, and at the 4 µs the firmware shipped with it is **8.6 % short**. Every
in-train measurement this firmware has ever taken was reading a transient.

`SJ_DAC_SETTLE_US` is now 9. It is a property of the analog path and the ADC aperture together —
not of the MCU or the backend — so it is not conditioned on either, which is a change from how the
other budgets are written.

The cost is real and is the main behaviour change of this phase. The free gap a measurement point
needs grows by 5 µs, to 24 µs for a single read and 47 µs for V+I on both channels, and **no
measurement point fits the default 20 µs ramp interval any more**. An `L` slot that wants in-train
measurement needs `DT` at 26 µs or more; an `S` slot is unaffected as long as its stages are that
long, which most stimulus stages are; a `W` train is limited to 332–651 Hz depending on how many
lines it reads. The tables in `docs/serial-protocol.md` §4 and
`figs/stimjim-timing-measurement.png` carry the new arithmetic.

That is the honest trade. The alternative — keeping a budget that buys coverage by measuring the
transient — is worse in a way that no warning could surface, because the numbers it produces look
perfectly reasonable.

### 3. Two engines started by one edge cannot latch at the same instant

Both player ISRs run at `SJ_PLAYER_PRIO`, so the second one waits for the first to return. With
`STARTLAT` correct and the arm no longer a factor, what remains is **7.3 µs** by the engine's own
overdue counter and **9–10 µs** at the output on the scope
(`figs/stimjim-trigger-contention.png`).

This is not a defect to tune away: the two engines share one SPI bus and one `NLDAC` line. It is
an argument about how to use the instrument, and it now has a number. Two channels that must be
sample-aligned belong in **one train that drives both**, where a single `dacProgramBoth` and a
single latch pulse serve both channels. An independent `TRIG` route is for two stimuli that merely
start together.

### 4. The delay measurement had a bench-circuit artefact, not a firmware fault

`capture.py delay` reported the 2000 µs delay 11 µs long while 5000 and 20000 were within a
microsecond. The cause was in the wiring, not the firmware: the reference pulse on CH1 was 2000 µs
long, so at a 2000 µs delay its *falling* edge coincided with the rising edge under test, and the
few hundred millivolts it couples onto scope channel A through the resistor chain dragged the
interpolated threshold crossing by ~10 µs.

Two changes fixed the method. The reference pulse is now 300 µs (`REF_HIGH_US`), so its fall is
clear of every delay the script measures. And the engine-to-engine offset is calibrated at a
**500 µs** delay instead of at 0, because at 0 it contains the latch contention of §3, which is
absent at every other delay — that case is now captured and reported separately, which is where
the contention figure comes from. Delay accuracy re-measured: 2000 µs set → 1999.3, 5000 →
4999.3, 20000 → 19996.0, every residual inside the scope's own sample interval.

## What was built

- **`BENCHARM,<slot>[,n]`** — the arm cost, bus-locked, train stopped inside the same lock.
- **`BENCHSETTLE,<ch>,<code>,<dmax_us>[,n]`** — the reading-versus-delay curve after a latch, plus
  a `#` line naming the first delay from which every later reading stays within two ADC codes of
  the final one. Drives the output, so it prints a `WARN` first.
- **The start-latency shortfall report** — `Completion.startNeedUs`, set in `startTrain` and
  printed with the other timing faults.
- **Corrected defaults** — `SJ_START_LATENCY_US` 20 → 60 (register) and 40 → 120 (portable),
  `SJ_DAC_SETTLE_US` 4 → 9.
- **EEPROM image v6.** The layout did not change; the version did, because a v5 image would
  restore the old budgets in silence. A budget that is merely *wrong* still passes
  `Cal::validate`, so the version bump is the only thing that could have caught it. Anyone who ran
  `P` before this phase loses their stored slots along with the stale budget, which is the right
  trade for not running a corrected firmware on uncorrected numbers.
- **`capture.py`** — the reference-pulse length and the offset-calibration delay above, plus a
  third reported quantity (latch contention) and its figure.
- **Five figures**, generated by two scripts under `docs/figures/`:
  `stimjim-timing-latch.png` (the start path and one latch, bars to scale),
  `stimjim-timing-measurement.png` (what a measurement point needs and whether it fits),
  and `stimjim-param-S/L/W.png` (each command line with every field numbered and the same numbers
  on the waveform). They are linked from `docs/serial-protocol.md` §2 and §4 and from the README.
- **Documentation** — the README is now a quick start for `stimjimAWG` (the board-building
  instructions moved to `docs/hardware-build.md` unchanged), `docs/bench-wiring.md` is new, and
  `docs/serial-protocol.md`, `docs/hardware-variants.md` and `tests/device/README.md` carry the
  measured numbers and the new commands.

## Verified

- **On silicon**, Teensy 3.5 at 120 MHz, register backends: `smoke.py` — 100 checks, all pass,
  including the `CAL`, `DT` and rotation sections that Phase 8 wrote and never ran.
  `capture.py delay` and `capture.py shapes` — all captures OK.
- **Host tests**: `test_cal`, `test_measure`, `test_trainstore`, `test_samplegen` — all pass.
- **Builds**, `--warnings more`, zero warnings, all four configurations: Teensy 3.5 register
  (168 760 B flash, 39 380 B RAM), Teensy 3.5 with both portable backends forced (169 828 B),
  Teensy 4.1, Teensy 4.0 with `-DSJ_EEPROM_SLOTS=6`.
- **Re-measured `BENCH` group** (n = 2000), unchanged from Phase 7 within noise: `BENCHDAC`
  1.42/1.43/2.81 µs, `BENCHDAC2` 2.75/2.75/4.14, `BENCHLATCH` 0.44/0.44/1.78, `BENCHADC`
  2.37/2.37/3.81, `BENCHSW` 4.62/4.63/6.06, `BENCHMISO` 2.24/2.27/3.73, `BENCHCYC` 0.27,
  `BENCHPIT,1000,2000` 0.30/0.37/0.47 late, `BENCHPIT,1000,2000,4` 0.117/0.125/0.158 — the 42 ns
  residual latch jitter reproduces. It is also independent of the event rate: the same 14/15/19
  cycles at 1000 µs, 200 µs and 50 µs periods, and a 2 µs preload already buys the same jitter as
  a 4 µs one.

## The requested-feature list in `firmware-spec.md`

| Requested | State |
|---|---|
| Piecewise-linear, piecewise-constant (0-duration = instant jump), sinusoidal waveforms | Done — `L`, `S`, `W` |
| Envelope ramping as a separate parameter and command | Done — `ENV`, and the `W` line's 5th triplet |
| Near-microsecond precision and duration | Done and measured — 42 ns residual latch jitter, sample instants on an exact CPU-cycle grid |
| Trigger inputs 0 and 1; synchronized or per-channel; programmable which waveform on which channel | Done — `TRIG<t>,<mode>,<slot0>,<slot1>,<edge>`. **Newly qualified:** "synchronized" means one train driving both channels; two engines on one edge are ~9 µs apart |
| Configurable what is measured and where; measurement programming takes time; DAC programming separable from execution | Done — `MEAS` with per-channel `what`, `when`, `stage`, `report`, `fit`. The program-early/latch-on-deadline design *is* the separation asked for, and it is what makes a measurement point placeable at all |
| Decide clock-driven vs calibrated delays; investigate DMA for non-blocking SPI | Decided: clock-driven, exact integer cycles on `cycles64()`. **DMA not implemented, and now with a measured basis for that:** `dacProgramBoth` costs 2.75 µs typical inside a 9 µs preload window, so DMA could return at most ~3 µs of CPU time per latch and would add a completion check before the latch. It is worth revisiting only if the arm cost (below) is reduced first, since that is 5× larger |
| Interrupt or loop; timing above serial | Done — PIT player ISRs at priority 64, triggers at 80, USB serial at 112 |
| Preparation for screen, buttons and rotary encoder | **Partial — the one unbuilt item.** The OLED and three buttons work and the views are done; the menu *editor* (button sets the waveform, rotary selects it) is not written |
| Fix the serial limitations: querying, evident defaults | Done — `?` on every settable object, the authoritative defaults table in protocol §5, `HELP`, `DUMP` |
| Measurement results with timestamps on the SD card; per-train selection; open/close/query commands; writes must never disturb timing | Done — `MEAS report` +2, `LOG`, the `SD` group, one ring buffer drained in `loop()` |

## Remaining

**Needs a scope and a wiring change** — `docs/bench-wiring.md` describes each from scratch:

- `CAL TRIGCOMP`, the pin-edge-to-ISR-entry delay. Configuration B. It is the last quantity in the
  firmware with no measured value; expect a few hundred nanoseconds.
- Dual-channel collision jitter: do the two channels of *one* train step together at the output?
  Configuration C1. The design says yes (one `dacProgramBoth`, one latch pulse) and that claim is
  the reason to prefer one train over an independent route, so it deserves a direct check.
- The real `SJ_FS_MAX_HZ`. Configuration C2, though step 2 of it needs no scope at all.

**Needs no wiring change:**

- Long-run drift: a multi-hour train, checking that the 64-bit cycle extension survives. The
  procedure is in `docs/bench-wiring.md`.
- The trigger-latency repeatability battery — spread over many edges, which configuration B
  produces as a by-product.
- The GPIO header trigger pins have still never been exercised; the bench uses the BNC IN0.
- The portable backend's whole `CAL` column, on a real Teensy 4.x. `SETTLE` should carry over
  unchanged (it is analog), `STARTLAT` starts at 120 µs and `BENCHARM` will say whether that is
  enough.

**Firmware work:**

- **The menu editor.** The last unbuilt item of the implementation plan and of the spec.
- **Reduce the arm cost.** 16.6 µs before anything train-specific happens is the number that sets
  `STARTLAT`, and `STARTLAT` is the trigger latency. It has not been profiled; the candidates
  visible from the measurements are `Measure::planBuild`'s `memset` of a ~1 kB plan (paid even by
  a train that measures nothing), the 64-bit divisions in the ramp and sine setup, and
  `ampToCode`'s float division per stage per channel. Halving it would halve the trigger latency.
- **Pre-build the measurement plan when the `TRIG` route is set**, not when the edge arrives. The
  plan depends only on the `TrainDef` and the `CAL` set, both known in advance, and it is 4.3 µs
  of the arm.
- **Issue the first `adcSelectLine` inside the `SETTLE` window.** The control-register write does
  not sample anything, so it can overlap the settling; that would return `ADCSWITCH` = 4 µs of the
  9 µs `SETTLE` now costs every measurement point, which is most of what this phase took away.
- The `WARN`/`#` strings contain UTF-8 em-dashes and `µ`. A strict-ASCII host parser would see
  multi-byte sequences where it expects one character. Nothing on this bench cares, and the
  BIST-frozen replies are unaffected, but it is a latent compatibility hazard worth a decision.

## Next entry point

Either the menu editor, which is self-contained and needs no bench, or a bench session in
configuration B for `CAL TRIGCOMP` followed by configuration C for the two-channel checks. If the
next session is a firmware one, the arm-cost profiling is the highest-value item, because every
microsecond it saves comes straight off the trigger latency.
