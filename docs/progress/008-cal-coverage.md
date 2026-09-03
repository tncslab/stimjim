# Phase 8: the timing budget as runtime state, and measurement coverage

Date: 2026-09-03

This phase turned every hardware timing budget the stimjimAWG scheduler and measurement engine
work from into runtime state (`CAL`), made the ramp sample interval a per-slot property (`DT`, or
an `L` line's 7th header field), and removed the wall that a fixed budget put in front of in-train
measurement: a point whose ADC reads do not fit the free gap they live in now rotates those reads
over consecutive repetitions instead of being refused. The trigger path gained an edge timestamp,
so interrupt entry and the arm-time precomputation no longer land on the delivered
trigger-to-output latency. Nothing here needed an oscilloscope or a change to the bench wiring —
that was the constraint the phase was scoped around. Plan:
[PLAN_calibration-coverage.md](../PLAN_calibration-coverage.md) (status: done).

**Not run on hardware.** The board was not on the bench this session. Host tests pass
(`test_measure`, `test_trainstore`, `test_samplegen` and a new `test_cal`), and all four
configurations compile clean with `--warnings more` (Teensy 3.5 register 163 KB, Teensy 3.5
portable, Teensy 4.1, Teensy 4.0 with `-DSJ_EEPROM_SLOTS=6`), but the `smoke.py` checks written
for the new commands have never been executed. **That is the next entry point.**

## What was built

### 1. `Cal` — nine timing budgets, settable over the serial port

`Cal.h`/`Cal.cpp` hold one struct of microsecond budgets, indexed by an enum so the command loop,
the `CAL?` listing and the EEPROM image cannot drift apart from each other:

| Name | Default (T3.5 registers) | What it budgets |
|---|---|---|
| `PRELOAD` | 4 | how early the player ISR wakes before a latch, then spins on `CYCCNT` |
| `DACPROG1` / `DACPROG2` | 3 / 5 | `dacProgram` one channel / `dacProgramBoth` |
| `ADCREAD` / `ADCSWITCH` | 3 / 4 | one conversion / the extra cost of a line switch |
| `GUARD` | 1 | margin between the last read and the next preload window |
| `SETTLE` | 4 | after a latch, before a reading means anything |
| `STARTLAT` | 20 | fixed start-request → first-latch latency |
| `TRIGCOMP` | 0 | hardware pin edge → trigger-ISR entry |

`Config.h` still holds the defaults per board and backend — the `#define`s did not move, they
became the initializers of `Cal::begin()`. `CAL?` / `CAL,<name>?` / `CAL,<name>,<us>` / `CALDEF`,
persisted by `P` in EEPROM image v5, one `DUMP` line per parameter that differs from the build
default, and `IDN`'s `# engine:` line now ends `cal=default` or `cal=custom`.

`Engine::startTrain` takes **one copy** of the set at arm time and uses it for the whole arm, so a
`CAL` line landing mid-arm cannot make one train use two budgets. `Cal::set` runs under
`FastIO::busLock()` because a trigger edge could otherwise arm a train between two words of the
assignment — the same reason `Triggers::setRoute` takes that lock.

Refusals, all in the host-tested `Cal::validate`: per-parameter floors (the three margins may be
zero, an operation budget may not), a 1000 µs ceiling, `DACPROG2 >= DACPROG1`, and
`STARTLAT >= PRELOAD + DACPROG2 + 3 µs + TRIGCOMP` — the first latch is programmed one preload
window before `t0`, so a start latency that does not cover it is unschedulable. A set is also
refused while a train runs. A stored budget that fails validation at boot is dropped with a `#`
line and the build defaults stay in force; `TrainStore::eepromRestore` reports that through an
out-parameter rather than printing, since nothing in that module prints.

**The portable backend's start latency had to grow.** Its programming budgets are twice the
register path's (`PRELOAD` 8, `DACPROG2` 11), which is 22 µs against a 20 µs `STARTLAT` — the new
invariant caught it immediately, and `SJ_START_LATENCY_US` is now 40 µs on that route. The board
also checks its own defaults at boot and warns if a build ships an inconsistent set.

### 2. Per-slot ramp sample interval

`TrainDef.dt_us` (0 = the build's `SJ_TARGET_DT_US`), set either as the optional 7th field of an
`L` header or with `DT<idx>,<us>`. It is positional, so a slot wanting an interval and no delay
writes the delay as `0`, and it round-trips that way. On `S` and `W` a 7th field is an `ERR`
rather than a silently ignored number — only a ramp has a sample interval. Omitting the field
resets the interval, which is the rule the delay already follows: a waveform line fully defines
its own header.

The parse-time range is 2…1 000 000 µs, a sanity bound only. What the player can actually keep up
with is `PRELOAD + DACPROG1/2 + SJ_MIN_SCHEDULE_US` — a `CAL` quantity — so an interval below it
is **refused at start** with the arithmetic, in the same style as the existing above-Fs/2 refusal.

### 3. Reads rotated over repetitions (`MEAS` `fit`)

`MeasDef.fit`: 0 = strict (refuse a point that does not fit, exactly the previous behaviour), 1 =
rotate, the new default. Rotation splits a point's reads into the smallest number of groups that
each fit one gap and fires **one group per repetition**, chosen by `pulseIdx % nGroups`.

The read windows on a Teensy 3.5 with the default budget are `room(1..4) = 21, 28, 35, 42 µs`
(one channel driven: 19, 26, 33, 40 — a one-channel train budgets `DACPROG1`, not `DACPROG2`).
So:

| Case | free gap | reads per gap | outcome |
|---|---|---|---|
| `L`, V+I both channels, `dt` 20 µs | 20 µs | 0 | refused: not even one read fits |
| `L`, V+I both channels, `dt` 25 µs | 25 µs | 1 | 4 groups |
| `L`, V+I both channels, `dt` 30 µs | 30 µs | 2 | 2 groups |
| `L`, V+I one channel, `dt` 20 µs | 20 µs | 1 | 2 groups |
| `W` 2 kHz, V+I one channel | 20 µs | 1 | 2 groups at any frequency |
| `W` 2 kHz, V+I both channels | 20 µs | 0 | refused by one microsecond |

The two remedies compose, and the table is the argument for both existing in the same phase: the
gap arithmetic is dominated by `PRELOAD + SETTLE` (13 of the 21 µs a *single* read needs), so
rotation alone cannot rescue a 20 µs dual-channel gap, and a wider `dt` alone would cost four
times the ramp resolution where rotation costs none.

**Why rotation rather than consecutive sample gaps**, which the earlier plan anticipated: on an
`L` ramp consecutive gaps carry values one ramp step apart, and around a `W` peak they carry
values several degrees off the peak — at 64 samples per cycle, four consecutive gaps span 17° and
4 % of amplitude. The four numbers of one record would then no longer describe one instant.
Rotation keeps every read at exactly the instant its label names and pays in `n` instead: each
line accumulates about `nPulses / nGroups` samples. For a repeating stimulus that is the better
trade, and it is the one that cannot be misread.

What it costs, and where each cost is reported:

- `MSUM` has one `n` field per point; with rotation it carries the largest of the four lines'
  counts, which differ by at most one. A per-point `#` line says the point was rotated and over
  how many repetitions.
- A `#` line at train start names the group count and how thin the per-line count gets, and
  points at the three ways out (`DT`, `CAL`, `fit=0`).
- A train with fewer repetitions than groups cannot cover every line. `planBuild` computes
  `ceil(duration_us / period_us)` and a `WARN MEAS:` says so.
- `MDATA` needs no format change: its `valid` mask already says which lines a row carries, so a
  rotated row is a partial one.

Rotation applies to all three train types. For `S` it makes a stage shorter than the full read
budget measurable, and the reads stay inside the stage they are labelled with. When not even one
read fits, the point is refused whatever `fit` says.

Implementation note: this needed **no new event machinery**. Rotation does not move a point in
time, so the plan keeps one entry per point and only `fire()` changed — it takes the read mask
from `grpMask[point][pulseIdx % nGrp[point]]` instead of from `lines[]`. What is new per point is
`nGrp`, `grpMask` and `budCyc` (the window actually reserved, which is now per point because the
gap differs per stage), about 90 bytes per plan.

The refusal message reports what the chosen `fit` asked for: the full window in strict mode, a
single read when rotating. That keeps the strict path's output identical to Phase 7's.

### 4. The trigger edge as the latency anchor

`Engine::startTrain` gained an `anchorCyc` parameter: the cycle count at which the start was
*requested*, or 0 for "now". `Triggers::edge` reads `FastIO::cycles64()` as its first statement
and passes it, so `t0 = edge − TRIGCOMP + STARTLAT + delay`. Two consequences:

- Interrupt entry and the whole arm-time precomputation (which includes a measurement plan build)
  drop out of the delivered latency, whatever the train's complexity. Phase 7 found a plan build
  costing the first latch 1.5 µs; with an edge anchor that class of bug cannot reach a
  trigger-started train's timing at all.
- Both engines armed by one edge now share one anchor, so an independent-mode route starts them
  on the same `t0` grid instead of one arm's cost behind each other. The ~7.5 µs arming skew
  `capture.py` subtracts should be gone — **unverified, and it is a scope measurement.**

What remains outside software's view is the hardware pin-to-ISR-entry delay, which is what
`TRIGCOMP` is for. It is 0 until someone measures it, and setting it is a serial line.

## Decisions taken in-phase

- *`Config.h` keeps the defaults; `CAL` is for one bench.* Editing the header is still right for
  a value wrong on every board of a type. This is why `DUMP` emits only the parameters that
  differ from the build default, and why `IDN` says which of the two a session is looking at.
- *Budgets stay whole microseconds.* The engine converts them with an exact integer multiply, and
  a budget that reads short is worse than one that is a fraction of a microsecond generous.
- *Rotation is the default.* A refusal is louder, but coverage out of the box with a `#` line
  explaining the cost serves the instrument's purpose better, and `fit=0` is one field away for
  anyone who wants the strictness. The strict path is bit-identical to Phase 7.
- *An omitted optional `MEAS` field takes its default, not the stored value* — the rule `S`/`L`/`W`
  headers already follow. `MEAS<i>,3,3,0,-1,0` therefore sets `fit` back to 1.
- *`DT` is refused on non-ramp slots* rather than stored and ignored, so a slot's answer to
  `DT<i>?` is never a number that does nothing.
- *EEPROM v5 rather than a migration.* The image gained `dt_us`, `fit` and the `CAL` block; older
  images are rejected outright as every previous version bump did. The `CAL` block is validated
  separately, so a stale budget loses the calibration without losing the slots.

## Verified

- Host tests: `test_measure` (rotation grouping for `S`/`L`/`W`, the balanced group split, the
  short-train warning, and the unchanged strict path), `test_trainstore` (the `dt_us` header
  field on `L` and its refusal on `S`/`W`, `DT` serialization, `fit` validation and
  serialization), `test_cal` (name lookup, floors, ceilings, both cross-parameter invariants,
  copy-apply-validate), `test_samplegen` — all pass.
- Builds, `--warnings more`, zero warnings: Teensy 3.5 register (163 KB flash, 38.4 KB RAM),
  Teensy 3.5 with both portable backends forced, Teensy 4.1, Teensy 4.0 with
  `-DSJ_EEPROM_SLOTS=6` (the image grew ~100 bytes, so that override is still required and the
  static_assert still names it).

## Owed

- **Run `smoke.py` on the board.** The new sections cover `CAL` (listing, single set, the
  invariant refusal, `DUMP` carrying it, `CALDEF`, refusal while a train runs), `DT` (both ways
  of setting it, the reset rule, refusal on an `S` slot, the start-time floor) and rotation
  (`fit=0` refusing a 25 µs stage, `fit=1` measuring it, the announcement, per-line coverage in
  `MSUM`, and the same A/B on an `L` ramp at 20 vs 25 µs). None of it has been executed.
- Everything Phase 7 still owed and this phase deliberately did not touch: dual-channel collision
  jitter and the measured `FsMax` table, long-run drift, the trigger-latency battery, the real
  GPIO header pins, and the two numbers that need a scope — `CAL SETTLE` (AD5752 settling) and
  `CAL TRIGCOMP`. Both are now one serial line from correct instead of one rebuild.
- The button menu editor, still the last unbuilt item of the implementation plan.

**Next entry point:** flash and run `python smoke.py COM4` with the new checks; then either the
bench session for the scope items or the menu editor.
