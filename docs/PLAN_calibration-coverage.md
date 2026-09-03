# Plan: runtime timing calibration and measurement coverage (phase 8)

Status: done, 2026-09-03.

This phase makes every hardware timing budget a runtime parameter instead of a compile-time
constant, and removes the two measurement-coverage walls that the fixed budgets created. Nothing
here needs an oscilloscope or a change to the bench wiring: the remaining scope items (dual-channel
collision jitter, the measured `FsMax` table, long-run drift, AD5752 settling) stay deferred, and
this phase's job is to make each of them a `CAL` line rather than a rebuild when the bench is
free again. Companion documents: [awg-implementation-plan.md](awg-implementation-plan.md),
[serial-protocol.md](serial-protocol.md).

## 1. Why

Three facts from phase 7 drive the work.

- The timing budgets (`SJ_PRELOAD_US`, `SJ_DAC_PROG1/2_US`, `SJ_ADC_READ_US`,
  `SJ_ADC_SWITCH_US`, `SJ_MEAS_GUARD_US`, `SJ_DAC_SETTLE_US`, `SJ_START_LATENCY_US`) are `#define`s
  compiled into the binary. Two of them are still guesses (`SJ_DAC_SETTLE_US`, and the whole
  portable-backend column), and re-measuring one costs a rebuild and a reflash.
- A measurement point needs `preload + nReads*(ADC_READ + ADC_SWITCH) + GUARD + SETTLE` of free
  gap. With four reads that is 42 µs, which no `L` train at the default 20 µs ramp interval can
  give and no `W` train above 372 Hz can give. The point is refused outright.
- The gap arithmetic is dominated by `preload + SETTLE` (13 µs of the 21 µs a *single* read needs
  on a dual-channel train), not by the reads. So the two remedies are independent: widen the gap,
  or put fewer reads in each gap.

The trigger path has a related gap: `t0` is taken inside the edge ISR as
`cycles64() + START_LATENCY`, so everything between the physical edge and that read — interrupt
entry, dispatch, the arm-time precomputation that precedes it — lands on the delivered latency
unmeasured and uncompensated.

## 2. What gets built

### 2.1 `Cal` — the timing budget as runtime state

New `stimjimAWG/Cal.h` / `Cal.cpp`. One struct of microsecond budgets, initialized from the
`Config.h` defaults at boot, read at arm time by `Engine::startTrain` and handed to
`Measure::planBuild` through the existing `Geometry`:

| Name | Default source | Meaning |
|---|---|---|
| `PRELOAD` | `SJ_PRELOAD_US` | how early the player ISR wakes before a latch |
| `DACPROG1` | `SJ_DAC_PROG1_US` | budget for `dacProgram` (one channel) |
| `DACPROG2` | `SJ_DAC_PROG2_US` | budget for `dacProgramBoth` |
| `ADCREAD` | `SJ_ADC_READ_US` | one conversion, line already selected |
| `ADCSWITCH` | `SJ_ADC_SWITCH_US` | extra cost of a control-register line switch |
| `GUARD` | `SJ_MEAS_GUARD_US` | margin before the next preload window opens |
| `SETTLE` | `SJ_DAC_SETTLE_US` | how long after a latch a reading means anything |
| `STARTLAT` | `SJ_START_LATENCY_US` | fixed arm→first-latch latency |
| `TRIGCOMP` | 0 | pin edge → trigger-ISR entry, subtracted for trigger starts |

Commands: `CAL?` prints one canonical `CAL,<name>,<value>` line per parameter then `OK`;
`CAL,<name>,<us>` sets one; `CALDEF` restores the compiled defaults. Every set is refused while a
train runs and validated against the invariant that the start latency must still cover a preload
plus a dual-channel program (`STARTLAT >= PRELOAD + DACPROG2 + SJ_MIN_SCHEDULE_US`, and
`TRIGCOMP` no larger than the slack). Values are persisted by `P` in the EEPROM image and shown by
`DUMP`; a restored budget that fails validation is dropped on its own — the slots come back, the
calibration does not, and a `#` line says so.

`TRIGCOMP` is the deferred trigger calibration made parametrizable, and it is only half of the
fix — the other half needs no constant at all: the edge ISR now timestamps the edge and hands that
anchor to `Engine::startTrain`, which places `t0 = anchor - TRIGCOMP + STARTLAT + delay`. Interrupt
entry and the arm-time precomputation therefore stop showing up in the delivered latency, and
`TRIGCOMP` is left carrying only the hardware pin→ISR-entry delay, to be measured on the scope
later.

### 2.2 Per-train ramp sample interval

`TrainDef` gains `dt_us` (0 = use `SJ_TARGET_DT_US`), settable two ways, mirroring `delay_us`:

- optional 7th header field of an `L` line: `L<i>,m0,m1,period,duration,delay,dt`. On `S`/`W` a
  7th field is an error — only a ramp has a sample interval.
- `DT<i>,<us>` / `DT<i>?`.

`Engine::startTrain` passes it to `SampleGen::rampStageInit`, so the ramp is sampled coarsely on
purpose, which is what widens the measurement gap. A `dt_us` below what the player can program
(`PRELOAD + DACPROG1/2 + SJ_MIN_SCHEDULE_US`, a `Cal` quantity) is refused at start with the
arithmetic, in the same style as the existing above-Fs/2 refusal.

### 2.3 Reads rotated across repetitions

`MeasDef` gains `fit`: 0 = strict (refuse a point that does not fit, today's behaviour), 1 =
rotate (the default). Rotation splits a point's reads into groups small enough to fit one gap and
fires **one group per repetition**, cycling by pulse index.

The alternative the earlier plan anticipated — spreading a point's reads across *consecutive*
sample gaps — was rejected: on an `L` ramp consecutive gaps carry values one ramp step apart, and
on a `W` peak they carry values a few degrees off the peak, so the four numbers of one record
would no longer describe one instant. Rotation keeps every read at exactly the instant its label
claims and pays in `n` instead: each line accumulates about `nPulses / nGroups` samples.

Consequences to keep honest about, all reported:

- `MSUM` carries one `n` per point; with rotation the lines' counts differ by at most one and the
  largest is reported. A `#` line states the group count and the per-line count.
- A train with fewer repetitions than groups cannot cover every line. `planBuild` knows
  `duration_us / period_us` and warns at arm time.
- `MDATA` rows already carry a per-row `valid` mask, so a partial row needs no format change.

Rotation applies to all three train types. For `S` it makes a stage shorter than the full read
budget measurable; the reads stay inside the stage they are labelled with.

## 3. Order of work

1. `Cal` module, `CAL`/`CALDEF`, EEPROM v5, `DUMP`/`IDN` reporting; `Engine`/`Measure` read the
   runtime values. Host test for the validator.
2. Trigger edge anchor + `TRIGCOMP` in `Engine::startTrain`.
3. `dt_us`: `WaveformDef`, parser, `DT` command, serializer, start-time floor check.
4. Rotation: `MeasDef.fit`, `planBuild` grouping, the ISR read mask, notes and summary lines.
5. Host tests (`test_measure`, `test_trainstore`, new `test_cal`), `smoke.py` cases, docs.

## 4. What this phase does *not* do

- No scope work. `SJ_DAC_SETTLE_US` stays a guess, `SJ_FS_MAX_HZ` stays a desk estimate, and
  `TRIGCOMP` stays 0 until someone measures it. Each is now one serial line away from correct.
- No menu editor (the last unbuilt item of the implementation plan).
- No change to the delivered waveform for any existing definition: a slot with `dt_us = 0` ramps
  exactly as before, and rotation only changes *when a read is taken*, never when a sample latches.
