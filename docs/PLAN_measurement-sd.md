# Phase 7 plan — measurement engine, SD logging, serial file retrieval

Status: done, 2026-09-03

This is the plan as written before the work, kept as a record of intent. Two things in it were
superseded by measurement and should be read from
[serial-protocol.md](serial-protocol.md) §4 instead: the measurement-window figures in §1.2
below (the estimate of "about 31 µs" became 19–42 µs depending on lines and channels, so the
`W` ceiling is 372–558 Hz rather than ~2 kHz), and `SDINFO`, whose used-space field turned out
to cost seconds and became opt-in. The outcome, including the defects the work uncovered, is in
[progress/007-measurement-sd-bench.md](progress/007-measurement-sd-bench.md).

This phase implements the two remaining data-path features of the plan: in-train measurement
(`MEAS` execution, `MSUM`, `MDATA`) and SD logging (`LOG`), plus one requirement added by the
user after Phase 6: **the SD socket sits under the instrument cover, so everything written to
the card must be readable back over the serial port.** That adds a small file-access group
(`SDINFO`/`SDLIST`/`SDGET`/`SDDEL`) which is not in the original plan. The hardware
recalibration of the Phase-1 desk constants (`BENCH` on silicon) stays open — it needs the
bench, not the keyboard.

## 1. Measurement execution

### 1.1 Where the events come from

`Measure::armPlan()` compiles a `Plan` per engine at arm time from the slot's `MeasDef`, the
train type and a `Geometry` struct that `Engine::startTrain` fills from its own arm-time
precomputation (stage boundaries `cum[]`, ramp sample counts, sine phase constants, the
player's `preloadCyc`). A plan is a short list of *points*, each with

- `atCyc[p]` — offset from `pulseStart` at which the ADC reads start,
- `label[p]` — the `<point>` field of `MSUM`/`MDATA`: the stage index for `S`/`L`, the peak
  phase in degrees (90/270) for `W`,
- `chMask[p]` — which channels this point reads,
- an accumulator triple (`n`, sum, sum of squares) per channel and line.

The player ISR takes `min(next latch deadline, next measurement deadline)` each pass, so
measurement events interleave with latch events on the same absolute-deadline machinery. No new
timer, no new priority.

### 1.2 The time budget, and why a point can be refused

One ADC value costs a control-register write (line select) plus a conversion. The window must
close before the next latch event starts *programming* the DAC, which happens `preloadCyc`
early. So a point needs

```
room = preloadCyc + budget + settle,   budget = nReads * (ADC_READ + ADC_SWITCH) + GUARD
```

and it is placed at `nextLatch - preloadCyc - budget`. `room` is compared against

- `S`: the stage duration,
- `L`: the stage's *sample interval* (stage duration / N) — ramp samples latch every
  `SJ_TARGET_DT_US`, so that, not the stage, is the free gap,
- `W`: the sine sample interval (`sampleCyc`); the reads are placed *after* the peak sample's
  latch (`settle` later), because the point of the measurement is the value that sample
  produced.

Points that do not fit are kept in the plan, marked in `skipMask`, never fired, and reported:
an `MSUM` line with `n = 0` and a `#` line naming the required and available times. This is
deliberate — the alternative (stretching the waveform to make room) would change the delivered
stimulus to suit the instrumentation.

With the current desk estimates a V+I measurement on both channels needs about 31 us of room,
so it fits `S` stages comfortably and does **not** fit an `L` train at the default 20 us ramp
interval, nor a `W` train above ~2 kHz. Remedies, in order of preference: measure one line
(`MEAS<i>,1,1,...`), measure one channel, use `S`, or raise `SJ_TARGET_DT_US`. A per-train ramp
interval and reads split across consecutive gaps are the documented follow-ups.

### 1.3 Sine peaks

The peak sample index is solved at arm time from the Q32 accumulator: for target phase `T`,
`k = ceil(((T - phaseInit) mod 2^32) / phaseInc)` is the first sample at or after the crossing.
`when` selects 90 degrees, 270 degrees or both; the phase restarts every burst, so the same `k`
applies to every burst and the offsets are constants of the plan.

The two channels can carry different frequencies and phases, in which case their peaks are at
different instants. Rather than doubling the event list, the plan solves the peaks for the
lower-numbered measured channel and warns when the other channel's frequency or phase differs.

### 1.4 Envelope gate

A measurement taken while the `ENV` envelope is ramping reads an attenuated waveform, and
averaging it together with full-amplitude repetitions produces a mean that describes neither.
So when an envelope is active, a point fires only where `env(t) == 1`; repetitions inside the
ramps are skipped and `n` reports what was actually accumulated. Without an envelope
(`env.on == false`) everything is measured — legacy behaviour is unchanged.

### 1.5 Numbers, not units, in the ISR

The ISR accumulates **raw ADC codes** as `n`, sum and sum of squares (integer adds only). The
channel offset is a float (`Stimjim.adcOffset10[]`) and the unit scale is a float, so both are
applied in loop context at summary time:

```
mean_phys = (sum/n - off) * PER_ADC          var_raw = (sumsq - sum^2/n) / (n-1)
sd_phys   = sqrt(var_raw) * PER_ADC          (the offset cancels in the spread)
```

### 1.6 MDATA has one consumer

Per-repetition records go into one SPSC ring. `Measure::poll()` is its only consumer: it prints
`MDATA` when `report & 1` and hands the row to `SdLog::writeRow()` when `report & 2`. Two
independent drains of one ring would each see half the records, so `SdLog::poll()` is left with
nothing but the flush timer. Ring overflow drops records and counts them; the count is reported
with the summary.

## 2. SD logging

`SdLog` uses the Teensy `SD` wrapper over the bundled SdFat (MIT) on `BUILTIN_SDCARD` (native
SDIO — never the DAC/ADC SPI bus). Only `loop()` touches the card. Present on Teensy 3.5/3.6
and 4.1; a Teensy 4.0 has no socket, so `SJ_USE_SD` defaults to 0 there and the whole group
answers `ERR ... no SD support in this build`.

- `LOG?` status, `LOG1[,name]` open (auto name `LOGnnnn.CSV`, lowest free index), `LOG0`
  close and flush.
- The file header carries the `IDN` line and the full session configuration as `#` comments —
  the same lines `DUMP` prints, so a log identifies the waveforms that produced it even when
  the train was started by a trigger edge. `handleDump` is refactored to write to a `Print&`
  for this.
- Rows: `<timestamp_us>,<slot>,<pulse>,<point>,<V0_mV>,<I0_uA>,<V1_mV>,<I1_uA>`, frozen in
  protocol §4. Flush on train end, every 64 rows, or every second.

## 3. Reading the card over the serial port

The socket is inaccessible with the cover on, so the card is served over USB:

| Command | Behaviour |
|---|---|
| `SD?` | list the group |
| `SDINFO` | card present, capacity, used, and the open log file |
| `SDLIST[,<dir>]` | one `SDLIST,<name>,<size>` line per entry, then `OK` |
| `SDGET,<name>[,<offset>[,<len>]]` | length-framed transfer |
| `SDDEL,<name>` | delete one file |

`SDGET` prints a header line `SDGET,<name>,<offset>,<len>,<total>`, then **exactly `<len>`
bytes verbatim**, then a newline, then `# crc32=<8 hex>`, then `OK`. The byte count in the
header is what makes the transfer unambiguous: log rows are CSV text, but a host must not have
to guess whether a payload line is data or protocol, and a length prefix settles it without an
escaping scheme. `len` defaults to the rest of the file, and the transfer loop calls
`cycles64()` every chunk so that a host which stops reading mid-transfer cannot stall the
64-bit timebase past its 35.8 s wrap.

Reading the log file that is currently open is the common case, so `SDGET` flushes first and
serves it through the open handle (save position, seek, read, restore) instead of opening a
second one.

## 4. Order of work

1. `Config.h`: measurement timing constants (`SJ_ADC_READ_US`, `SJ_ADC_SWITCH_US`,
   `SJ_MEAS_GUARD_US`, `SJ_DAC_SETTLE_US`), `SJ_USE_SD` per board.
2. `Measure.h/.cpp`: plan build (pure, host-testable), ISR fire path, accumulators, MDATA ring,
   `MSUM`/`MDATA` formatting, start notes.
3. `Engine.cpp`: `Geometry` fill, plan arm, the measurement branch of `playerRun`, plan reset on
   stop.
4. `SdLog.h/.cpp` plus `LOG` and the `SD` group in `Commands.cpp`; `handleDump(Print&)`.
5. `tests/host/test_measure.cpp`: fit arithmetic, stage selection, peak solving, summary math.
6. `tests/device/smoke.py`: measurement and SD checks over serial.
7. Docs: protocol §4 (record formats, the `SD` group, the fit rule), plan §6, hardware-variants
   (SD per board), progress entry 007.
