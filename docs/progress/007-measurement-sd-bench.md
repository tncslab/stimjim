# Phase 7: measurement engine, SD logging with serial retrieval, bench constants

Date: 2026-09-03

This phase built the two remaining data-path features — in-train measurement (`MEAS` execution,
`MSUM`, `MDATA`) and SD logging (`LOG`) — added a file-access group so the card can be read back
over the serial port, and closed the Phase-6 debt of running the `BENCH` group on silicon. The
measured numbers replaced two estimated constants and the exercise found three defects, one of
them in this phase's own code. Plan: [PLAN_measurement-sd.md](../PLAN_measurement-sd.md)
(status: done).

**Done** (Teensy 3.5 on COM4 with a 29 GB card; host tests pass, `smoke.py` all-pass at 62
checks, `--warnings more` clean on Teensy 3.5 / 3.5-portable / 4.1 / 4.0):

1. **Measurement engine.** `Measure::planBuild` compiles a per-engine `Plan` at arm time from the
   slot's `MeasDef`, the train type and a `Geometry` struct the player fills from its own
   precomputation. A plan is a short ascending list of *points*, each with an offset from
   `pulseStart`, a label (stage index for `S`/`L`, peak degrees for `W`), a channel mask and an
   accumulator triple per channel and line. The player ISR takes
   `min(next latch deadline, next measurement deadline)` each pass, so measurement rides the
   existing absolute-deadline machinery: no new timer, no new priority, no new lock.

   The ISR accumulates **raw ADC codes only** (`n`, Σ, Σ²; integer adds). The channel offset and
   the unit scale are floats, so both are applied in loop context when `MSUM` is formatted —
   `sqrt` and `printf` never appear above priority 112.

2. **The window-fit rule, and refusing rather than squeezing.** A point needs
   `(preload + DAC programming) + nReads*(ADC_READ + ADC_SWITCH) + GUARD + SETTLE` of free gap.
   The free gap is the stage for `S`, but only one *ramp sample interval* for `L` (stage
   duration / N, 20 µs by default) and one sine sample interval for `W` (`1/Fs`, never shorter
   than 20 µs because Fs is capped at 50 kHz). A point that does not fit is kept, flagged, never
   fired, and reported twice — a `WARN MEAS:` at start naming the required and available times,
   and an `MSUM` line with `n = 0` plus a `#` reason. With the Teensy 3.5 constants
   (`PRELOAD` 4, `DAC_PROG1/2` 3/5, `ADC_READ` 3, `ADC_SWITCH` 4, `GUARD` 1, `SETTLE` 4, all µs):

   | What is measured | `nReads` | room | Fits `L` at 20 µs? | Highest `W` frequency |
   |---|---|---|---|---|
   | one line, one channel | 1 | 19 µs | yes | any |
   | V+I, one channel | 2 | 26 µs | no | 601 Hz |
   | one line, both channels | 2 | 28 µs | no | 558 Hz |
   | V+I, both channels | 4 | 42 µs | no | 372 Hz |

   This is the phase's main design decision and it goes against convenience: **an `L` train at
   the default 20 µs ramp interval can carry only a single value on a single channel, and a `W`
   train can be measured on both channels only below 558 Hz (372 Hz for V+I).** Stretching the
   ramp interval to fit the instrumentation would change the delivered stimulus, which is the one
   thing this firmware must not do quietly. The documented remedies are to measure fewer lines or
   channels, use `S`, or rebuild with a larger `SJ_TARGET_DT_US`; each was verified on hardware.

   The room figure depends on how many channels the train drives, because a one-channel train
   budgets `SJ_DAC_PROG1_US` rather than `SJ_DAC_PROG2_US` — which is why an `L` train with one
   measured value fits 19 µs into a 20 µs gap and one with two does not.

3. **Sine peaks solved at arm time.** `peakSampleIndex` gives the first sample at or after a
   phase target as `ceil(((target − phaseInit) mod 2^32) / phaseInc)`, so 90°/270° become sample
   indices and, because the phase restarts every burst, constants of the plan. The reads are
   placed `SETTLE` *after* the peak sample latches — that sample is the value being measured.
   When both channels are measured with different frequencies or phases the peaks follow the
   lower-numbered channel and say so, rather than doubling the event list.

4. **The envelope gates measurement.** With a non-zero `ENV`, points fire only where the envelope
   is fully on; skipped repetitions are counted and reported. Averaging attenuated readings
   together with full-amplitude ones produces a mean that describes neither. Without an envelope
   nothing changes, so legacy averaging behaviour is untouched.

5. **SD logging, and the card served over serial** (the user's requirement: the socket is under
   the cover, so nothing written there may depend on opening the case). `LOG?`/`LOG1[,name]`/
   `LOG0`, plus `SDINFO`/`SDLIST`/`SDGET`/`SDDEL`. `SDGET` frames its payload with a byte count
   and follows it with a CRC-32 of the bytes actually sent, so arbitrary file content can never
   be mistaken for protocol and a chunked download is verifiable piece by piece. Reading the log
   file while it is still open works — it is flushed and served through the same handle.

   The log is self-describing: the identity block and the whole `DUMP` configuration at open, the
   column names, then a `# train:` block for every train that arms while the file is open. That
   last part is what makes a log usable when the trains were fired by trigger edges with no host
   attached.

6. **`BENCH` on silicon** (Phase-6 debt). Teensy 3.5 at 120 MHz, register backends, n = 2000 per
   bench, nothing else running:

   | Bench | min / avg / max | What it sizes |
   |---|---|---|
   | `BENCHDAC` | 1.41 / 1.43 / 2.88 µs | `SJ_DAC_PROG1_US` = 3 |
   | `BENCHDAC2` | 2.74 / 2.75 / 4.23 µs | `SJ_DAC_PROG2_US` = 5 |
   | `BENCHLATCH` | 0.44 / 0.44 / 1.93 µs | the NLDAC pulse itself |
   | `BENCHADC` | 2.21 / 2.22 / 3.81 µs | `SJ_ADC_READ_US` = 3 |
   | `BENCHSW` | 4.53 / 4.58 / 6.10 µs | `SJ_ADC_READ_US + SJ_ADC_SWITCH_US` = 7 |
   | `BENCHMISO` | 2.27 / 2.29 / 3.29 µs | the PORT-mux swap costs nothing |
   | `BENCHCYC` | 0.27 µs | `cycles64()` overhead |
   | `BENCHK` | residual −23 / −16 / −12 cycles | `K_RELOAD` spread ≈ 11 cycles (92 ns) |
   | `BENCHPIT,1000,2000` | 0.37 / 0.41 / 0.58 µs late | raw PIT wake latency |
   | `BENCHPIT,1000,2000,4` | 0.117 / 0.133 / 0.158 µs late | **residual latch jitter** |

   The two headline results: `SJ_PRELOAD_US` = 4 is seven times the worst observed wake latency,
   and **residual latch jitter is a 42 ns spread** against the design target of < 200 ns. Boot
   `K_RELOAD` values ranged 84–98 cycles across the session's reflashes, which is exactly why it
   is calibrated closed-loop at every boot rather than written down.

**Defects found by running it, and what they say**

- **`BENCHDAC2` never reached the dual-channel bench.** The command word is the leading
  *alphabetic* run of a line, which stops in front of the `2`, so `BENCHDAC2,2000` ran `BENCHDAC`
  with a repetition count of 2. It had been wrong since Phase 1 and was invisible except in the
  reply's own `n=` field. `BENCH` sub-commands are now scanned across digits too.
- **The plan build sat on the wrong side of `t0`.** I put `Measure::armPlan` after the
  `t0 = now + START_LATENCY` assignment, so ~1 KB of plan zeroing and precomputation was
  subtracted from the 20 µs start latency — and the first latch of *every measured train* was
  ~1.5 µs late. The existing comment at that line already said precomputation must precede `t0`;
  I had read it and still inserted below it.
- **The stale "MDATA streaming is deferred" warning** in `validateMeas` now contradicted the
  firmware and was removed.

**The counter that found the second one.** `progLatch` now compares each latch against its own
deadline, and `playerRun` separately notes events that were already due on arrival. Two counters,
two different fixes:

- `WARN engine: <n> latch(es) overran their deadline by up to <t> ns` — programming for a
  still-future event finished late. A defect: a `Config.h` budget is too small for this board.
- `# engine: <n> event(s) were already due when the player reached them` — an earlier event ran
  long.

Events that **share** a deadline by definition are excluded from both, because that is the
waveform's shape and not a fault: an `L` train latches its last ramp sample and then parks at the
same stage boundary, and a `W` burst with `burst_us == period_us` puts the off event on top of a
sample. Getting that exclusion right took two iterations — the first version reported 49 faults
for a continuous sine and 300 for a perfectly healthy `L` train, which is how a diagnostic
teaches people to ignore it.

This is worth keeping for a reason beyond this phase: it makes the whole timing design
**acceptance-testable over the serial port alone**, with no oscilloscope. That is the cheapest
possible regression test for the Teensy 4 and portable backends, which still have not run on
silicon.

**Constants changed by measurement**

| Constant | Was | Now | Why |
|---|---|---|---|
| `SJ_ADC_READ_US` | 2 (estimate) | 3 | `BENCHADC` max 3.81 µs; budgets carry the maximum, not the average |
| `SJ_ADC_SWITCH_US` | 3 (estimate) | 4 | `BENCHSW` max 6.10 µs for select+read, so 7 µs per read covers it outright |
| `SJ_DAC_SETTLE_US` | 2 (new, estimate) | 4 | the player's own post-latch bookkeeping costs ~3.2 µs, so a smaller value scheduled reads at an instant the ISR cannot reach |

`SJ_DAC_PROG1_US` (3) and `SJ_DAC_PROG2_US` (5) were left alone: measured maxima are 2.88 and
4.23 µs, so the estimates were already correct as budgets. `SJ_PRELOAD_US` (4) was left alone
too — lowering it would shrink the measurement window's `room`, but it is the margin that absorbs
outliers, and there is no benefit worth spending it on.

**Decisions taken in-phase**

- *Budgets carry maxima, not averages.* A 1-in-900 outlying ADC read is enough to push a latch
  late, and one late latch is a stimulus artefact. The cost is measurable — raising the ADC
  budget from 20 to 29 µs is what put `W` peak measurement at 500 Hz out of reach on both
  channels — and it is the right way round for a stimulator.
- *One consumer for the MDATA ring.* `Measure::poll()` both prints `MDATA` and hands rows to
  `SdLog::writeRow`. Two independent drains of one SPSC ring would each see half the records.
  `SdLog::poll()` is left with nothing but the flush timer.
- *`SDINFO`'s used-space field is opt-in.* `SD.usedSize()` walks the whole free-cluster chain:
  seconds on a 29 GB card, during which `loop()` drains no `MDATA` and redraws nothing. Plain
  `SDINFO` reports `-1` in that field and `SDINFO,1` scans, refused while a train runs. The field
  stays in place either way, so the record shape does not depend on the argument.
- *A manual stop still prints its summary*, since `T-1` pushes no completion record and the
  accumulators would otherwise be thrown away. Verified: 89 repetitions summarised from an
  interrupted train.
- *The log header freezes at open time*, so the per-train `# train:` block carries anything
  changed afterwards. Writing the configuration at each train start is the only option that works
  for trigger-started trains, because an edge ISR cannot touch the card.
- *`sjcon.cmd1` now skips `#` comments*, which is the machine-parse rule protocol §1 states. It
  was `SDINFO`'s explanatory note that exposed the gap.

**Verified on hardware.** Everything below was checked over the serial port alone — the new
per-latch counters replace what a scope was needed for in Phase 6, and no oscilloscope was
connected. The StimJim outputs still drove the Phase-6 load chain, which is what the millivolt
readings describe:

```
StimJim CH0(+) --1k-- ... the chain, from CH1(+) round to CH1(-):
  A(CH1+) --1k-- B(CH0+) --1k-- C(CH0-) --[two antiparallel LEDs]-- D(CH1-)
```

A programmed 5 V on CH0 therefore reads back around 4.49 V, because the antiparallel LED pair
clamps at its ~2.2 V forward drop and the two 1 k resistors divide what is left; a programmed
1000 µA on CH1 reads back 988–1103 µA. **Amplitude accuracy is not what these checks test** —
what matters is that the readings are stable (spreads of 2–8 mV and < 1 µA over 300
repetitions), correctly signed per stage, and taken at the right instants.

| Case | Result |
|---|---|
| `S`, V+I on both channels, 2 ms stages, 300 pulses | 2 `MSUM` lines, n = 300, zero timing faults |
| `S` with `MEAS` report bit 0 | one `MDATA` line per point per pulse, 8 fields |
| `S` with a 20 µs stage | refused with the arithmetic, `MSUM … n=0` plus the reason |
| `L` ramp, one line on one channel | measured; stage-end values 4095 mV and 33 mV — the ramp's endpoints |
| `L` ramp, V+I on both channels | refused (20 µs gap against 42 µs needed), as designed |
| `W` 500 Hz gapped burst, both peaks, one line | 90° → +2555 mV, 270° → −2612 mV |
| `W` 2 kHz continuous burst | zero faults after the coincident-deadline exclusion |
| `T-1` mid-train | summary printed from 89 accumulated repetitions |
| SD log → `SDGET` → host CRC-32 | byte-exact, 914 and 1171 byte files, chunked reads verified |

**Open questions / still owed**

- **Dual-channel collision jitter and the published `FsMax` table** — the displaced Phase-6 item,
  still displaced. A single unmeasured sine train keeps every deadline to Fs = 49.9 kHz, so
  `SJ_FS_MAX_HZ = 50000` is not obviously wrong, but two independent trains contending for the
  SPI bus have never been measured and that is what the table needs.
- **Long-run drift** (≤ 1 µs cumulative over a 10 s train) and the full trigger-latency battery.
- **Bench-verify item 2** (AD5752 settling vs NLDAC on a scope) is the one number still guessed.
  `SJ_DAC_SETTLE_US = 4` is set by the player's bookkeeping, not by the DAC; if the DAC needs
  more, in-train readings are taken too early and nothing currently would say so.
- **Item 4** (the real GPIO header pins; `Stimjim.h:52-62` defines every `GPIO_x` as pin 36) is
  untouched and still needs the PCB netlist.
- **Measurement coverage for `L` and fast `W`**: a per-train ramp interval, or splitting a
  point's reads across consecutive sample gaps (the fallback plan §7 anticipated). Neither built.
- **Neither Teensy 4 nor the portable backends have run on silicon.** They now have a serial-only
  acceptance test (the timing-fault counters), which makes qualifying them much cheaper than it
  was.
- The `MDATA` ring overflow path is coded and counted but has not been provoked; a train fast
  enough to overrun 128 records needs the coverage work above first.

**Next entry point:** the bench session for dual-channel collision jitter, the measured `FsMax`
table and long-run drift — everything that still needs the scope. After that, the button menu
editor is the last unbuilt item in the plan.
