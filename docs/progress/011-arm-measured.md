# 011 — the arm measured on silicon

The phase-10 firmware was built for a Teensy 3.5, flashed, and `Engine::startTrain` measured with
`BENCHARM` across every waveform shape, every stage count from 1 to 10, and every measurement-plan
size from 1 to 10 points, on both halves of the `SJ_CODE_IN_RAM` A/B. The arm fell from the
16.2–37.5 µs of phase 9 to 8.45–28.5 µs on a re-arm — a `W` sine arm fell 37.5 → 10.1 µs and a
ten-stage `L` train, never measured before and projected at ~61 µs, measures 28.5 µs. Three things
came out that the plan had not predicted. The cost of the *first* arm after a slot edit separated
cleanly from every arm after it, because the plan tag added in phase 10 works: the difference is
the measurement-plan compile, and it costs 3.1 µs per point, which makes a ten-point plan's first
arm 54.3 µs against that same train's 23.6 µs re-arm — the largest single term in the arm and above
what `CAL STARTLAT` = 60 µs covers (that case needs 66 µs). `SJ_CODE_IN_RAM` is worth 13–17 %, not
the 30–50 % the flash-wait-state hypothesis implied, and the saving tracks function size rather
than call count: the two multi-kilobyte functions gain 1.4–4.3 µs while moving the 180-byte
`rampStageInit` into RAM as well gained 0.33 µs over ten stages. And the 8 µs goal the whole
arm-cost plan was written against is unreachable from inside the arm, because an arm that drives
nothing already costs 8.45 µs. `CAL STARTLAT` therefore stays at 60 µs, and the next lever is
moving the plan compile out of the arm rather than making anything in it faster.

## What was run

Hardware: StimJim on a Teensy 3.5 (K64, 120 MHz, 120 cycles/µs), register backends
(`fastio=registers`, `timer=raw-PIT`), SD card mounted, on COM4. Firmware built with the Arduino
IDE's bundled `arduino-cli` at
`C:\Users\stipp\AppData\Local\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe`
(it is not on `PATH`; the VS Code Arduino extension ships a second copy):

```
arduino-cli --config-file tmp/arduino-cli.yaml compile --fqbn teensy:avr:teensy35 \
    --warnings more --build-path tmp/b10-t35 stimjimAWG
arduino-cli --config-file tmp/arduino-cli.yaml upload  --fqbn teensy:avr:teensy35 \
    -p COM4 --input-dir tmp/b10-t35 stimjimAWG
```

and the flash-resident half with
`--build-property "build.flags.defs=-D__MK64FX512__ -DTEENSYDUINO=160 -DSJ_CODE_IN_RAM=0"` into
`tmp/b10-t35-flash`. `IDN` reports `hot=RAM` or `hot=flash`, which is how a result is attributed to
a binary.

Measurements: `tests/device/bench_arm.py` (the eight-case sweep, written in phase 10 and run here
for the first time), plus two throwaway sweeps kept in `tmp/` — `tmp/stage_sweep.py` for arm cost
against stage count and `tmp/meas_sweep.py` for arm cost against measurement-point count. Raw
tables in `tmp/arm-p10-ram.csv`, `tmp/arm-p10-flash.csv`, `tmp/arm-p10-ram2.csv`.

`BENCHARM` re-arms the same slot n times inside one `busLock`, so with `n = 200` the *minimum* is
the steady-state re-arm and the *maximum* is the first arm — the one that misses the plan tag. That
turned out to be the most useful property of the harness and it was not designed in; it was
confirmed by running `BENCHARM,<slot>,1` twice in a row with a slot write in between, which
reproduces the max and then the min exactly (8458 ns → 8458 ns, write, 11292 ns, then 8450 ns).

## Results

Arm cost, `hot=RAM`, µs:

| Case | re-arm | first arm |
|---|---|---|
| undriven, 0 stages | 8.45 | 11.1 |
| `S`, 1 ch, 1 stage | 9.58 | 12.5 |
| `S`, 2 ch, 1 stage | 9.88 | 12.8 |
| `S`, 2 ch, 1 stage, `MEAS` V+I both | 10.62 | 18.0 |
| `S`, 2 ch, 10 stages | 17.88 | 20.9 |
| `S`, 2 ch, 10 stages, `MEAS` V+I every stage | 23.58 | 54.3 |
| `L`, 2 ch, 1 stage | 11.20 | 14.2 |
| `L`, 2 ch, 10 stages | 28.46 | 31.4 |
| `W`, 2 ch | 10.12 | 13.1 |

The stage and point sweeps are linear to within the 8.3 ns resolution of the cycle counter, so the
whole table reduces to a floor and four marginal costs:

| Term | Cost |
|---|---|
| floor (gatekeeping, `CAL` copy, train-level scalars, `envInit`, `t0`, `pitProgram`) | 8.45 µs |
| per `S` stage | 0.89 µs |
| per `L` stage | 1.93 µs (0.89 shared with `S`, 1.04 in `rampStageInit`) |
| per measurement point, every arm | 0.55 µs |
| per measurement point, first arm only | 3.1 µs |

`SJ_CODE_IN_RAM` A/B, re-arm figures, µs:

| Case | `hot=RAM` | `hot=flash` | saved |
|---|---|---|---|
| undriven | 8.45 | 9.88 | 1.43 |
| `S` 2 ch 1 stage | 9.88 | 11.92 | 2.04 |
| `S` 2 ch 10 stages | 17.88 | 21.04 | 3.16 |
| `L` 2 ch 1 stage | 11.38 | 13.75 | 2.37 |
| `L` 2 ch 10 stages | 28.79 | 33.04 | 4.25 |
| `W` 2 ch | 10.12 | 12.29 | 2.17 |

## Decisions

**`CAL STARTLAT` stays at 60 µs.** The plan for this work said it would be lowered to what the
measurements support. They do not support lowering it: the worst measured arm, the first start of
a ten-stage train measuring V and I on every stage, is 54.3 µs and needs `54.3 + 12 = 66 µs`. An
independent two-engine `TRIG` route of a ten-stage `L` slot needs 75 µs on its first arm. Both are
first-arm-only and both disappear once the plan compile moves out of the arm; until then, raising
the default would be as wrong as lowering it, because every steady-state case fits inside 45 µs and
the two that do not are exactly the ones the next phase removes. The firmware already reports the
shortfall rather than hiding it — a train whose arm did not fit names the `STARTLAT` it needed in
its completion (`startNeedUs`).

**`SampleGen::rampStageInit` and `rampStep` are marked `SJ_HOT`.** This was the one code change of
the phase, made after the first bench run and measured on its own: 296 B of RAM for 0.33 µs across
ten ramp stages. Small, but it removes a call from RAM into flash on the arm's hot path, and
`rampStep` shares the file and runs once per ramp sample inside the player ISR, where it was not
measured and where the same argument applies more strongly. Kept on those grounds, not on the
0.33 µs. `SampleGen.cpp` includes `Config.h` only under `#ifdef ARDUINO` and defines `SJ_HOT` empty
otherwise, because `Config.h` pulls in `Arduino.h` and the host tests compile that file alone.

**The 8 µs goal is retired.** The arm-cost plan opened with "the arm has to fit in 8 µs" so that a
trigger-started train never needs a wider budget than the waveform's own sample interval. An arm
that drives nothing costs 8.45 µs, so no amount of moving work out of `startTrain` gets there. The
route to a single-digit latency is the pre-armed one in `timing.md` §3 — arm before the edge, and
let the edge ISR pulse `NLDAC` on an already-programmed DAC — which makes the arm's cost irrelevant
instead of smaller.

## Open questions and where the next phase starts

Ranked by what the measurements say each is worth:

1. **Compile the measurement plan outside the arm** — 3.1 µs per point, and the reason the worst
   arm is 54.3 µs instead of 23.6. The plan depends on `(slot, definition, CAL)` and nothing the arm
   learns, so `loop()` can compile it as soon as a slot or the `CAL` set is written. What it needs
   is the train geometry — `cum[]`, the ramp `N` per stage, the sine constants — computed outside
   `startTrain`, i.e. a `buildGeometry(slot, geo)` factored out of it. That collapses the two
   columns of the results table into one and lets `STARTLAT` go to ~45 µs.
2. **Zero the accumulators outside the arm** — 0.55 µs per point, 5.7 µs on a ten-point plan. No
   memory operation makes this free: 96 bytes is 24 SRAM words, the generic `memset` takes ~66
   cycles for them, and a hand-rolled `STMIA` clear would at best halve that. Two result buffers per
   engine, arm on the clean one, clear the retired one in `loop()` after `printSummary` has read it,
   and fall back to the in-arm clear when both are dirty — the fallback keeps the worst case at
   today's and never loses a measurement, which refusing to measure would. It also removes the
   reason the reset sits in the arm today (a re-arm that beats `loop()` to the drain would otherwise
   have its own accumulators wiped by that drain) and leaves the previous train's statistics
   readable after the next start.
3. **Derive stage i+1 during stage i** — 0.89 µs per `S` stage, 1.93 µs per `L` stage, up to 17.4 µs
   on a ten-stage `L` train, and the only way the arm's cost stops depending on stage count. What
   moves is the two `ampToCode` conversions and, for `L`, `rampStageInit`. What cannot move while
   item 1 is unbuilt is `cum[]` and the per-stage `N`, because the plan compile reads them — which
   is why item 1 comes first. The risk is that the derivation lands in the tail of a stage-boundary
   ISR pass and has to be shown to fit the sample interval; the arm-cost plan's prerequisite (no two
   consecutive 0-duration `L` stages) bounds the look-ahead to one stage.
4. **`CAL TRIGCOMP`** is still 0 and unmeasured — the pin-to-ISR-entry delay, the one term of the
   delivered latency software cannot see. `docs/bench-wiring.md` configuration B measures it.

Nothing on the list is a correctness question. Every case above plays correctly today; what a late
first arm costs is a first latch that lands after `t0` and a completion that says by how much.
