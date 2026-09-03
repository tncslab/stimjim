# 013 — the stage derivation moved into the player

`Engine::startTrain` used to convert every stage of a train before it took `t0`, and after phase 12
that was the whole of what was left in the arm above its floor: 0.89 µs for each `S` stage and
1.93 µs for each `L` stage, so a ten-stage `L` train armed in 28.7 µs and set `CAL STARTLAT` = 45 µs
on its own. This phase moved the work into the player. The arm now derives stage 0 — the only stage
whose DAC codes are due at `t0` — and the player derives stage i+1 in the gap before a latch it is
already waiting for. Measured on the board: a ten-stage `L` arm fell **28.7 → 15.0 µs** and a
ten-stage `S` arm **17.5 → 13.3 µs**, so nine extra stages now cost 2.8 µs and 2.2 µs instead of
17.4 and 8.0. The six trains that stress the stage machinery — including ten `L` stages at the 12 µs
sample-interval floor and a chain of alternating 0-duration jumps — all run with no late and no
overdue latch, and their measured output matches the pre-change firmware to within 7.3 mV against
per-point standard deviations of 2–10 mV. `CAL STARTLAT` drops **45 → 35 µs**, the second successive
reduction. The phase also found that no `T`/`U` start can qualify a start latency at all, which is
why a new harness drives real trigger edges from the bench signal generator.

## What was built

**`Engine::deriveStage(pl, i)`** (`SJ_HOT`) — stage i's two DAC-code deltas and, for a ramp, the
Bresenham constants that carry it from stage i−1's end value. Strictly in order, because a ramp
stage starts where its predecessor finished. `Player::nDerived` is the invariant: stages below it
are derived.

- **The arm** derives stage 0 and nothing else, then copies the stage triplets it will need later.
- **`deriveAhead`** tops `nDerived` up on the one path in `playerRun` that programs the timer and
  returns — idle time by construction, since the ISR's next act is to wait for that instant. It only
  runs while the gap is wider than `SJ_STAGE_DERIVE_US` (4 µs, a new `Config.h` constant), and it
  derives as many stages as fit rather than exactly one.
- **`ensureDerived`** at each use site is the guarantee, so correctness never depends on the
  look-ahead having run. The call sites sit *after* a latch — in the `L` path between the last
  sample of a stage and the entry to the next, in the `S` path right after a stage's latch, for the
  following stage — so a derivation that does fall back spends the interval to the next latch and
  never a preload window.

Deriving *as many as fit* is what covers the 0-duration `L` jump without a special case. Such a
stage's single sample shares a deadline with the previous stage's last one, so entering it yields no
gap of its own and the stage after it has to be ready already. Two 0-duration stages in a row are
refused at parse time (phase 10), so that chain is at most two long and one idle gap always covers
it.

**The player carries a 120-byte copy of the stage triplets** (`StageDef stages[10]`, +336 B of RAM
for both engines including alignment). It must not read `TrainStore` while a train runs: a slot
write is refused only for the slot a train is *attached to*, and that check is made in `loop()`
while a trigger edge can arm from an ISR. Copy-on-arm was already the rule for everything else the
player reads; this extends it to the one thing the arm used to consume immediately.

**`ampToCode` became `ampToDelta(amp, mode, off)`.** Every caller subtracted the channel's park code
from the result, and the park code *is* the offset `ampToCode` added, so the two cancel except at
saturation. Taking `off` as an argument makes the conversion a pure function and keeps the ISR path
free of `Stimjim`'s live calibration tables. The expression is unchanged, so every DAC code is.

**`CAL STARTLAT` 45 → 35 µs.** The worst warmed arm is 15.0 µs and needs
15.0 + `PRELOAD` + `DACPROG2` = 24; the worst cold arm — an engine re-triggered with `loop()`
starved, compiling its own ten-point plan — is 20.0 µs and needs 29. 35 covers both. As in phase 12
the EEPROM version is deliberately not bumped: a stored 45 or 60 is conservative rather than wrong,
and rejecting the image would destroy the operator's slots 0–9 to deliver an improvement that
`CAL,STARTLAT,35` and `P` adopt.

**Two harnesses, both kept:**

- `tests/device/stage_timing.py` — the six trains above, each run to completion and checked against
  the firmware's per-latch deadline counters, printing every case's `MSUM` rows so two builds can be
  diffed for changed DAC codes. Serial only.
- `tests/device/startlat_trig.py` — puts real trigger edges on the two heaviest slots at a candidate
  `CAL STARTLAT` and reads the firmware's verdict. Needs the bench AWG.

## Measurements

Teensy 3.5, 120 MHz, register backends, `hot=RAM`, `tests/device/bench_arm.py`, warmed column
(one arm per `loop()` pass, which is what a running board delivers):

| Case | phase 12 | phase 13 |
|---|---|---|
| undriven, 0 stages | 8.83 | 9.04 |
| `S`, 1 ch, 1 stage | 10.17 | 10.79 |
| `S`, 2 ch, 1 stage | 10.42 | 11.12 |
| `S`, 2 ch, 1 stage, `MEAS` V+I both | 10.42 | 11.12 |
| `S`, 2 ch, 10 stages | 17.46 | **13.29** |
| `S`, 2 ch, 10 stages, `MEAS` on every stage | 17.46 | **13.29** |
| `L`, 2 ch, 1 stage | 11.75 | 12.17 |
| `L`, 2 ch, 10 stages | **28.67** | **15.00** |
| `W`, 2 ch | 10.83 | 10.96 |

Per-stage cost after the change: 0.25 µs for `S`, 0.31 µs for `L`. That is `cum[i+1]` and, for a
ramp, the sample-count division — the two things the measurement plan is compiled against, which is
why they stay in the arm. `Engine::buildGeometry` is shared with the `loop()`-side plan warm, and
the plan tag's guarantee that a warmed plan describes the geometry the arm computes rests on there
being exactly one copy of that function.

**The floor rose 0.2–0.6 µs on every train, including one with no stages at all**, and the reason is
worth recording because it is the same one phase 12 hit. Nothing was added to those paths: the
`Player` grew by 128 bytes, which moves `player[1]`'s fields past some of the addressing modes the
compiler was using, and `deriveStage` is a `SJ_HOT` (therefore `noinline`) call where the two
conversions used to be inlined into `startTrain`. 0.5 µs paid once against 1.6 µs a stage saved nine
times over is not worth chasing.

**Timely execution, which is where the arm's saving had to be paid for.**
`tests/device/stage_timing.py` on both firmware builds:

| Case | phase 12 | phase 13 |
|---|---|---|
| `S` 10 stages, 1 ms apart, measured | clean | clean |
| `S` 10 stages, 12 µs apart (the closest a two-channel latch schedules) | clean | clean |
| `L` 10 stages, `dt` = 20 µs (default) | clean | clean |
| `L` 10 stages, `dt` = 12 µs (the floor, where samples run inline and the look-ahead never gets a gap) | clean | clean |
| `L` alternating 0-duration jump chain | clean | clean |
| `L` 10 stages, `dt` = 50 µs, measured | clean | clean |

"clean" means `lateEvents` and `overdueEvents` both zero on a five-pulse train. The two measured
cases' `MSUM` rows agree between the builds to within **7.3 mV** on any point, against per-point
standard deviations of 2–10 mV — the derivation produces the same codes, just later.

**`CAL STARTLAT` verified by bisection under real trigger edges** (`startlat_trig.py`, AWG at 1 Hz
into IN0). 35, 30, 27 and 25 µs all run clean on both heavy slots. At 23 the ten-stage `L` train
reports `set CAL STARTLAT >= 24 us` on every trigger, and at 21 the ten-point measured train joins
it with `>= 22`. Both figures are the `BENCHARM` warmed arm plus `PRELOAD + DACPROG2` to the
microsecond (15.00 + 9 = 24, 13.29 + 9 = 22), so the arithmetic and the silicon agree.

## Two things the phase found that were not on the plan

**A `T`/`U` start cannot test a start latency.** When no anchor is passed, `startTrain` reads its
own clock at the *end*, so `t0` is one `STARTLAT` after the arm finished and a serial start is never
charged for the arm. `CAL,STARTLAT,21` and `T` on the worst train reports nothing wrong; the same
value with a trigger edge reports a fault on every one. That is the anchor working as designed — the
point of it is that a trigger's latency does not depend on how long arming took — but it means the
first four attempts at qualifying 35 µs proved nothing, and that any future `STARTLAT` change needs
the signal generator, not just the serial port. `startlat_trig.py` exists for that reason and says
so in its own docstring.

**A measurement point does not fit a default ramp.** `S`/`L` modes 0 and 1 mean "measure"; 90 and 91
are the same drive without it. A ten-stage `L` train written with modes 0,0 therefore asks for a V+I
point on both channels inside every 20 µs sample interval, which needs 26 µs, and the firmware
refuses all ten points with a `WARN MEAS` line apiece. That is correct behaviour and it is
documented, but it made the first version of `stage_timing.py` measure nothing on three of its
cases. The script now uses 90/91 for the timing-only cases and `dt` = 50 µs for the measured ramp.

## Verification

- Host suites `test_cal`, `test_measure`, `test_samplegen`, `test_trainstore` — all pass.
- `tests/device/smoke.py` — ALL CHECKS PASSED.
- `tests/device/stage_timing.py` — no timing faults, six cases.
- `tests/device/startlat_trig.py COM4 35` — every candidate fits; the bisection above locates the
  break point.
- `tests/device/bench_arm.py` — the table above, reproduced twice.
- Builds, all clean at `--warnings more`: Teensy 3.5 register (170 892 B flash, 50 436 B RAM),
  Teensy 3.5 portable (172 064 B, 50 692 B), Teensy 4.1, Teensy 4.0 with `SJ_EEPROM_SLOTS=6`.

## Next entry point

The arm is finished as a target. Its floor is 9.0 µs for a train that drives nothing, the per-stage
term is 0.3 µs, and in-train measurement costs it nothing — there is no term left inside
`startTrain` worth a phase. What remains, in order:

- **`CAL TRIGCOMP` is still 0 and unmeasured** — the pin-edge-to-ISR-entry delay, the one part of
  the delivered latency software cannot see. `docs/bench-wiring.md` configuration B is the
  procedure, and the harness this phase built is most of what it needs: `startlat_trig.py` already
  drives the AWG and routes a trigger; what it does not do is capture the two edges.
- **Pre-arm the train and fire on a bare `NLDAC` pulse** (`docs/timing.md` §3) — the only route to
  a single-digit total trigger latency, because the 9.0 µs floor puts everything else out of reach.
  It makes the arm's cost irrelevant rather than smaller.
- **The menu UI's editing FSM**, still the last unbuilt item of `firmware-spec.md`.
