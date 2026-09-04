Status: implemented 2026-09-04; §6 (the silicon qualification that lowers `CAL STARTLAT`)
outstanding — the code change is complete and the default is unchanged until then.

# Plan — pre-armed players, a structural stage look-ahead, and a short-stage warning

This phase moves the whole of `Engine::startTrain` out of the trigger ISR. Each engine gets a
second `Player`, `loop()` fills it with everything a train's start does not depend on the start
*time*, and a trigger edge then only reads the clock, places `t0`, attaches the measurement plan,
swaps the live pointer and programs the timer. The estimate is a 2–4 µs arm against today's
9.0–15.0 µs, which would put `CAL STARTLAT` at the scheduling floor of
`PRELOAD + DACPROG2 + MIN_SCHEDULE` = 12 µs instead of at the arm's cost. Two smaller items ride
along: the per-stage look-ahead stops asking the clock how many stages it can afford and instead
derives the stage due next plus, for a 0-duration stage alone, the one behind it; and a serial
`S`/`L` line that asks for stage boundaries closer together than one latch costs now says so at
definition time instead of only through the per-latch deadline counters at train end.

Nothing here lowers `SJ_START_LATENCY_US`. The default is measured, not estimated, and this
machine has no board attached — the phase ends with the `BENCHARM` procedure that qualifies a
lower value and leaves the adoption to one `CAL,STARTLAT,<us>` line.

## 1. Why the floor cannot be cut term by term

`Engine::startTrain` costs 9.04 µs for a train that drives nothing, which at 120 MHz is 1085
cycles. What runs in it: the busy check, a `Cal::live()` copy, `buildGeometry`, the
channel-conflict check, about twenty `Player` field stores, `Measure::armPlan`, a dozen counter
resets, `envInit`, `t0`, two LED writes and `pitProgram`. That is on the order of 150–250
instructions, so the measured figure is 4–7 cycles per instruction — what a Cortex-M4 costs when
the work is 64-bit `SJ_US_TO_CYC` multiplies, struct copies through memory, and five PIT register
writes that cross the peripheral bus at F_BUS = 60 MHz.

There is no single fat term left; there are ten thin ones, and no instruction-level measurement
would change that conclusion. The original `stimjimPulser` was quicker to its first pulse for one
structural reason, not because it computed less: `startIT0` called `pulse0()` and latched inside
the trigger ISR. That is the right idea. This phase takes it, and does the conversion *before* the
edge rather than after it, which is what the original could not do.

## 2. Pre-armed players

### Storage and ownership

```
static Player playerBuf[2][2];     // [engine][buffer]
static Player* live[2];            // what the player ISR plays
static Player* warm[2];            // what loop() prepares; swapped in by the arm
```

`sizeof(Player)` grows by the geometry it now carries and there are four of them instead of two:
about +2.6 kB of RAM, against 211 kB free on a Teensy 3.5.

The invariant is one flag. `Player::armReady` is set only by `prepareArm()` in `loop()` and cleared
only by the arm that consumes the buffer, so a buffer that has ever played is never mistaken for a
prepared one — its counters, `evIdx`, `evPhase`, `nDerived` and ramp cursor are all mid-train
state. The tag beside it is `(armSlot, armDefEpoch, armCalEpoch)`, exactly the measurement plan's
tag from phase 12, which is what keeps a live slot edit effective: an edge whose warm buffer no
longer matches the current definition falls back to preparing it in place, so an edit is never
ignored and pre-arming costs nothing in correctness.

### What moves to `loop()`

Everything in today's `startTrain` between the conflict check and the `t0` assignment:
`buildGeometry` and the `Measure::Geometry` it fills (now stored in the `Player`, together with the
ramp sample counts, so the arm's fallback compile needs no stack storage), the type/mask/mode
fields, the park codes, the sine constants and amplitudes, `dtUs`, the `memcpy` of the stage
triplets, `deriveStage(0)`, the period/duration/preload/delay scalars, every counter reset, the
`evIdx`/`evPhase`/`prevEvDl` initialisation, and the envelope's shape.

Two `Cal` values are folded in at the same time — `startLatCyc` and `trigCompCyc` — because the tag
already carries `Cal::epoch()`. The fast arm therefore never reads the `Cal` set at all.

### What the edge ISR still does

1. `live[eng]->active` — the busy refusal.
2. Tag check against `(slot, TrainStore::epoch(), Cal::epoch())`; a miss calls `prepareArm` in
   place and pays what today's arm pays.
3. Channel-conflict refusal against the other engine.
4. `base` from the anchor, then `t0 = base − trigCompCyc + startLatCyc + delayCyc`, `pulseStart`,
   and `sampT` for a sine.
5. `SampleGen::envRebase(env, t0)` — four 64-bit adds. `envInit` splits into a t0-free `envShape`
   (which keeps the two FP32 reciprocals) and this, so no division happens after the edge.
6. Re-read the channel park codes from the live calibration tables. `ampToDelta` returns a delta
   that is independent of the offset except at saturation, so a recalibration between the
   preparation and the edge can only shift a code that was already out of range and WARNed at parse
   time; the park level itself, which the ISR does use, is always current.
7. `Measure::armPlan` with the stored geometry — one buffer swap in the warm case, unchanged in the
   cold one. No change to `Measure` is needed.
8. LEDs, `armReady = false`, swap `live`/`warm`, `active = true`, `pitProgram`, and the existing
   start-latency shortfall check.

### Expected cost

Two `cycles64()` reads, one `pitProgram`, and on the order of forty other instructions: **2–4 µs**,
flat across every train shape, which also removes the one case that exceeds the current default —
a `TRIG` independent route of two ten-stage `L` slots, which pays the arm twice and needs ~39 µs
today.

`BENCHARM` already separates the two paths without any change: its repetitions run back to back
inside one bus lock with no `loop()` pass between them, so they measure the cold path, and
`bench_arm.py`'s warmed column — one arm per invocation — measures the pre-armed one. That is the
column `CAL STARTLAT` is set from.

### Risks

- **A stale pending timer IRQ entering `playerRun` after a swap.** It reads `live[p]`, which is
  the freshly armed player, exactly as a single-buffer build reads the freshly overwritten one.
  Unchanged behaviour, and `pitStop` at ISR entry plus the `active` check still cover it.
- **`prepareArm` running while a train plays.** It writes only `warm[eng]`, which no *player* ISR
  reads. That is what makes back-to-back triggers cheap, and it is the same argument phase 12 made
  for the second measurement plan.
- **A trigger edge landing inside `prepareArm`.** This one is real, and it is what the
  implementation added over the plan: the edge ISR *does* read `warm[eng]`, so a preparation
  interrupted half-way would be resumed afterwards into a player that had meanwhile started
  playing. Masking the trigger ISR is not an option — it would delay the edge timestamp the whole
  latency is anchored to, invisibly — so a `volatile prepBusy[2]` handshake covers it exactly as
  `Measure`'s `loopBusy` does: an edge inside the window arms the live buffer in place and swaps
  nothing, and `prepareArms` reads the pointer only after raising the flag.
- **`timingFaults` after a manual stop** reads `live[eng]`, the buffer that played, which
  `prepareArm` never touches. The counters survive as they do today.

## 3. A structural stage look-ahead

`deriveAhead` currently derives as many stages as the clock says fit, and its justification is a
two-premise argument: the parse-time refusal of consecutive 0-duration `L` stages bounds the chain
at two, and one idle gap covers two derivations. The first premise is `L`-only — `TrainStore.cpp`
tests `type == PIECEWISE_RAMP` — and what the rule buys is ~1.6 µs on a latch that cannot be on
time anyway, because a 0-duration stage's sample shares its deadline with the previous stage's last
one and the second latch of a coincident pair is late by construction (`playerRun` excludes such
pairs from `overdueEvents`, and the 8–9 µs analog settling means the intermediate value never
reaches the output).

Replace it with what the definition says:

```
derive the stage due next; while the stage just derived has dur_us == 0, derive the next one too
```

A 0-duration stage is the only kind that yields no gap of its own, so it is the only kind whose
successor must already be there. The loop is bounded by `nStages`, needs no clock, and covers an
`S` chain of any length as well as the `L` pair. `SJ_STAGE_DERIVE_US` survives as a single entry
guard — do not start a derivation with less than that left before the wake-up — which is the one
clock reading the function keeps, and `ensureDerived` remains the correctness guarantee at every
use site.

## 4. Warn about stage boundaries closer than one latch

`buildGeometry` refuses a ramp sample interval below `PRELOAD + DACPROG* + MIN_SCHEDULE`, but
nothing checks stage durations. `rampStageN(3, 20)` is 1, and a stage with one sample puts it at the
stage *end*, so ten 3 µs `L` stages schedule ten latches 3 µs apart while one latch costs 12 µs.
Every one is late. An `S` train has no check at all.

Add a definition-only quantity to `TrainStore` (host-testable, no Arduino dependency):

```c
uint32_t minLatchGapUs(const TrainDef& t, uint32_t dtUs);
```

the shortest nonzero gap the definition puts between two consecutive latches — `dur_us` per stage
for `S`, `dur_us / rampStageN(dur_us, dtUs)` for `L`, `UINT32_MAX` for a sine or an undriven train.
Zero gaps are excluded: coincident latches are what an instant jump means. The `S`/`L`/`W` command
handler compares it with the live budget and appends a warning to the line it already prints. A
warning and not a refusal — the shortest deliverable stage is a runtime quantity, the deadline
counters already report the outcome, and nothing under about 10 µs settles at the output regardless.

## 5. Order of work

1. `SampleGen`: split `envInit` into `envShape` + `envRebase`, keep `envInit` as the wrapper, extend
   `tests/host/test_samplegen.cpp` with the equivalence.
2. `Engine`: `deriveAhead` structural; `Player` gains the arm tag, the geometry and the stage
   counts; `player[2]` becomes `playerBuf[2][2]` with `live`/`warm`; `prepareArm` extracted from
   `startTrain`; `warmPlans` renamed `prepareArms` and calling it.
3. `TrainStore::minLatchGapUs` + host test; the warning in `Commands.cpp`.
4. Four builds (`teensy35` register and portable, `teensy41`, `teensy40` with
   `SJ_EEPROM_SLOTS=6`), four host suites.
5. Docs: `timing.md` §1/§7 (the floor review, the pre-arm path, the new item), `Config.h`'s
   `SJ_STAGE_DERIVE_US` and `SJ_START_LATENCY_US` comments, `serial-protocol.md` (the one sentence
   on serial-start latency and the short-stage warning), `tests/device/README.md`.

## 6. What qualifies the result

On silicon, in this order:

```
python tests/device/bench_arm.py COM4 --csv tmp/arm-p14.csv     # the warmed column is the answer
python tests/device/stage_timing.py COM4                        # the look-ahead still fits
python tests/device/smoke.py COM4
python tests/device/startlat_trig.py COM4 <candidate>           # needs the bench AWG
```

`STARTLAT` = warmed arm + `PRELOAD` + `DACPROG2`, floored at 12 µs by `Cal::validate`. Only
`startlat_trig.py` can qualify it: with no anchor a `T`/`U` start is not charged for the arm, so
every candidate passes over the serial port alone (phase 13).
