# Plan — derive stage i+1 while stage i plays

Status: done, 2026-09-03. Built and measured on silicon: the ten-stage `L` arm fell 28.7 → 15.0 µs
and the ten-stage `S` arm 17.5 → 13.3 µs, the six trains of `tests/device/stage_timing.py` all
complete with no late and no overdue latch, and `CAL STARTLAT` drops 45 → 35 µs.

`Engine::startTrain` still converts every stage of a train before it takes `t0`, and that is the
whole of what is left in the arm above its 8.8 µs floor: 0.89 µs for each `S` stage and 1.93 µs for
each `L` stage, so a ten-stage `L` train arms in 28.7 µs and sets `CAL STARTLAT` = 45 µs on its own.
The work is per-stage but it is not needed per-arm: stage i's DAC codes and Bresenham constants are
first read when stage i is entered, which for i > 0 is at least one stage duration after the first
latch. This plan moves that derivation out of the arm and into the player, so the arm derives
exactly one stage — stage 0, the only one due at `t0` — and the player derives stage i+1 in the idle
gap before a latch it is already waiting for. The arm's cost then stops depending on stage count.

This is item 6 of [PLAN_arm-cost.md](PLAN_arm-cost.md), unblocked by phase 12: the measurement plan
is compiled in `loop()` now, so the arm no longer has to leave `cum[]` and the per-stage sample
count behind for it.

## What moves, and what does not

Per stage, `startTrain` does three things today:

| Work | Cost | Moves? |
|---|---|---|
| `ampToCode(a0)`, `ampToCode(a1)` → `d0[i]`, `d1[i]` | ~0.6 µs/stage | yes |
| `SampleGen::rampStageInit` → `rst[i]` (`L` only) | 1.04 µs/stage | yes |
| `cum[i+1]` and `stageN[i]` in `buildGeometry` | ~0.3 µs/stage | no |

`cum[]` and `stageN[]` stay because `buildGeometry` is shared with the `loop()`-side plan warm,
which needs both arrays whole and pays nothing for them there. Making them lazy would split that
function in two for ~0.3 µs per stage, against 1.6 µs per stage for the two that move.

Nothing about the *values* changes: the same expressions produce the same DAC codes and the same
Bresenham constants, just later. `ampToCode` adds the channel's calibration offset and `startTrain`
immediately subtracts `pl.off0`/`pl.off1`, which is the same offset — so the derivation is rewritten
as one function of `(amp, mode, off)` and reads no global state at all. That is what makes it safe
to run in ISR context.

## Mechanism

**The player carries the stage definitions.** `Player` gains `StageDef stages[SJ_MAX_STAGES]`
(120 B) and `dtUs`, copied in the arm with one `memcpy` of `nStages × 12` bytes. Copy-on-arm is an
existing invariant — the player must not read `TrainStore` while a train runs, because a slot write
is only refused for the slot a train is *attached to*, and that check is made in `loop()` while a
trigger edge can arm from an ISR.

**`nDerived` is the invariant.** Stages `0 … nDerived-1` have valid `d0`, `d1` and (for `L`) `rst`.
Derivation is strictly in order, because a ramp stage's Bresenham constants start from the previous
stage's end value.

- The **arm** derives stage 0 and nothing else. Stage 0 is the one stage whose values are due at
  `t0` itself, inside the first latch's preload window, so there is nowhere else to put it.
- The **player** tops up in the gap before a latch it is about to sleep for: on the path that
  programs the timer and returns, `deriveAhead` derives stages while the gap to the wake-up is
  wider than `SJ_STAGE_DERIVE_US`. This is idle time by construction — the ISR is about to return
  and wait for it.
- Every use site calls `ensureDerived` first, so correctness never depends on the look-ahead
  having run. In the `L` path the call sits after the last sample of a stage is latched and before
  the next stage is entered; in the `S` path after a stage's latch, for the following stage.

The whole derivation is a first-pulse cost: `nDerived` reaches `nStages` and stays there for the
rest of the train.

**Bounded look-ahead.** A 0-duration `L` stage is an instant jump whose single sample shares its
deadline with the previous stage's last sample, so entering it yields no idle gap and stage i+2 has
to be ready too. Two of them in a row are refused at parse time (phase 10), so the chain is at most
two stages long — and `deriveAhead` derives as many stages as fit the gap rather than exactly one,
which covers it without a special case.

## Risks, and what answers each

1. **The derivation lands in the tail of an ISR pass and delays a latch.** The look-ahead is gated
   on the measured gap, so it only runs when there is room. The fallback is not gated, but it runs
   immediately after a latch, where it spends the interval to the next one and nothing else.
   Answered by the engine's own late/overdue counters on the worst-case trains — that is what they
   are for.
2. **A ramp interval too tight for the look-ahead.** At `dt` near its floor
   (`PRELOAD + DACPROG2 + 3` = 12 µs) samples are processed inline in one ISR pass and the
   look-ahead never runs; every stage boundary then pays the fallback. Answered by running an `L`
   train at the floor and reading the counters.
3. **`S` stages have no minimum spacing.** A stage a microsecond long makes its successor's
   derivation land between two latches that are already back to back. It is a first-pulse cost of
   ~0.6 µs per stage, and the same counters see it.

## Done when

- ~~`BENCHARM` shows the arm's cost flat in stage count for both `S` and `L`.~~ Not flat, but
  nearly: nine extra stages cost 2.2 µs on an `S` train and 2.8 µs on an `L` one, against 8.0 and
  17.4 before. What is left is `cum[]` and the per-stage sample count, which stay in the arm for
  the reason given above.
- ~~The worst-case trains run with `lateEvents` and `overdueEvents` both 0, at the default `dt` and
  at the floor.~~ Done, and the six cases are now a script: `tests/device/stage_timing.py`.
- ~~A measured train still reports the same values it did before the change.~~ Done — the same
  script's `MSUM` rows, before and after, agree to within 7.3 mV against per-point standard
  deviations of 2–10 mV.
- ~~`CAL STARTLAT` is lowered to what the measurements support~~ — 45 → **35 µs**. The worst
  warmed arm (15.0 µs) needs 24, which the firmware confirms by asking for exactly that at
  `CAL,STARTLAT,23` under a real trigger edge; the worst cold arm (20.0 µs) needs 29; 35 covers
  both. A `TRIG` independent route of two ten-stage `L` slots needs ~39 and still reports itself,
  as it did at 45.
- ~~`docs/timing.md` carries them.~~ §1 and §7.
- ~~Four host suites and four build configurations pass.~~ Done.

## What the bench found that the plan did not predict

**The floor rose 0.2–0.5 µs on every train, including one with no stages at all.** An undriven arm
went 8.83 → 9.04 µs and a one-stage `S` 10.42 → 11.12. Nothing was added to those paths: the
`Player` grew by 128 bytes for the stage copy, which moves `player[1]`'s fields past some of the
addressing modes the compiler was using, and `deriveStage` is a `SJ_HOT` (therefore `noinline`)
call where the conversions used to be inlined into `startTrain`. It is the same effect phase 12
saw when `buildGeometry` became a separate function, and the same conclusion: 0.5 µs paid once
against 1.6 µs a stage saved nine times over is not worth chasing.

**A `T`/`U` start cannot test a start latency at all.** `startTrain` reads its own clock at the
*end* when no anchor is passed, so `t0` is one `STARTLAT` after the arm finished and the arm costs
a serial start nothing. `CAL,STARTLAT,21` and `T` on the worst train reports no fault; the same
value with a real edge from the bench AWG into IN0 reports one on every trigger. Only a trigger
start charges the arm to the latency, which is the whole point of the anchor — and it means the
acceptance test for a `STARTLAT` value needs the signal generator, not just the serial port.
