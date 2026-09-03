# Plan — take the measurement plan out of the start latency

Status: done, 2026-09-03. Built, host-tested and measured on silicon; `CAL STARTLAT` is
45 µs. What the bench found is at the bottom of this file, and the current numbers are in
[timing.md](timing.md) §7.

The arm was measured on silicon in phase 11 and reduces to a floor plus four marginal costs. Two of
the four belong to the measurement plan and together dominate everything else: compiling a plan
costs **3.1 µs per point** and zeroing its accumulators **0.55 µs per point on every arm**, so a
ten-stage train measuring V and I on every stage arms in 54.3 µs the first time after an edit and
23.6 µs afterwards — against 8.45 µs for a train that drives nothing. That first-arm case needs a
66 µs `CAL STARTLAT` and the default is 60. This plan moves both terms out of `Engine::startTrain`
into `loop()`, where microseconds cost nothing.

It is also the prerequisite for deriving ramp stages lazily. That scheme wants to leave the arm
with nothing per stage, but the plan compile reads `cum[]` and the per-stage sample count `N`, so
while the compile is inside the arm those two have to stay there too. With the compile in `loop()`
the arm owes the plan nothing at all.

## The one structural change: two plan buffers per engine

Everything below follows from a single invariant.

> `plan_[eng][live[eng]]` is the only plan the player ISR of engine `eng` ever touches.
> The other buffer belongs to `loop()`, which may compile into it and clear it at will.

`Measure::Plan` becomes `plan_[2][2]` with a `volatile uint8_t live[2]`, costing 2448 B of extra
RAM (4.9 kB total, against 214 kB free). `live[eng]` changes only in `startTrain`, which runs with
the player of that engine masked or stopped, so no reader ever sees it move.

The arm then does not compile and does not zero — it *swaps*:

| At arm time, the incoming buffer is | Cost | When |
|---|---|---|
| clean and already compiled for this slot | one index write | the normal case once warming is in: every trigger of an experiment |
| clean, compiled for another slot or a stale epoch | 3.1 µs/point | first start of a slot `loop()` had no reason to warm (a `T`/`U` command on an arbitrary slot) |
| still holding a summary nobody has printed | 0.55 µs/point + maybe the compile | two trains completed without `loop()` running in between |

The last row is the fallback, and it is deliberately a fallback rather than a refusal: it costs
what today's arm costs and never loses a measurement, where dropping the measurement to save the
microsecond would.

Two things fall out that are worth having on their own:

- **A re-arm no longer destroys the previous train's summary.** Today `armPlan` resets in place, so
  a train that completes and is re-armed before `loop()` drains the completion prints
  `# MSUM: engine N was re-armed before its summary printed — dropped`. With two buffers the
  finished train's plan *and* its accumulators sit intact in the retired buffer while the new train
  runs on the other one, so the summary prints correctly.
- **The retired buffer is exactly the buffer the next arm will take**, which is what makes warming
  a one-liner: `loop()` compiles into the buffer that is not live, and the next arm finds it ready.

`printSummary` therefore has to read the buffer the *completed* train used, not the live one.
`Measure::trainDone(eng)`, called from the player ISR where the completion is pushed, records
`summaryIdx[eng] = live[eng]`; `printSummary` reads that buffer and checks its slot as before.

## Work items

### 1. `Engine::buildGeometry` — the geometry outside the arm

`Measure::Geometry` is currently assembled from `Player` fields at the end of the arm
(`Engine.cpp`, the block before `armPlan`). Compiling in `loop()` needs the same geometry without a
`Player`, and — this is the correctness point, not a tidiness one — it must be *the same code*: two
implementations that drift would let a warm plan carry a tag that says it describes a geometry it
does not.

```cpp
// Everything the measurement plan is compiled against, derived from the slot's
// definition and the CAL set alone. `cum` (SJ_MAX_STAGES+1) and `stageN`
// (SJ_MAX_STAGES) are the caller's storage; geo points into them. Fills dtUs,
// which the arm needs for the Bresenham constants. Returns false, with a reason
// in err when err is non-NULL, for a train that startTrain would refuse anyway.
bool buildGeometry(uint8_t slotIdx, const TrainDef& def, const Cal::Def& cal,
                   uint64_t* cum, uint32_t* stageN, Measure::Geometry& geo,
                   uint32_t* dtUs, char* err, size_t errsz);
```

It absorbs from `startTrain`: `driveMask`, the `progUs` budget, the ramp-interval refusal, the
Nyquist refusal, the `cum[]` accumulation, the per-stage `N`, the sine-constant lookup and the
CAL-derived ADC fields. `startTrain` keeps the busy check, the channel-conflict check, the
amplitude conversions (they read the calibration offsets, which are deliberately *not* in the plan
tag) and everything that writes the `Player`.

`stageN[i]` moves to a new `SampleGen::rampStageN(dur_us, targetDt_us)` — the `N` computation lifted
out of `rampStageInit`, which then calls it. The arm pays one extra `UDIV` per ramp stage for the
split (~0.04 µs), and gets in exchange the exact separation the lazy-ramp phase needs: `cum[]` and
`N` on one side, everything else per stage on the other.

### 2. Compile in `loop()`

`Engine::warmPlans()`, called from `loop()` after `Commands::poll()` has drained completions:

- for each engine, find the slot an enabled `TRIG` route would start on it (mode 1 → engine 0 gets
  `slot0`; mode 2 → engine 0 gets `slot0`, engine 1 gets `slot1`; the two inputs are scanned in
  order and the first hit wins, so a board whose two inputs point different slots at one engine
  warms the input-0 one);
- skip unless the non-live buffer is free (not the pending summary) and its tag misses — a tag
  comparison is four loads, so the steady-state cost is nothing and no geometry is built;
- on a miss, build the geometry and compile into that buffer.

`TrainStore::epoch()` and `Cal::epoch()` are already in the tag, so an edit invalidates the warm
plan and the next `loop()` pass rebuilds it. Warming is best-effort throughout: if `loop()` never
gets there, the arm compiles exactly as it does today.

### 3. Zero in `loop()`

`Measure::housekeep()`, in the same place: for each engine, if the non-live buffer holds results
and is not the pending summary, `planResetResults` it and mark it clean. `armPlan` clears only when
it finds the incoming buffer still dirty.

### 4. Re-measure and set `CAL STARTLAT`

`tests/device/bench_arm.py` on the same eight cases plus the ten-point measured case, before and
after. The expectation to check against, from the phase-11 constants: every case collapses onto its
re-arm column, so the worst arm becomes the ten-stage `L` train's 28.5 µs and the ten-point measured
train drops from 54.3 to about 18 µs. That would put `STARTLAT` at **45 µs**, and it is the first
time in this project the budget has been lowered rather than raised, so it gets measured on the
bench and not inferred.

## Risks, and what each is checked by

| Risk | Check |
|---|---|
| Warming writes a buffer the player is reading | The live/non-live invariant, plus `live[eng]` only ever written in `startTrain`. `tests/host/test_measure.cpp` cannot see this — it is checked by reading the code and by a device run that triggers while `loop()` warms. |
| A warm plan's tag says it describes a geometry it does not | One `buildGeometry`, used by both paths. |
| The summary reads the wrong buffer | `summaryIdx` set in the ISR at completion, slot checked in `printSummary` as today. |
| A stale plan survives an edit | Unchanged from phase 10: the tag carries both epochs. |
| Accumulators cleared while a point can still fire | Only the non-live buffer is cleared in `loop()`, and the live one only in the arm, which is where it is already safe. |

## What the bench found

The prediction held: every case collapsed onto its re-arm column, and the ten-point measured train
fell from 54.3 µs to 17.5 µs — the same figure as the identical train with measurement switched
off, so **in-train measurement now costs the arm nothing**, one point or ten. `CAL STARTLAT` is
45 µs, sized by the ten-stage `L` train's 28.7 µs, and verified by running both worst-case trains
at that budget with no arm-overrun reported.

Three things the plan did not anticipate:

- **The harness could not see the work.** `BENCHARM`'s repetitions run back to back inside one bus
  lock, so `loop()` never runs between them and nothing gets warmed; worse, with two buffers the
  arms alternate, so its maximum became the *second* arm rather than the first. The sweep gained a
  column measured with one arm per invocation. The old maximum is still worth reporting — it is a
  real regime, an engine re-triggered before `loop()` ran — so `armPlan` now also prefers the live
  buffer when the spare's tag misses and the live one's matches, which makes the repeated start of
  one slot free even with `loop()` starved.
- **`SJ_HOT` on a five-line helper made things worse.** `rampStageN` split out of `rampStageInit`
  as intended, but as an out-of-line `SJ_HOT` function it cost 0.35 µs per ramp stage — `FASTRUN`
  carries `noinline, noclone`, so a function too small to justify a call frame got one anyway,
  twice per stage. It is a header `inline` now.
- **The warm needed a fallback target.** A `TRIG` route names the slot to warm, but a `T`/`U` start
  names none in advance, so the first serial start of a freshly edited measured slot still compiled
  inside its own latency. `Engine::warmPlans` falls back to `TrainStore::lastWritten()`, which is
  what such a start almost always follows.

The floor rose 0.4 µs and a one-stage train 0.5 µs, which is what `buildGeometry` costs as a
separate function. It buys 3.1 µs per measurement point.

## Done when

- ~~`BENCHARM` shows the first-arm and re-arm columns collapsed for the warmed slots, measured on
  silicon.~~ Done, with a new column in `tests/device/bench_arm.py` that can see it.
- ~~`CAL STARTLAT` is set to what those measurements support, and `docs/timing.md` §7 carries
  them.~~ Done — 45 µs. The EEPROM version is deliberately not bumped: a stored 60 µs is
  conservative rather than wrong, so rejecting a saved image over it would destroy the operator's
  slots 0–9 to deliver an improvement they can adopt with `CAL,STARTLAT,45` and `P`.
- ~~The four host suites pass, and the Teensy 3.5 register build is clean.~~ Done, and so are the
  Teensy 3.5 portable, Teensy 4.0 and Teensy 4.1 builds. `smoke.py` passes on the board.
