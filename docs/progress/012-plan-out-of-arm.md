# 012 — the measurement plan out of the start latency

Phase 11 measured the arm on silicon and found that the two largest terms in it both belonged to
the measurement plan: compiling one cost 3.1 µs per point and clearing its accumulators 0.55 µs per
point, so a ten-stage train measuring V and I on every stage armed in 54.3 µs — needing a 66 µs
`CAL STARTLAT` against a 60 µs default. This phase moved both into `loop()`. The mechanism is one
structural change: each engine now holds **two** measurement plans and the arm swaps between them
instead of rewriting one in place, so the player ISR always reads the live buffer while `loop()`
owns the other and can compile into it and clear it with no lock. `Engine::warmPlans` prepares the
buffer the next arm will take, for whatever slot a `TRIG` route names — falling back to the last
slot written, which is what a `T`/`U` start almost always follows. Measured on the board:
**in-train measurement now costs the arm nothing at all**, one point or ten. The ten-point case
fell 54.3 → 17.5 µs, the same figure as the identical train with measurement switched off, and the
worst arm of any shape is now the ten-stage `L` train's 28.7 µs. `CAL STARTLAT` therefore drops
from 60 to **45 µs** — the first time in this project a timing budget has been lowered — verified
by running both worst-case trains at 45 µs with no arm-overrun warning and all ten measurement
points reporting n=50. The second buffer also fixes something that was never the point: a train
re-armed before `loop()` drained its completion used to lose its summary, and now keeps it.

## What was built

**`Engine::buildGeometry`** (new, `SJ_HOT`). Everything the measurement plan is compiled against —
`chMask`, `cum[]`, the per-stage ramp sample count, the sine constants, the CAL-derived ADC and
preload windows — derived from `(slot, definition, CAL)` and nothing else. It was previously
assembled from `Player` fields at the end of the arm. There is exactly one copy because two would
let a warm plan carry a tag claiming a geometry the arm computes differently, which is the one
failure this design could have that would be silent. It also absorbed the two refusals that depend
on the CAL set (ramp interval below the per-sample budget, sine above Fs/2), taking an optional
error buffer so the warm path can call it without one.

**`SampleGen::rampStageN`**, split out of `rampStageInit`: the sample count is all the plan needs of
a ramp stage. It lives in the header as an `inline` — see the measurements below for why.

**Two plans per engine** (`Measure::plan_[2][2]`, `live[2]`), +2448 B of RAM. The invariant is that
`plan_[eng][live[eng]]` is the only plan the player ISR of that engine touches. `live[eng]` moves
only in `armPlan`, which runs with that engine's player masked or stopped.

- `warmPlan` / `housekeep` (loop context) compile into and clear the spare buffer.
- `armPlan` takes the spare, compiling and clearing only if `loop()` did not.
- `trainDone`, called from the player ISR where the completion is pushed, freezes the finished
  train's buffer; `printSummary` reads that one rather than the live one and releases it.
- `printSummary` returns a `bool` instead of the caller asking `hasPlan` first, so that the
  release happens for every completion including unmeasured ones.

**The race, and how it is closed.** A trigger edge can arm an engine while `loop()` is compiling
that engine's spare buffer. A `volatile bool loopBusy[2]` handshake covers it: `loop()` publishes
the claim and then re-reads `live[]`, so either it sees an arm that beat it and backs off, or the
arm sees the flag and stays on the live buffer, rewriting it in place exactly as the single-buffer
firmware always did. Exactly one of the two happens, and the fallback is a path that ran in
production for ten phases. Masking the trigger ISRs for the length of a compile was rejected: it
would put up to 30 µs of jitter on a trigger edge, which is the opposite of what the timing design
is for.

**Two ordering bugs found by reading rather than by testing**, both fixed before the first upload:

- The accumulator reset has to follow *every* compile, not only a compile of a dirty buffer. A plan
  cleared while it had fewer points than the next one leaves the points beyond that count holding
  an older train's sums, and only a reset taken after the compile knows how many points to clear.
- The completion path asked `hasPlan`, which reads the live buffer, so a measured train whose
  successor measures nothing would have been reported as "no measurement carried out". It now asks
  about the frozen buffer.

## Measurements

Teensy 3.5, 120 MHz, register backends, `hot=RAM`, `tests/device/bench_arm.py`. The harness gained a
third column and a ninth case, because the old one could not see this work at all: `BENCHARM`'s
repetitions run back to back inside one bus lock, so `loop()` never runs between them and nothing
gets warmed. The **warmed** column runs one arm per `BENCHARM` invocation.

| Case | phase 11 (re-arm / first) | phase 12 warmed |
|---|---|---|
| undriven, 0 stages | 8.45 / 11.1 | 8.83 |
| `S`, 2 ch, 1 stage | 9.88 / 12.8 | 10.42 |
| `S`, 2 ch, 1 stage, `MEAS` V+I both | 10.62 / 18.0 | 10.42 |
| `S`, 2 ch, 10 stages | 17.88 / 20.9 | 17.46 |
| `S`, 2 ch, 10 stages, `MEAS` on every stage | 23.58 / **54.3** | **17.46** |
| `L`, 2 ch, 1 stage | 11.20 / 14.2 | 11.75 |
| `L`, 2 ch, 10 stages | 28.46 / 31.4 | 28.67 |
| `W`, 2 ch | 10.12 / 13.1 | 10.83 |

The rows to read in pairs are 2/3 and 4/5: measurement is free now. The floor rose 0.4 µs and a
one-stage train 0.5 µs, which is what `buildGeometry` costs as a separate function; it buys 3.1 µs
per measurement point, so it is not worth chasing.

**Two intermediate results worth keeping, because both contradicted an expectation:**

- Marking `rampStageN` `SJ_HOT` and leaving it in the `.cpp` cost **0.35 µs per ramp stage** —
  `SJ_HOT` expands to `FASTRUN`, which carries `noinline, noclone`, so a function too small to
  justify a call frame got one anyway, twice per stage. Moving it to the header as `inline` restored
  the per-stage cost exactly. The lesson generalises: `SJ_HOT` is for functions large enough that
  the call is noise, and putting it on a five-line helper makes things worse.
- The first version of the warm path measured as having done nothing, and the reason was the
  harness rather than the firmware. With two buffers, 200 back-to-back arms alternate between them,
  so arm 1 hits the warm buffer and arm 2 misses on the other one — `BENCHARM`'s max became the
  *second* arm, not the first. That is a real regime (an engine re-triggered before `loop()` ran),
  so rather than only relabelling the column, `armPlan` now prefers the live buffer when the spare's
  tag misses and the live one's matches: the repeated start of one slot with `loop()` starved is
  free instead of a full compile. The worst column fell from 53 to 24 µs for the ten-point case.

**`CAL STARTLAT` 60 → 45 µs.** The worst warmed arm is 28.67 µs and needs 40.7; 45 leaves margin.
Verified on the board rather than inferred: at `CAL,STARTLAT,45`, `T` on a ten-stage `L` train and
on a ten-stage train with a measurement point per stage both completed with no timing-fault line
and no `startNeedUs`, and the measured one printed all ten `MSUM` rows with n=50.

Two cases still exceed 45 µs and both report themselves at train end: a `TRIG` independent route of
two ten-stage `L` slots (~70 µs, because it pays the arm twice), and an engine re-triggered with
`loop()` starved for a whole train, which prepares its plan inside the arm.

**The EEPROM version is deliberately not bumped.** A stored 60 µs is conservative, not wrong, so
rejecting a v6 image over it would destroy the operator's saved slots 0–9 to deliver a latency
improvement they can adopt with `CAL,STARTLAT,45` and `P`. That is the opposite trade from phase 9,
where the stored value was too small and the image had to be rejected.

## Verification

- Host suites `test_cal`, `test_measure`, `test_samplegen`, `test_trainstore` — all pass.
- `tests/device/smoke.py` — ALL CHECKS PASSED, including the completion reporting and the MDATA/SD
  paths that the buffer change touches.
- Functional measurement check on real hardware: a two-stage measured train reports both points
  with the LED clamp's polarity asymmetry visible in the summary (+3629 mV against −4174 mV), which
  is the load behaving as `docs/bench-wiring.md` describes.
- Builds: Teensy 3.5 register (170 812 B flash, 50 100 B RAM), Teensy 3.5 portable (171 928 B,
  50 364 B), Teensy 4.0 and 4.1 — all clean at `--warnings more`.

## Next entry point

The remaining arm cost is entirely the per-stage work, and that is item 3 of
`docs/PLAN_arm-cost.md`, now unblocked:

**Derive stage i+1 during stage i.** 0.89 µs per `S` stage and 1.93 µs per `L` stage — up to 17.4 µs
on a ten-stage `L` train, and the only way the arm's cost stops depending on stage count. What moves
is the two `ampToCode` conversions and, for `L`, `rampStageInit`. What had to stay in the arm while
the plan was compiled there — `cum[]` and the per-stage `N` — no longer does, which is why this
phase came first. The risks are that the derivation lands in the tail of a stage-boundary ISR pass
and has to be shown to fit the sample interval, and that `S` stage boundaries have no minimum
spacing the way an `L` train's `dt` does, so restricting the scheme to `L` may be the right call.
The prerequisite from phase 10 is in place: consecutive 0-duration `L` stages are refused at parse
time, so look-ahead is bounded to one stage.

Below that: `CAL TRIGCOMP` is still 0 and unmeasured (`docs/bench-wiring.md` configuration B), and
pre-arming the whole train to fire on a bare `NLDAC` pulse remains the only route to a single-digit
total trigger latency — the arm's 8.83 µs floor puts everything else out of reach.
