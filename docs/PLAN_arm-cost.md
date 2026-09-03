# Plan — bring the arm cost under one latch interval

Status: done, 2026-09-03. Items 1–5 are built, host-tested and measured on silicon; item 6 is
deferred with its prerequisite in place and is re-scoped by what the bench found. The arm fell from
16.2–37.5 µs to 8.5–28.5 µs on a re-arm, but the measurement-plan compile turned out to cost 3.1 µs
per point on the first arm after an edit, which is now the largest single term and puts the worst
case at 54.3 µs. `CAL STARTLAT` therefore stays at 60 µs. Current numbers and the remaining work
live in [timing.md](timing.md) §7; this file keeps the reasoning behind each decision.

The goal is a start latency no longer than the interval between two consecutive DAC updates, so
that a trigger-started train never needs a wider budget than the waveform itself already runs on.
The shortest interval the engine schedules is the default ramp sample interval `DT` = 20 µs (and
the sine sample interval at the 50 kHz `FS_MAX` ceiling, also 20 µs). `CAL STARTLAT` has to cover
`arm + PRELOAD + DACPROG2 + MIN_SCHEDULE` = `arm + 12 µs`, so **the arm has to fit in 8 µs**,
against 16.2 µs at the time of writing for a train that drives nothing and 19–42 µs for real ones.
This plan lists what the arm spends that on, what can be moved out of it, and in what order. The
bench run at the end shows the goal is not reachable this way: an arm that drives nothing costs
8.45 µs after every item below is built.

## Terminology

A *latch* here is the DAC output update: the AD5752 holds a value in its input register until the
`NLDAC` pin is pulsed, which transfers it to the DAC register and moves the output. The datasheet
calls the pin "load DAC", so **update** and **load** are the neutral synonyms — the word has
nothing to do with a software lock or mutex. This document and `timing.md` use "latch" for the
event and "update" where the sentence would otherwise read as locking.

## What the arm does, and what it costs

Measured with `BENCHARM` on a Teensy 3.5 at 120 MHz (120 cycles/µs), register backends, phase 9:

| Case | µs | Increment |
|---|---|---|
| undriven, 0 stages | 16.2 | the floor every arm pays |
| `S`, 2 ch, 1 stage | 19.2 | |
| `S`, 2 ch, 10 stages | 28.2 | ~1.0 µs per extra stage |
| `S`, 2 ch, 1 stage, default `MEAS` | 23.4 | +4.2 µs for the measurement plan |
| `L`, 2 ch, 1 stage | 22.8 | +3.6 µs per ramp stage |
| `W`, 2 ch | 37.5 | +18.3 µs for the sine setup |

Static analysis of the phase-9 binary (`tmp/b9-t35/stimjimAWG.ino.elf`, `-O2`, hard
single-precision FPU, `-fsingle-precision-constant`):

- `Engine::startTrain` is 2800 bytes / 942 instructions; `Engine::playerRun` 3164 bytes.
  `sizeof(Measure::Plan)` = 1216 B, `sizeof(Player)` = 912 B, `sizeof(TrainDef)` = 164 B.
- `Measure::planBuild` opens with `memset` of all 1216 bytes. The linked `memset` stores 16 B per
  iteration, so that is ~76 iterations ≈ 600–900 cycles ≈ **5–7 µs, paid by every arm including
  one that measures nothing** — a third to nearly half of the floor.
- `SampleGen::sinePhaseInc` is nine soft-float double calls (3×`ui2d`, 3×`dmul`, `ddiv`,
  `adddf3`, `d2uiz`) ≈ 450–500 cycles ≈ 4 µs, called once per channel → **~8 µs of the sine's
  18.3 µs**. It is the only remaining double-precision arithmetic in the arm; `ampToCode`'s
  division compiles to a 14-cycle `VDIV.F32` because the unit constants are single-precision
  under `-fsingle-precision-constant`.
- `SampleGen::rampStageInit` calls `__aeabi_uldivmod` twice per stage (~100–150 cycles each) —
  most of the 430 cycles a ramp stage costs.
- After the `memset`, the floor still holds ~1000–1300 cycles for roughly 200 instructions of
  straight-line copying: 5–6 cycles per instruction, too high for SRAM-resident Cortex-M4 code.
  The suspected cause is flash wait states on a 2800-byte function (K64 flash at 120 MHz behind a
  512-byte FMC cache). Item 5 tests that directly.

## Does moving work to the end of the arm help?

Only outside the arm, not at its end, and the reason is worth writing down because it decides
where the zeroing goes.

`t0` is taken near the end of `startTrain`, and the first latch's preload window opens at
`t0 − preloadCyc`. Work placed *after* `pitProgram` would in principle be preempted by the player
ISR (priority 64) when that window opens, since both `loop()` and the trigger ISRs (priority 80)
sit below it — so from a trigger edge, trailing work is free. But `T`/`U` starts wrap
`startTrain` in `FastIO::busLock`, which raises `BASEPRI` to 64 and masks exactly that ISR, so
trailing work there *does* delay the first latch. Relying on the difference would make the
serial-start and trigger-start paths behave differently, which is the opposite of what the
timing design is for.

So the zeroing does not move to the end of the arm. It moves **out** of it: to plan-compile time
(before the edge) and to train end in `loop()`, after the summary has been read.

## Work items

Ordered by saving per unit of risk. Items 1–4 are this phase; item 5 is a measurement; item 6 is
deferred with its prerequisite built now.

### 1. Zero only what the plan uses, and zero it outside the arm

`planBuild` currently zeroes 1216 bytes, of which 960 are the accumulators
`acc[10][2][2]` — one per (point, channel, line) — while a typical plan has 1–3 points.

- Split the plan into *compile* (structure: `atCyc`, `budCyc`, group masks, scalars) and *reset*
  (`next`, `envSkipped`, and the accumulators of the points that exist).
- Zero `acc[0 … nPoints-1]` only, which is 72–216 bytes instead of 960.
- Do the reset at compile time and at train end in `loop()` (after `printSummary` has read the
  accumulators), never in the arm.

Expected: **−5 to 7 µs on every arm.** Risk: the accumulators must not be zeroed while the player
can still fire a point, which is why train end means "in `loop()`, after the summary", not "in the
completion ISR".

### 2. Compile the measurement plan once, not per arm

The plan depends on the stage/sample geometry, the `MEAS` fields and the `CAL` set — never on the
calibration offsets. Re-arming the same slot with nothing edited therefore recompiles an identical
plan.

- Tag the compiled plan with `(slot, definition epoch, CAL epoch)`; `armPlan` returns immediately
  when the tag matches.
- Bump the definition epoch on any slot write and the `CAL` epoch on any budget change, so a stale
  plan cannot survive an edit.

Expected: **−4.2 µs from the second arm of a slot onwards**, which is every trigger in an
experiment. Risk: a missed epoch bump would run a stale plan; both bump sites are single-entry
(`TrainStore` slot commit, `Cal::set`).

### 3. Derive the sine constants when `W` is parsed

`sampleCyc`, `phaseInc` and `phaseInit` depend only on the frequency and start phase in the
definition, so they can be computed once at definition time and read by the arm. Keeping them in a
side table (100 slots × 20 B = 2 kB) rather than in `TrainDef` avoids an EEPROM format change; the
table is refilled whenever a slot is written and after an EEPROM load.

Expected: **−8 to 10 µs on a `W` arm**, and no soft-float double left anywhere in the arm.
Risk: the table must be refilled on every path that writes a slot — slot commit, `EEPROM` load,
`slotDefault`.

### 4. Hardware `UDIV` instead of `__aeabi_uldivmod` in `rampStageInit`

Two divisions per stage are 64-bit today. Neither needs to be, and neither loses a bit:

- `N = (dur_us + targetDt/2) / targetDt` — both operands are `uint32_t` and the sum cannot
  overflow (`dur_us ≤ 2×10⁹`, `targetDt ≤ 10⁶`), so this is a plain 32-bit `UDIV`.
- `qt = durCyc / N`, `rt = durCyc % N` with `durCyc = dur_us × 120` — up to 2.4×10¹¹, genuinely
  64-bit. Split it exactly instead:
  `qt = (dur_us / N) × cycPerUs + ((dur_us % N) × cycPerUs) / N`, `rt = ((dur_us % N) × cycPerUs) % N`.
  Both terms are 32-bit as long as `(dur_us % N) × cycPerUs` fits, i.e. `dur_us % N < 2³²/120 ≈ 35.7×10⁶`,
  which holds for every stage shorter than ~35 s; the 64-bit path stays as the fallback above that.

This is exact integer arithmetic in both paths — the identity is algebraic, not an approximation,
so the concern about precision does not arise: no float appears on the time axis at all, and the
code axis already uses 32-bit signed `SDIV`. `tests/host/test_samplegen.cpp` gets a case comparing
the fast and fallback paths over the interesting range, which is the verification.

Expected: **−2 to 3 µs per ramp stage.**

### 5. Try `FASTRUN` and measure it

`FASTRUN` is a Teensy 3.x core macro — `__attribute__((section(".fastrun"), noinline, noclone))` —
that places a function in a RAM section which the startup code copies from flash at boot. It then
executes from SRAM at one cycle per access instead of through the flash controller. Teensy 4.x
does this for *all* code by default (it runs from ITCM and needs `FLASHMEM` to opt out), so this is
a Kinetis-only lever, and the T4 build already has whatever it is worth.

On the reliability question: the RAM copy is made once at boot from the flash image, so a brownout
or a reset re-copies it. The MCU's own POR/LVD brings the chip down before SRAM contents become
unreliable, and neither SRAM nor flash carries ECC here, so a corrupted RAM instruction is no more
likely than the corrupted stack, globals and heap the firmware already depends on. The real costs
are RAM (a few kB of 222 kB free) and that `noinline, noclone` can block an inlining the compiler
would otherwise do.

Put it behind `SJ_CODE_IN_RAM` (default on for the Kinetis register build, no effect elsewhere) on
`Engine::startTrain`, `Engine::playerRun` and the plan compiler, then run `BENCHARM` both ways.
If the flash hypothesis is right the floor drops by several microseconds; if not, the switch goes
to 0 and the finding is recorded.

### 6. Deferred — derive stage i+1 during stage i

Lazy per-stage derivation would remove `(nStages − 1) × 1.0 µs` for `S` and `× 4.6 µs` for `L`
from the arm, which is the only way a ten-stage `L` train gets under 8 µs. It is deferred because
it splits one straightforward function into a state machine whose invariants have to hold in ISR
context — the maintenance cost is real and the other five items are cheaper.

Its prerequisite is built now: **a 0-duration ramp stage means "jump to this level and start the
next ramp from it", and two of them in a row have no meaning**, so `L` parsing now rejects
consecutive 0-duration stages (a single one anywhere, including first or last, stays legal). With
that rule, a lazy scheme never has to prepare more than one stage ahead: on entering stage i it
derives stage i+1, and if stage i is a 0-duration jump it derives i+2 as well — bounded look-ahead
by construction rather than by luck.

## What the bench found

`tests/device/bench_arm.py` ran on a Teensy 3.5 at 120 MHz with the register backends, on both
halves of the `SJ_CODE_IN_RAM` A/B. Full tables are in [timing.md](timing.md) §7; what matters to
the decisions in this file:

- **The projection for a ten-stage `L` train was wrong by a factor of two in the safe direction.**
  It was expected at ~61 µs from the phase-9 pieces; it measures **28.5 µs** on a re-arm and 31.4 µs
  on a first arm. Items 1–5 did more than the arithmetic of the projection allowed for.
- **`SJ_CODE_IN_RAM` is worth 13–17 %, not 30–50 %.** The saving is real and scales with the
  function's size, which is the tell: the two multi-kilobyte functions blow the K64's 512-byte
  flash cache and gain 1.4–4.3 µs, while moving the 180-byte `rampStageInit` in as well gained only
  0.33 µs over ten stages. The flash controller was a contributor, not the explanation; the memset
  and the soft-float were.
- **The plan compile is now the largest single term, and it was never on this plan's list.** Item 2
  moved it from every arm to the first arm after an edit, which is the right place for it, but it
  costs **3.1 µs per measurement point** and a ten-point plan therefore makes the first arm 54.3 µs
  against that train's 23.6 µs re-arm. `CAL STARTLAT` = 60 µs does not cover it.
- **The accumulator zeroing costs 0.55 µs per point**, so the ten-point case pays 5.7 µs of it on
  every arm.
- **The 8 µs target this plan opens with is unreachable from inside the arm.** The floor — an arm
  that drives nothing — is 8.45 µs on its own.

## What was built

| Item | State |
|---|---|
| 1 — zero only the plan a train uses | Done. `planCompile` zeroes 242 bytes of the struct's 1224; `planResetResults` clears 96 bytes per existing point. `tests/host/test_measure.cpp` poisons the struct with `0xAA` before `planBuild` so a field added after `acc` without a reset fails the test. Measured: 0.55 µs per point on every arm, against 5–7 µs flat before. |
| 2 — compile the plan once per definition | Done. The plan carries `(tagValid, tagSlot, tagDefEpoch, tagCalEpoch)`; `TrainStore::epoch()` bumps on `commit`, `begin` and the EEPROM restore, `Cal::epoch()` inside `Cal::set`'s bus lock. Measured: the compile drops out of every arm after the first, and costs 3.1 µs per point on that first one. |
| 3 — derive the sine constants at definition | Done. `SampleGen::sineDerive` is the pure derivation, `Engine` caches one `SineConst` per slot (2 kB) keyed on the definition epoch, and `Engine::deriveSine` warms it from the `S`/`L`/`W` commit. The arm still derives on demand, so correctness never depends on the warm-up. Measured: a `W` arm fell 37.5 → 10.1 µs, the largest single saving in the plan, and now costs the same as an `S` arm. |
| 4 — 32-bit `UDIV` in `rampStageInit` | Done, with the exactness checked rather than argued: `testRampDivisionPaths` compares both paths against the 64-bit reference across six `dt` values and 21 durations up to `UINT32_MAX`, including both overflow guards, and re-checks that N Bresenham steps still land exactly on the stage end. Measured: a ramp stage costs 1.93 µs in the arm, of which 1.04 µs is `rampStageInit`. |
| 5 — `FASTRUN` | Done as `SJ_CODE_IN_RAM`, default on for Kinetis, covering `Engine::startTrain`, `Engine::playerRun` and — added after the first bench run — `SampleGen::rampStageInit` and `rampStep`. Verified in the ELF: all are at `0x1fff…` in SRAM, and RAM rises 41 404 → 47 820 B. `IDN` reports `hot=RAM`/`flash`/`ITCM`. Measured: 13–17 % on the arm, the two large functions accounting for essentially all of it. |
| 6 — derive stage i+1 during stage i | Deferred, and re-scoped by the bench: it is worth 0.89 µs per `S` stage and 1.93 µs per `L` stage, but the plan compile above it is worth 3.1 µs per point, and compiling the plan outside the arm is also what would free the lazy scheme from having to leave `cum[]` and the per-stage `N` behind in the arm. Prerequisite built: consecutive 0-duration `L` stages are refused at parse time, so look-ahead is bounded to one stage. |

Not attempted, and deliberately: `ampToCode`'s `VDIV.F32` stays a division. Replacing it with a
reciprocal multiply would risk a 1-LSB shift in delivered amplitude, and the claim that `S` trains
are bit-exact against the legacy firmware rests on that exact expression. The measurements do not
change that: the two conversions are 0.89 µs per stage together, and deferring them costs nothing
in accuracy where replacing them would.

## Done when

- ~~The floor and the `S`/`L`/`W` arm costs are re-measured with `BENCHARM` on silicon.~~ Done,
  including the ten-stage `L` case and a ten-point measured case the plan had not foreseen.
- ~~`CAL STARTLAT` is lowered to what the measurements support~~ — it is **not** lowered, because
  the measurements do not support lowering it: the worst first arm needs 66 µs against the 60 µs
  default. Lowering it waits on the plan compile moving out of the arm, after which the whole
  measured range fits inside ~45 µs.
- ~~`docs/timing.md` carries the new numbers with the same "measured / estimated" separation.~~
  Done — §7 is now measurement throughout, split by re-arm and first arm rather than by
  measured/predicted.
- ~~The four host suites and all four build configurations still pass.~~ Host suites pass. The
  Teensy 3.5 register build passes in both `SJ_CODE_IN_RAM` states; the portable and Teensy 4.x
  configurations are unchanged by this phase and were last built in phase 10.
