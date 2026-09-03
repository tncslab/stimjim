# Plan — bring the arm cost under one latch interval

Status: in progress, 2026-09-03

The goal is a start latency no longer than the interval between two consecutive DAC updates, so
that a trigger-started train never needs a wider budget than the waveform itself already runs on.
The shortest interval the engine schedules is the default ramp sample interval `DT` = 20 µs (and
the sine sample interval at the 50 kHz `FS_MAX` ceiling, also 20 µs). `CAL STARTLAT` has to cover
`arm + PRELOAD + DACPROG2 + MIN_SCHEDULE` = `arm + 12 µs`, so **the arm has to fit in 8 µs**,
against 16.2 µs today for a train that drives nothing and 19–42 µs for real ones. This plan lists
what the arm spends that on, what can be moved out of it, and in what order.

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

## Bench work this plan needs

`BENCHARM` has never been run on a ten-stage `L` slot. From the measured pieces it projects to
`19.2 + 9 × 4.6 ≈ 61 µs`, i.e. above the 60 µs `STARTLAT` default, and ~65 µs with measurement on.
`tests/device/bench_arm.py` now defines the representative slots and runs the sweep, so the
projection can be replaced by a number, before and after items 1–5.

## Done when

- The floor and the `S`/`L`/`W` arm costs are re-measured with `BENCHARM` on silicon.
- `CAL STARTLAT` is lowered to what the measurements support, and `docs/timing.md` carries the new
  numbers with the same "measured / estimated" separation it has now.
- The four host suites and all four build configurations still pass.
