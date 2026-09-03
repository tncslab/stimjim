# Phase 10: the arm's cost accounted for, and most of it moved out of the start latency

Date: 2026-09-03

This phase answered a set of questions about the firmware's latency and then acted on the answer.
The trigger-to-output latency of `stimjimAWG` is `CAL STARTLAT`, 60 µs on a Teensy 3.5 with the
register backends, and what sizes it is not the DAC write (2.75 µs) but `Engine::startTrain`, which
costs 16.2 µs for a train that drives nothing and up to 37.5 µs for a sine. Disassembling the
phase-9 binary named the terms that the `BENCHARM` deltas had only bounded: the 1224-byte
measurement plan was `memset` in full on every arm even when nothing was measured (5–7 µs of the
floor), `SampleGen::sinePhaseInc` is nine soft-float double calls per channel (~8 µs of the sine's
18.3), `rampStageInit` paid two `__aeabi_uldivmod` calls per stage, and what remained ran at 5–6
cycles per instruction, which points at flash wait states on a 2.8 kB function rather than at the
instruction mix. Five changes moved that work out of the arm rather than making it faster: the plan
is compiled once per `(slot, definition, CAL)` instead of once per arm, only the plan a train
actually uses is zeroed, the sine constants are derived when `W` is parsed, the ramp's time-axis
division became four exact 32-bit `UDIV`s, and `SJ_CODE_IN_RAM` places the arm and the player event
loop in SRAM. The predicted arm costs are 10.7–24 µs against the measured 16.2–37.5, but **nothing
is measured yet**, so `CAL STARTLAT` deliberately stays at 60 µs; `tests/device/bench_arm.py` is
the sweep that will settle it, including the ten-stage `L` case that has never been benchmarked and
projects to ~61 µs on the old firmware. A separate outcome of the same investigation is documented
rather than built: the AD5752 can hold a preloaded code and fire on a 0.44 µs `NLDAC` pulse, so a
pre-armed trigger path could reach 1–3 µs, but only if the whole arm moves before the edge.

## The questions this started from, and where the answers live

The phase began as six questions about latency, and the answers were written into
`docs/timing.md`, which is new: trigger-to-output latency and its breakdown, the comparison with
`stimjimPulser`, whether the DAC can be preloaded to fire faster, what the trigger interrupt
actually starts, whether generation occupies the CPU, and what SD logging costs and stores. Two of
those answers were worth more than the rest.

**The original firmware has no fixed trigger latency at all.** Its trigger ISR plays the first
pulse itself: `startIT0ViaInputTrigger` → `startIT0` does two `memset`s, `micros()`,
`IntervalTimer::begin`, **two USB `Serial.print` calls**, the LED writes and only then `pulse0()`
(`stimjimPulser/stimjimPulser.ino:663-726`). A buffered CDC write costs microseconds; one that
finds the transmit buffer full blocks for milliseconds. So the delivered latency depends on whether
a host is listening, the arm's cost is *added* to the onset instead of subtracted from a budget,
and the first pulse can be preempted by its own repeat (pin ISR at the core default priority, the
repeat timer at 64). `stimjimAWG` differs by construction — no ISR prints, `t0` is anchored to the
edge timestamp, the first latch always happens in the player ISR, and the trigger ISR sits *below*
the players. The trade is a constant that is currently larger than the original's best case and far
smaller than its worst, and unlike either, repeatable to 42 ns.

**The trigger interrupt does not start signal generation.** It timestamps the edge, arms, computes
`t0` and programs its player's PIT channel to wake one `PRELOAD` before it. Every sample, the first
included, is latched from the timer ISR. The interrupt is the time reference; the clock is the
emitter. That is what makes the latency independent of the train, of `loop()` and of interrupt
entry, and it is also why work placed after `pitProgram` inside `startTrain` would *not* delay the
first latch from a trigger edge — the player ISR at priority 64 preempts the trigger ISR at 80. It
would delay it from a `T`/`U` start, though, because `Commands::handleStart` wraps the arm in
`FastIO::busLock`, which raises `BASEPRI` to 64 and masks exactly that ISR. Making the two start
paths behave differently is the opposite of what the timing design exists for, so nothing relies on
that asymmetry: the zeroing moved *out* of the arm, not to its end.

## What the arm costs, and why

Measured with `BENCHARM` on a Teensy 3.5 at 120 MHz, register backends (the phase-9 firmware):

| Case | µs | Increment |
|---|---|---|
| undriven, 0 stages | 16.2 | the floor |
| `S`, 2 ch, 1 stage | 19.2 | |
| `S`, 2 ch, 10 stages | 28.2 | ~1.0 µs per extra stage |
| `S`, 2 ch, 1 stage, default `MEAS` | 23.4 | +4.2 µs for the plan |
| `L`, 2 ch, 1 stage | 22.8 | +3.6 µs per ramp stage |
| `W`, 2 ch | 37.5 | +18.3 µs for the sine setup |

Static analysis of `tmp/b9-t35/stimjimAWG.ino.elf` (`-O2`, hard single-precision FPU,
`-fsingle-precision-constant`) attributed those numbers:

- `Engine::startTrain` is 2800 bytes / 942 instructions, `Engine::playerRun` 3164 bytes.
  `sizeof(Measure::Plan)` was 1216 B (1224 now, with the tag), `sizeof(Player)` 912 B,
  `sizeof(TrainDef)` only 164 B — which is the fact that makes a future lazy scheme possible: the
  *definition* is cheap to copy, the derived state is expensive to compute.
- `planBuild` opened with `memset` of the whole Plan. The linked `memset` stores 16 B per
  iteration: ~76 iterations, 600–900 cycles, **5–7 µs on every arm** including one that measures
  nothing.
- `sinePhaseInc` compiles to nine soft-float calls (3×`ui2d`, 3×`dmul`, `ddiv`, `adddf3`,
  `d2uiz`) ≈ 450–500 cycles, once per channel. It is the only double-precision arithmetic in the
  arm; `ampToCode`'s division is a 14-cycle `VDIV.F32` because `-fsingle-precision-constant`
  narrows the unit constants in `Stimjim.h`.
- After the `memset`, ~1000–1300 cycles remain for ~200 instructions — 5–6 cycles each, too high
  for SRAM-resident M4 code, which is what the flash-placement experiment is for.

## What was built

1. **The measurement plan is compiled once per definition.** Its compiled region depends only on
   `(slot, definition, CAL)` — never on the calibration offsets — so the Plan carries
   `tagValid/tagSlot/tagDefEpoch/tagCalEpoch` and `Measure::armPlan` skips `planCompile` when the
   tag matches. `TrainStore::epoch()` is bumped by `commit()`, `begin()` and the EEPROM restore —
   the only three writers of `slots[]` — and `Cal::epoch()` inside `Cal::set`'s existing bus lock,
   so no arm can read a new budget with an old epoch. One counter covers all slots: editing slot 7
   costs the next arm of slot 3 one recompile, which is cheaper than per-slot bookkeeping.
2. **Only the plan a train uses is zeroed.** `planCompile` zeroes the 242-byte compiled region
   (`offsetof(Plan, tagValid)`), and `planResetResults` clears 96 bytes per *existing* point rather
   than all ten. An unmeasured train, and any re-arm that hits the tag, zeroes nothing.
   The result state is still cleared **in the arm**, not at the end of the previous train: a
   trigger that re-arms the engine before `loop()` has drained the previous completion would
   otherwise have its own accumulators wiped by that drain. Keeping the invariant "an armed train
   starts from zeroed accumulators" costs ~0.5 µs and removes a race that would only appear under
   fast triggering.
3. **The sine constants are derived when `W` is parsed.** `SampleGen::sineDerive` is the pure
   derivation (host-testable, takes `cycPerUs`, the samples-per-cycle policy and the Fs clamps as
   parameters); `Engine` caches one 20-byte `SineConst` per slot, 2 kB total, dropped wholesale
   when the definition epoch changes; `Engine::deriveSine(slot)` warms it from the `S`/`L`/`W`
   commit in command context. The arm still derives on demand if the cache was invalidated, so
   correctness never depends on the warm-up — only the latency does.
4. **`rampStageInit`'s time axis is 32-bit.** With `a = dur_us/N` and `b = dur_us%N`,
   `qt = a·cycPerUs + (b·cycPerUs)/N` and `rt = (b·cycPerUs)%N` give the same quotient and
   remainder as the 64-bit `durCyc/N`, because `a·N·cycPerUs` divides by `N` exactly. Four hardware
   `UDIV`s replace two `__aeabi_uldivmod` calls. Two guards keep it exact: the 64-bit form is used
   when `b·cycPerUs` would overflow 32 bits (stages beyond ~35 s) and when `dur_us + targetDt/2`
   would overflow, which is reachable because stage durations are only bounded by `strtoul`.
   No approximation is involved and no float touches the time axis.
5. **`SJ_CODE_IN_RAM`** (default on for Kinetis) marks `Engine::startTrain` and
   `Engine::playerRun` `FASTRUN`, i.e. `.fastrun`, which the core copies from flash into SRAM at
   boot. Verified in the ELF: both symbols sit at `0x1fff…`, and RAM goes 41 404 → 47 524 B, which
   is the 6120 B the two functions occupy. `IDN` now reports `hot=RAM`, `hot=flash` or `hot=ITCM`
   (Teensy 4.x runs all code from RAM already), because a `BENCHARM` figure only means something
   next to a binary with the same answer. The reliability question it raises has a short answer:
   the RAM copy is remade from the flash image on every reset, the MCU's POR/LVD brings the chip
   down before SRAM contents could decay, and neither flash nor SRAM carries ECC here — so a
   corrupted RAM instruction is no more likely than the corrupted stack, globals and heap the
   firmware already depends on.
6. **Consecutive 0-duration `L` stages are refused at parse time.** One 0-duration stage is the
   documented instant jump and stays legal anywhere; two in a row would ask for two levels at the
   same instant, so the first could never be delivered — one sample latches, carrying the second
   value. Refusing the pair keeps the syntax honest *and* bounds the look-ahead any future
   per-stage derivation needs to one stage. `S` is untouched: a 0-duration `S` stage is a
   rectangular step of zero length, legal in any number, and `S` keeps its bit-exact legacy
   semantics.

`ampToCode`'s division was deliberately left alone. A reciprocal multiply would be faster and would
risk a 1-LSB shift in delivered amplitude, and the claim that `S` trains are bit-exact against the
legacy firmware rests on that exact expression.

## Verified

- **Host tests**: `test_cal`, `test_measure`, `test_trainstore`, `test_samplegen` — all pass. Two
  new cases carry the risky parts of this phase:
  - `testRampDivisionPaths` compares the new 32-bit path against the 64-bit reference over six
    `targetDt` values × 21 durations up to `UINT32_MAX` (both overflow guards included), and
    re-checks that N Bresenham steps still land exactly on the stage end.
  - `testPlanResetsWhatItMustNotInherit` fills a `Plan` with `0xAA` before `planBuild` and checks
    that the compiled region, the accumulators of the points that exist, `next`, `envSkipped` and
    the tag all come out clean. A field added after `acc` without a reset fails this test, which is
    the guard the narrowed `memset` needs.
- **Builds**, `--warnings more`, zero warnings: Teensy 3.5 register with `hot=RAM` and with
  `hot=flash`, Teensy 3.5 with both portable backends forced, Teensy 4.1, and Teensy 4.0 with
  `-DSJ_EEPROM_SLOTS=6`. Sizes are in `docs/hardware-variants.md`.
- **Not verified on silicon.** No `BENCHARM` figure in this entry is new; the predictions are
  arithmetic on the phase-9 measurements.

## A trap worth remembering

`--build-property "compiler.cpp.extra_flags=-DX"` is **silently ignored** by the Teensy platform —
its compile recipe never references that property, so the define never reaches the compiler and the
build looks successful while testing the wrong thing. It cost a build round here: an
`SJ_CODE_IN_RAM=0` binary came out byte-identical to the default one. `build.flags.defs` (repeating
the board's own `-D__MK64FX512__ -DTEENSYDUINO=160`) or `build.flags.optimize` work.
`docs/hardware-variants.md` already warned about this; the warning is now also next to the A/B
command in `timing.md` §7.

## Remaining

**The bench run, which is the point of the phase:**

- `tests/device/bench_arm.py <port>` — `BENCHARM` over eight slots (empty, `S` 1/10 stages,
  measured `S`, `L` 1/10 stages, `W`), printing what `STARTLAT` each needs from the board's own
  `CAL` values. Run it on both binaries (`hot=RAM` and `hot=flash`): that settles the
  flash-wait-state hypothesis and the predicted arm costs in one session.
- Then lower `CAL STARTLAT` to what the worst measured case supports, and update
  `docs/timing.md` §7 and `docs/serial-protocol.md` §4. The ten-stage `L` case is the one to watch:
  it has never been measured and projected to ~61 µs before this phase.
- `CAL TRIGCOMP` is still 0 and unmeasured — the one term of the delivered latency software cannot
  see. `docs/bench-wiring.md` configuration B, which is also the only configuration that measures
  the *absolute* edge-to-output latency; every other bench measurement here is differential.

**Firmware, in the order the plan ranks it:**

- Derive stage i+1 during stage i (`docs/PLAN_arm-cost.md` item 6) — the only route to a
  multi-stage `L` train inside 8 µs. Its prerequisite is built.
- Pre-build the plan when a `TRIG` route is set. The tag makes every arm after the first free, but
  the first arm after an edit still compiles inside the start latency.
- Pre-arm the whole train and fire on a bare `NLDAC` pulse (`timing.md` §3): an estimated 1–3 µs
  delivered latency, which makes the arm's cost irrelevant rather than smaller. It changes when
  copy-on-arm happens, so slot edits would need a re-arm to take effect — a real semantic cost, and
  the reason it is written down rather than done.
- The menu editor, still the one unbuilt item of the spec.

## Next entry point

The bench session: flash the default build, run `bench_arm.py`, reflash with
`-DSJ_CODE_IN_RAM=0`, run it again, and put both columns in `timing.md` §7. Everything else in this
phase is already reversible from a single switch or a single `CAL` line, so that measurement is
what decides which of them stay.
