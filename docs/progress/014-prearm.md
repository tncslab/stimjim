# 014 — the arm moved out of the trigger path

`Engine::startTrain` used to run entirely inside the trigger ISR, and after phase 13 it was still
9.0 µs for a train that drives nothing and 15.0 µs for a ten-stage `L` train — a floor made of ten
thin terms rather than one fat one, which is why phase 13 declared it finished as a target. This
phase stopped trying to shrink it and moved it instead. Each engine now holds two `Player` buffers;
`loop()` writes everything a start does not need the start *time* for into the spare one, for
whichever slot a `TRIG` route would fire next, and a trigger edge places `t0`, rebases the
envelope, re-reads the park codes, attaches the plan, swaps the live pointer and programs the
timer. Two `cycles64()` reads, one `pitProgram` and a few dozen instructions, on a train of any
shape. Two smaller items came with it: the per-stage look-ahead stopped asking the clock how many
stages it could afford and now derives the stage due next plus, for a 0-duration stage only, the
one behind it; and an `S`/`L` line that asks for stage boundaries closer together than one latch
costs says so when it is defined instead of only through the deadline counters at train end. Four
builds and four host suites are clean. **`CAL STARTLAT` is unchanged at 35 µs**, because a start
latency is a measurement and no board was attached to this machine — the phase ends with the
procedure that lowers it.

## What was built

**`Engine::prepareArm(Player&, slot, def, err, errsz)`** (`SJ_HOT`) — the whole of the old arm
except the parts that need `t0`: `buildGeometry` and the `Measure::Geometry` it fills, the
type/mask/mode fields, the park codes, the sine constants and amplitudes, `dtUs`, the `memcpy` of
the stage triplets, `deriveStage(0)`, the period/duration/preload/delay scalars, `startLatCyc` and
`trigCompCyc`, every counter reset, `evIdx`/`evPhase`/`prevEvDl`, and the envelope's shape.

**`Engine::prepareArms()`**, called every `loop()` pass, is the old `warmPlans()` with the player
preparation in front of the plan warm. It reuses `routedSlot()` unchanged — the slot a `TRIG` route
would fire, falling back to the last slot written — so the "we know which two signals to prepare
for" reasoning that phase 12 built for the measurement plan now serves the whole arm. The plan warm
takes the geometry out of the player it just prepared, so there is still exactly one copy of
`buildGeometry` in the firmware; two would let a warmed plan's tag claim a geometry the arm
computes differently.

**Two players per engine.** `static Player playerBuf[2][2]` with `live[2]` and `warm[2]` pointers;
the player ISR reads `*live[p]` (one pointer load it was already doing as an array index), and the
arm swaps the two. +2.6 kB of RAM, and 209 kB still free on a Teensy 3.5. `Player` also grew the
`Measure::Geometry` and the ramp sample counts, so a prepared buffer is self-sufficient: the arm
can hand it to `Measure::armPlan` even on the path that has to compile the plan itself, which is
what let the whole change happen without touching `Measure` at all.

**The invariant is one flag.** `armReady` is set only by `prepareArm` and cleared only by the arm
that consumes the buffer, so a buffer that has ever played is never mistaken for a prepared one —
its counters, `evIdx`, `evPhase`, `nDerived` and ramp cursor are all mid-train state. Beside it sits
the measurement plan's tag: slot, `TrainStore::epoch()`, `Cal::epoch()`. **A preparation is a cache,
not a snapshot**, which is the point: an edge whose prepared player fails the tag prepares one in
place and pays the old cost, so a slot edited a microsecond before an edge still plays as edited.
That property is what `docs/timing.md` §3 listed as the price of pre-arming, and the tag buys it
back.

**The race the plan did not have, and the handshake that covers it.** The edge ISR reads
`warm[eng]`, so a `loop()`-side preparation interrupted half-way would resume into a player that
had meanwhile started playing. Masking the trigger ISR is not an option — it would delay the edge
timestamp the entire latency is anchored to, and delay it invisibly, which is exactly the failure
mode the anchor exists to prevent. A `volatile bool prepBusy[2]` covers it the way `Measure`'s
`loopBusy` covers the plan buffers: an edge inside the window arms the *live* buffer in place
(idle, because the busy check passed) and swaps nothing, which is what the single-buffer firmware
always did. `prepareArms` raises the flag and only then reads the pointer — reading it first would
have moved the same bug rather than fixed it — and drops it before the plan warm, so the steady
state, where there is nothing to prepare, never raises it at all.

**The park codes are the one thing a preparation cannot freeze.** `A`/`B`/`C` and the boot
calibration move them, they are outside the tag, and the player ISR latches them directly as the
inter-pulse level, so the arm re-reads them: two loads. Stage amplitudes need no such treatment —
`ampToDelta` returns a delta the offset cancels out of, except at saturation, where a recalibration
between the preparation and the edge can shift a code that was already out of range and WARNed at
parse time.

**`SampleGen::envInit` split into `envShape` + `envRebase`.** The shape carries the two FP32
reciprocals — the only divisions the envelope needs — and the three lengths; the rebase places them
on a `t0` with four 64-bit adds. `envInit` remains as the wrapper the cold path and the host tests
call, so nothing that used it changed. `EnvCoef` grew 24 bytes.

**The stage look-ahead is structural, not clock-driven.** `deriveAhead` derives the stage due next
and keeps going while the stage it has just derived has `dur_us == 0`:

```c
do { deriveStage(pl, pl.nDerived); }
while (pl.nDerived < pl.nStages && pl.stages[pl.nDerived - 1].dur_us == 0);
```

A 0-duration stage is the one kind that yields no gap of its own — its single sample shares a
deadline with the previous stage's last one, so entering it hands the player no idle time and the
stage behind it must already be there. That covers the `L` pair (a second 0-duration stage in a row
is refused at parse time) and an `S` run of any length, which nothing refuses — the old rule's
justification quietly assumed the `L`-only refusal applied to both. `SJ_STAGE_DERIVE_US` survives
as the single entry guard, the only clock reading the function makes, and `ensureDerived` at each
use site is still the correctness guarantee.

What the old rule bought was at most one derivation's worth of microseconds on a latch that cannot
be on time anyway: the second latch of a coincident pair is late by definition, `playerRun`
excludes such pairs from `overdueEvents`, and the 8–9 µs analog settling means the intermediate
value never reaches the output. Deriving as many stages as the clock said would fit was work in
service of an invisible microsecond.

**`TrainStore::minLatchGapUs(def, dtUs)` and `Cal::minLatchUs(cal, bothChannels)`**, joined in the
`S`/`L`/`W` handler. The first is definition-only and host-tested: the shortest nonzero interval a
definition puts between two consecutive latches — `dur_us` for an `S` stage, `dur_us / N` for an
`L` stage of `N` samples, `UINT32_MAX` for a sine or an undriven train, and 0 gaps excluded because
an instant jump *means* two latches at one instant. The second is `PRELOAD + DACPROG1/2 + 3`, which
is 12 µs on a two-channel train with the measured defaults. A line that asks for less is accepted
with

```
WARN S: 3 us between two latches is under the 12 us one latch costs on this board — those latches will be late
```

The hole it closes: `buildGeometry` refuses a ramp *sample interval* below that figure, but nothing
checked stage durations, and `rampStageN(3, 20)` is 1 — a stage with one sample puts it at the
stage *end*, so ten 3 µs `L` stages schedule ten latches 3 µs apart and every one is late. An `S`
train had no check at all. It is a warning and not a refusal: the figure is runtime state, the
deadline counters report what actually happened, and nothing under about 10 µs settles at the
output regardless.

## What was reviewed and left alone

**The 9.04 µs floor, term by term, without instruction-level measurement.** 1085 cycles at 120 MHz
over roughly 150–250 instructions — the busy check, a `Cal::live()` copy, `buildGeometry`, the
conflict check, twenty-odd `Player` stores, `Measure::armPlan`, a dozen counter resets, `envInit`,
`t0`, two LED writes and `pitProgram` — is 4–7 cycles an instruction, which is what a Cortex-M4
costs when the work is 64-bit `SJ_US_TO_CYC` multiplies, struct copies through memory, and five PIT
register writes across a 60 MHz peripheral bus. There is no single fat term left; there are ten thin
ones. The original `stimjimPulser` was quicker to its first pulse for one structural reason and not
because it computed less: `startIT0` called `pulse0()`, so the pulse was latched inside the trigger
ISR. This phase takes that idea and does the conversion before the edge instead of after it.

**Serial-start latency, which is deliberately not a number.** A `T`/`U` command's arrival is
quantized to the 1 ms USB frame and batched by the host driver on top of that, and it then waits for
`Protocol::poll()` in a `loop()` pass that may be flushing the SD card — so it is repeatable to
milliseconds at best, and nothing in the firmware can see, let alone compensate, what happens
before the byte arrives. `startTrain` already reflects that: with no anchor it reads its own clock
at the *end*, so a `T` start means "first latch one `STARTLAT` from now" and is never charged for
the arm (which is also why phase 13 found that a `T` start cannot qualify a `STARTLAT`). Everything
*inside* such a train is still exact from `t0`, and the completion and `MSUM` timestamps report the
`t0` it got. Recorded in `docs/timing.md` §2 and on the `T` row of `docs/serial-protocol.md` §3.

## The figures

`docs/figures/make_timing_figures.py` was reworked around this change and now emits three figures
instead of two. Rows are execution contexts — `loop()`, the trigger ISR at priority 80, the player
ISR at priority 64, the analog output — tinted and named on the y axis, with each row's explanation
in a notes column inside that row, so no label crosses a row boundary. Bar colour keeps its own
meaning: blue DAC programming, green ADC reads, pink settling and guard, orange software arming,
dark orange the `NLDAC` pulse, grey idle. The `NLDAC` pulse is drawn explicitly wherever it happens,
because "which interrupt moves the output" is the question the old figures did not answer.

- `stimjim-timing-latch.png` panel A is the start path at `STARTLAT` = **20 µs**, showing the
  prepared arm (2–4 µs, hatched and labelled as an estimate) against the cold arm (9.0–20.0 µs
  measured, with its three known points marked), which makes visible that a cold arm does not fit
  20 µs. Panel B is one latch event, with the SPI write separated from the pulse that acts on it.
- `stimjim-timing-contexts.png` is new: the same division of labour over three pulses of a
  millisecond-period train, where each latch is one tick because its 9 µs preload window is thinner
  than a line at that scale.
- `stimjim-timing-measurement.png` keeps its arithmetic — no `CAL` value in it changed — and gains
  the row structure, which is what says that a measurement point's reads happen inside one
  player-ISR pass rather than between interrupts.

20 µs is a candidate and not the shipped default: `SJ_START_LATENCY_US` is still 35, and the
figure's own caption says so. It was chosen for the figures because it is what the prepared path
makes reachable (4 + 9 = 13 needed, against a 12 µs floor) while still being a round number a
reader can check.

## Verification

- Host suites `test_cal`, `test_samplegen`, `test_trainstore`, `test_measure` — all four build clean
  at `-Wall -Wextra` and pass. New checks: `envShape`+`envRebase` reproduce `envInit` field for
  field at three start times and a shape re-placed twice leaves no trace of the first placement;
  `minLatchGapUs` over `S` with and without a 0-duration stage, an undriven train, an `L` stage
  sampled at `dt`, an `L` stage under `dt/2`, and a `W` train.
- Builds, all clean at `--warnings more`: Teensy 3.5 register (170 988 B flash, 53 012 B RAM,
  against 170 892 / 50 436 before), Teensy 3.5 portable (172 088 B), Teensy 4.1 (RAM1 variables
  69 088 B), Teensy 4.0 with `SJ_EEPROM_SLOTS=6`.
- **Not run on silicon.** No board was attached to the machine this phase was written on. Every
  timing claim above about the *new* path is an instruction count, not a measurement.

## Next entry point

The one thing this phase owes: **measure the prepared arm and lower `CAL STARTLAT`.** In order, on
a board:

```
python tests/device/bench_arm.py COM4 --csv tmp/arm-p14.csv   # the warmed column is the answer
python tests/device/stage_timing.py COM4                      # the look-ahead still fits
python tests/device/smoke.py COM4
python tests/device/startlat_trig.py COM4 <candidate>          # needs the bench AWG
```

`STARTLAT` = warmed arm + `PRELOAD` + `DACPROG2`, floored at 12 µs by `Cal::validate`; adopt it
with `CAL,STARTLAT,<us>` and `P`. Only `startlat_trig.py` can qualify it — over the serial port
alone every candidate passes, because a `T` start is not charged for the arm. `SJ_START_LATENCY_US`
should then be changed in `Config.h` too, with the EEPROM version left alone for the reason phases
12 and 13 gave: a stored 35 is conservative, not wrong, and rejecting the image would destroy the
operator's slots 0–9 to deliver an improvement that two serial lines adopt.

One trade to decide when that number is picked: a `STARTLAT` tuned to the prepared path (~13–15 µs)
does not cover a cold arm, so the first edge after a slot edit — or after a `loop()` pass long
enough to starve the preparation — reports `set CAL STARTLAT >= N` and delivers its first latch
late by the shortfall. Nothing else about the train changes: deadlines are absolute and derived from
`t0`, so the grid, the envelope and every later sample stay where they belong. 20 µs covers a bare
cold arm (9 + 9 = 18) but not a heavy one; the value that never warns at all is cold arm + 9.

Then, in order:

- **`CAL TRIGCOMP` is still 0 and unmeasured** — the pin-edge-to-ISR-entry delay, the one part of
  the delivered latency software cannot see. `docs/bench-wiring.md` configuration B is the
  procedure; `startlat_trig.py` already drives the AWG and routes a trigger, and what it does not do
  is capture the two edges.
- **Preload the DAC and fire the first latch on a bare `NLDAC` pulse** (`docs/timing.md` §3). This
  phase built the half that made it hard: the first sample's code is now known before the edge. What
  remains is writing it into the input register, pulsing `NLDAC` in the edge ISR, and re-doing the
  preload whenever the preparation is re-done — which the epoch tag already signals. That is the
  only route below the 12 µs scheduling floor, and it is worth an estimated 1–3 µs delivered
  latency.
- **The menu UI's editing FSM**, still the last unbuilt item of `firmware-spec.md`.
