# 017 — the start latency inside one latch interval: STARTLAT 20, TRIGCOMP 2

The Teensy 3.5 in hand now runs `CAL STARTLAT` = 20 µs and `CAL TRIGCOMP` = 2 µs, persisted to its
EEPROM image, and delivers its first output step **20.26 µs (sd 41 ns)** after the physical trigger
edge crosses the input pin's own switching threshold. The whole start latency therefore fits inside
one default 20 µs ramp sample interval, which is what the change was for. This is the first time
the project has both lowered a timing budget and compensated the hardware delay underneath it, and
both halves were verified on silicon rather than derived.

The number is available because phase 14 moved the arm's preparation into `loop()` and phase 16
measured the hardware delay. Neither had been cashed in: `bench_arm.py` had not been re-run on
silicon since phase 14, so `docs/` still carried the phase-13 arm figures and the 35 µs budget
sized from them.

One consequence has to be accepted with it: **an independent `TRIG` route no longer fits.** It
arms two engines from one edge and needs 30–35 µs. Joint routes — one train driving one or both
channels — have 3 µs of margin.

## What the arm actually costs now

`python tests/device/bench_arm.py COM4`, warmed column:

| case | warmed arm |
|---|---|
| empty, drives nothing | 5.92 µs |
| `S` 1 stage, 1 ch, no meas | 5.92 µs |
| `S` 1 stage, 2 ch, V+I both | 5.96 µs |
| `S` 10 stages, 2 ch | 5.96 µs |
| `L` 1 stage / `L` 10 stages | 6.00 / 5.96 µs |
| `W` sine, 2 ch | 6.00 µs |
| `S` 10 stages + 10 MEAS points | 6.00 µs |

**Flat.** Stage count, waveform type and in-train measurement all cost nothing, which is exactly
what "the preparation happens in `loop()`" is supposed to mean and is the first silicon evidence
that phase 14 delivered it. The phase-13 figures this replaces were 9.0 µs undriven rising to
15.0 µs for a ten-stage `L`. The *cold* path — an engine re-triggered before `loop()` could prepare
it — is unchanged and still costs the old 9–20 µs; the worst column of the same run shows it
(11.5–24.8 µs).

## Sizing the budget, and why the arithmetic is not the answer

The window between the ISR timestamping the edge and the first DAC programming starting is
`STARTLAT − TRIGCOMP − PRELOAD − DACPROG2`, and the arm plus the scheduler's 3 µs slack has to fit
in it. So

```
STARTLAT >= arm + PRELOAD + DACPROG2 + SJ_MIN_SCHEDULE_US + TRIGCOMP
         =  6.0 +    4    +    5     +        3           +    2      = 20 us
```

which puts 20 exactly on the floor. It is not: `tests/device/startlat_trig.py` puts real trigger
edges on the heaviest trains at a candidate and reads the firmware's own verdict, and the break is
between 16 and 18 — at 16 the completion says `set CAL STARTLAT >= 17 us`, and 18 and 20 run clean.
So the arithmetic is about 3 µs conservative, and **20 carries 3 µs of real margin** on a joint
route.

`Cal::validate`'s own floor is looser still and unrelated: `PRELOAD + DACPROG2 + 3 + TRIGCOMP` =
14 µs, which only ensures the scheduler can place the first latch at all, and knows nothing about
the arm.

That the qualifier has to use a trigger and not `T`/`U` is the point phase 13 established and it
still holds: a `T` start reads its own clock *after* arming, so the arm is never charged to the
latency and every candidate passes.

## The independent route is what 20 µs costs

An independent route (`TRIG<n>,2,<slotA>,<slotB>,<edge>`) arms two engines inside one ISR, so it
pays the remainder twice. Measured with two ten-stage single-channel `S` trains:

| `CAL STARTLAT` | joint | independent |
|---|---|---|
| 16 | FAIL, wants 17 | — |
| 18 | ok | FAIL, wants 35 |
| 20 | ok | FAIL, wants 35 |
| 30 | ok | ok |
| 35 | ok | ok |

The firmware names 35 while 30 passes, because the value it names comes from the heaviest arm it
actually observed, which includes a cold one.

This is a real narrowing and is documented as such in `docs/timing.md` §1: a bench that routes two
slots independently from one input has to put `STARTLAT` back to 30 or more and give up the
sample-aligned start latency. It is not a silent failure — the train still runs, and its completion
names the value that would have covered it. And a single train driving both channels, which is
already the right construct for sample-aligned stimulation (independent routes contend for their
first latch by ~10 µs, configuration A), keeps both properties.

`startlat_trig.py` now covers both route modes. Two changes were needed to make its verdict mean
anything:

- **The independent case has to be `S`, not `L`.** An `L` stage subdivides into 20 µs ramp samples,
  so two engines ramping at once latch every ~10 µs between them and collide all train long. The
  first attempt drowned the arm's verdict under ~2470 "already due" notes per train, which measure
  dual-engine throughput and not the arm at all. Ten `S` stages latch ten times per period.
- **The verdict has to separate the arm from throughput.** `WARN … took longer than CAL STARTLAT`
  and `WARN … overran their deadline` decide the candidate; `# … already due when the player
  reached them` is reported as a note, because two busy engines produce it at any `STARTLAT` and
  raising `STARTLAT` does not fix it.

## Compensation, and what it does not fix

`CAL TRIGCOMP` = 2 subtracts 2 µs from `t0`, so `t0` is scheduled 18 µs after the edge and the
output moves at 18 + 2.26 = 20.26 µs. Re-measured with the new budget in force: **20.263 µs** at
8000 mV and **20.278 µs** at 2000 mV, sd 41 ns. The hardware term comes out 2.26 µs against the
2.27 measured at `STARTLAT` 35 — the check that it does not depend on the budget it is measured
against.

The residual 0.26 µs is arithmetic, not error: `CAL` takes whole microseconds and the hardware
delay is 2.27.

**A triggered start now leads a `T`/`U` start by about 2 µs.** `TRIGCOMP` is subtracted on the
trigger path only, while two of its three terms — the PIT wake and the DAC's latch-to-output delay
— are common to both. That is the accepted trade: it buys an absolute, repeatable edge-to-output
latency, which is what a trigger input exists to provide.

## The compiled defaults are unchanged, on purpose

`SJ_START_LATENCY_US` is still 35 and `TRIGCOMP` still defaults to 0. Only this board's EEPROM
image carries 20 and 2. That is deliberate: `TRIGCOMP` is a property of the board, not a constant
to copy, and changing the compiled default would claim a value for boards nobody has measured — and
would need a rebuild and a reflash of all four configurations. A second board adopts the setting
with `CAL,STARTLAT,20`, `CAL,TRIGCOMP,2`, `P` — after re-running `bench_arm.py`, `startlat_trig.py`
and configuration B on it.

`P` does persist `CAL`, though its reply does not say so: `TrainStore::eepromSave` writes
`eeImg.cal = Cal::live()` (`TrainStore.cpp:548`) and `eepromRestore` validates it before applying
(`:569`). The message "First 10 slot definitions and the trigger table saved to EEPROM." is
misleading and is worth one line of firmware some time.

## Two harness bugs fixed on the way

- **`sjcon.ASYNC` never learned about `MRANGE`.** Phase 15 added an `MRANGE` record beside every
  `MSUM`, and `WARN engine:` follows a completion whose arm did not fit; neither was in the tuple
  of lines `cmd1` treats as asynchronous. A train completing in the middle of any command therefore
  derailed the next `cmd1` with an assertion. It only surfaced now because this is the first
  session to run commands while trains were completing *and* warning.
- **`make_bench_figures.py` hardcoded `CAL_STARTLAT = 35`.** Re-measuring at 20 would have redrawn
  phase 16's figures with the wrong label. `trigcomp.py` now writes `startlat_us` and `trigcomp_us`
  into every row of its per-shot CSV and the figures read them, so a figure describes the run that
  produced its data. `make_timing_figures.py`'s `CAL` dict was updated by hand and its panel-A
  annotation, which read "TRIGCOMP = 0 … unseen by software", now names the three delays the 2 µs
  covers together.

## Open

1. **The 4× voltage-mode current readback** (phase 16) is untouched and still the top item.
2. **Decide whether the compiled defaults should follow.** That is a rebuild, a reflash of four
   configurations, and a re-run of `smoke.py`.
3. **Separating the three delays inside `TRIGCOMP`** needs a probe on `NLDAC`, which no connector
   carries.
4. Configuration C — dual-channel collision jitter and the real `SJ_FS_MAX_HZ` — is still unrun.

## Entry point

`python tests/device/startlat_trig.py COM4 20` says in ten seconds whether this board still holds
the budget, for both route modes. `python tests/device/bench_arm.py COM4` says what the arm costs.
`CAL` on the serial port says what is in force.
