# Phase 6: first hardware run, hardware variants, post-trigger delay, triggers, UI

Date: 2026-09-02

**Scope note:** this phase replaces the originally planned Phase 6 ("dual channel: dualSync,
collision-jitter benchmark, published FsMax table"). Three things displaced it: the firmware had
never actually run on hardware and did not boot; the user asked for a Teensy 4 route and a
per-waveform post-trigger delay; and the delay is untestable without the trigger start path, so
the trigger half of Phase 8 came forward. The dual-channel benchmark and the measured `FsMax`
table move to Phase 7.

**Done** (Teensy 3.5 on COM4 + PicoScope 2204A; all host tests pass; `--warnings more` clean on
Teensy 3.5 / 3.5-portable / 4.1 / 4.0):

1. **The firmware booted for the first time — and did not.** Phases 1–5 were compile-only. Two
   fatal defects in the existing Kinetis path, both found by a `-DSJ_BOOT_TRACE` build that makes
   `setup()` wait for a serial host and announce each init step:
   - `Engine::begin()` read PIT `TCTRL` **before the PIT clock was ungated**. On the K64 a load
     from a clock-gated peripheral is a bus fault, so the board locked up in `setup()` with
     interrupts disabled: USB stayed enumerated, nothing ever answered, and it looked exactly
     like a broken upload. Fixed with an explicit `pitClockEnable()` mirroring the core's own
     sequence (`SIM_SCGC6 |= SIM_SCGC6_PIT`, `nop`, `PIT_MCR = 1`).
   - **Both players were given PIT channel 0.** `IntervalTimer` claims a channel by scanning for
     `TCTRL == 0`; the old `grabChannel()` claimed one, immediately cleared its `TCTRL` during
     takeover, and the second claim then found the same channel free. The banner said
     `PIT channels 0/0`. Fixed by claiming both before disarming either (`grabChannels()`).
   The unbounded `while (!(TFLG & 1))` spin in the `K_RELOAD` calibration is what turned the
   first defect into an unrecoverable hang; it is now bounded, and a timeout leaves `K_RELOAD`
   untouched with a warning rather than poisoning every future deadline.

2. **Hardware variants (`#if` backends).** `SJ_FASTIO_REGISTER` and `SJ_TIMER_REGISTER` select
   register-level Kinetis code or a portable route on the standard Arduino `SPI` library and
   `IntervalTimer`; either can be forced from the build to A/B the portable path on a known-good
   Teensy 3.5. `SJ_CYC_PER_US` now derives from `F_CPU` (600 on a Teensy 4.x), and two constants
   that silently assumed 120 MHz — the NLDAC hold pulse and the `BENCH` cycles→ns conversion —
   are derived rather than literal. Full rationale, the recalibration procedure and the
   Teensy 4.0 EEPROM limit are in [hardware-variants.md](../hardware-variants.md).

3. **Per-waveform post-trigger delay** (the user's request). Optional 6th field of the `S`/`L`/`W`
   header, plus a `DELAY<idx>,<us>` setter and query. It applies to every start path, so it is
   testable over serial before any trigger is wired. **Measured on the scope: 2000 µs set →
   1999.3 µs, 5000 → 5000.2, 20000 → 20002.7** (skew-corrected; the residual is the scope's own
   sample interval).

4. **Triggers** (Phase 8 brought forward): `TRIG` and `R` setters, edge ISRs at priority 80 that
   arm and start in place, the stimulus-marker output, and deferred reject warnings printed from
   `loop()`. `DUMP` now emits `TRIG` lines as real set-commands, so a dump pastes back complete.

5. **Display redesign + `SCREEN`.** Three views (browse / waiting / running) with a title bar,
   engine tags and a progress bar, redrawn only when the composed text changes. `SCREEN` prints
   the framebuffer as ASCII art *and* the composed row text — the panel cannot be photographed
   over a serial link, so this is how layout changes get reviewed. Captures:
   [006-screens.txt](006-screens.txt).

6. **Bench harness** under `tests/device/`: `sjcon.py` (serial console), `pico2000.py` (ctypes
   binding for the legacy `ps2000` driver — the 2204A needs it, and it uses the DLL that ships
   with PicoScope 7, so nothing had to be installed), `smoke.py` (28 serial checks + screen
   captures) and `capture.py` (scope acceptance, figures to `figs/`, raw samples to `tmp/`).

**Decisions taken in-phase:**

- *Delay semantics*: the delay shifts the whole train timebase, so it never changes train length;
  outputs stay grounded through it. Omitting the 6th field **resets** the delay to 0, matching
  the rule the `W` envelope triplet already follows — a waveform line fully defines its header.
  That makes replayed legacy scripts reproduce legacy behaviour exactly, at the cost of an
  asymmetry with the `DELAY` setter, which is documented in protocol §2 and in the command's own
  help text. Cap 2 000 000 000 µs, chosen because `strtol` bounds the setter at `LONG_MAX`, so a
  higher cap would be reachable through one entry path and not the other.
- *STAT is unchanged* (still 8 fields). "Waiting out the delay" is exposed through `EngineStatus`
  for the display only, so no GUI parser has to change; a waiting engine reports `elapsed_us = 0`.
- *Trigger mode 1 ("joint")* starts one slot on engine 0. Which physical channels move is a
  property of the slot's own mode fields, so channel synchronisation is expressed in the waveform
  definition, not the routing. Mode 2 arms the two engines sequentially inside one ISR, which
  puts engine 1 **7.5 µs** (measured) behind engine 0 — documented, and the reason sample-
  synchronous work belongs in one slot rather than two.
- *Reentrancy*: loop-context `T`/`U` takes the bus lock around `startTrain`, since a trigger edge
  (priority 80) can preempt `loop()` and the busy check plus the arm writes are not atomic.
  Trigger ISRs need no lock: they cannot preempt each other, and the players — the only thing
  above them — never start trains.
- *Teensy 4.0 EEPROM*: the ~1.6 KB image does not fit its 1080 B, and `EEPROM.put()` past `E2END`
  is a silent no-op in the core, so the build **fails** with a `static_assert` naming the fix
  (`-DSJ_EEPROM_SLOTS=6`) rather than silently persisting less than asked.

**Scope-verified.** Traces below (PNGs committed under `figs/`, raw samples written to `tmp/` by
`tests/device/capture.py`). All three waveform types had been compile-only until now, so this
closes the Phase 3/4/5 shape acceptance. The traces also carry the bench circuit's own signature
— the 1 k/1 k divider cross-talk onto the idle channel and the ~2.2 V LED clamp — which confirms
the measurement is of the real outputs and not of an artefact.

| Figure | What it shows |
|---|---|
| [stimjim-trigger-skew.png](../../figs/stimjim-trigger-skew.png) | Both engines started by one trigger edge with delay 0: the −7.5 µs arming skew that the delay measurements are corrected for |
| [stimjim-trigger-delay-2000us.png](../../figs/stimjim-trigger-delay-2000us.png) | 2000 µs delay set → 1999.3 µs measured |
| [stimjim-trigger-delay-5000us.png](../../figs/stimjim-trigger-delay-5000us.png) | 5000 µs delay set → 5000.2 µs measured |
| [stimjim-trigger-delay-20000us.png](../../figs/stimjim-trigger-delay-20000us.png) | 20000 µs delay set → 20002.7 µs measured |
| [stimjim-shape-S-biphasic.png](../../figs/stimjim-shape-S-biphasic.png) | `S`: biphasic rectangular, ±5 V, 2 ms per phase |
| [stimjim-shape-L-ramp.png](../../figs/stimjim-shape-L-ramp.png) | `L`: a clean triangle, 4 ms up and 4 ms down, with the 20 µs Bresenham steps visible |
| [stimjim-shape-W-sine.png](../../figs/stimjim-shape-W-sine.png) | `W`: 500 Hz sine burst with the 32 kHz sample staircase (Fs = 64·f) |

## Addendum: what was measured, on what circuit, and how

Written after the fact, because the table above names the figures without saying what produced
them. Nothing here changes the results; it makes them reproducible.

### Where the traces come from

**Every trace in the table is oscilloscope data. The firmware reports no waveform samples of its
own, and none of these figures contain anything it said about itself.** A PicoScope 2204A
digitises the two StimJim outputs; `tests/device/capture.py` writes the samples to `tmp/<name>.csv`
and the plot to `figs/<name>.png`. The CSV columns are `t_s`, `chA_V`, `chB_V` — seconds and volts
at the scope's inputs, nothing else.

The firmware's own reports are a separate and much coarser thing: `STAT` counters polled over
USB, good to roughly ±200 ms because one poll costs 60–150 ms. `smoke.py` uses them as a sanity
check and never as a measurement. That is the whole reason a scope is on this bench.

### The test circuit

```
PicoScope AWG   ->  StimJim IN0            (trigger input)
StimJim CH0(+)  ->  PicoScope channel A
StimJim CH1(+)  ->  PicoScope channel B
StimJim CH0(-)  ->  PicoScope ground       (the common reference for both scope inputs)

load chain, four nodes in series:

    CH1(+) ---- 1k ---- CH0(+) ---- 1k ---- CH0(-) ---[antiparallel LEDs]--- CH1(-)
      |                    |                   |      (one red, one blue)
   scope B              scope A            scope gnd
```

Both scope inputs sit on the ±10 V range. The four load nodes are one series chain, not two
independent loads, and that is deliberate: the isolated outputs need *some* return path, and a
shared chain gives one. (The original wiring note labelled the chain nodes A–D, which collides
with the scope's own channel names A and B — the labels above are explicit instead, because the
collision is easy to trip over.)

**Both channels are grounded, not hi-Z, whenever a train is not driving them.** `mode = 3` is
`GROUND` and `mode = 2` is `HIGH-Z` in the output-enable decoder, and `Stimjim.begin()` ends by
setting both channels to 3, as does the end of every train. A `mode1 = 3` in a *train
definition* means only "this train does not drive channel 1 — leave its pins alone", so what
channel 1 is actually doing during a CH0-only capture is holding the standing state: grounded.

That matters for the question of whether scope channel B has a reference at all, and the answer
is that it always does, for two independent reasons. A scope input is a high-impedance voltage
measurement and needs no return current of its own — only a defined potential — and CH1(+)
reaches scope ground through 1 k + 1 k whatever the output stage does. Grounding CH1 adds a
second path: it ties CH1(+) to CH1(−), which reaches scope ground through the LED branch. Had
CH1 been left hi-Z (`M1,2`) the measurement would still have been defined, through the two
resistors alone.

**What the circuit puts on the traces.** Both signatures are worth recognising before reading a
figure as a fault, and both were checked against the captured samples in `tmp/`:

- **A CH0 pulse appears on scope channel B, LED-clamped.** With CH1 grounded, CH1(+) is tied to
  CH1(−), which reaches scope ground only through the antiparallel LEDs. Below their turn-on no
  current flows in that branch, so none flows through the 1 k beside it either and CH1(+) simply
  sits at CH0(+)'s potential: in the `L` ramp capture scope B tracks scope A within 1–4 % up to
  about 1.8 V. Above turn-on the LEDs conduct and clamp it — scope B saturates at **+2.2 V one
  way and −2.9 V the other**, the two antiparallel devices not being identical. So the flat tops
  on scope B in the shape figures are the LED drop, and the channel is *not* emitting.
- **A CH1 pulse divides down onto scope channel A.** The other direction, visible in the delay
  captures: while CH1 drives its 8 V reference pulse and CH0 is grounded, scope A reads
  **0.49–0.58 V, about 11 %** of it — the residue across CH0's grounded output stage and the
  divider, not an output.

Neither signature affects amplitude accuracy, which is not what these captures test: scope A
reads about 4.1–4.2 V for a programmed 5 V on CH0. Shape and timing are what they test.

### What each figure actually played

Slot numbers 10–14 are reserved for the bench so nothing a user stored gets overwritten, and
`capture.py` resets them afterwards.

**The three shape figures — CH0 only.** All three are defined with `mode0 = 0` (voltage) and
`mode1 = 3`, so **only channel 0 emits** and channel 1 stays grounded throughout. The scope B
trace in those figures is therefore not an output at all: it is CH0's own waveform reaching
CH1(+) through the load chain, clamped by the LEDs as described above. Each train repeats every
20 ms and runs for 3 s, and the scope triggers on the stimulus itself:

| Figure | Slot line | In words |
|---|---|---|
| `stimjim-shape-S-biphasic` | `S12,0,3,20000,3000000;5000,0,2000;-5000,0,2000` | Two rectangular steps back to back: +5 V held for 2 ms, then −5 V held for 2 ms, then the output parks and grounds. Repeats every 20 ms. |
| `stimjim-shape-L-ramp` | `L13,0,3,20000,3000000;5000,0,4000;0,0,4000` | A triangle: a straight climb from 0 to +5 V over 4 ms, then a straight fall back to 0 over 4 ms. The 20 µs Bresenham sample steps are visible on the slopes. |
| `stimjim-shape-W-sine` | `W14,0,3,20000,3000000;5000,0,10000;500,0,0;0,0,0;0,0,0` | A 500 Hz sine at 5 V amplitude, starting at phase 0, burst-gated: 10 ms of sine (five cycles) per 20 ms period, output parked and grounded in between. No envelope. Sampled at Fs = 64 × 500 Hz = 32 kHz, so the staircase in the figure is 32 kHz. |

**The four delay figures — both channels, one per engine.** These need two simultaneous outputs,
so each engine drives one channel:

- **Channel 0 (engine 0) is the pulse under test**: `S10,0,3,50000,1000,<delay>;5000,0,2000` —
  voltage, +5 V, a single 2 ms pulse per trigger, started `<delay>` µs after the trigger edge.
  (The train's `duration` of 1000 µs is shorter than one period, so exactly one pulse runs; its
  2 ms stage then plays to completion, which is the legacy stage semantics.)
- **Channel 1 (engine 1) is the zero-delay reference**: `S11,3,0,50000,1000,0;0,8000,2000` —
  voltage, +8 V, a single 2 ms pulse with no delay. 8 V rather than 5 V because CH1 drives
  through both 1 k resistors *and* the LEDs, so it arrives at the scope smaller.
- One trigger edge starts both, through `TRIG0,2,10,11,0` (mode 2 = independent, rising edge).
- The trigger stimulus is the scope's own AWG: a 1 Hz, 0–2 V square wave into IN0.

`stimjim-trigger-skew` is this same pair with the delay set to 0, which is what isolates the
engine-to-engine arming skew.

### Measuring a delay when you cannot see the trigger, and cannot trigger on it either

Two constraints shaped the method, and both are ordinary bench constraints rather than anything
specific to this instrument:

1. **The trigger source is not observable.** The AWG wire runs to IN0 and is not teed into a
   scope input, so there is no trace of the edge whose delay is being measured. (Teeing it would
   have been the easy answer; the scope has only two inputs and both were needed for the
   outputs.)
2. **Triggering near ground does not work.** The 2204A is 8-bit, so on the ±10 V range one code
   is 78 mV. A trigger threshold set within a few codes of the baseline sits inside the trigger
   hysteresis and the scope simply never fires. Thresholds have to stand ≥1 V clear of the
   baseline, which rules out catching a signal at the instant it leaves zero.

The way around both is to **stop trying to time against the trigger, and time against a second
output instead** — a differential measurement:

1. Route the same trigger edge to both engines (`TRIG` mode 2). Engine 1 plays a **reference
   pulse with zero delay**; engine 0 plays the **pulse under test** with the delay being checked.
   The reference pulse is now a proxy for the trigger instant, and unlike the trigger it is a
   several-volt edge on a scope input.
2. **Trigger the scope on the reference pulse**, not on the trigger signal — at 4.0 V on an 8 V
   pulse, far outside the hysteresis. Set 5 % pre-trigger so the baseline before the edge is
   visible and the crossing can be interpolated from both sides.
3. Find both edges in the captured trace by linear interpolation between the two samples that
   straddle a threshold (`first_cross`): 4.0 V on the reference, 2.5 V on the pulse under test.
   Interpolating recovers timing well below one sample interval, which matters because the
   buffer is only 3968 samples for both channels together.
4. **Calibrate away the systematic offset.** The two engines are not armed at the same instant —
   the trigger ISR arms them one after the other — so the raw difference contains a fixed skew.
   Measure it by running the identical capture with the delay set to 0 (`stimjim-trigger-skew`:
   7.5 µs, engine 1 behind engine 0), then subtract it from every subsequent reading:

   ```
   delay = (t_test − t_reference) − skew
   ```

   This is the standard trick of measuring a difference and removing it with a null measurement
   taken through the same signal path: whatever is common to both — cable lengths, scope channel
   skew, threshold placement, the DAC's own settling — cancels, and only the quantity under test
   is left.

What survives after the subtraction is the sample interval, which is why the residuals in the
table are a fraction of a µs: 2000 µs set → 1999.3 measured, 5000 → 5000.2, 20000 → 20002.7.
`capture.py` chooses the timebase per delay so the whole interval plus 4 ms of margin fits the
3900-sample capture, so the longest delay is also the one with the coarsest samples — the 2.7 µs
residual at 20 ms is one sample interval, not an instrument error.

The same "no usable trigger source" problem appears in the shape captures, where it has a
simpler answer: there is no trigger routing at all (`TRIG0,0,-1,-1,0`), the train is started over
the serial port, and **the scope triggers on the stimulus itself** at 2.0 V (S), 1.5 V (L) and
2.0 V (W) — again ≥1 V clear of the baseline. That only works because the train is long: a
one-shot train cannot be armed against, since the serial round trip that starts it is tens of
milliseconds, far longer than the capture. So the shape trains run for 3 s and repeat every
20 ms, and the capture is armed calmly inside one and catches a whole pulse.

**Bench notes worth keeping** (the addendum above explains what each one forced): the 2204A is
8-bit, so on the ±10 V range one code is 78 mV and a trigger threshold within a few codes of
ground sits inside the trigger hysteresis and never fires. Its capture buffer is 3968 samples
with both channels enabled. A one-shot train cannot be started *after* arming the scope from the
host, because the serial round trip alone is tens of milliseconds. Close the PicoScope
application before running `capture.py`: it holds the USB device exclusively and `ps2000_open_unit`
then returns 0.

**Open questions / still owed:**

- The register-path timing constants (`SJ_DAC_PROG*_US`, `SJ_PRELOAD_US`, `SJ_FS_MAX_HZ`) are
  still Phase-1 desk estimates: the `BENCH` group has never been run on hardware, even though the
  board is now known to work. That is the first thing to do in Phase 7 and it is now cheap.
- Plan §7 items 1–4 (AD7321 first-conversion validity after a line switch, AD5752 settling vs
  NLDAC, MISO mux glitch, the real GPIO header pins) remain open; 1 and 2 gate the Phase 7
  measurement GUARD.
- Neither Teensy 4 build nor the portable backends have run on silicon — they compile and are
  structurally correct, nothing more.
- `K_RELOAD` measured 96–112 cycles across boots; `BENCHK` should quantify that spread.

**Next entry point:** Phase 7 — run the `BENCH` group on hardware and replace the estimated
constants; then the measurement engine (`MeasurePlan` execution, `MSUM` summaries), `SdLog` and
the MDATA ring. The displaced dual-channel collision benchmark and the measured `FsMax` table
belong with that bench session.
