# 016 — configuration B: the absolute trigger latency, and two faults found on the way

The last quantity in the stimjimAWG firmware without a measured value now has one. On a Teensy 3.5
running fw 0.8.0 with the register backends, the channel-0 output leaves its baseline **37.27 µs
(sd 49 ns over 30 shots)** after the physical rising edge at trigger input IN0 crosses the level
that pin actually switches at. Against the `CAL STARTLAT` of 35 µs in force, that leaves
**2.27 µs** for `CAL TRIGCOMP` — an order of magnitude more than the "few hundred nanoseconds"
the bench-wiring document predicted, because the measurement sees not just the pin-to-ISR delay
`TRIGCOMP` is named for but also the PIT wake with its 0.8 µs quantum and the AD5752's
latch-to-output delay, and this wiring cannot separate the three. `CAL TRIGCOMP` is left at **0**:
it is subtracted on the trigger path only, while two of its three terms apply to a `T`/`U` start
as well, so applying it would make triggered trains lead software-started ones by ~2 µs. That is a
decision about what the instrument should guarantee, not a calibration, and it is left open.

Getting there took correcting the prescribed procedure in two places, and turned up two faults
that had nothing to do with the measurement: the board's channel-0 current readback is 4× low in
voltage mode, and the scope driver had been throwing away 32× of its own time resolution since it
was written.

## What was on the bench, and what the checks say

Configuration B as `docs/bench-wiring.md` describes it, with a 1 kΩ resistor across CH0. All six
wiring checks pass. They are worth listing because each one fails differently and names its own
wire:

| | check | how it is proved | result |
|---|---|---|---|
| 1 | the AWG reaches scope B | a 1 kHz square is visible on B | 2.16 V swing |
| 2 | the AWG reaches IN0 | trains fire from the edges | 7 in 3 s at 2 Hz |
| 3 | scope A is on CH0(+) | a ±8 V train appears on A | 13.96 V swing |
| 4 | scope ground is at CH0(−) | A is single-ended, not a divider tap | +6.90 / −7.06 V |
| 5 | CH1 and the LED branch are gone | a CH0 pulse does not appear on B | B moves 0.09 V |
| 6 | the load is a plain resistor | the same value through both output modes | 993 Ω |

Check 5 is the one that separates configuration B from configuration A: in A a CH0 pulse couples
onto scope B through the LED branch, so B staying flat while CH0 swings ±7 V is proof the chain is
out. Check 6 changed shape during the session and is discussed below.

`tests/device/trigcomp.py` runs all of it — `check`, `threshold`, `measure` — and is the entry
point for repeating any of this.

## Two corrections to the prescribed procedure

**The reference instant has to be the pin's own threshold, not half the AWG's amplitude.** The
PicoScope 2204A's square wave slews its middle 1.58 V in 2.08 µs — 0.75 V/µs — so where on that
ramp the clock starts matters enormously. The same captured output foot gives 39.01 µs referenced
to 0.50 V and 37.27 µs referenced to 1.76 V: a 1.7 µs spread, which is *most of the quantity being
measured*. The original procedure said "half the AWG amplitude", i.e. 1.00 V, which would have
given 38.61 µs and a `TRIGCOMP` of 3.6 µs.

`Triggers::begin` sets the pin to plain `INPUT` with no pull, so the pin sees the BNC directly and
its threshold can simply be measured: walk the AWG's peak down until trains stop firing. The
transition is sharp — 1.780 V fires every edge, 1.735 V fires none — giving **1.757 ± 0.022 V**,
about 0.53·VDD, and reducing the reference-instant doubt to ±29 ns. That is the `threshold`
subcommand.

**The output's foot has to be timed, not its 50 % point.** The 50 % crossing arrives after roughly
half the output's settling, and the settling depends on the step size, so the 50 % estimator gives
38.96 µs on an 8 V step and 39.40 µs on a 2 V one — a 0.44 µs disagreement that is pure artefact.
The first departure from baseline gives 37.267 µs and 37.261 µs for the same two steps: **six
nanoseconds apart across a 4× change of slew rate.** That agreement is the internal control that
says the load and the output settling are not inside the answer, and it is why the measurement was
run at two amplitudes rather than one.

Three estimators are reported (foot, 50 %, and the 10–90 % chord extrapolated back to baseline)
precisely because they disagree by more than the quantity being measured; the foot is the one
`TRIGCOMP` is derived from.

## Fault 1 — the channel-0 current readback is 4× low in voltage mode

The load check was meant to be a formality. Driving a bipolar train in voltage mode and dividing
the board's own `MSUM` voltage by its own `MSUM` current gave **3988 Ω**, stably, linearly and
symmetrically at every amplitude from ±1 V to ±4 V, for a resistor colour-coded 1 kΩ.

Stable, linear and symmetric is what rules out the obvious explanations. A bad contact is none of
those. So the load was measured a second way, through the other output mode: the Howland pump
forces a known current and the scope reads the voltage across the load, which puts neither of the
board's readbacks in the loop. That gives **993 Ω ± 3 %** over 500–1500 µA, both polarities — the
resistor that is fitted.

The board's *voltage* readback agrees with the scope throughout (6.82 V read against 6.90 V
measured on an 8000 mV command). So the fault is on the current path in voltage mode alone, low by
a factor of **4.02**. A clean constant factor across amplitude and polarity points at a range or
gain setting — the AD7321 has programmable input ranges, and a 4× error is two range steps — rather
than at anything analog and load-dependent. It has not been chased into the firmware yet.

This is not a bench problem. Every voltage-mode `MSUM`, `MDATA` and `READ` current figure is a
quarter of the truth, and the load resistance the OLED result pages added in phase 015 divides by
it, so a 1 kΩ preparation displays as 4 kΩ. Current-mode measurement is unaffected. **This is the
most consequential finding of the session and the obvious next piece of work.**

`trigcomp.py check` now measures the load through both modes for exactly this reason: one route on
its own cannot tell a bad contact from a miscalibrated readback, and two routes can.

## Fault 2 — the scope driver was discarding 32× of its own resolution

`ps2000_get_timebase` returns a sample interval and a `time_units` code. The interval is in
nanoseconds always; the units code describes the timestamp array of `ps2000_get_times_and_values`,
a call this project never makes. `pico2000.py` multiplied the two. For timebases 6 and slower the
driver happens to report `units = 2` (nanoseconds) and the product came out right, which is why
every capture ever taken here was correct — but timebases 1 to 5 report `units = 1`, so their
intervals came out a thousand times too small, and `pick_timebase`, which scans upward for the
first interval at least as coarse as asked, never selected them. Every capture in this repository
was taken at 640 ns or slower when 20 ns was available.

Fixed, and it changes no existing result: re-running `pick_timebase` for every `want_dt` that
`capture.py` asks for returns the same timebase indices as before, because all of them are 6 or
slower. What it unlocks is finer requests, and this measurement wanted one — the shots were taken
at 20 ns instead of 640.

**A false alarm on the way, recorded because it is an easy trap.** The first check of which
timebases are honest measured a 20 µs reference period as 15.01 µs at timebase 1 and 6.68 µs at
timebase 0 — exactly 3/4 and 1/3 — which looks precisely like a driver accepting timebases it
cannot deliver, and was written into the code as such. It was wrong. The AWG's edge is slow enough
that samples dither across a fixed threshold, so one real edge yields several upward crossings;
counting them all shortens the mean spacing by a clean-looking factor. Skipping to the end of each
high run before hunting the next edge removes it, and then **every timebase the driver accepts
delivers what it claims**, to better than 1 %. `tests/device/scope_timebase.py` does it correctly
and is the standing check.

## What else the session measured

- **The analog output settles much faster than a reading of it does.** Foot to 90 % is 1.91 µs for
  an 8 V step and 2.93 µs for a 1.8 V one. `BENCHSETTLE` reports 8–9 µs on the same board, so most
  of `CAL SETTLE` is the ADC path — conversion, input-line switch, isolator — not the output stage.
  `CAL SETTLE` remains the right budget for *when a reading means anything*, which is what it is
  used for; it is not the time the output takes to arrive.
- **The voltage-mode output runs 10–15 % below the commanded amplitude**, worsening with load
  current: 500 mV → 448 mV, 8000 mV → 6820 mV (6.90 V on the scope). The incremental gain falls
  from 0.885 to 0.80 across that span, so it is a gain error of ~0.89 plus compression above about
  4 mA. Current mode does not show it (1000 µA set puts 1.017 V across 1 kΩ).
- **Trigger latency repeatability is 29–54 ns rms**, comparable to the 42 ns residual latch jitter
  phase 007 measured, and small enough that every systematic discussed above dominates it.

## Figures

`docs/figures/make_bench_figures.py` builds four from the CSVs the device scripts leave in `tmp/`;
it touches no hardware, so re-running the scripts and then it keeps figures and numbers in step.

- `figs/stimjim-load-two-modes.png` — the same resistor through both output modes, and why a
  stable, linear, symmetric wrong answer indicts a readback rather than a contact.
- `figs/stimjim-trigcomp-reference.png` — the slow input edge with the measured threshold band on
  it, and the resulting latency against the level the clock is started at.
- `figs/stimjim-trigcomp-budget.png` — the 37.27 µs decomposed into what the firmware budgets, what
  it does not, and what is inside the part it does not.
- `figs/pico-timebase-units.png` — the units bug and what believing it cost.

## Open, in the order they are worth doing

1. **Find the 4× current-readback error in voltage mode.** Start at the AD7321 range register and
   the input-line selection in the voltage-mode read path, and compare it against the current-mode
   path, which is right. Until this is fixed the OLED result pages report load resistances four
   times too high in voltage mode.
2. **Decide what `CAL TRIGCOMP` should be**, given that it compensates a triggered start only and
   two of its three terms are common to both start paths. Setting it to 2 is right if absolute
   trigger-to-output latency is the guarantee; 0 is right if the two start paths must agree.
3. **Characterise the voltage-path gain.** The 10–15 % shortfall is a calibration constant nobody
   has measured; `MILLIVOLTS_PER_DAC` assumes a gain of 1.505.
4. Configuration C is untouched: dual-channel collision jitter and the real `SJ_FS_MAX_HZ`.

## Entry point

`tests/device/trigcomp.py --help`, then its module docstring. `python trigcomp.py check` is the
one command that says whether the bench is still wired for any of this.
