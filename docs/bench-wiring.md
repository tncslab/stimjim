# Bench wiring for the stimjimAWG measurements that are still open

This document describes, from scratch, how to wire a StimJim board and an oscilloscope for each
measurement the `stimjimAWG` firmware still needs, and what to do once it is wired. It assumes
nothing beyond a StimJim running `stimjimAWG`, a two-channel oscilloscope with a signal generator
(a PicoScope 2204A is what the existing scripts drive), a handful of resistors and two LEDs. Every
number quoted as "measured" comes from a Teensy 3.5 at 120 MHz running the register backends.

Four of the firmware's timing budgets used to need an oscilloscope. Three no longer do:
`BENCHDAC`/`BENCHADC`/`BENCHPIT` measure the DAC, ADC and timer costs, `BENCHARM` measures the
cost of arming a train, and `BENCHSETTLE` measures how long after a latch a reading means
anything. What is left needs a scope because it involves an instant the firmware cannot observe —
the physical edge on a trigger pin — or a comparison between two outputs.

Three wiring configurations cover everything below. Configuration A is the bench as it stands and
is the one `tests/device/capture.py` already drives; B and C are changes to it.

---

## Safety and setup, common to all three

- The two channels are isolated from each other and from the USB supply. An oscilloscope is not:
  its channel grounds are common. **Tie the scope ground to exactly one node of the load**, and
  never to two nodes that the StimJim drives independently, or the isolation is defeated through
  the scope.
- Keep the outputs grounded (`M0,3` and `M1,3`, which is the boot state) whenever you are
  changing wiring.
- The 2204A is 8-bit: on the ±10 V range one code is 78 mV. Keep every scope trigger level at
  least 1 V clear of the baseline, or the threshold sits inside the trigger hysteresis and never
  fires.
- Close the PicoScope application before running any script. It holds the USB device exclusively.
- With both scope channels enabled the capture buffer is 3968 samples and the fastest usable
  sample interval is 640 ns. That floor matters: it is why sub-microsecond settling cannot be
  measured this way, and why `BENCHSETTLE` exists instead.

---

## Configuration A — the load chain (what is on the bench now)

```
PicoScope AWG  ------------------------------> StimJim trigger input IN0

load chain, four nodes in series:

    CH1(+) ---- 1k ---- CH0(+) ---- 1k ---- CH0(-) ---[antiparallel LEDs]--- CH1(-)
      |                    |                   |
   scope B              scope A            scope gnd
```

The two antiparallel LEDs are deliberately different colours — one red, one blue. Their forward
voltages differ by about a volt, so the branch clamps at a different level in each polarity: the
load a positive half of a bipolar pulse sees is not the load the negative half sees. An `S` train
with a positive and a negative stage must read back two *different* clamp levels; a pair of equal
magnitudes would mean the firmware is reporting one stage's value for both.

The chain couples the channels, and the captures show it in both directions. Neither of these is
a fault:

- A CH0 pulse appears on **scope channel B**, because a channel no train drives is *grounded*,
  which ties CH1(+) to CH1(−) and hence to the LED branch. Below the LEDs' turn-on that branch
  carries no current, so scope B simply sits at CH0(+)'s potential; above it the LEDs clamp and
  scope B saturates at +2.2 V one way and −2.9 V the other.
- A CH1 pulse divides down onto **scope channel A**: with CH0 grounded, an 8 V pulse on CH1 reads
  about 0.5 V there.

### What A measures

| Test | Script | Result on this bench |
|---|---|---|
| `S`, `L` and `W` shapes on a real load | `python capture.py shapes` | `figs/stimjim-shape-*.png` |
| Post-trigger delay accuracy | `python capture.py delay` | 2000 µs set → 1999.3 measured; 5000 → 4999.3; 20000 → 19996.0 |
| Engine-to-engine offset | same run | −0.25 µs |
| Same-instant latch contention | same run | −10.1 µs |

The delay is measured differentially, because the AWG wire is not teed to a scope input: one
trigger edge starts a reference pulse on CH1 and the pulse under test on CH0, the scope triggers
on the reference, and the constant engine-to-engine offset is subtracted. That offset is
calibrated at a **500 µs** delay, not at 0 — at 0 the two first latches fall on the same instant
and the second player ISR waits for the first to return, which is the contention figure above and
is absent at every other delay.

---

## Configuration B — trigger edge and output on the scope at the same time

**Measures: `CAL TRIGCOMP`, the delay from the physical edge at the input pin to the trigger
ISR's first instruction. It is the last quantity in the firmware with no measured value.**

Change from A: tee the AWG output so the scope sees the edge it generates.

```
PicoScope AWG ---+---> StimJim trigger input IN0
                 |
                 +---> scope channel B          (the edge itself)

    CH0(+) ---- 1k ---- CH0(-)
      |                    |
   scope A             scope gnd

CH1 unused; leave it grounded (M1,3).
```

The 1 kΩ across CH0 is a plain resistive load: no LEDs, so there is no clamp to distort the edge
under test. Drop the load chain entirely for this measurement — the coupling that makes
configuration A informative only adds uncertainty here.

### Procedure

1. `M0,0` (channel 0 to voltage mode).
2. Define a one-pulse train with no delay and a large step, on channel 0 only:
   `S20,0,3,100000,1000;8000,0,2000`
3. Route the trigger to it, rising edge, joint mode: `TRIG0,1,20,-1,0`
4. `CAL,TRIGCOMP,0` and `CAL,STARTLAT,60` — you are measuring against a known budget.
5. Scope: channel B on the trigger edge, channel A on CH0(+), both DC, ±10 V range. Trigger on
   channel B rising at half the AWG amplitude, 10 % pre-trigger, 640 ns/sample.
6. AWG: a 1 Hz square wave, 0 to 2 V (`scope.square(1.0, 2.0)` in `pico2000.py`).
7. Measure `t(CH0 crosses half its step) − t(trigger edge crosses its threshold)`. Average over at
   least 20 shots; the spread is the trigger-latency repeatability, which is worth recording on
   its own.

### What the number means

The delivered latency is `TRIGCOMP + STARTLAT`. With `TRIGCOMP` at 0 the measurement should read
about 60 µs plus the hardware part. Set `CAL,TRIGCOMP,<measured − 60>` and re-run: the delivered
latency then reads 60 µs exactly, and `P` persists it. If the measured value is *less* than 60 µs
something is wrong with the setup, not with the firmware — `STARTLAT` is a floor the engine
schedules against, not an average.

Expect a few hundred nanoseconds. `attachInterrupt` on a Kinetis port ISR plus the dispatch to the
handler is the whole of it, and the value is a property of the MCU and the core, not of the board.

---

## Configuration C — the two channels compared against each other

**Measures: dual-channel collision jitter (do two channels driven by *one* train latch together?)
and the real `SJ_FS_MAX_HZ` ceiling for sine playback.**

Change from A: both channels get their own resistive load and their own scope channel, with a
single common ground node.

```
    CH0(+) ---- 1k ---- CH0(-) ---+
      |                           |
   scope A                        +--- scope gnd
                                  |
    CH1(+) ---- 1k ---- CH1(-) ---+

PicoScope AWG ---> StimJim trigger input IN0   (only needed for the trigger tests)
scope B on CH1(+)
```

Tying CH0(−) and CH1(−) together defeats the channel-to-channel isolation, which is exactly what
makes this measurement possible and exactly why it is a separate configuration. **Do not use this
wiring with anything connected to a preparation.**

### C1 — do both channels of one train step together?

A train that drives both channels programs them with a single `dacProgramBoth` and latches them
with a single `NLDAC` pulse, so they should step within the latch pulse itself (~100 ns) and not
within a DAC programming time. This is the claim that makes "one train driving both channels" the
right construct for sample-aligned stimulation, and it has never been checked at the output.

1. `M0,0`, `M1,0`.
2. `S21,0,0,20000,3000000;8000,8000,2000;0,0,2000` and `T21`.
3. Scope: A and B, ±10 V, 640 ns/sample, trigger on A rising at 4 V, 10 % pre-trigger.
4. Measure the time between A and B crossing the same fraction of their steps, over 20 shots.

Pass: the two edges coincide within one sample interval. Fail: they differ by a DAC programming
time (~1.4 µs), which would mean the two channels are not being latched together.

Then repeat with an **independent** trigger route (`TRIG0,2,21,22,0` with two single-channel
slots) to see the ~10 µs contention that configuration A already measures indirectly, this time
directly on the two outputs.

### C2 — the sine sample-rate ceiling

`SJ_FS_MAX_HZ` is 50 kHz on the register backend and is a desk estimate: the sample period at
50 kHz is 20 µs against a preload plus a dual-channel program of 9 µs, so there is margin, but the
number has never been walked up to its limit with both channels driven.

1. Define a two-channel sine train and step its frequency up: `W23,0,0,20000,2000000;`
   `4000,4000,20000; <f>,<f>,0; 0,0,0` for `f` in 200, 400, 600, 800 Hz — `Fs` is `64·f`, so
   800 Hz is `Fs = 51.2 kHz` and is refused at the clamp.
2. For each, read the completion line. `WARN engine: <n> latch(es) overran their deadline` is the
   failure the ceiling exists to prevent, and it needs no scope at all.
3. Use the scope to confirm that the last frequency that reports no overrun still produces a
   clean sine on both channels rather than a waveform whose samples are being dropped.

Set `SJ_FS_MAX_HZ` to about 70 % of the first frequency that overruns, and record the measured
number in `docs/hardware-variants.md`.

---

## Tests that need no wiring change at all

These are the ones worth running first, because they are cheap and two of them replace scope work
that used to be required.

| Question | Command | Expected on a Teensy 3.5 |
|---|---|---|
| `CAL SETTLE` | `M0,0` then `BENCHSETTLE,0,8000,20,64` | readings stop moving at 8–9 µs |
| `CAL STARTLAT` | `BENCHARM,<slot>,200` | 19–24 µs for `S`/`L`, 42 µs for a measured `W` |
| DAC and ADC costs | `BENCHDAC`, `BENCHDAC2`, `BENCHADC`, `BENCHSW` | 2.81 / 4.14 / 3.81 / 6.06 µs worst |
| Timer wake and latch jitter | `BENCHPIT,1000,2000` and `BENCHPIT,1000,2000,4` | 0.47 µs worst wake, 42 ns residual jitter |
| Does anything miss a deadline? | run a train, read the completion | silence |
| Full serial regression | `python smoke.py COM4` | `ALL CHECKS PASSED` |

**Long-run drift** also needs no wiring: start a train with `duration_us` set to several hours and
a long period, leave it, and read the completion. The 64-bit cycle counter is extended by the
player's own wake-ups, so the failure mode this looks for is a missed extension, which shows up as
a completion that never arrives or a pulse count that does not match `duration/period`.
