# On-target bench harness for stimjimAWG

Scripts that talk to a StimJim running `stimjimAWG` over USB serial and, where a measurement
needs microsecond accuracy, to a PicoScope. They need `pyserial` and `matplotlib`; the PicoScope
driver is loaded straight from the PicoScope application's install directory by `ctypes`, so
there is nothing else to install.

| File | What it is |
|---|---|
| `sjcon.py` | Serial console. Library (`StimJim.cmd`, `.cmd1`, `.reset`) and a CLI: `python sjcon.py COM4 IDN "S0?" DUMP` |
| `smoke.py` | Serial-only regression run: identity, backward compatibility, the post-trigger delay set both ways, rejection paths, completion reporting, and an OLED capture in each of the three views |
| `pico2000.py` | ctypes binding for the legacy `ps2000` driver, which is what the PicoScope 2204A needs. Run it directly to probe the scope |
| `pico2000a.py` | the same for the newer `ps2000a` driver. Unused on this bench; kept for 2000a-series models |
| `capture.py` | Oscilloscope acceptance: trigger-to-output delay, and the `S`/`L`/`W` shapes. Figures to `figs/`, raw samples to `tmp/` |

```
python smoke.py COM4 --screens ../../tmp/screens
python capture.py all
```

`smoke.py` exits with the number of failed checks.

## What needs no oscilloscope

Three of the four things a scope used to be needed for are now serial commands, so a board can be
qualified over USB alone:

| Question | Command | What to look for |
|---|---|---|
| Does a latch make its deadline? | any train, then read the completion | `WARN engine: … latch(es) overran their deadline` |
| How long after a latch does a reading mean anything? (`CAL SETTLE`) | `M0,0` then `BENCHSETTLE,0,8000,20,64` | the delay at which the readings stop moving; 8–9 µs here |
| How long does arming cost? (`CAL STARTLAT`) | `BENCHARM,<slot>,200` | the max; `STARTLAT` needs that plus `PRELOAD + DACPROG2 + 3 µs`, doubled for a two-engine trigger route |
| How much does one DAC/ADC operation cost? | `BENCHDAC`, `BENCHDAC2`, `BENCHADC`, `BENCHSW`, `BENCHPIT` | the max, not the average |

`BENCHSETTLE` and `BENCHSQ`/`BENCHSQL` drive the output; everything else leaves it parked.

What still needs an oscilloscope, and how to wire it, is in
[../../docs/bench-wiring.md](../../docs/bench-wiring.md).

## Bench wiring

```
PicoScope AWG  -> StimJim trigger input IN0
StimJim CH0(+) -> PicoScope channel A ("channel 1")
StimJim CH1(+) -> PicoScope channel B ("channel 2")
StimJim CH0(-) -> PicoScope ground

load chain, four nodes in series (node names spelled out: the scope's own
channels are also called A and B, and mixing the two up is easy):

    CH1(+) ---- 1k ---- CH0(+) ---- 1k ---- CH0(-) ---[antiparallel LEDs]--- CH1(-)
      |                    |                   |
   scope B              scope A            scope gnd
```

**The two antiparallel LEDs are deliberately different colours — one red, one blue.** Their
forward voltages differ by roughly a volt (red ~1.8 V, blue ~2.7 V), so the branch clamps at a
different level in each polarity: the load a positive half of a bipolar pulse sees is not the
load the negative half sees. That asymmetry is the point of the pair. An `S` train with a
positive and a negative stage should read back two clamp levels that differ, and a pair of equal
magnitudes would mean the firmware is reporting one stage's value for both, or averaging across
polarities. Where a capture or an `MSUM` line shows the positive and negative excursions
clipping at unequal voltages, that is the diodes, not the instrument.

The AWG wire is not teed to a scope input, so `capture.py` measures the trigger-to-output delay
differentially: the same edge starts a reference pulse on CH1 and the delayed pulse under test on
CH0, the scope triggers on the reference, and the constant engine-to-engine offset is subtracted.

That offset is calibrated with the delay set to **500 µs, not to 0**. At 0 both engines' first
latches fall on the same instant and the second player ISR has to wait for the first to return,
which adds about 10 µs that is absent at every other delay. `capture.py` reports that case
separately as *latch contention* (`figs/stimjim-trigger-contention.png`) because it is a real
property worth knowing: two independently routed trains cannot be sample-aligned, and two
channels that must be belong in one train that drives both.

The reference pulse is deliberately short (`REF_HIGH_US`, 300 µs). The resistor chain couples the
channels, so the reference's own falling edge lands on scope channel A as a step of a few hundred
millivolts; if it coincides with the pulse under test it drags the interpolated threshold
crossing by ~10 µs and looks exactly like a firmware timing error. Keeping the reference short
keeps its fall clear of every delay the script measures.

The resistor network couples the channels, and the captures show it in both directions. That is
the circuit, not the instrument, and neither trace is an output of the channel it appears on:

- **A CH0 pulse shows up on scope channel B**, because a channel that no train is driving is
  *grounded* (mode 3, the state `Stimjim.begin()` and every train end leave behind), which ties
  CH1(+) to CH1(−) and hence to the LED branch. Below the LEDs' turn-on that branch carries no
  current, so scope B simply sits at CH0(+)'s potential (within a few per cent up to ~1.8 V);
  above it the LEDs clamp, and scope B saturates at +2.2 V one way, −2.9 V the other — the two
  levels differ because the two diodes do (above).
- **A CH1 pulse divides down onto scope channel A**: with CH0 grounded, its 8 V reference pulse
  reads about 0.5 V there, roughly 11 %.

A scope input measures voltage at high impedance and needs no return current of its own, so
these readings are well defined whether the quiet channel is grounded or hi-Z — the two
resistors alone give CH1(+) a path to scope ground.

## Gotchas found on this bench

- **Close the PicoScope application first.** It holds the USB device exclusively and
  `ps2000_open_unit` then returns 0.
- The 2204A is 8-bit: on the ±10 V range one code is 78 mV, so a trigger threshold within a few
  codes of the baseline sits inside the trigger hysteresis and never fires. Keep scope trigger
  levels ≥1 V clear of ground.
- Capture buffer is 3968 samples with both channels enabled.
- A one-shot train cannot be started *after* arming the scope from the host — the serial round
  trip alone is tens of milliseconds. Shape captures run against a multi-second train instead.
- Host-side timing over serial is good to roughly ±200 ms because a `STAT` poll costs 60–150 ms.
  `smoke.py` only uses it as a sanity check; `capture.py` is the real measurement.

## Building and flashing

The Arduino IDE keeps no `arduino-cli.yaml`, so a bare `arduino-cli` invocation does not know the
Teensy board index and reports "Platform 'teensy:avr' not found". `tmp/arduino-cli.yaml` (created
on demand, not tracked) supplies the index URL and the IDE's data/user directories:

```
arduino-cli --config-file tmp/arduino-cli.yaml compile --fqbn teensy:avr:teensy35 --warnings more stimjimAWG
arduino-cli --config-file tmp/arduino-cli.yaml upload  --fqbn teensy:avr:teensy35 -p COM4 stimjimAWG
```

Build with `-DSJ_BOOT_TRACE` to make `setup()` wait for a serial host and announce each init
step; that is how a hang in initialization gets pinned down in one flash cycle.
