# 019 — Configuration C: the two channels compared against each other

Everything the stimjimAWG firmware still needed an oscilloscope for is now measured. Three
questions were open, all of them about what happens when both output channels are driven at once,
and all three have numbers. One train that drives both channels latches them **0.162 µs** apart —
one latch, two analog paths — while two independently routed trains that want the same instant
land **7.392 µs** apart, and that figure is contention and nothing else, because a control that
moves the two engines' latches 20 µs apart reads a residual of 0.061 µs. The two-channel sine
generator is far faster than the 50 kHz its constant claimed: it misses no deadline at all up to
115 kHz and only starts failing in proportion to the sample count at 150 kHz, so `SJ_FS_MAX_HZ` is
now **100 kHz**, measured rather than estimated. A worked example — a biphasic current stimulus on
channel 0 and a gating burst on channel 1, delayed half a second, started by one trigger edge —
shows what the 7.39 µs costs in practice: when the gate's delay is an exact multiple of the
stimulus period its rising edges collide with stimulus events and its falling edges do not, so the
first three gate pulses come out about 9 µs short of the 50 µs asked for. Moving the gate 250 µs off
the stimulus grid restores all four to full width and silences the firmware's own timing counters.

## The bench

A StimJim carrying a Teensy 3.5 at 120 MHz, `stimjimAWG` 0.8.0, register backend, `hot=RAM`,
with `CAL STARTLAT` 20 µs and `CAL TRIGCOMP` 2 µs persisted in EEPROM. A PicoScope 2204A
(8-bit, 3968 samples with both channels enabled, 20 ns fastest interval) supplies both the
capture and, through its AWG, the trigger edge.

The wiring is configuration C: each output channel gets its own 1 kΩ resistor and its own scope
input, and the two negative terminals are tied to a single common ground node.

```
    CH0(+) ---- 1k ---- CH0(-) ---+
      |                           |
   scope A                        +--- scope gnd
                                  |
    CH1(+) ---- 1k ---- CH1(-) ---+

scope B on CH1(+);  PicoScope AWG ---> StimJim trigger input IN0
```

Tying the two returns together defeats the channel-to-channel isolation. It is what makes a
two-channel comparison possible and it is why this is a separate bench configuration: nothing may
be connected to a preparation while it is wired this way.

`tests/device/dualchan.py check` verifies nine things separately, so a failure names a wire rather
than corrupting a number: each channel reaches its own scope input at full swing, each scope
ground sits at that channel's own return rather than partway down a chain, driving one channel
does not move the other (this is the property that distinguishes configuration C from A, where a
series chain deliberately couples them), each load is a plain resistor measured in current mode
with the scope as the voltage reference, and the AWG reaches IN0. The loads came out at **992 Ω**
across CH0 and **1010 Ω** across CH1, both linear and symmetric from 500 to 2000 µA.

The AWG wire was missing on the first run — configuration B tees it to a scope input, configuration
C does not, and it had come off with the tee. Check 9 caught it and named it.

## C1 — do the two channels of one train step together?

A train that drives both channels programs them with one `dacProgramBoth` and latches them with
one `NLDAC` pulse, so they should step together rather than a DAC programming time apart. That is
the claim that makes "one train driving both channels" the right construct for sample-aligned
stimulation, and it had never been checked at the output.

Four cases, 20 ns per sample, trigger half way into the record, 20 shots each, reported as CH1
minus CH0 at the same fraction of each channel's own step:

| Case | at 50 % of the step | at the foot |
|---|---|---|
| one train drives both channels (`S30,90,90,20000,3000000;8000,8000,2000;0,0,2000`, `T30`) | **+0.162 µs**, sd 6 ns | +0.492 µs |
| two single-channel trains, one edge, independent route, both wanting the same instant | **+7.453 µs**, sd 16 ns | +7.543 µs |
| the same route, caught tens of pulses into a 3 s train | +7.454 µs | +7.541 µs |
| **control:** the same route with CH1 delayed 20 µs, so the latches never coincide | **+0.061 µs**, sd 80 ns | +0.150 µs |

**The control was the missing piece and it should have been obvious from the start.** The first
version of this measurement had only the first two rows, and they cannot distinguish three
different things: contention, a constant lead of one engine over the other, and the scope's own
inter-channel sampling skew. Every latch in the coincident case collides, so a per-event delay and
a constant offset produce identical numbers. Delaying CH1 by 20 µs — far enough that neither
player waits for the other, near enough that both steps still fit one 79 µs capture — separates
them: the residual is 61 ns, so there is no constant engine-to-engine offset and the instrument
floor is under 100 ns. Subtracting the control from the coincident case:

> **contention = 7.392 µs at the 50 % crossing, 7.393 µs at the foot.**

Two independent estimates, taken at different points on the edge with different sensitivity to
output settling, agreeing to a nanosecond. That agreement is what says the 7.39 µs is a delay
rather than a difference in how the two channels settle. It supersedes the "9–10 µs at the output"
that configuration A measured differentially through its LED chain, and it sits close to the
7.3 µs the engine's own overdue counter has always reported.

**The first latch after the arm is not special.** The two arms run one after the other inside the
trigger ISR, so the first latch might have been expected to cost more than a later one. It does
not — 7.453 against 7.454 µs. The arms complete before `t0`; what collides is the two player ISRs,
and that happens identically at every coincident latch for the life of the train. This matters for
how the figure is quoted: it is not a start-up cost, it is a permanent property of two engines
sharing one SPI bus and one latch line.

**One train's two channels pass.** 0.162 µs is well under the 2 µs that programming a second
channel separately would cost (`CAL DACPROG2` minus `DACPROG1`), so the two channels really are
latched together. The original pass criterion in the bench document — "coincide within one sample
interval" — was wrong and has been replaced. One sample interval is a property of the instrument
doing the watching, not of the claim being tested, and sub-sample agreement was never on offer:
the 50 % crossing and the foot disagree by 0.33 µs, and a pure timing skew would move both
equally. That 0.33 µs is a difference in settling between two output stages driving two resistors
that themselves differ by 2 %. The number reads as *one latch, two analog paths*.

## C2 — the sine sample-rate ceiling

`SJ_FS_MAX_HZ` was 50 kHz on the register backend, a desk estimate from comparing a 20 µs sample
period against a 9 µs preload-plus-program budget. `Config.h` said in a comment that it "must be
replaced by the measured dual-channel ceiling with ~30 % margin".

Walking past the constant needs a build with the constant raised, because the firmware clamps
`Fs = 64·f_max` to it. The definition was not overridable, so it is now wrapped in `#ifndef` like
the other build-time knobs in `Config.h`, and the sweep ran on

```
arduino-cli --config-file tmp/arduino-cli.yaml compile --fqbn teensy:avr:teensy35 \
  --build-property "build.flags.defs=-D__MK64FX512__ -DTEENSYDUINO=160 -DSJ_FS_MAX_HZ=250000" \
  --build-path tmp/build-fsmax stimjimAWG
```

Each rate plays for 1 s twice: once as **one continuous burst**, which measures the sustained
sample rate, and once as **50 bursts of 20 ms**, which adds a burst boundary 50 times.

| Fs | µs/sample | continuous: late latches in 1 s | in 50 bursts |
|---|---|---|---|
| 50.0 kHz | 20.00 | 0 of 50 000 | 0 |
| 64.0 kHz | 15.63 | 0 of 64 000 | 49 |
| **100.0 kHz** | **10.00** | **0 of 100 000** | 49 |
| 115.2 kHz | 8.68 | 0 of 115 163 | 116 |
| 128.0 kHz | 7.82 | 1 of 127 931 | 149 |
| 147.2 kHz | 6.79 | 3 of 147 239 | 323 |
| 150.4 kHz | 6.65 | 22 of 150 375 | 444 |
| 153.6 kHz | 6.51 | 220 of 153 649 | 645 |
| 156.9 kHz | 6.38 | 597 of 156 862 | 1 268 |
| 160.0 kHz | 6.25 | **33 903** of 160 000 | 34 289 |
| 179.1 kHz | 5.58 | 4 late, but **179 101 of 179 104 already due** | the same |

**The two counters have to be read together, and reading only one of them is a trap.** The
firmware's `WARN engine: … overran their deadline` counts events whose DAC programming finished
after a deadline that was still in the future, so it only counts while the player is keeping up at
all. `# engine: … already due` counts events the player reached after their instant had already
passed. Past about 175 kHz the first collapses back to almost nothing while the second goes to one
per sample. A plot of overruns alone shows a peak at 160 kHz and a fall afterwards, which reads as
the generator recovering when in fact it has stopped working entirely. The figure plots both.

Three regimes, then: nothing late at all up to 115 kHz; one late latch per *train* — not per
sample — from 128 to 147 kHz; lateness proportional to the sample count from 150 kHz, with a knee
at 160 kHz. `SJ_FS_MAX_HZ` is now **100 000** for the register backend: the fastest rate measured
with zero late latches, and a third below where lateness begins to scale. The portable backend
keeps its hand-halved 25 kHz, still never measured.

**A bursted sine costs about one late latch per burst, and that is a separate effect.** With
`burst_us == period_us` the off event at each burst boundary lands off the sample grid, because
`Fs` is realized as a whole number of CPU cycles and most rates do not divide 120 MHz evenly —
50 kHz does (2400 cycles), 57.6 kHz does not (2083.33, rounded to 2083). One or two latches per
boundary then finish a few µs late. It shows from 64 kHz upward, grows to about four per burst at
141 kHz, never exceeds a few µs, does not scale with the sample count, and lands on the park at the
end of a burst rather than inside the sine. That is why the ceiling is quoted from continuous
playback; applying the "70 % of the first overrun" rule to burst mode would have given 38 kHz and
would have been measuring the wrong thing.

## C3 — the worked example: stimulate on CH0, gate on CH1

The construct most experiments want is two rhythms from one trigger. Two rhythms means two trains,
two trains means two engines, and one edge means an independent trigger route — the one
arrangement on this instrument that cannot sample-align its channels.

```
S34,1,3,10000,1000000;2000,0,500;-2000,0,500   # CH0: 2 mA biphasic, 500+500 us, every 10 ms for 1 s
S35,3,0,500,2000,500000;0,2500,50              # CH1: 2.5 V gate, 50 us every 500 us x 4, 0.5 s later
TRIG0,2,34,35,0
CAL,STARTLAT,35                                # two engines armed from one edge do not fit 20
```

The gate delay of 500 000 µs is an exact multiple of the 10 000 µs stimulus period, so the two
engines' event grids interlock: the gate's rising edges at +0, +500 and +1000 µs land on stimulus
events (the two phase starts and the park) and its falling edges at +50, +550 and +1050 µs land on
nothing. The rises are pushed back by the contention, the falls are not, and the pulses come out
short.

| gate mode | pulse widths on the stimulus grid | with the gate 250 µs off the grid | gate edge behind the stimulus edge |
|---|---|---|---|
| voltage | 40.1, 40.8, 39.3, **49.0** µs | 49.0, 48.8, 48.8, 49.0 µs | 8.74 µs |
| current | 40.5, 41.8, 40.1, **49.5** µs | 49.4, 49.3, 49.6, 49.6 µs | 8.26 µs |

The last column is the coincidence measured directly at 20 ns per sample, and it accounts for the
width deficit by itself. It is larger than C1'"'"'s 7.39 µs because contention costs however long
the *other* engine'"'"'s ISR takes, and the latch it collides with here is heavier — a current-mode
stage carrying in-train measurement, against C1'"'"'s unmeasured single-stage voltage train. Run to
run these move by a few tenths of a microsecond.

The fourth pulse is full width in both cases, because its rise at +1500 µs has no stimulus event on
it. The firmware reports the same thing with no scope involved: `# engine: 3 event(s) were already
due …, worst by 7425 ns` on the aligned run, and no timing fault at all on the staggered one. The
remedy is to move the gate off the stimulus grid — a delay of 500 250 µs instead of 500 000 puts
every gate event a quarter period from every stimulus event and all four pulses come out 49 µs
wide with 500.0 µs between rises. One train driving both channels, which would avoid the problem
entirely, is not available here: a slot carries one period and one stage list, and these two
signals share neither.

**Voltage and current mode differ in what reaches the load, not in timing.** The two rows agree to
within the measurement. What differs is amplitude: 2500 mV commanded delivers 2.207 V across the
1010 Ω load while 2500 µA delivers 2.479 V — the ~135 Ω of source impedance measured in the
previous phase, worth a 12 % droop in voltage mode and nothing in current mode. The board's own
readback agrees with the scope in both modes, and in voltage mode it reports no current at all,
which is correct on this hardware: the sense shunt sits in a branch the output mux parks on an
on-board dummy resistor whenever the channel is not in current mode. The stimulus is undisturbed
throughout — 2000 µA into 992 Ω reads +1.96 / −1.96 V with a charge imbalance under 0.2 %, and
`MSUM` gives 1981 µA and −1984 µA over 100 pulses with a 0.5 µA standard deviation.

## Long-run drift

`cycles64()` extends a 32-bit cycle counter that wraps every 35.79 s at 120 MHz, and the extension
is carried by the player's own wake-ups rather than by a periodic interrupt, so the failure mode is
a missed extension: deadlines that jump backwards by 35.79 s. `tests/device/longrun.py` plays one
long train and checks the pulse count, the wall-clock duration and the firmware's timing counters.
A 300 s run crosses the wrap 8.4 times and passed: 300 pulses of the 300 asked for, the
completion arriving 0.4 s from the requested duration by the host clock (which is only good to
a few hundred ms over USB, and a missed extension would be 35.79 s out, not 0.4 s), and no
timing fault of any kind reported. The hours-long version the bench document describes has not
been run; the wrap is crossed either way, and eight crossings is enough to break an extension
that does not work.

## Harness faults found and fixed

Four, three of which produced numbers that looked plausible:

- **Captures were not of the run they appeared to be.** The capture helper started the AWG and
  then armed the scope, without stopping the engines first. `Triggers::fireRoute` arms the two
  engines separately, so an edge arriving while the previous 1 s stimulus train still held engine 0
  started the *gate alone*. A capture triggered on the gate then showed a gate burst with no
  stimulus anywhere in a 2.54 ms window, and — because nothing collided — four perfectly formed
  50 µs pulses on a clean 500 µs grid. That was very nearly written up as "the collision costs
  nothing". Stopping both engines before each capture makes every edge start a complete pair.
  The generator cannot be started from the scope's armed-callback instead: the ps2000 driver
  refuses `ps2000_set_sig_gen_built_in` while a block is running.
- **An edge finder on a flat trace returns a number, not an error.** The collision skew was
  computed by taking the levels from a record's sorted extremes and interpolating the 50 %
  crossing. On a channel that never steps, those extremes are the noise floor and the "crossing"
  is the first noise sample above the middle of it — which reported the gate edge as 8.4 µs from a
  stimulus edge that was not in the window at all. The check is now that the trace actually moves
  by more than a volt before any skew is reported.
- **A failed run left the board on the raised two-engine budget.** `CAL STARTLAT` was raised to 35
  inside the use-case step and restored at its end, so the exception thrown by the driver above
  left it at 35. It is now captured and restored in the outermost `finally`.
- **Two outputs shared one filename.** The C2 sweep table and the scope trace of the top clean
  sine both wrote `tmp/stimjim-sine-ceiling.csv`, and the trace silently overwrote the sweep.

Also worth recording: `K_RELOAD`, printed in the `IDN` build line, changed from 97 to 112 cycles
across reflashes. It is self-calibrated at boot and the 0.125 µs difference is absorbed by
`PRELOAD`, so it is normal variation and not a symptom.

## What changed

- `stimjimAWG/Config.h` — `SJ_FS_MAX_HZ` 50 000 → **100 000** for the register backend, wrapped in
  `#ifndef` so a `-D` can override it, with the measurement in the comment. The board was reflashed
  with this build and its EEPROM survived (`cal=custom`, `STARTLAT` 20, `TRIGCOMP` 2).
- `tests/device/dualchan.py` — new: `check`, `c1`, `c2`, `usecase`.
- `tests/device/longrun.py` — new: the cycle-wrap test.
- `docs/bench-wiring.md` — configuration C rewritten from prescription to result; status line says
  all three configurations are measured; configuration A's −10.1 µs contention marked superseded.
- `docs/serial-protocol.md`, `docs/timing.md`, `docs/hardware-variants.md`,
  `docs/awg-implementation-plan.md` — the 10 µs contention figure replaced by 7.39 µs with its
  control, the sine ceiling replaced by the measured 100 kHz, and the `W` measurement gap floor
  corrected from 20 µs to 10 µs since `Fs` can now reach 100 kHz.
- Figures: `figs/stimjim-dual-latch.png`, `figs/stimjim-sine-overruns.png`,
  `figs/stimjim-sine-ceiling.png`, `figs/stimjim-usecase-overview.png`,
  `figs/stimjim-usecase-gate.png`, `figs/stimjim-usecase-collision.png`.

## Open questions

- **The portable backend's 25 kHz is still a guess.** It was halved by hand from the register
  backend's old 50 kHz, and the register backend's real figure turned out to be twice that. The
  same sweep on a portable build would settle it; nothing about the method is register-specific.
- **The 0.162 µs between one train's two channels is not broken down.** It is analog — the 50 %
  crossing and the foot disagree — but whether it is the AD5752's two output stages, the two
  Howland pumps or the 2 % difference between the two load resistors is not separated. Swapping the
  two resistors and re-running would answer it in one step.
- **Why some burst boundaries are late and others are not** is only half explained. The
  cycles-per-sample rounding accounts for the effect existing, but 64 kHz (1875 cycles, dividing
  the 20 ms burst exactly) still shows 49 late latches while 50 kHz shows none, so the rounding
  alone does not predict which rates suffer.
- **The scope's own inter-channel skew is bounded, not measured.** The 20 µs control puts it
  under 100 ns together with the channels' analog difference. Parallelling both scope inputs onto
  one node for one capture would measure it on its own; nothing here needed that resolution.

## Next entry point

The oscilloscope work on this firmware is complete: configurations A, B and C are all measured and
every timing budget in `CAL` now rests on a measurement rather than an estimate. The natural next
step is a portable-backend build on a Teensy 4.x, where `SJ_FS_MAX_HZ`, `CAL STARTLAT` and the
whole `CAL` group need re-measuring with the same scripts — `bench_arm.py`, `trigcomp.py` and
`dualchan.py` all run unchanged against any board that answers the protocol.
