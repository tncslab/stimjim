Stimjim — a flexible, precise, inexpensive open-source stimulator
=================================================================

Stimjim is a two-channel current and voltage stimulator for neural tissue. Each channel has an
isolated supply, switches between voltage and current output, and measures what it actually
delivered. This repository holds the hardware design and two firmwares: the original
`stimjimPulser`, and **`stimjimAWG`**, an arbitrary-waveform firmware that adds ramps, sine
bursts, envelopes, per-slot trigger delays, in-train measurement with SD logging, and a runtime
timing calibration. This page is a quick start for `stimjimAWG`; the full command reference is
[docs/serial-protocol.md](docs/serial-protocol.md).

![Stimjim picture](images/photo.png)

## Specifications

- Two independently controllable channels, each in current or voltage mode
- Current mode −3.33 mA to +3.33 mA; voltage mode −15 V to +15 V
- Compliance ±13.7 V in current mode
- Channels isolated from the supply and from each other; powered over USB
- On-board ADC measurement of the delivered current or voltage
- Waveform timing: 42 ns latch jitter, sample instants on an exact CPU-cycle grid
  (Teensy 3.5 at 120 MHz, measured — [docs/serial-protocol.md §4](docs/serial-protocol.md))

## Build one

Ordering, assembly and enclosure instructions have moved to
[docs/hardware-build.md](docs/hardware-build.md), along with the PCB and BOM links.

## Flash `stimjimAWG`

1. Install the [Arduino IDE](https://www.arduino.cc/en/main/software) with
   [Teensyduino](https://www.pjrc.com/teensy/td_download.html).
2. Copy [lib/stimjim](lib/) into your Arduino `libraries` folder (`Documents/Arduino/libraries`
   on Windows, `~/Arduino/libraries` on Linux).
3. Open [stimjimAWG/stimjimAWG.ino](stimjimAWG/), select **Teensy 3.5** and **120 MHz**, upload.

From a shell instead, with `arduino-cli`:

```
arduino-cli --config-file tmp/arduino-cli.yaml compile --fqbn teensy:avr:teensy35 \
    --warnings more stimjimAWG
arduino-cli --config-file tmp/arduino-cli.yaml upload --fqbn teensy:avr:teensy35 \
    -p COM4 stimjimAWG
```

Teensy 4.0 and 4.1 build from the same sources through a portable backend; see
[docs/hardware-variants.md](docs/hardware-variants.md) for what has to be recalibrated there.

## First contact

Open the Arduino serial monitor (or any terminal — it is USB CDC, so the baud rate is ignored)
and send `IDN`:

```
IDN
IDN,stimjimAWG,Teensy3.5,fw=0.7.0,proto=1
# build: Teensy3.5, F_CPU=120 MHz, fastio=registers, timer=raw-PIT, sd=yes
# engine: PIT channels 0/1, K_RELOAD=112 cycles, cal=default
```

`HELP` lists every command with its units and defaults. Lines end with `\n`; a reply is one line
per setter, or a block ending in `OK`.

At boot the firmware calibrates its ADC and DAC offsets and prints them. ADC offsets of a few
tens of ADC units are normal; ±4096 means something is wrong with the board.

## Define a waveform and run it

There are three waveform types, all sharing one header. Each has a figure showing which number
sets which part of the waveform:

| Type | What a stage means | Figure |
|---|---|---|
| `S` | jump to the amplitudes and hold | [figs/stimjim-param-S.png](figs/stimjim-param-S.png) |
| `L` | ramp linearly to the amplitudes | [figs/stimjim-param-L.png](figs/stimjim-param-L.png) |
| `W` | sine burst; amplitude, frequency and phase per channel | [figs/stimjim-param-W.png](figs/stimjim-param-W.png) |

```
S0,0,1,2000,1000000;100,0,150;-100,-100,200
```

reads as: **slot 0**, channel 0 in voltage mode and channel 1 in current mode, a pulse every
**2000 µs** for **1 000 000 µs**, then two stages — 150 µs at (100 mV, 0) and 200 µs at
(−100 mV, −100 µA). Start it on engine 0 with `T0`, stop it with `T-1`; `U0` / `U-1` are the same
on engine 1. `S0?` returns the line that reproduces the slot, so a configuration always
round-trips.

Modes are the original numbering: **0** voltage, **1** current, **2** hi-Z, **3** grounded. In a
train definition 2 and 3 both mean *this train does not drive that channel*. **90** and **91** are
voltage and current with measurement switched off.

Slots 0–99 exist; `P` saves slots 0–9 (with their envelopes, measurement settings, trigger table
and timing budget) to EEPROM.

## Trigger it

```
TRIG0,1,3,-1,0     input 0, rising edge, starts slot 3 on engine 0
TRIG1,3,-1,-1,0    input 1 becomes a stimulus-marker output (the boot default)
```

`TRIG<t>,<mode>,<slot0>,<slot1>,<edge>` — mode 0 off, 1 joint (one slot, one engine), 2
independent (slot0 on engine 0, slot1 on engine 1), 3 output marker. The legacy `R<t>,<idx>` form
still works.

Delay a train relative to its trigger with `DELAY<idx>,<us>`, or with the optional 6th field of
the waveform line. The delay applies to `T`/`U` starts too, so it can be checked over the serial
port before anything is wired.

**Two channels that must be sample-aligned belong in one train that drives both**, not in an
independent route: two engines started by one edge cannot latch at the same instant and the
second one lands ~9 µs late. See [docs/serial-protocol.md §4](docs/serial-protocol.md).

A trigger edge puts signal on the output **45 µs later** (`CAL STARTLAT`, adjustable), repeatable
to 42 ns, plus whatever `DELAY` asks for. The 45 µs is almost all the arm — the copy and
precomputation the edge ISR does before it hands the train to the timer — not the DAC write, which
is 2.75 µs. The edge ISR itself emits nothing: it timestamps the edge, computes `t0` and programs
a timer, and every sample is latched from the timer ISR, which is why the latency does not depend
on the train, on `loop()`, or on interrupt entry.
[docs/timing.md](docs/timing.md) works through that, the comparison with the original firmware,
whether a preloaded DAC could fire faster, and what SD logging costs.

## Measure what was delivered

Measurement is on by default: every train summarises what it put out.

```
MEAS0,3,3,0,-1,0      slot 0: V+I on both channels, near each stage end, all stages,
                      end-of-train summary only
T0
...
Train #1 complete. Delivered 500 pulses.
MSUM,0,500,0,99.8,0.4,,,,,,
```

`MSUM` carries the mean and standard deviation per point, per channel, per line, accumulated over
every repetition. `MEAS<idx>,…,<report>` with `report` +1 also streams a per-repetition `MDATA`
line, +2 writes to the SD card (`LOG` opens and closes the file; the `SD` group reads the card
back over the serial port, so the socket under the cover never has to be opened).

A measurement point needs a free gap between two latches — 24 µs for a single reading, 47 µs
for V+I on both channels. When the waveform does not leave that much, the reads are split and
rotated over consecutive repetitions rather than the waveform being stretched. On an `L` ramp
the gap is the sample interval, so in-train measurement needs `DT` raised above the 20 µs
default.
[figs/stimjim-timing-measurement.png](figs/stimjim-timing-measurement.png) shows the arithmetic.

## Calibrate the timing to your board

Every hardware timing budget the scheduler works from is runtime state, not a compiled constant:

```
CAL?                     list all nine budgets
BENCHDAC2,2000           measure what dacProgramBoth actually costs
BENCHARM,0,200           measure what arming slot 0 costs
CAL,STARTLAT,80          widen the trigger-to-output latency
P                        persist it
```

The defaults in [stimjimAWG/Config.h](stimjimAWG/Config.h) are measured on a Teensy 3.5 at
120 MHz with the register backends. On any other board, re-measure with the `BENCH` group before
trusting the firmware for stimulation. [figs/stimjim-timing-latch.png](figs/stimjim-timing-latch.png)
shows what each budget pays for, drawn to scale.

The engine checks itself: if a latch or an arm misses its budget, the train's completion says so
and names the value that would have covered it. No oscilloscope is needed for that.

## Test benches

[tests/host/](tests/host/) holds host-side C++ tests of the parser, the measurement planner, the
sample generator and the calibration validator — no board required. [tests/device/](tests/device/)
talks to a real Stimjim over USB (`smoke.py`, and `bench_arm.py` for the arm cost that sizes the
trigger latency) and, for the microsecond measurements, to a PicoScope (`capture.py`); its README
documents the bench wiring.

## Where things are

| Path | What |
|---|---|
| [stimjimAWG/](stimjimAWG/) | the arbitrary-waveform firmware |
| [stimjimPulser/](stimjimPulser/) | the original firmware, kept for reference |
| [lib/](lib/) | the `Stimjim` hardware library (pins, DAC/ADC helpers) |
| [docs/serial-protocol.md](docs/serial-protocol.md) | every command, with defaults and measured timing |
| [docs/timing.md](docs/timing.md) | trigger latency, ISR-vs-busy-loop, DAC preloading, SD cost and content |
| [docs/hardware-variants.md](docs/hardware-variants.md) | Teensy 3.5 / 4.x differences and what to recalibrate |
| [docs/hardware-notes.md](docs/hardware-notes.md) | board-level notes and known hardware limits |
| [docs/bench-wiring.md](docs/bench-wiring.md) | how to wire board and scope for each remaining measurement |
| [docs/hardware-build.md](docs/hardware-build.md) | ordering, assembling and boxing a board |
| [docs/PROGRESS.md](docs/PROGRESS.md) | development log index, newest first |
| [figs/](figs/) | the figures this page links, plus oscilloscope captures |
