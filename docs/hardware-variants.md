# stimjimAWG on other hardware: backends and what must be recalibrated

stimjimAWG runs on the Teensy 3.5, where it drives the SPI and timer peripherals through their
registers, and on the Teensy 4.x boards open-ephys is moving the StimJim to now that the 3.5 is
out of stock. Every register-level path therefore has a second implementation built on the
standard Arduino APIs, selected by `#if` in `Config.h`. The two are functionally identical, but
the portable one is slower per event, so the timing constants listed in §4 must be re-measured on
a new board before the firmware is used for stimulation. Nothing else is board-specific: the
protocol, the waveform math, the slot store and the display are shared.

Build status (Teensyduino 1.62.0, `--warnings more`, zero warnings):

| Target | Command | Result |
|---|---|---|
| Teensy 3.5 (register backends) | `--fqbn teensy:avr:teensy35` | 163 KB flash, 38.4 KB RAM |
| Teensy 3.5 (portable backends) | `--fqbn teensy:avr:teensy35 -DSJ_FASTIO_REGISTER=0 -DSJ_TIMER_REGISTER=0` | builds clean, 164 KB flash |
| Teensy 4.1 | `--fqbn teensy:avr:teensy41` | 139 KB flash, RAM1 59.2 KB |
| Teensy 4.0 | `--fqbn teensy:avr:teensy40 -DSJ_EEPROM_SLOTS=6` | 97 KB flash — no SD, see below |

Only the Teensy 3.5 register build has been run on hardware. The others compile and are
structurally correct; they are untested silicon until someone runs the bench list in §4.

Build-time defines go through the Teensy platform's optimisation flag variable, which is where
the compile recipe puts them:

```
arduino-cli --config-file tmp/arduino-cli.yaml compile --fqbn teensy:avr:teensy35 \
  --warnings more --build-property "build.flags.optimize=-O2 -DSJ_EEPROM_SLOTS=6" stimjimAWG
```

`compiler.cpp.extra_flags` does *not* work here — the Teensy recipe does not reference it, so the
defines are silently dropped and the build behaves as if they were never passed.

## 0. SD card support

`SJ_USE_SD` selects the SD logging and the `SD` file-access group. It defaults to 1 everywhere
except a Teensy 4.0, which has no card socket; there it defaults to 0 and every `LOG`/`SD`
command answers `ERR … no SD support in this build (Teensy4.x)` instead of failing to link. The
Teensy 3.5, 3.6 and 4.1 all carry a socket on native SDIO, which never touches the DAC/ADC SPI
bus. `IDN` reports `sd=yes` or `sd=no` so a session can tell without probing.

Cost of having it: about 50 KB of flash for SdFat, which is why the Teensy 4.0 build is so much
smaller than the 4.1 one. Setting `-DSJ_USE_SD=0` on a board that has a socket is a legitimate
way to reclaim that space.

## 1. Backend selection

`Config.h` derives two independent switches from the MCU macros, and either can be forced from
the build:

| Switch | 1 (default on Kinetis K64/K66) | 0 (everything else) |
|---|---|---|
| `SJ_FASTIO_REGISTER` | SPI0 driven through its DSPI registers, CTAR0/CTAR1 preconfigured for DAC and ADC, MISO swapped by writing the PORT mux | `SPI.beginTransaction()` per DAC word or ADC frame, `SPI.setMISO()` for the mux |
| `SJ_TIMER_REGISTER` | two PIT channels claimed through `IntervalTimer`, then their vectors and NVIC priorities taken over one by one | `IntervalTimer::begin()` re-armed per event, with the core's own dispatcher |

Forcing the portable backends on a Teensy 3.5 (`-DSJ_FASTIO_REGISTER=0 -DSJ_TIMER_REGISTER=0`)
is the intended way to measure the portable route's constants: it isolates the backend change
on hardware whose behaviour is already known.

## 2. Why Teensy 4 does not reuse the register path

The obvious port — swap `KINETISK_SPI0` for `IMXRT_LPSPI4` and `KINETISK_PIT_CHANNELS` for
`IMXRT_PIT_CHANNELS` — does not carry over cleanly, for one structural reason:

**On the i.MX RT1062 all four PIT channels share a single `IRQ_PIT` vector.** (Verified in the
Teensyduino 1.62 core, `cores/teensy4/IntervalTimer.cpp`: `beginCycles` calls
`attachInterruptVector(IRQ_PIT, &pit_isr)` and `pit_isr` polls all four `TFLG` bits.) The Kinetis
design gives each player its own vector at its own NVIC priority, which is what makes the two
players serialize against each other for free and lets a player preempt everything else. With one
shared vector that scheme has no direct equivalent — it needs a dispatcher and a single shared
priority. Routing through `IntervalTimer` lets the core's dispatcher do that work, at the cost of
an `end()` + `begin()` pair per scheduled event.

That cost is absorbed rather than hand-tuned: `K_RELOAD` is calibrated closed-loop at boot by
running the *real* scheduling path and measuring when the timer actually fires, so it takes on
whatever the selected backend costs. Only the `CAL` preload and DAC budgets are numbers a
human must set — and those are settable at runtime, not compiled in.

A native LPSPI4 + shared-vector PIT backend remains a worthwhile optimization, but it is not a
prerequisite for running on a Teensy 4.

## 3. Clocking

`SJ_CYC_PER_US` derives from `F_CPU`, and the build fails if `F_CPU` is not a whole number of
MHz: every µs↔cycle conversion in the engine is an integer multiply and must stay exact. Teensy
3.5 at 120 MHz gives 120 cycles/µs, Teensy 4.x at 600 MHz gives 600. The `F_BUS == F_CPU/2`
relationship is asserted only for the Kinetis register backend, which is the one that converts
cycles to timer ticks with a shift.

Two other constants must scale with the clock rather than be written as cycle counts, because a
literal tuned for 120 MHz becomes five times shorter at 600 MHz:

- the NLDAC hold pulse in `FastIO::dacLatch`, expressed as `SJ_CYC_PER_US / 10` so it stays
  ~100 ns on any clock — comfortably above the AD5752's ~20 ns minimum;
- the cycles→ns conversion in the `BENCH` printouts.

## 4. What must be recalibrated, and how

Everything in this list is a `CAL` parameter (protocol §4) whose default lives in `Config.h`
marked `RECALIBRATE`. Measure on the board, set it with `CAL,<name>,<us>`, check the train, then
`P` to persist — no rebuild in the loop. Editing `Config.h` is still the right move once a value
is known to be wrong on *every* board of that type. Run the benches with nothing else going on,
in this order:

| Command | Sets | Note |
|---|---|---|
| `BENCHDAC` / `BENCHDAC2` | `DACPROG1` / `DACPROG2` | round up; the portable route pays an extra `beginTransaction`/`endTransaction` per word |
| `BENCHADC` / `BENCHSW` | `ADCREAD` / `ADCSWITCH` — the measurement budget (plan §3.6) | carry the **maximum**, not the average: one outlying read pushes the next latch late. `BENCHSW` also answers bench-verify item 1, first-conversion validity after a line switch |
| a scope on one output | `SETTLE`, `GUARD` | how long after a latch a reading means anything; the floor is also the player's own post-latch bookkeeping, ~3.2 µs on the register path |
| `BENCHMISO` | confirms the mux swap does not glitch the first bit | on the portable route this covers `SPI.setMISO` instead of the PORT mux |
| `BENCHPIT,1000,5000` | `PRELOAD` — raw ISR wake latency | the portable route wakes later: `IntervalTimer::begin()` is slower than writing `LDVAL` |
| `BENCHPIT,1000,5000,<preload>` | acceptance: residual latch jitter must stay < 200 ns | if it does not, raise `PRELOAD` |
| `BENCHK` | spread of the boot `K_RELOAD` calibration | the constant itself is self-calibrated; this only checks it is stable |
| `BENCHSQ` vs `BENCHSQL` | scope A/B of the FastIO path against `Stimjim.writeToDac` | the shape check for a new FastIO backend |
| a scope on the trigger input and one output | `TRIGCOMP` — pin edge to ISR entry | the delivered latency is `STARTLAT` once this is set; everything from the ISR's first instruction onwards is already compensated |

`STARTLAT` has to cover `PRELOAD + DACPROG2 + 3 µs` beyond `TRIGCOMP`, which is why the portable
default is 40 µs against the register path's 20: its programming budgets are twice as wide.
`Cal::validate` refuses a set that breaks the relation, and reports it at boot if a build's own
defaults do.

Then `SJ_FS_MAX_HZ`, the sine sample-rate ceiling: it must sit about 30 % below the rate at
which the measured preload + DAC programming budget fills the sample period. It is 50 kHz for the
register backend and starts at 25 kHz for the portable one — both still provisional, though a
single unmeasured sine train on a Teensy 3.5 does meet every deadline right up to Fs = 49.9 kHz.

**Acceptance without an oscilloscope.** Every latch compares itself against its own deadline, so
a recalibrated board can be qualified over the serial port alone: run a train of each type with
measurement on and read the lines that follow the completion. `WARN engine: … latch(es) overran
their deadline` means a budget above is still too small; silence means every event in that train
met its deadline. This is the cheapest possible regression test for a new backend, and it is
what caught two of the numbers in the table above.

The figures the Teensy 3.5 register backend actually measured are tabulated in
[serial-protocol.md](serial-protocol.md) §4, next to the `BENCH` command list — that is the
reference a new board's numbers should be compared against.

`IDN` prints which backends the running binary uses, so a measurement can always be attributed:

```
# build: Teensy3.5, F_CPU=120 MHz, fastio=registers, timer=raw-PIT, sd=yes
# engine: PIT channels 0/1, K_RELOAD=96 cycles, cal=default
```

## 5. EEPROM size

The persisted image (slots 0–9, the trigger table and the CAL budget, ~1.7 KB) fits the 4096 B EEPROM of the
Teensy 3.5 and the 4284 B of the Teensy 4.1, but **not** the 1080 B of a Teensy 4.0. Since
`EEPROM.put()` past `E2END` is a silent no-op in the Teensy core, an image that does not fit
would appear to save and then restore as garbage. `TrainStore.cpp` therefore static_asserts the
image against `E2END`, and a Teensy 4.0 build stops with:

```
error: static assertion failed: EepromImage exceeds this board's EEPROM —
rebuild with a smaller -DSJ_EEPROM_SLOTS (a Teensy 4.0 fits about 6)
```

`SJ_EEPROM_SLOTS` is a build override for exactly this. Failing the build is deliberate: silently
persisting fewer slots than the operator asked for is the kind of surprise this firmware exists
to remove.
