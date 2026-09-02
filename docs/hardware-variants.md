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
| Teensy 3.5 (register backends) | `--fqbn teensy:avr:teensy35` | 107 KB flash, 30.6 KB RAM |
| Teensy 3.5 (portable backends) | `--fqbn teensy:avr:teensy35 -DSJ_FASTIO_REGISTER=0 -DSJ_TIMER_REGISTER=0` | builds clean |
| Teensy 4.1 | `--fqbn teensy:avr:teensy41` | 88 KB flash, RAM1 44.8 KB |
| Teensy 4.0 | `--fqbn teensy:avr:teensy40 -DSJ_EEPROM_SLOTS=6` | 87 KB flash (see EEPROM below) |

Only the Teensy 3.5 register build has been run on hardware. The others compile and are
structurally correct; they are untested silicon until someone runs the bench list in §4.

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
whatever the selected backend costs. Only `SJ_PRELOAD_US` and the DAC budgets are constants a
human must set.

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

Everything in this list is a constant in `Config.h` marked `RECALIBRATE`. Run the benches with
nothing else going on, in this order:

| Command | Sets | Note |
|---|---|---|
| `BENCHDAC` / `BENCHDAC2` | `SJ_DAC_PROG1_US` / `SJ_DAC_PROG2_US` | round up; the portable route pays an extra `beginTransaction`/`endTransaction` per word |
| `BENCHADC` / `BENCHSW` | the measurement budget (plan §3.6) | `BENCHSW` also answers bench-verify item 1, first-conversion validity after a line switch |
| `BENCHMISO` | confirms the mux swap does not glitch the first bit | on the portable route this covers `SPI.setMISO` instead of the PORT mux |
| `BENCHPIT,1000,5000` | `SJ_PRELOAD_US` — raw ISR wake latency | the portable route wakes later: `IntervalTimer::begin()` is slower than writing `LDVAL` |
| `BENCHPIT,1000,5000,<preload>` | acceptance: residual latch jitter must stay < 200 ns | if it does not, raise `SJ_PRELOAD_US` |
| `BENCHK` | spread of the boot `K_RELOAD` calibration | the constant itself is self-calibrated; this only checks it is stable |
| `BENCHSQ` vs `BENCHSQL` | scope A/B of the FastIO path against `Stimjim.writeToDac` | the shape check for a new FastIO backend |

Then `SJ_FS_MAX_HZ`, the sine sample-rate ceiling: it must sit about 30 % below the rate at
which the measured preload + DAC programming budget fills the sample period. It is 50 kHz for the
register backend and starts at 25 kHz for the portable one — both provisional until measured.

`IDN` prints which backends the running binary uses, so a measurement can always be attributed:

```
# build: Teensy3.5, F_CPU=120 MHz, fastio=registers, timer=raw-PIT
# engine: PIT channels 0/1, K_RELOAD=96 cycles
```

## 5. EEPROM size

The persisted image (slots 0–9 plus the trigger table, ~1.6 KB) fits the 4096 B EEPROM of the
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
