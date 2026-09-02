# Phase 1: scaffold + FastIO + bench harness

Date: 2026-07-09

**Done:** `stimjimAWG/` created on branch `arbitrary_waveform`; all 14 planned modules compile
clean (`arduino-cli compile --fqbn teensy:avr:teensy35 --warnings more`: 0 warnings, 67 KB flash /
23.7 KB RAM). Functional in this phase: `FastIO` (register-level SPI0 split-phase DAC/ADC, MISO
PORT-mux swap, `cycles64()`, nestable BASEPRI `busLock()`), `Engine` PIT reservation +
deadline-programming core + boot `K_RELOAD` self-calibration (two passes, median of 65),
`Protocol` bounded line assembler, and the `BENCH` command group (now documented in
serial-protocol.md §4): DAC/ADC/latch/mux/cycles64 timing, `BENCHK`, `BENCHPIT` jitter histogram
(raw ISR wake or preload+spin latch mode), `BENCHSQ`/`BENCHSQL` square generators for the scope
A/B against `Stimjim.writeToDac`. `UiInput` (debounced ISR event ring) and a display splash work;
SampleGen/Measure/SdLog/Triggers are documented stubs; TrainStore holds the 100 slots with
protocol-§5 defaults. Legacy single-letter commands answer a one-line `ERR … Phase 2+`.

**Findings vs plan §7 (core source, Teensyduino 1.59):** (a) DWT CYCCNT is **not** enabled by the
Teensy 3 startup code (only AudioStream enables it on demand) — `FastIO::begin()` enables
TRCENA+CYCCNTENA itself; (b) `IntervalTimer` does allocate PIT channels low-to-high, but
`Engine::grabChannel()` doesn't rely on it: it diffs the PIT `TCTRL` enable bits around
`begin()` to identify the granted channel, then takes over its vector at priority 64 while
keeping the `IntervalTimer` allocated so the core can never hand the channel out again.

**Implementation deviations from plan (documented in code):** the 24-bit DAC word is shifted as
**two 12-bit frames** (not 8+16) because both frames must share CTAR0 (CTAR1 is the ADC's) and
K64 FIFO frames are ≤16 bits — the AD5752 only counts SCLK edges while SYNC is low, so this is
equivalent. `dacProgramBoth` is sequential per channel (shared MOSI ⇒ inherent), the FIFO overlap
is within each 24-bit word. `K_RELOAD` is calibrated closed-loop by running the real
`pitProgram()` path and polling `TFLG`; the few-cycle poll-detection latency stays inside the
constant, which is fine because players wake `PRELOAD` (≈4 µs) early and spin on CYCCNT.

**Open questions / next bench session (hardware needed):** run `BENCHDAC/ADC/SW/MISO` to replace
the calibrated 2.75/4.5 µs figures with measured ones; `BENCHPIT,1000,5000` (raw) to size
`SJ_PRELOAD_US`, then `BENCHPIT,1000,5000,4` for the latch-jitter acceptance (<200 ns);
`BENCHK` variance; scope A/B `BENCHSQ,0,10000,50,100` vs `BENCHSQL,…` (Phase-1 exit criterion);
AD7321 first-conversion validity after line switch and AD5752 settling (plan §7 items 1–2) remain
open for the measurement GUARD. Real GPIO header pins still unverified (`Stimjim.h` bug).

**Next entry point:** Phase 2 — TrainStore staging/validate/commit, legacy `S`/`W` + new `L`
parsers with atomic slot updates, `?` queries and round-trip serializers, `M/V/A/E` byte-exact
replies; verify against StimJimBIST and the README/header example lines.
