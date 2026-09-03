//    stimjimAWG (c) 2026- TNCS, Dept of Comp Sci, HUN-REN Wigner RCP, Hungary
//
//    Arbitrary-waveform-generator firmware for the StimJim board.
//    Design documents: docs/awg-implementation-plan.md, docs/serial-protocol.md,
//    docs/hardware-notes.md, docs/hardware-variants.md.
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//    <https://www.gnu.org/licenses/>

#ifndef STIMJIMAWG_CONFIG_H
#define STIMJIMAWG_CONFIG_H

#include <Arduino.h>
#include <Stimjim.h>   // pin macros (CS0_x, NLDAC_x, ...) and unit-conversion constants

// ------------------------------------------------------------------ identity
#define SJ_FW_NAME       "stimjimAWG"
#define SJ_FW_VERSION    "0.7.0"
#define SJ_PROTO_VERSION 1

// ---------------------------------------------------------- hardware variant
//
// Teensy 3.5/3.6 (Kinetis K64/K66) run the register-level fast path the whole
// timing design was calibrated on: SPI0 driven through its DSPI registers and
// two PIT channels whose vectors we take over one by one.
//
// Every other target -- notably the Teensy 4.x boards open-ephys is moving to
// now that the 3.5 is out of stock -- builds a portable route on the standard
// Arduino SPI library and IntervalTimer. It is functionally identical but
// slower and less predictable per event, so every timing constant marked
// RECALIBRATE below must be re-measured on the new board with the BENCH group
// (docs/serial-protocol.md §4) before the firmware is trusted for stimulation.
//
// Why Teensy 4 does not simply reuse the register path: on the i.MX RT1062 all
// four PIT channels share a single IRQ_PIT vector (Teensyduino 1.62 core,
// cores/teensy4/IntervalTimer.cpp), so the per-channel attachInterruptVector
// and per-channel NVIC priority of the Kinetis path have no equivalent. Going
// through IntervalTimer lets the core's own shared dispatcher do that work.
// A native LPSPI4 + shared-vector PIT backend is a later optimization.
#if defined(__MK64FX512__)
  #define SJ_MCU_KINETISK  1
  #define SJ_HW_NAME       "Teensy3.5"
#elif defined(__MK66FX1M0__)
  #define SJ_MCU_KINETISK  1
  #define SJ_HW_NAME       "Teensy3.6"
#elif defined(__IMXRT1062__)
  #define SJ_MCU_KINETISK  0
  #define SJ_HW_NAME       "Teensy4.x"
#else
  #define SJ_MCU_KINETISK  0
  #define SJ_HW_NAME       "generic-ARM"
#endif

// Backend selection. Both default to the MCU family but can be forced from the
// build (-DSJ_FASTIO_REGISTER=0) to A/B the portable route on a Teensy 3.5 --
// that is how the portable path's timing constants get measured on known-good
// hardware instead of guessed.
#ifndef SJ_FASTIO_REGISTER
  #define SJ_FASTIO_REGISTER SJ_MCU_KINETISK   // 1 = DSPI registers, 0 = Arduino SPI library
#endif
#ifndef SJ_TIMER_REGISTER
  #define SJ_TIMER_REGISTER  SJ_MCU_KINETISK   // 1 = raw PIT + own vectors, 0 = IntervalTimer
#endif

// The 64-bit timebase and the bus lock are Cortex-M3/M4/M7 features (DWT cycle
// counter, BASEPRI). Both Teensy families have them; a target without them
// needs a different timebase design, not a #define.
#if !defined(ARM_DWT_CYCCNT)
  #error "stimjimAWG needs the Cortex-M DWT cycle counter (Teensy 3.x / 4.x)"
#endif

// ------------------------------------------------------------ feature switches
#define SJ_USE_DISPLAY   1   // SSD1306 128x32 on Wire @0x3C (geometry below)

// SD logging + the SD file-access group. The Teensy 3.5/3.6 and 4.1 carry a
// micro-SD socket on native SDIO (BUILTIN_SDCARD); a Teensy 4.0 has none, so
// the group is compiled out there and answers ERR instead of failing to link.
#ifndef SJ_USE_SD
  #if defined(ARDUINO_TEENSY40)
    #define SJ_USE_SD    0
  #else
    #define SJ_USE_SD    1
  #endif
#endif

// ---------------------------------------------------------------- clocking
// 1 us must be an exact whole number of CPU cycles -- every us<->cycle
// conversion in the engine is an integer multiply and must not drift.
// Teensy 3.5 @120 MHz: 120 cycles/us. Teensy 4.x @600 MHz: 600 cycles/us.
#if (F_CPU % 1000000) != 0
#error "stimjimAWG needs an integer number of CPU cycles per microsecond"
#endif
#define SJ_CYC_PER_US    ((uint32_t)(F_CPU / 1000000u))
#define SJ_US_TO_CYC(us) ((uint64_t)(us) * SJ_CYC_PER_US)
#define SJ_CYC_TO_US(cy) ((uint64_t)(cy) / SJ_CYC_PER_US)

#if SJ_TIMER_REGISTER
// The raw-PIT backend converts CPU cycles to PIT ticks by a shift, which
// assumes the Kinetis F_BUS = F_CPU/2 relationship (60 MHz at stock clocks).
#if F_CPU != 120000000
#error "the Kinetis register backend is calibrated for 120 MHz (Tools > CPU Speed)"
#endif
#if F_BUS != 60000000
#error "the Kinetis register backend assumes F_BUS = F_CPU/2 = 60 MHz"
#endif
#define SJ_PIT_PER_US    60u    // PIT (F_BUS) ticks per microsecond (exact)
#endif

// -------------------------------------------------- NVIC priorities (plan §3.3)
// Lower value = higher priority. SysTick stays at core default 32;
// USB serial stays at core default 112 (preempted by players by design).
#define SJ_PLAYER_PRIO   64   // PIT player ISRs (both channels; equal prio => serialized)
#define SJ_TRIG_PRIO     80   // IN0/IN1 edge ISRs

// ------------------------------------------------- engine knobs (plan §3.1)
//
// The nine hardware timing budgets below are the *defaults* of the runtime
// `CAL` parameter set (Cal.h): the engine arms from Cal::live(), which starts
// as a copy of these and can be changed over the serial port and persisted
// with `P`. Editing this file is still the right move for a value that is
// wrong on every board of a given type; `CAL` is for calibrating one bench
// without a rebuild. `CAL?` prints what a running board actually uses.
//
// RECALIBRATE all of the following whenever the MCU, the clock speed or a
// backend changes. Procedure, in this order, with nothing else running:
//   BENCHDAC / BENCHDAC2          -> SJ_DAC_PROG1_US / SJ_DAC_PROG2_US (round up)
//   BENCHADC / BENCHSW            -> the measurement budget of plan §3.6
//   BENCHPIT,1000,5000            -> raw ISR wake latency, sizes SJ_PRELOAD_US
//   BENCHPIT,1000,5000,<preload>  -> residual latch jitter, must stay <200 ns
//   BENCHK                        -> K_RELOAD spread (self-calibrated at boot anyway)
// The register-path values are desk estimates for the Teensy 3.5, not measured
// figures. The portable route pays an extra SPI-library
// beginTransaction/endTransaction per DAC word and an IntervalTimer
// end()+begin() per scheduled event, so its budgets are deliberately generous.
#if SJ_FASTIO_REGISTER
  #define SJ_DAC_PROG1_US    3   // budgeted dacProgram cost, single channel (calibrated 2.75)
  #define SJ_DAC_PROG2_US    5   // dacProgramBoth
#else
  #define SJ_DAC_PROG1_US    6   // RECALIBRATE: SPI-library transaction overhead included
  #define SJ_DAC_PROG2_US   11
#endif
#if SJ_TIMER_REGISTER
  #define SJ_PRELOAD_US      4   // ISR wakes this early, spins on CYCCNT, latches on deadline
#else
  #define SJ_PRELOAD_US      8   // RECALIBRATE: IntervalTimer re-arm is slower than a raw LDVAL
#endif
// Measurement budget (plan §3.6). One in-train ADC value costs a control-
// register write selecting the input line plus the conversion read; GUARD is
// the margin left before the next latch event starts programming the DAC, and
// SETTLE is how long the AD5752 output needs after a latch before a reading
// means anything. Together they decide whether a measurement point fits its
// stage (S), its ramp sample interval (L) or its sine sample interval (W) --
// Measure::planBuild refuses the point rather than delaying a latch.
// These are *budgets*, so they carry the measured worst case, not the average:
// a single outlying read that overruns its window pushes the next latch late,
// and Engine::progLatch counts exactly that. BENCHADC measured adcRead at
// 2.22 us avg / 3.81 us max and BENCHSW measured select+read at 4.70 / 6.10 on
// a Teensy 3.5; 3 + 4 covers the maximum outright instead of leaning on GUARD.
// RECALIBRATE with BENCHADC / BENCHSW (item 1) and a scope (item 2).
#if SJ_FASTIO_REGISTER
  #define SJ_ADC_READ_US     3   // adcRead, line pre-selected (measured 2.22 avg, 3.81 max)
  #define SJ_ADC_SWITCH_US   4   // the extra cost of a line switch (measured 2.5 avg, 2.3 max)
#else
  #define SJ_ADC_READ_US     6   // RECALIBRATE: SPI-library transaction overhead included
  #define SJ_ADC_SWITCH_US   8
#endif
#define SJ_MEAS_GUARD_US     1   // margin between the last read and the next preload window
// How long after a latch a reading means anything. Two things set the floor:
// the AD5752 output settling (bench item 2) and the player's own post-latch
// bookkeeping, measured at ~3.2 us on the register path -- a smaller value
// would place the reads at an instant the ISR cannot reach, which is harmless
// for the reading (zero derivative at a sine peak) but makes the schedule a
// fiction. RECALIBRATE with a scope.
#define SJ_DAC_SETTLE_US     4

#define SJ_MIN_SCHEDULE_US   3         // events closer than this are run inline in the same ISR pass
#define SJ_MAX_SLICE_US      10000000  // 10 s: chunk longer gaps (PIT max ~71 s; keeps cycles64 alive)
// Fixed arm->first-latch latency, which is what makes the trigger latency
// deterministic. It must cover a preload plus a dual-channel DAC program plus
// SJ_MIN_SCHEDULE_US (Cal::validate refuses a set that does not), so the
// portable route — whose program budget is twice the register path's — needs a
// wider one. A trigger-started train subtracts the CAL TRIGCOMP parameter from
// it, so the sum of the two is what the hardware actually has to cover.
#if SJ_TIMER_REGISTER && SJ_FASTIO_REGISTER
  #define SJ_START_LATENCY_US 20
#else
  #define SJ_START_LATENCY_US 40       // RECALIBRATE with the other portable budgets
#endif
#define SJ_TARGET_DT_US      20        // default ramp sample interval; per-train override: TrainDef.dt_us
// SJ_MAX_DELAY_US (per-slot post-trigger delay ceiling) lives in WaveformDef.h:
// the host-testable parser validates against it and never includes this file.

// Sine sample-rate policy (plan §3.5): Fs = clamp(SAMPLES_PER_CYC * f_max,
// FS_MIN, FS_MAX), realized as an exact integer number of CPU cycles per
// sample. FS_MAX is provisional pre-bench (min sample period 20 us >> the
// preload+program budget) and must be replaced by the measured dual-channel
// ceiling with ~30 % margin. f_max above FS_MAX/2 is refused at start.
#define SJ_SINE_SAMPLES_PER_CYC 64
#define SJ_FS_MIN_HZ            1000
#if SJ_FASTIO_REGISTER
  #define SJ_FS_MAX_HZ          50000
#else
  #define SJ_FS_MAX_HZ          25000   // RECALIBRATE: halved for the slower portable route
#endif

// ------------------------------------------------------------------ protocol
#define SJ_LINE_MAX      999   // longest accepted serial line (excl. terminator)

// ---------------------------------------------------------------- UI geometry
#define SJ_OLED_WIDTH    128
#define SJ_OLED_HEIGHT   32
#define SJ_OLED_ADDR     0x3C
#define SJ_OLED_COLS     (SJ_OLED_WIDTH / 6)    // 6x8 default GFX font
#define SJ_OLED_ROWS     (SJ_OLED_HEIGHT / 8)
#define SJ_UI_MIN_MS     100   // minimum interval between physical display writes
#define SJ_BTN_OK        17    // Btn0
#define SJ_BTN_PREV      39    // Btn1
#define SJ_BTN_NEXT      16    // Btn2
#define SJ_BTN_DEBOUNCE_MS 25

#endif // STIMJIMAWG_CONFIG_H
