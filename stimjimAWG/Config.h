//    stimjimAWG (c) 2026- TNCS, Dept of Comp Sci, HUN-REN Wigner RCP, Hungary
//
//    Arbitrary-waveform-generator firmware for the StimJim board.
//    Design documents: docs/awg-implementation-plan.md, docs/serial-protocol.md,
//    docs/timing.md (what the budgets below deliver as latency), docs/hardware-notes.md,
//    docs/hardware-variants.md.
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

// --------------------------------------------------- code placement (Kinetis)
//
// The K64 fetches instructions from flash through a controller with a 512-byte
// cache at 120 MHz, so a long straight-line function pays wait states its
// instruction mix does not explain: the arm spends ~5-6 cycles per instruction
// where SRAM-resident code would spend 1-2 (docs/timing.md §7). SJ_CODE_IN_RAM
// puts the two functions whose cost is a latency -- the arm and the player
// event loop -- in the .fastrun section, which the core's startup copies from
// flash into SRAM at boot; they then execute at SRAM speed. It costs RAM equal
// to their code size and nothing else: the copy is remade from the flash image
// on every reset, so a brownout cannot leave stale code behind (and the MCU's
// own POR/LVD resets it long before SRAM contents decay).
//
// Teensy 4.x already runs *all* code from ITCM RAM and needs FLASHMEM to opt
// out, so this switch is Kinetis-only and a no-op elsewhere. Build with
// -DSJ_CODE_IN_RAM=0 to A/B it with BENCHARM.
#ifndef SJ_CODE_IN_RAM
  #define SJ_CODE_IN_RAM SJ_MCU_KINETISK
#endif
#if SJ_CODE_IN_RAM && defined(FASTRUN)
  #define SJ_HOT      FASTRUN
  #define SJ_HOT_NAME "RAM"
#elif defined(__IMXRT1062__)
  #define SJ_HOT                  // Teensy 4.x: the core already runs all code from ITCM
  #define SJ_HOT_NAME "ITCM"
#else
  #define SJ_HOT
  #define SJ_HOT_NAME "flash"
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
// How long after a latch a reading means anything: the AD5752 output settling
// and the ADC's own aperture together, which is the composite the measurement
// engine actually depends on. MEASURED with BENCHSETTLE, which needs no scope
// -- it latches the same step repeatedly and reads it back at increasing
// delays. On this board the reading is still climbing 4 us after the latch
// (8.6 % short of the final value) and reaches it at 8-9 us, on either
// channel, either polarity and at 2000 or 8000 DAC codes of step. 9 us is the
// worst of those, and a budget carries the worst case.
//
// This is a property of the analog path, not of the MCU or the backend, so it
// is not conditioned on either. RECALIBRATE with BENCHSETTLE on a new board.
#define SJ_DAC_SETTLE_US     9

#define SJ_MIN_SCHEDULE_US   3         // events closer than this are run inline in the same ISR pass
#define SJ_MAX_SLICE_US      10000000  // 10 s: chunk longer gaps (PIT max ~71 s; keeps cycles64 alive)
// Fixed arm->first-latch latency, which is what makes the trigger latency
// deterministic. Two things have to fit inside it, and only the first is
// checked by Cal::validate:
//
//   1. a preload plus a dual-channel DAC program plus SJ_MIN_SCHEDULE_US,
//      because the first latch is programmed one preload window before t0;
//   2. the whole of Engine::startTrain, because t0 is measured from the start
//      request (a trigger edge timestamps itself) and every microsecond the
//      arm spends comes out of the latency.
//
// Term 2 dominates and depends on the train: BENCHARM measures it per slot,
// and tests/device/bench_arm.py sweeps the shapes. Measured on a Teensy 3.5 at
// 120 MHz with the register backends, on a board whose loop() is running (which
// is what prepares each engine's next measurement plan -- Engine::warmPlans):
// 8.8 us for a slot that drives nothing, 10.4 us for a one-stage two-channel
// `S` train measured or not, 10.8 us for a `W` sine, 17.5 us for a ten-stage
// `S` train with a measurement point on every stage, and 28.7 us for a
// ten-stage `L` train -- the worst case, because a ramp stage costs the arm
// twice what a rectangular one does. 45 us covers all of them with margin.
//
// What it does not cover is a TRIG independent route -- which arms two engines
// from one edge, paying term 2 twice -- of two ten-stage `L` slots, which needs
// about 70 us. Nor does it cover an engine re-triggered so fast that loop()
// never ran in between, which falls back to preparing the plan inside the arm.
// Both cases still play; the engine names the STARTLAT that would have covered
// the arm at train end, and CAL sets it without a rebuild.
//
// Raising this default rather than lowering it is the safe direction, so a
// board that has an EEPROM image saved keeps whatever budget it stored -- the
// image is not rejected over this, because a stored 60 us is merely
// conservative, not wrong. `CAL,STARTLAT,45` then `P` adopts the new one.
//
// A trigger-started train subtracts the CAL TRIGCOMP parameter from it, so the
// sum of the two is what the hardware actually has to cover.
#if SJ_TIMER_REGISTER && SJ_FASTIO_REGISTER
  #define SJ_START_LATENCY_US 45
#else
  #define SJ_START_LATENCY_US 120      // RECALIBRATE with BENCHARM on the target board
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
