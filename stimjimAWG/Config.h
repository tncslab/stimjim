//    stimjimAWG (c) 2026- TNCS, Dept of Comp Sci, HUN-REN Wigner RCP, Hungary
//
//    Arbitrary-waveform-generator firmware for the StimJim board (Teensy 3.5).
//    Design documents: docs/awg-implementation-plan.md, docs/serial-protocol.md,
//    docs/hardware-notes.md.
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
#define SJ_FW_VERSION    "0.1.0"        // Phase 1 scaffold
#define SJ_PROTO_VERSION 1
#define SJ_HW_NAME       "Teensy3.5"

// ------------------------------------------------------------ feature switches
#define SJ_USE_DISPLAY   1   // SSD1306 128x32 on Wire @0x3C (geometry below)
#define SJ_USE_SD        0   // SdLog arrives in Phase 7

// ---------------------------------------------------------------- clocking
// The entire timing design assumes Teensy 3.5 at stock clocks:
// F_CPU = 120 MHz, F_BUS = 60 MHz  =>  1 us = exactly 120 CPU cycles
// = exactly 60 PIT ticks. All us<->cycle conversions are exact integers.
#if F_CPU != 120000000
#error "stimjimAWG assumes Teensy 3.5 at 120 MHz (Tools > CPU Speed: 120 MHz)"
#endif
#if F_BUS != 60000000
#error "stimjimAWG assumes F_BUS = 60 MHz"
#endif
#define SJ_CYC_PER_US    120u   // CPU cycles per microsecond (exact)
#define SJ_PIT_PER_US    60u    // PIT (F_BUS) ticks per microsecond (exact)
#define SJ_US_TO_CYC(us) ((uint64_t)(us) * SJ_CYC_PER_US)
#define SJ_CYC_TO_US(cy) ((uint64_t)(cy) / SJ_CYC_PER_US)

// -------------------------------------------------- NVIC priorities (plan §3.3)
// Lower value = higher priority. SysTick stays at core default 32;
// USB serial stays at core default 112 (preempted by players by design).
#define SJ_PLAYER_PRIO   64   // PIT player ISRs (both channels; equal prio => serialized)
#define SJ_TRIG_PRIO     80   // IN0/IN1 edge ISRs

// ------------------------------------------------- engine knobs (plan §3.1)
#define SJ_PRELOAD_US        4         // ISR wakes this early, spins on CYCCNT, latches on deadline
#define SJ_MIN_SCHEDULE_US   3         // events closer than this are run inline in the same ISR pass
#define SJ_MAX_SLICE_US      10000000  // 10 s: chunk longer gaps (PIT max ~71 s; keeps cycles64 alive)
#define SJ_START_LATENCY_US  20        // fixed arm->first-latch latency: trigger latency is deterministic
#define SJ_TARGET_DT_US      20        // default ramp sample interval (per-train overridable later)

// ------------------------------------------------------------------ protocol
#define SJ_LINE_MAX      999   // longest accepted serial line (excl. terminator)

// ---------------------------------------------------------------- UI geometry
#define SJ_OLED_WIDTH    128
#define SJ_OLED_HEIGHT   32
#define SJ_OLED_ADDR     0x3C
#define SJ_BTN_OK        17    // Btn0
#define SJ_BTN_PREV      39    // Btn1
#define SJ_BTN_NEXT      16    // Btn2
#define SJ_BTN_DEBOUNCE_MS 25

#endif // STIMJIMAWG_CONFIG_H
