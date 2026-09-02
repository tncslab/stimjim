//    stimjimAWG (c) 2026- TNCS, Dept of Comp Sci, HUN-REN Wigner RCP, Hungary
//
//    Arbitrary-waveform-generator firmware for the StimJim dual-channel
//    isolated stimulator, targeting Teensy 3.5 (MK64FX512, 120 MHz).
//    Successor of stimjimPulser (szinuszgenerator lineage). Design documents:
//    docs/awg-implementation-plan.md, docs/serial-protocol.md,
//    docs/hardware-notes.md; progress log: docs/PROGRESS.md.
//
//    What this firmware does: 100 waveform slots defined over a serial protocol
//    (`S` rectangular, `L` linear-ramp, `W` sine, each with an amplitude
//    envelope and an optional post-trigger delay), played by two independent
//    channel players on a deadline scheduler with copy-on-arm semantics, from
//    `T`/`U` commands or from an edge on either trigger input. It also carries
//    the register-level DAC/ADC path, a BENCH timing harness, EEPROM
//    persistence and an OLED status display. Register and portable backends
//    coexist so the same source runs on Teensy 3.x and 4.x — see
//    docs/hardware-variants.md.
//
//    Not implemented: in-train measurement (`MEAS` execution, MSUM/MDATA),
//    SD logging (`LOG`), and the button menu editor.
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

#include "Config.h"
#include "FastIO.h"
#include "Engine.h"
#include "Protocol.h"
#include "SampleGen.h"
#include "TrainStore.h"
#include "Triggers.h"
#include "Measure.h"
#include "SdLog.h"
#include "UiInput.h"
#include "UiMenu.h"

// Boot tracing: build with -DSJ_BOOT_TRACE to make setup() wait for a serial
// host and announce each init step. Without it the board boots headless as it
// must; with it, a hang in any init step is pinned down in one flash cycle.
#ifdef SJ_BOOT_TRACE
#define SJ_TRACE(msg)  do { Serial.println("# boot: " msg); Serial.send_now(); } while (0)
#else
#define SJ_TRACE(msg)  do { } while (0)
#endif

void setup() {
  Serial.begin(9600);        // USB CDC — the rate is irrelevant
#ifdef SJ_BOOT_TRACE
  while (!Serial) ;          // block until a host opens the port
  delay(200);
  SJ_TRACE("serial up");
#endif

  // Legacy library boot: SPI init, DAC range/power-up, ADC + current offset
  // calibration (outputs stay grounded throughout; takes a few hundred ms).
  SJ_TRACE("-> Stimjim.begin");
  Stimjim.begin();

  // Take over SPI0 (register-level CTARs), enable CYCCNT, set MISO mux state.
  // Must come after Stimjim.begin() — the SPI library clobbers the CTARs.
  SJ_TRACE("-> FastIO::begin");
  FastIO::begin();

  SJ_TRACE("-> Engine::begin");
  Engine::begin();           // reserve 2 PIT channels + K_RELOAD self-calibration
  SJ_TRACE("-> SampleGen::sineTabInit");
  SampleGen::sineTabInit();  // 2 KB Q15 sine table (double sin, boot only)
  SJ_TRACE("-> TrainStore::begin");
  TrainStore::begin();
  SJ_TRACE("-> Triggers::begin");
  Triggers::begin();

  // Restore the EEPROM image (slots 0-9 + trigger table) if magic, version and
  // CRC all check out. setRoute() also wires the pins, so a restored trigger
  // is live from boot.
  SJ_TRACE("-> eepromRestore");
  TriggerRoute trig[2];
  if (TrainStore::eepromRestore(trig)) {
    Triggers::setRoute(0, trig[0]);
    Triggers::setRoute(1, trig[1]);
    Serial.println("# EEPROM: restored slots 0-9 and the trigger table");
  } else {
    Serial.println("# EEPROM: no valid image — using boot defaults");
  }

  SJ_TRACE("-> Measure::begin");
  Measure::begin();
  SJ_TRACE("-> SdLog::begin");
  SdLog::begin();
  SJ_TRACE("-> UiInput::begin");
  UiInput::begin();
  SJ_TRACE("-> UiMenu::begin");
  UiMenu::begin();

  SJ_TRACE("-> Protocol::begin");
  Protocol::begin();         // boot banner + IDN + engine info
}

void loop() {
  Protocol::poll();          // serial in -> command dispatch (all printing here)
  Engine::poll();            // cycles64 keep-alive
  Commands::poll();          // completion-ring drain: train result summaries
  Triggers::poll();          // deferred trigger-reject WARNs (ISRs never print)
  Measure::poll();           // MDATA ring drain (no-op until MEAS execution exists)
  SdLog::poll();             // SD row writer (no-op until SD logging exists)
  UiMenu::tick();            // event drain + throttled render
}
