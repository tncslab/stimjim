//    stimjimAWG (c) 2026- TNCS, Dept of Comp Sci, HUN-REN Wigner RCP, Hungary
//
//    Arbitrary-waveform-generator firmware for the StimJim dual-channel
//    isolated stimulator, targeting Teensy 3.5 (MK64FX512, 120 MHz).
//    Successor of stimjimPulser (szinuszgenerator lineage). Design documents:
//    docs/awg-implementation-plan.md, docs/serial-protocol.md,
//    docs/hardware-notes.md; progress log: docs/PROGRESS.md.
//
//    Phase 3 state: FastIO + BENCH harness (Phase 1); full waveform-definition
//    protocol with atomic staging, queries, EEPROM (Phase 2); PIT deadline
//    scheduler playing rectangular (`S`) trains via T/U with copy-on-arm and
//    the completion ring, plus READ manual measurement (Phase 3). L/W playback
//    arrives in Phases 4-5, the measurement engine + SD in Phase 7.
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
#include "TrainStore.h"
#include "Triggers.h"
#include "Measure.h"
#include "SdLog.h"
#include "UiInput.h"
#include "UiMenu.h"

void setup() {
  Serial.begin(9600);        // USB CDC — the rate is irrelevant

  // Legacy library boot: SPI init, DAC range/power-up, ADC + current offset
  // calibration (outputs stay grounded throughout; takes a few hundred ms).
  Stimjim.begin();

  // Take over SPI0 (register-level CTARs), enable CYCCNT, set MISO mux state.
  // Must come after Stimjim.begin() — the SPI library clobbers the CTARs.
  FastIO::begin();

  Engine::begin();           // reserve 2 PIT channels + K_RELOAD self-calibration
  TrainStore::begin();
  Triggers::begin();

  // Restore the EEPROM v2 image (slots 0-9 + trigger table) if magic/version/
  // CRC check out; edge-ISR wiring on the restored routes arrives in Phase 8.
  TriggerRoute trig[2];
  if (TrainStore::eepromRestore(trig)) {
    Triggers::setRoute(0, trig[0]);
    Triggers::setRoute(1, trig[1]);
    Serial.println("# EEPROM: restored slots 0-9 and the trigger table");
  } else {
    Serial.println("# EEPROM: no valid image — using boot defaults");
  }

  Measure::begin();
  SdLog::begin();
  UiInput::begin();
  UiMenu::begin();

  Protocol::begin();         // boot banner + IDN + engine info
}

void loop() {
  Protocol::poll();          // serial in -> command dispatch (all printing here)
  Engine::poll();            // cycles64 keep-alive
  Commands::poll();          // completion-ring drain: train result summaries
  Triggers::poll();          // Phase 8: deferred trigger-reject WARNs
  Measure::poll();           // Phase 7: MDATA ring drain
  SdLog::poll();             // Phase 7: SD row writer
  UiMenu::tick();            // event drain + throttled render
}
