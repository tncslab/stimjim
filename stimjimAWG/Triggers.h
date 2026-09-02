//    stimjimAWG — Triggers: IN0/IN1 routing table, edge ISRs at SJ_TRIG_PRIO,
//    the stimulus-marker output, and the legacy `R` <-> `TRIG` mapping.
//    Trigger-conflict policy: ignore + WARN (plan §2 decision 5).
//    GPL-3.0-or-later; see Config.h header.

#ifndef STIMJIMAWG_TRIGGERS_H
#define STIMJIMAWG_TRIGGERS_H

#include "WaveformDef.h"

namespace Triggers {

void begin();   // boot default: both inputs mode 3 (output marker) — legacy state
void poll();    // print deferred WARNs (dropped triggers) from loop context

const TriggerRoute& route(uint8_t input);
// Store a routing entry and reconfigure the pin (interrupt, marker output or
// plain input). Loop context only — takes the bus lock.
void setRoute(uint8_t input, const TriggerRoute& r);
// NULL if the route is acceptable, else a static reason.
const char* validateRoute(const TriggerRoute& r);
// Drive every marker-mode pin high/low. Called by the player ISRs around a
// stimulus, so it must stay a couple of GPIO writes.
void marker(bool on);
uint32_t rejectCount();

} // namespace Triggers

#endif // STIMJIMAWG_TRIGGERS_H
