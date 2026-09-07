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

// Dispatch input `input`'s route as if an edge had just arrived: start the
// routed slot on one engine (joint) or on both (independent). `at` is the
// cycle count the start was *requested* at, which is what the delivered latency
// is measured from; 0 means "now".
//
// The edge ISRs call it with their own entry timestamp. The trigger buttons
// call it from loop() with 0, because a press is a human action and a
// loop()-scale dispatch delay does not matter — that is what keeps arming to a
// single context and leaves IRQ_PORTA/IRQ_PORTB at the core's priority.
//
// Returns false when the route drives no train at all (mode 0 or 3), so a
// caller can say so; a start refused by a busy engine is counted instead and
// reported by poll(), the same as for an electrical edge.
bool fireRoute(uint8_t input, uint64_t at);

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
