//    stimjimAWG — Triggers: IN0/IN1 routing table, edge ISRs at SJ_TRIG_PRIO,
//    legacy `R` <-> `TRIG` mapping. Implemented in Phase 8 (plan §4);
//    trigger-conflict policy: ignore + WARN (decision 5).
//    GPL-3.0-or-later; see Config.h header.

#ifndef STIMJIMAWG_TRIGGERS_H
#define STIMJIMAWG_TRIGGERS_H

#include "WaveformDef.h"

namespace Triggers {

void begin();   // boot default: both inputs mode 3 (output marker) — legacy state
void poll();    // print deferred WARNs (trigRejectCount) from loop context

const TriggerRoute& route(uint8_t input);
uint32_t rejectCount();

} // namespace Triggers

#endif // STIMJIMAWG_TRIGGERS_H
