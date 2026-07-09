//    stimjimAWG — Triggers implementation, Phase 1 scope (state only).
//    GPL-3.0-or-later; see Config.h header.

#include "Triggers.h"

namespace Triggers {

static TriggerRoute routes[2];
static volatile uint32_t rejects = 0;

void begin() {
  // legacy boot state: both triggers are output markers
  routes[0] = {3, -1, -1, 0};
  routes[1] = {3, -1, -1, 0};
}

void poll() {}

const TriggerRoute& route(uint8_t input) { return routes[input]; }
uint32_t rejectCount()                   { return rejects; }

} // namespace Triggers
