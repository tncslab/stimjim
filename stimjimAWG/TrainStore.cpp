//    stimjimAWG — TrainStore implementation, Phase 1 scope (defaults only).
//    GPL-3.0-or-later; see Config.h header.

#include "TrainStore.h"
#include <string.h>

namespace TrainStore {

static TrainDef slots[SJ_NUM_SLOTS];

void slotDefault(TrainDef& t) {
  memset(&t, 0, sizeof(t));
  t.type        = PIECEWISE_HOLD;
  t.mode0       = 5;          // grounded
  t.mode1       = 5;
  t.period_us   = 10000;
  t.duration_us = 500000;
  t.nStages     = 0;
  t.meas.what0  = 3;          // both V and I
  t.meas.what1  = 3;
  t.meas.when   = 1;          // all stages (auto-switches to 2 when type becomes SINE)
  t.meas.report = 0;          // end-of-train summary only
}

void begin() {
  for (auto& s : slots) slotDefault(s);
}

TrainDef&       slot(uint8_t idx)      { return slots[idx]; }
const TrainDef& slotConst(uint8_t idx) { return slots[idx]; }

} // namespace TrainStore
