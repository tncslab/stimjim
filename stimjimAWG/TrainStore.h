//    stimjimAWG — TrainStore: 100 waveform slots with staging/validate/commit
//    (atomic — a malformed line never half-updates a slot), defaults per
//    protocol §5, EEPROM v2 persistence. Full logic arrives in Phase 2.
//    GPL-3.0-or-later; see Config.h header.

#ifndef STIMJIMAWG_TRAINSTORE_H
#define STIMJIMAWG_TRAINSTORE_H

#include "WaveformDef.h"

namespace TrainStore {

void begin();   // initialize all slots to defaults (Phase 2 adds EEPROM restore)

// Boot default (protocol §5): grounded modes, period 10 ms, duration 500 ms,
// 0 stages, type S; ENV 0,0,0; MEAS 3,3,auto-when,0.
void slotDefault(TrainDef& t);

TrainDef&       slot(uint8_t idx);         // idx asserted < SJ_NUM_SLOTS by caller
const TrainDef& slotConst(uint8_t idx);

// Phase 2: staging buffer + validate + commit, round-trip serializers,
// running-slot edit refusal, EEPROM v2 save/restore (`P`).

} // namespace TrainStore

#endif // STIMJIMAWG_TRAINSTORE_H
