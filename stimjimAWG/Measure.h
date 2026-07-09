//    stimjimAWG — Measure: per-train measurement plans, accumulation, MDATA ring
//    and summary formatting (plan §3.6). Implemented in Phase 7.
//    GPL-3.0-or-later; see Config.h header.

#ifndef STIMJIMAWG_MEASURE_H
#define STIMJIMAWG_MEASURE_H

#include <stdint.h>

namespace Measure {

void begin();
void poll();   // drain MDATA ring: serial streaming / SD rows (loop context only)

// Phase 7: MeasurePlan compiled at arm time — for each selected stage a MEASURE
// event at t_meas = stageEnd - sum(adcRead) - lineSwitch - GUARD; SINE_PEAK at
// the first 90-degree crossing after envelope ramp-in. Results accumulate as
// int32 sums + counts per stage/line/channel (legacy averaging semantics).
// MDATA record format (frozen): MDATA,<slot>,<pulse>,<stage>,<V0_mV>,<I0_uA>,<V1_mV>,<I1_uA>

} // namespace Measure

#endif // STIMJIMAWG_MEASURE_H
