//    stimjimAWG — SdLog: timestamped measurement logging to the Teensy 3.5
//    onboard micro-SD (native SDIO — never touches the DAC/ADC SPI bus).
//    Implemented in Phase 7 (plan §3.7); only loop() ever writes to the card.
//    GPL-3.0-or-later; see Config.h header.

#ifndef STIMJIMAWG_SDLOG_H
#define STIMJIMAWG_SDLOG_H

#include <stdint.h>

namespace SdLog {

void begin();
void poll();   // drain the MDATA ring to the open file; flush at train end

// Phase 7 (`LOG` command backend): LOG? status, LOG1[,name] open, LOG0 close.
// CSV row: <timestamp_us>,<slot>,<pulse>,<stage>,<V0_mV>,<I0_uA>,<V1_mV>,<I1_uA>
// (timestamps = cycles64()/120 since boot; header line carries IDN + train defs)

} // namespace SdLog

#endif // STIMJIMAWG_SDLOG_H
