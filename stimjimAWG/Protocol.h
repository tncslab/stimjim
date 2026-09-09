//    stimjimAWG — Protocol: bounded line assembler + dispatch (plan §4, protocol §1).
//    GPL-3.0-or-later; see Config.h header.

#ifndef STIMJIMAWG_PROTOCOL_H
#define STIMJIMAWG_PROTOCOL_H

#include <stdint.h>

class Print;   // Arduino output sink; only referenced, never dereferenced here

namespace Protocol {
void begin();   // prints the boot banner + identity block
void poll();    // drain Serial, assemble lines, dispatch complete ones
// IDN line plus `#` build/engine detail — shared by the banner, `IDN` and the
// SD log header, hence the explicit sink.
void printIdentity(Print& out);

// Decimal text of a 64-bit value; buf must hold >= 21 bytes. Teensyduino's
// printf is not relied on for %llu, and card sizes and file offsets exceed
// 32 bits.
void u64str(uint64_t v, char* buf);
}

namespace Commands {
// Dispatch one complete, NUL-terminated line (no terminator, not blank/comment).
void handleLine(const char* line);
// Drain the engine completion ring and print result summaries (loop context —
// the player ISRs never print; plan §3.3).
void poll();
// The `DUMP` body — every non-default slot, ENV/MEAS and both TRIG lines, as
// paste-back-able set-commands, without the trailing `OK`. Written to Serial by
// `DUMP` and into the SD log header by SdLog.
void writeDump(Print& out);
// Open an SD log if any slot's `MEAS` report asks for one (SJ_REPORT_SD).
// Called after every slot commit and once from setup() after the EEPROM
// restore, which is what makes a headless box log from boot. Silent and
// harmless with no card in the socket — see SdLog::autoOpen.
void logIfConfigured();
}

#endif // STIMJIMAWG_PROTOCOL_H
