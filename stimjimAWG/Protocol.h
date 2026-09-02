//    stimjimAWG — Protocol: bounded line assembler + dispatch (plan §4, protocol §1).
//    GPL-3.0-or-later; see Config.h header.

#ifndef STIMJIMAWG_PROTOCOL_H
#define STIMJIMAWG_PROTOCOL_H

namespace Protocol {
void begin();   // prints the boot banner + identity block
void poll();    // drain Serial, assemble lines, dispatch complete ones
// IDN line plus `#` build/engine detail — shared by the banner and `IDN`.
void printIdentity();
}

namespace Commands {
// Dispatch one complete, NUL-terminated line (no terminator, not blank/comment).
void handleLine(const char* line);
// Drain the engine completion ring and print result summaries (loop context —
// the player ISRs never print; plan §3.3).
void poll();
}

#endif // STIMJIMAWG_PROTOCOL_H
