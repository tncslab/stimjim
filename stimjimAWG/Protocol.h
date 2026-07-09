//    stimjimAWG — Protocol: bounded line assembler + dispatch (plan §4, protocol §1).
//    GPL-3.0-or-later; see Config.h header.

#ifndef STIMJIMAWG_PROTOCOL_H
#define STIMJIMAWG_PROTOCOL_H

namespace Protocol {
void begin();   // prints the boot banner + IDN line
void poll();    // drain Serial, assemble lines, dispatch complete ones
}

namespace Commands {
// Dispatch one complete, NUL-terminated line (no terminator, not blank/comment).
void handleLine(const char* line);
}

#endif // STIMJIMAWG_PROTOCOL_H
