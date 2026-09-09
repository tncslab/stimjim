//    stimjimAWG — SdLog: timestamped measurement logging to the onboard micro-SD
//    (native SDIO — never the DAC/ADC SPI bus) and serial access to the card.
//    Only loop() ever touches the card (plan §3.7, protocol §4).
//    GPL-3.0-or-later; see Config.h header.
//
//    The socket sits under the instrument cover, so the card cannot be pulled to
//    read a log: everything written here is readable back over the serial port
//    through the SD command group. SDGET frames its payload with a byte count so
//    a host never has to decide whether a line is data or protocol.
//
//    Present on Teensy 3.5/3.6 and 4.1. A Teensy 4.0 has no socket: SJ_USE_SD is
//    0 there and every entry point reports that instead of failing to link.

#ifndef STIMJIMAWG_SDLOG_H
#define STIMJIMAWG_SDLOG_H

#include <stdint.h>
#include <stddef.h>

namespace SdLog {

void begin();
void poll();   // flush timer; the MDATA ring is drained by Measure::poll()

// One CSV row, already formatted by Measure (protocol §4):
// <timestamp_us>,<slot>,<pulse>,<point>,<V0_mV>,<I0_uA>,<V1_mV>,<I1_uA>
void writeRow(const char* row);
void flushNow();          // called at train end
bool isOpen();
bool cardPresent();       // a card is mounted (false on a build with no socket)
// The open log's name ("" when none) and how many bytes have been written to
// it — what the OLED status page reports, and what `LOG?` prints.
const char* name();
uint32_t    bytes();

// One `#` block naming the waveform a just-armed train plays, so a log stays
// self-describing when trains are started by trigger edges long after the file
// was opened. No-op while no file is open.
void noteTrain(uint8_t slot);

// One `# <tag>: us=<n> <text>` session-event line, stamped with the same
// microsecond timebase the rows carry. The tags in use are `set` (a
// configuration change, `text` being the canonical round-trip line the setter
// echoed over serial), `done` (a train's own account of itself) and `stop` (a
// train ended by hand). Between them these are what make a file describe what
// happened *after* it was opened, rather than only the DUMP block from when it
// was. Gated on a file being open and nothing else: `report` selects
// measurement destinations, a session event is not a measurement. No-op while
// no file is open.
void noteEvent(const char* tag, const char* text);

// Open the next free LOGnnnn.CSV without being asked, because something is
// configured to write to a log. Differs from openLog() in the two ways that
// matter on a board with an empty socket: a missing card is not an error and
// prints nothing, and the SDIO bus is never re-probed (begin() mounted once at
// boot; LOG1 and SDINFO still retry). Whether to call it at all -- and whether
// an explicit LOG0 has latched the automatic path off -- is Commands' decision,
// so that opening BENCHSD's scratch file cannot disturb it.
//
// Returns whether there was a card to try, so the caller can spend its one
// automatic attempt on a real attempt: with an empty socket nothing happened
// and nothing should be counted against the box.
bool autoOpen();

// ----------------------------------------------------------- `LOG` backend
// Each prints exactly one status line: LOG,<open>,<name>,<bytes>.
void status();
void openLog(const char* name);   // name == NULL: next free LOGnnnn.CSV
void closeLog();

// ------------------------------------------------------- `SD` group backend
// withUsed: also scan the FAT for the used-space field (slow, see SdLog.cpp).
void info(bool withUsed);
void list(const char* dir);                                  // dir == NULL: root
void get(const char* name, uint64_t offset, uint64_t len);    // len == 0: to end of file
void del(const char* name);

} // namespace SdLog

#endif // STIMJIMAWG_SDLOG_H
