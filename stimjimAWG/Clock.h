//    stimjimAWG — Clock: the wall clock, and where its epoch came from.
//    GPL-3.0-or-later; see Config.h header.
//
//    This module is deliberately the *coarse* clock. Every waveform instant,
//    every measurement deadline and the log's timestamp_us column come from
//    FastIO::cycles64() and nothing here touches them. What this provides is a
//    label: enough to tie a log file to a computer's log, and no more.
//
//    Three facts decide the design:
//
//      - The RTC counts 32768 Hz, so about 30.5 us of readable resolution, and
//        its crystal is uncompensated (tens of ppm, i.e. seconds per day).
//      - Its *epoch* cannot be trusted unless a host has set it. A Teensy 3.5
//        with no battery starts the clock at the binary's compile time in the
//        *build host's local zone* (the core's `__rtc_localtime` defsym); a
//        Teensy 4.x starts it at 2019-01-01T00:00:00Z. Either is right just
//        after an upload and wrong by however long the binary has been in
//        service.
//      - So the firmware reports the source on every clock line and never
//        presents the reading as accurate on its own. A host that wants an
//        accurate anchor sends `CLK,<unix>[,<ms>]`, or reads `CLK?` and keeps
//        the offset against the us column itself.
//
//    isoString/sourceName carry no Arduino dependency and are host-tested by
//    tests/host/test_uifmt.cpp.

#ifndef STIMJIMAWG_CLOCK_H
#define STIMJIMAWG_CLOCK_H

#include <stdint.h>
#include <stddef.h>

namespace Clock {

enum Source : uint8_t {
  SRC_BUILD = 0,   // the core set it from the binary's compile time (local zone!)
  SRC_BATT  = 1,   // running from VBAT since some earlier upload — epoch unverified
  SRC_HOST  = 2,   // a host sent `CLK` this session: UTC, good to about a ms
};

// ------------------------------------------------------------ pure section

// Broken-down UTC. The FAT directory timestamp needs the same fields the ISO
// rendering does, so the conversion lives here once.
struct Civil {
  uint16_t year;                       // full year, e.g. 2026
  uint8_t  mon, day, hour, min, sec;   // mon 1-12, day 1-31
  uint16_t ms;
};
void civilFromUnix(uint32_t unixSec, uint16_t subMs, Civil& out);

// "YYYY-MM-DDThh:mm:ss.mmmZ" — 24 characters plus the terminator. Always UTC:
// nothing in the firmware knows a zone, and a SRC_BUILD reading is local time
// mislabelled as UTC, which is exactly why the source travels with it.
#define SJ_ISO_MAX 25
void isoString(char* out, size_t n, uint32_t unixSec, uint16_t subMs);

// "build" / "batt" / "host" — the token the protocol and the log use.
const char* sourceName(Source s);

// ----------------------------------------------------------- device section
#ifdef ARDUINO

void   begin();          // classify the source; prints nothing
Source source();

// Consistent read of seconds and the sub-second fraction. The core's rtc_get()
// returns seconds only and reading the two registers separately can straddle a
// second boundary, so this pairs them and retries. Returns false when the RTC
// is not running at all (no epoch to report).
bool read(uint32_t& unixSec, uint16_t& subMs);

// Set the clock, fraction included (the core's rtc_set() zeroes it), and move
// the source to SRC_HOST.
void set(uint32_t unixSec, uint16_t subMs);

// Both clocks, read back to back: the wall clock and the microseconds since
// boot that every log row is stamped with. The pair is the anchor — a host
// subtracts its own clock from it and can bound the error by timing the round
// trip. Returns what read() returned.
bool anchor(uint32_t& unixSec, uint16_t& subMs, uint64_t& us);

// "2026-09-07T14:32:05.123Z src=host us=41234567", the one line that appears in
// the boot banner, in `CLK?` and in the log. Needs ~64 bytes.
void anchorLine(char* out, size_t n);

#endif // ARDUINO

} // namespace Clock

#endif // STIMJIMAWG_CLOCK_H
