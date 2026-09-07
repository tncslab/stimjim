//    stimjimAWG — Clock implementation. GPL-3.0-or-later; see Config.h header.

#include "Clock.h"
#include <stdio.h>

namespace Clock {

// --------------------------------------------------------------- pure section

const char* sourceName(Source s) {
  switch (s) {
    case SRC_HOST: return "host";
    case SRC_BATT: return "batt";
    default:       return "build";
  }
}

void civilFromUnix(uint32_t unixSec, uint16_t subMs, Civil& out) {
  const uint32_t secOfDay = unixSec % 86400u;
  out.hour = (uint8_t)(secOfDay / 3600u);
  out.min  = (uint8_t)((secOfDay / 60u) % 60u);
  out.sec  = (uint8_t)(secOfDay % 60u);
  out.ms   = subMs;
  // Days since the Unix epoch, shifted to an era starting on 0000-03-01 so the
  // leap day falls at the end of a 400-year era. The whole conversion is then
  // integer division with no month table and no leap-year branch (Howard
  // Hinnant's civil_from_days, public domain).
  uint32_t z   = unixSec / 86400u + 719468u;
  uint32_t era = z / 146097u;
  uint32_t doe = z - era * 146097u;                                         // [0, 146096]
  uint32_t yoe = (doe - doe / 1460u + doe / 36524u - doe / 146096u) / 365u; // [0, 399]
  uint32_t y   = yoe + era * 400u;
  uint32_t doy = doe - (365u * yoe + yoe / 4u - yoe / 100u);                // [0, 365]
  uint32_t mp  = (5u * doy + 2u) / 153u;                                    // [0, 11], March = 0
  out.day      = (uint8_t)(doy - (153u * mp + 2u) / 5u + 1u);               // [1, 31]
  out.mon      = (uint8_t)(mp + (mp < 10u ? 3u : (uint32_t)-9));            // [1, 12]
  if (out.mon <= 2u) y++;      // January and February belong to the following year
  out.year     = (uint16_t)y;
}

void isoString(char* out, size_t n, uint32_t unixSec, uint16_t subMs) {
  Civil c;
  civilFromUnix(unixSec, subMs, c);
  snprintf(out, n, "%04u-%02u-%02uT%02u:%02u:%02u.%03uZ",
           (unsigned)c.year, (unsigned)c.mon, (unsigned)c.day,
           (unsigned)c.hour, (unsigned)c.min, (unsigned)c.sec, (unsigned)c.ms);
}

// ------------------------------------------------------------- device section
#ifdef ARDUINO

} // namespace Clock

#include "Config.h"
#include "FastIO.h"
#include "Protocol.h"   // u64str: microseconds since boot outgrow 32 bits

namespace Clock {

static Source src = SRC_BUILD;

Source source() { return src; }

#if SJ_MCU_KINETISK

// The VBAT register file word the core writes 0x5A94C3A5 into when it had to
// set the RTC from a stale compile time (cores/teensy3/mk20dx128.c:1128-1155).
// It clears the flag again on the reset that follows an upload, which is the
// one moment the compile time is fresh.
#define SJ_RTC_STALE_FLAG (*(volatile uint32_t *)0x4003E01C)
#define SJ_RTC_STALE_MAGIC 0x5A94C3A5u

// The linker defsym the IDE fills in from {extra.time.local}: the *address* of
// this symbol is the build host's local time at compile, as a Unix count.
extern "C" void* __rtc_localtime;
static inline uint32_t buildTime() { return (uint32_t)(uintptr_t)&__rtc_localtime; }

bool read(uint32_t& unixSec, uint16_t& subMs) {
  if (!(RTC_SR & RTC_SR_TCE)) return false;      // counter disabled: no epoch at all
  // TSR increments when TPR[14:0] overflows, so TPR[14:0] is the sub-second in
  // 1/32768ths and TPR bit 15 plays no part in the second boundary (K64 RM,
  // RTC chapter). Reading the two registers separately can straddle a second,
  // so bracket the seconds read with the prescaler and retry when it rolled.
  for (uint8_t tries = 0; tries < 4; tries++) {
    uint32_t p1 = RTC_TPR & 0x7FFFu;
    uint32_t s  = RTC_TSR;
    uint32_t p2 = RTC_TPR & 0x7FFFu;
    if (p2 >= p1) {
      unixSec = s;
      subMs   = (uint16_t)((p2 * 1000u) >> 15);
      return true;
    }
  }
  unixSec = RTC_TSR;
  subMs   = 0;
  return true;
}

void set(uint32_t unixSec, uint16_t subMs) {
  // The core's rtc_set() zeroes TPR, which throws the fraction away; writing
  // both is what makes a host's millisecond survive the transfer.
  RTC_SR  = 0;                                    // TCE off: TSR/TPR are writable
  RTC_TPR = (uint32_t)(((uint32_t)subMs * 32768u) / 1000u);
  RTC_TSR = unixSec;
  RTC_SR  = RTC_SR_TCE;
  src     = SRC_HOST;
}

void begin() {
  uint32_t s;
  uint16_t ms;
  if (!read(s, ms)) { src = SRC_BUILD; return; }
  if (SJ_RTC_STALE_FLAG == SJ_RTC_STALE_MAGIC) {
    src = SRC_BUILD;                              // core set it from a stale compile time
    return;
  }
  // The flag is clear, which means either the reset right after an upload (the
  // core set the clock from a *fresh* compile time and cleared the flag) or a
  // battery-backed clock set at some earlier upload. Only the distance from the
  // compile time separates them, so this is a heuristic and nothing more: a
  // reading still within a minute of it is a fresh upload.
  const uint32_t bt = buildTime();
  const uint32_t d  = (s > bt) ? (s - bt) : (bt - s);
  src = (d <= 60u) ? SRC_BUILD : SRC_BATT;
}

#else   // i.MX RT1062 (Teensy 4.x) and any other target

// The i.MX core starts the SRTC at 2019-01-01T00:00:00Z when it finds it
// stopped (cores/teensy4/startup.c:178-182) and never uses the compile time, so
// there is no stale flag to read: a reading still inside that first day is the
// core's default and anything later came from somewhere else.
#define SJ_IMX_DEFAULT_EPOCH 1546300800u   // 2019-01-01T00:00:00Z

bool read(uint32_t& unixSec, uint16_t& subMs) {
#if defined(__IMXRT1062__)
  // The same paired read as the core's rtc_get(), keeping the 15 fractional
  // bits it discards (ref manual 20.3.3.1.3).
  uint32_t hi1 = SNVS_HPRTCMR, lo1 = SNVS_HPRTCLR;
  for (uint8_t tries = 0; tries < 4; tries++) {
    uint32_t hi2 = SNVS_HPRTCMR, lo2 = SNVS_HPRTCLR;
    if (lo1 == lo2 && hi1 == hi2) {
      unixSec = (hi2 << 17) | (lo2 >> 15);
      subMs   = (uint16_t)(((lo2 & 0x7FFFu) * 1000u) >> 15);
      return true;
    }
    hi1 = hi2;
    lo1 = lo2;
  }
  // Four straddled reads in a row means the low word changed under every one of
  // them, which cannot happen at 32768 Hz; report the last pair rather than
  // spinning in what is only a label.
  unixSec = (hi1 << 17) | (lo1 >> 15);
  subMs   = 0;
  return true;
#else
  (void)unixSec; (void)subMs;
  return false;                                   // no RTC on this target
#endif
}

void set(uint32_t unixSec, uint16_t subMs) {
#if defined(__IMXRT1062__)
  // Same enable/disable dance as the core's rtc_set(), but writing the 15
  // fractional bits instead of zeroing them.
  const uint32_t frac = (uint32_t)(((uint32_t)subMs * 32768u) / 1000u) & 0x7FFFu;
  SNVS_HPCR &= ~(SNVS_HPCR_RTC_EN | SNVS_HPCR_HP_TS);
  while (SNVS_HPCR & SNVS_HPCR_RTC_EN) ;
  SNVS_LPCR &= ~SNVS_LPCR_SRTC_ENV;
  while (SNVS_LPCR & SNVS_LPCR_SRTC_ENV) ;
  SNVS_LPSRTCLR = (unixSec << 15) | frac;
  SNVS_LPSRTCMR = unixSec >> 17;
  SNVS_LPCR |= SNVS_LPCR_SRTC_ENV;
  while (!(SNVS_LPCR & SNVS_LPCR_SRTC_ENV)) ;
  SNVS_HPCR |= SNVS_HPCR_RTC_EN | SNVS_HPCR_HP_TS;
#else
  (void)unixSec; (void)subMs;
#endif
  src = SRC_HOST;
}

void begin() {
  uint32_t s;
  uint16_t ms;
  if (!read(s, ms)) { src = SRC_BUILD; return; }
  src = (s < SJ_IMX_DEFAULT_EPOCH + 86400u) ? SRC_BUILD : SRC_BATT;
}

#endif  // SJ_MCU_KINETISK

bool anchor(uint32_t& unixSec, uint16_t& subMs, uint64_t& us) {
  // Back to back and in this order: the wall clock first, then the timebase, so
  // the us value a host pairs with the reading is never the older of the two.
  const bool ok = read(unixSec, subMs);
  us = SJ_CYC_TO_US(FastIO::cycles64());
  return ok;
}

void anchorLine(char* out, size_t n) {
  uint32_t s;
  uint16_t ms;
  uint64_t us;
  const bool ok = anchor(s, ms, us);
  char iso[SJ_ISO_MAX], usStr[24];
  if (ok) isoString(iso, sizeof iso, s, ms);
  else    snprintf(iso, sizeof iso, "no-rtc");
  Protocol::u64str(us, usStr);
  snprintf(out, n, "%s src=%s us=%s", iso, sourceName(src), usStr);
}

#endif // ARDUINO

} // namespace Clock
