//    stimjimAWG — host-side tests for the OLED result formatting and the wall
//    clock's calendar rendering. No Arduino dependencies — build & run on the
//    development machine:
//
//      g++ -std=c++17 -Wall -Wextra -I stimjimAWG tests/host/test_uifmt.cpp
//          stimjimAWG/UiFmt.cpp stimjimAWG/Clock.cpp -o test_uifmt   (one line)
//      ./test_uifmt             (exit code = number of failed checks)
//
//    GPL-3.0-or-later; see stimjimAWG/Config.h header.

#include "UiFmt.h"
#include "Clock.h"
#include <cstdio>
#include <cstdint>
#include <cstring>

static int failures = 0;

#define CHECK(cond) do { \
    if (!(cond)) { failures++; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

#define CHECK_EQ(a, b) do { \
    long long va = (long long)(a), vb = (long long)(b); \
    if (va != vb) { failures++; \
      printf("FAIL %s:%d  %s=%lld != %s=%lld\n", __FILE__, __LINE__, #a, va, #b, vb); } \
  } while (0)

#define CHECK_STR(got, want) do { \
    if (strcmp((got), (want))) { failures++; \
      printf("FAIL %s:%d  \"%s\" != \"%s\"\n", __FILE__, __LINE__, (got), (want)); } \
  } while (0)

// The two readings of one channel at one point, with standard errors small
// enough that the 1 % floor decides the precision — the normal case for a train
// of a few hundred repetitions.
static void ohmAtFloor(char* out, size_t n, int32_t uV, int32_t nA) {
  UiFmt::fmtOhm(out, n, true, uV, 1, true, nA, 1);
}

static void testOhmDecades() {
  char b[SJ_UI_FIELD_MAX];
  // Four worked examples, all at the 1.4 % floored uncertainty: three
  // significant figures, the unit chosen so the mantissa lands in [1, 1000).
  ohmAtFloor(b, sizeof b,   471000,  1000000); CHECK_STR(b, "471");     // 471 ohm
  ohmAtFloor(b, sizeof b,  1010000,  1000000); CHECK_STR(b, "1.01k");
  ohmAtFloor(b, sizeof b,  4710000,   100000); CHECK_STR(b, "47.1k");
  ohmAtFloor(b, sizeof b, 10200000,    10000); CHECK_STR(b, "1.02M");
  // Unit boundaries: 999.5 ohm is where the ohm field would round to 1000.
  ohmAtFloor(b, sizeof b,   999000,  1000000); CHECK_STR(b, "999");
  ohmAtFloor(b, sizeof b,   999600,  1000000); CHECK_STR(b, "1.00k");
  // A negative reading keeps its sign: V and I disagreeing is worth seeing.
  ohmAtFloor(b, sizeof b,  -471000,  1000000); CHECK_STR(b, "-471");
}

static void testOhmPrecisionFollowsUncertainty() {
  char b[SJ_UI_FIELD_MAX];
  // A noisy train loses digits. 10 % on the current alone puts the window a
  // decade above the floor's, and the printed value coarsens with it.
  UiFmt::fmtOhm(b, sizeof b, true, 1010000, 1, true, 1000000, 100000);
  CHECK_STR(b, "1.0k");
  // The same 10 % on a value whose mantissa already spends two digits on its
  // integer part leaves room for no decimal at all.
  UiFmt::fmtOhm(b, sizeof b, true, 4710000, 1, true, 100000, 10000);
  CHECK_STR(b, "47k");
  // ... and a quiet one never gains more than three significant figures, however
  // small the standard errors are, because the floor stands in for the ADC
  // path's uncharacterised gain accuracy.
  UiFmt::fmtOhm(b, sizeof b, true, 1010000, 0, true, 1000000, 0);
  CHECK_STR(b, "1.01k");
}

static void testOhmSpecialCases() {
  char b[SJ_UI_FIELD_MAX];
  // A line that was never read.
  UiFmt::fmtOhm(b, sizeof b, false, 0, 0, true, 1000000, 1); CHECK_STR(b, "--");
  UiFmt::fmtOhm(b, sizeof b, true, 1000000, 1, false, 0, 0); CHECK_STR(b, "--");
  // No current path: |I| under three quantisation deviations (3 * 245 nA).
  ohmAtFloor(b, sizeof b, 5000000, 700);  CHECK_STR(b, "open");
  ohmAtFloor(b, sizeof b, 5000000, -700); CHECK_STR(b, "open");
  // Just outside it a value appears again, with the single significant figure a
  // current only three deviations from zero can support.
  ohmAtFloor(b, sizeof b, 5000000, 800);  CHECK_STR(b, "6M");
  // Neither V nor I differs from zero: an undriven or grounded channel.
  ohmAtFloor(b, sizeof b, 1000, 500);     CHECK_STR(b, "--");
  // A voltage that does not differ from zero while a real current flows is a
  // short, not a missing reading — so it renders as the small number it is.
  ohmAtFloor(b, sizeof b, 100, 1000000);  CHECK_STR(b, "0.1");
  // A large standard error raises the zero threshold, so a reading the floor
  // would have accepted becomes indistinguishable from zero.
  UiFmt::fmtOhm(b, sizeof b, true, 5000000, 1, true, 3000, 2000); CHECK_STR(b, "open");
}

static void testHeadroom() {
  // Full scale of both converters, and the smallest current the module still
  // divides by (three quantisation deviations). The intermediate is
  // uV * 1e6 = 1.5e13, three orders below the int64 ceiling.
  CHECK_EQ(UiFmt::ohmMilli(15000000, 3333000), 4500450);
  CHECK_EQ(UiFmt::ohmMilli(-15000000, 3333000), -4500450);
  CHECK_EQ(UiFmt::ohmMilli(15000000, 735), 20408163265LL);
  char b[SJ_UI_FIELD_MAX];
  // 20 M is the largest resistance this rule will print: a current three
  // deviations from zero is 33 % uncertain, hence the single figure.
  ohmAtFloor(b, sizeof b, 15000000, 735);     CHECK_STR(b, "20M");
  ohmAtFloor(b, sizeof b, 15000000, 3333000); CHECK_STR(b, "4.50k");
}

static void testIsqrtAndRel() {
  CHECK_EQ(UiFmt::isqrt64(0), 0);
  CHECK_EQ(UiFmt::isqrt64(1), 1);
  CHECK_EQ(UiFmt::isqrt64(2), 1);
  CHECK_EQ(UiFmt::isqrt64(999999ull * 999999ull), 999999u);
  CHECK_EQ(UiFmt::isqrt64(2ull * 10000ull * 10000ull), 14142u);   // the 1.4 % floor
  // The floor applies from below and the 100 % cap from above; a zero mean
  // carries no digits at all.
  CHECK_EQ(UiFmt::relPpm(1000000, 1), SJ_UI_REL_FLOOR_PPM);
  CHECK_EQ(UiFmt::relPpm(1000000, 200000), 200000u);
  CHECK_EQ(UiFmt::relPpm(1000, 5000000), 1000000u);
  CHECK_EQ(UiFmt::relPpm(0, 1), 1000000u);
}

static void testVoltCurrFields() {
  char b[SJ_UI_FIELD_MAX];
  UiFmt::fmtVolt(b, sizeof b, true,   100200); CHECK_STR(b, "100.2");
  UiFmt::fmtVolt(b, sizeof b, true,   -99800); CHECK_STR(b, "-99.8");
  UiFmt::fmtVolt(b, sizeof b, true,        0); CHECK_STR(b, "0.0");
  UiFmt::fmtVolt(b, sizeof b, false,  100200); CHECK_STR(b, "--");
  // Above 1000 of the printed unit the decimal is dropped, which is what keeps
  // full scale inside the panel's seven-character field.
  UiFmt::fmtVolt(b, sizeof b, true,  15000000); CHECK_STR(b, "15000");
  CHECK(strlen(b) <= 7);
  UiFmt::fmtVolt(b, sizeof b, true, -15000000); CHECK_STR(b, "-15000");
  CHECK(strlen(b) <= 7);
  UiFmt::fmtCurr(b, sizeof b, true,     99500); CHECK_STR(b, "99.5");
  UiFmt::fmtCurr(b, sizeof b, true,   -100000); CHECK_STR(b, "-100.0");
  UiFmt::fmtCurr(b, sizeof b, true,  -3333000); CHECK_STR(b, "-3333");
  CHECK(strlen(b) <= 7);
}

static void testLimitFlags() {
  // A reading at the driver IC's ceiling or at the current pump's design limit
  // is marked, whichever sign it has, and nothing below it is.
  CHECK_EQ(UiFmt::voltFlag(8999999), ' ');
  CHECK_EQ(UiFmt::voltFlag(9000000), '*');
  CHECK_EQ(UiFmt::voltFlag(-9000000), '*');
  CHECK_EQ(UiFmt::voltFlag(14988000), '*');
  CHECK_EQ(UiFmt::currFlag(2999999), ' ');
  CHECK_EQ(UiFmt::currFlag(3000000), '*');
  CHECK_EQ(UiFmt::currFlag(-3333000), '*');
}

static void testIso() {
  char b[SJ_ISO_MAX];
  Clock::isoString(b, sizeof b, 0, 0);             CHECK_STR(b, "1970-01-01T00:00:00.000Z");
  Clock::isoString(b, sizeof b, 1546300800u, 0);   CHECK_STR(b, "2019-01-01T00:00:00.000Z");
  Clock::isoString(b, sizeof b, 1767225599u, 999); CHECK_STR(b, "2025-12-31T23:59:59.999Z");
  Clock::isoString(b, sizeof b, 1709164800u, 0);   CHECK_STR(b, "2024-02-29T00:00:00.000Z");
  Clock::isoString(b, sizeof b, 1709251200u, 0);   CHECK_STR(b, "2024-03-01T00:00:00.000Z");
  Clock::isoString(b, sizeof b, 1740787200u, 0);   CHECK_STR(b, "2025-03-01T00:00:00.000Z");
  Clock::isoString(b, sizeof b, 4294967295u, 0);   CHECK_STR(b, "2106-02-07T06:28:15.000Z");
  CHECK(strlen(b) == SJ_ISO_MAX - 1);

  // The broken-down fields the FAT directory timestamp is built from.
  Clock::Civil c;
  Clock::civilFromUnix(1757255525u, 123, c);
  CHECK_EQ(c.year, 2025); CHECK_EQ(c.mon, 9);  CHECK_EQ(c.day, 7);
  CHECK_EQ(c.hour, 14);   CHECK_EQ(c.min, 32); CHECK_EQ(c.sec, 5);
  CHECK_EQ(c.ms, 123);

  CHECK_STR(Clock::sourceName(Clock::SRC_BUILD), "build");
  CHECK_STR(Clock::sourceName(Clock::SRC_BATT),  "batt");
  CHECK_STR(Clock::sourceName(Clock::SRC_HOST),  "host");
}

int main() {
  testOhmDecades();
  testOhmPrecisionFollowsUncertainty();
  testOhmSpecialCases();
  testHeadroom();
  testIsqrtAndRel();
  testVoltCurrFields();
  testLimitFlags();
  testIso();
  if (failures) printf("test_uifmt: %d check(s) FAILED\n", failures);
  else          printf("test_uifmt: all checks passed\n");
  return failures;
}
