//    stimjimAWG — host-side tests for Cal: the parameter name lookup, the
//    validator (per-value floors and ceilings, the two cross-parameter
//    invariants) and the copy-apply-validate helper the `CAL` handler uses.
//    No Arduino dependencies — build & run on the development machine:
//
//      g++ -std=c++17 -Wall -Wextra -I stimjimAWG tests/host/test_cal.cpp
//          stimjimAWG/Cal.cpp -o test_cal                  (one command line)
//      ./test_cal               (exit code = number of failed checks)
//
//    GPL-3.0-or-later; see stimjimAWG/Config.h header.

#include "Cal.h"
#include <cstdio>
#include <cstring>
#include <initializer_list>

using namespace Cal;

static int failures = 0;

#define CHECK(cond) do { \
    if (!(cond)) { failures++; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

#define CHECK_EQ(a, b) do { \
    long long va = (long long)(a), vb = (long long)(b); \
    if (va != vb) { failures++; \
      printf("FAIL %s:%d  %s=%lld != %s=%lld\n", __FILE__, __LINE__, #a, va, #b, vb); } \
  } while (0)

// The Teensy 3.5 register-path defaults of Config.h, which is the set every
// measured figure in docs/serial-protocol.md §4 belongs to.
static void registerDefaults(Def& c) {
  c.us[PRELOAD]   = 4;
  c.us[DACPROG1]  = 3;
  c.us[DACPROG2]  = 5;
  c.us[ADCREAD]   = 3;
  c.us[ADCSWITCH] = 4;
  c.us[GUARD]     = 1;
  c.us[SETTLE]    = 4;
  c.us[STARTLAT]  = 20;
  c.us[TRIGCOMP]  = 0;
}

static void testNames() {
  CHECK_EQ(indexOf("PRELOAD"), PRELOAD);
  CHECK_EQ(indexOf("TRIGCOMP"), TRIGCOMP);
  CHECK_EQ(indexOf("STARTLAT"), STARTLAT);
  CHECK_EQ(indexOf("nonsense"), -1);
  CHECK_EQ(indexOf(""), -1);
  CHECK_EQ(indexOf("preload"), -1);          // the names print in upper case
  // Every id has a name and every name maps back to its id, so `CAL?` and the
  // EEPROM image cannot drift apart from the enum.
  for (uint8_t i = 0; i < N_ID; i++) {
    CHECK(NAME[i] != nullptr && NAME[i][0] != '\0');
    CHECK_EQ(indexOf(NAME[i]), i);
  }
}

static void testDefaultsValidate() {
  Def c;
  registerDefaults(c);
  CHECK(validate(c) == nullptr);
  // The portable backend's budgets are twice as generous, which is why its
  // start latency had to grow with them: 8 + 11 + 3 = 22 us does not fit 20.
  c.us[PRELOAD]  = 8;
  c.us[DACPROG1] = 6;
  c.us[DACPROG2] = 11;
  CHECK(validate(c) != nullptr);
  c.us[STARTLAT] = 40;
  CHECK(validate(c) == nullptr);
}

static void testBounds() {
  Def c;
  registerDefaults(c);

  // The three margins may legitimately be zero; the operation budgets may not.
  for (uint8_t id : {ADCSWITCH, GUARD, SETTLE, TRIGCOMP}) {
    Def d = c;
    d.us[id] = 0;
    CHECK(validate(d) == nullptr);
  }
  for (uint8_t id : {PRELOAD, DACPROG1, DACPROG2, ADCREAD, STARTLAT}) {
    Def d = c;
    d.us[id] = 0;
    CHECK(validate(d) != nullptr);
  }

  Def d = c;
  d.us[SETTLE] = SJ_CAL_MAX_US;
  CHECK(validate(d) == nullptr);
  d.us[SETTLE] = SJ_CAL_MAX_US + 1;
  CHECK(validate(d) != nullptr);

  // Programming both channels cannot cost less than programming one.
  d = c;
  d.us[DACPROG2] = d.us[DACPROG1];
  CHECK(validate(d) == nullptr);
  d.us[DACPROG2] = (uint16_t)(d.us[DACPROG1] - 1);
  CHECK(validate(d) != nullptr);
}

static void testStartLatencyInvariant() {
  Def c;
  registerDefaults(c);
  // Exactly PRELOAD + DACPROG2 + slack is the smallest start latency that
  // still leaves the first latch schedulable.
  c.us[STARTLAT] = (uint16_t)(c.us[PRELOAD] + c.us[DACPROG2] + SJ_CAL_MIN_SLACK_US);
  CHECK(validate(c) == nullptr);
  c.us[STARTLAT]--;
  CHECK(validate(c) != nullptr);

  // TRIGCOMP moves t0 earlier, so it spends the same slack.
  registerDefaults(c);
  c.us[TRIGCOMP] = (uint16_t)(c.us[STARTLAT] - c.us[PRELOAD] - c.us[DACPROG2] -
                              SJ_CAL_MIN_SLACK_US);
  CHECK(validate(c) == nullptr);
  c.us[TRIGCOMP]++;
  CHECK(validate(c) != nullptr);
}

static void testApply() {
  Def base, out;
  registerDefaults(base);

  CHECK(apply(base, SETTLE, 7, out) == nullptr);
  CHECK_EQ(out.us[SETTLE], 7);
  CHECK_EQ(base.us[SETTLE], 4);              // the base is left alone
  CHECK_EQ(out.us[PRELOAD], base.us[PRELOAD]);

  CHECK(apply(base, SETTLE, -1, out) != nullptr);
  CHECK(apply(base, SETTLE, SJ_CAL_MAX_US + 1, out) != nullptr);
  CHECK(apply(base, N_ID, 5, out) != nullptr);
  // A value inside its own bounds but breaking an invariant is still refused,
  // which is what keeps a one-parameter `CAL` line from arming a broken set.
  CHECK(apply(base, PRELOAD, 100, out) != nullptr);
  CHECK(apply(base, STARTLAT, 200, out) == nullptr);
}

int main() {
  testNames();
  testDefaultsValidate();
  testBounds();
  testStartLatencyInvariant();
  testApply();
  if (failures == 0) printf("all checks passed\n");
  else               printf("%d check(s) FAILED\n", failures);
  return failures;
}
