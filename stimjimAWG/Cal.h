//    stimjimAWG — Cal: the hardware timing budget as runtime state (protocol §4).
//    GPL-3.0-or-later; see Config.h header.
//
//    Every number the scheduler and the measurement engine budget for a
//    hardware operation lives here instead of in a `#define`: how early the
//    player ISR wakes before a latch, what a DAC program and an ADC read cost,
//    how long the output needs to settle before a reading means anything, the
//    fixed arm->first-latch latency, and the pin->ISR-entry delay of the
//    trigger path. `Config.h` still supplies the defaults — this module makes
//    them adjustable over the serial port (`CAL`) and persists them with `P`.
//
//    Why runtime and not compile time: two of the budgets are still desk
//    estimates (the AD5752 settling time and the whole portable-backend
//    column), the measurement window is decided by their *sum* to within a
//    microsecond, and re-measuring one on the bench must not cost a rebuild
//    and a reflash. A wrong value cannot corrupt a waveform silently: the
//    engine's per-latch deadline counters report a budget that is too small,
//    and calValidate() refuses a set that would make the start latency
//    unreachable.
//
//    The values are *budgets*, so they carry the measured worst case, not the
//    average (docs/serial-protocol.md §4).
//
//    calIndexOf/calApply/calValidate carry no Arduino dependency and are
//    host-tested by tests/host/test_cal.cpp.

#ifndef STIMJIMAWG_CAL_H
#define STIMJIMAWG_CAL_H

#include <stdint.h>

namespace Cal {

// Parameter identity. The order is the order `CAL?` prints and the order the
// EEPROM image stores, so new parameters go on the end (and bump the image
// version) — never in the middle.
enum Id : uint8_t {
  PRELOAD = 0,   // player ISR wakes this early, then spins on CYCCNT
  DACPROG1,      // dacProgram, one channel
  DACPROG2,      // dacProgramBoth
  ADCREAD,       // adcRead with the line already selected
  ADCSWITCH,     // extra cost of a control-register line switch
  GUARD,         // margin between the last read and the next preload window
  SETTLE,        // after a latch, before a reading means anything (BENCHSETTLE)
  STARTLAT,      // fixed arm -> first-latch latency
  TRIGCOMP,      // pin edge -> trigger-ISR entry (0 = uncompensated)
  N_ID
};

// All values in microseconds. Whole microseconds only: the engine converts
// them to cycles with an exact integer multiply, and a budget that reads short
// is worse than one that is a fraction of a microsecond too generous.
struct Def {
  uint16_t us[N_ID];
};

// Parameter names as `CAL` spells them, indexed by Id.
extern const char* const NAME[N_ID];

// Ceiling for every parameter. 1000 us is far past anything a Cortex-M SPI
// path can cost and still leaves the products inside 32-bit cycle counts.
#define SJ_CAL_MAX_US 1000
// Slack the start latency must keep beyond preload + a dual-channel program,
// so the first latch is still schedulable. Must equal SJ_MIN_SCHEDULE_US;
// Cal.cpp static_asserts that against Config.h.
#define SJ_CAL_MIN_SLACK_US 3

// Index of `name` (case-sensitive, as printed), or -1 when unknown.
int8_t indexOf(const char* name);

// NULL when the set is consistent, else a static reason. Checks the per-value
// bounds and the two cross-parameter invariants: programming both channels
// cannot cost less than programming one, and the start latency must cover a
// preload plus a dual-channel program plus SJ_CAL_MIN_SLACK_US after TRIGCOMP
// has been subtracted from it.
const char* validate(const Def& c);

// Copy `base` into `out` with parameter `id` set to `value`, then validate.
// Returns NULL on success (out holds the new set) or the reason on failure
// (out is undefined). Rejects a value outside [0, SJ_CAL_MAX_US] before the
// narrowing cast, so a caller may pass an unchecked parsed long.
const char* apply(const Def& base, uint8_t id, long value, Def& out);

// ------------------------------------------------------------- device side
#ifdef ARDUINO

void begin();                  // load the Config.h defaults
const Def& live();             // the values the engine arms with
const Def& defaults();         // the compiled-in defaults (DUMP compares against these)
void       set(const Def& c);  // replace wholesale (EEPROM restore); caller validates first
bool       isDefault();        // true when nothing has been changed from the build
// Bumped by every set(). Anything that caches a value derived from the budget
// stores this alongside it and recomputes when it no longer matches — the
// measurement plan does (Measure::armPlan).
uint32_t   epoch();

#endif // ARDUINO

} // namespace Cal

#endif // STIMJIMAWG_CAL_H
