//    stimjimAWG — Cal implementation: the name table, the validator (both pure
//    and host-tested) and the live/default value pair.
//    GPL-3.0-or-later; see Config.h header.

#include "Cal.h"
#include <string.h>

namespace Cal {

// --------------------------------------------------------------- pure section

const char* const NAME[N_ID] = {
  "PRELOAD", "DACPROG1", "DACPROG2", "ADCREAD", "ADCSWITCH",
  "GUARD", "SETTLE", "STARTLAT", "TRIGCOMP",
};

// Per-parameter floor. Zero is meaningful for the three margins (a build may
// legitimately decide the AD5752 needs no extra settling, or that the line
// switch is free), but a preload of 0 would remove the spin the whole jitter
// design rests on, and a 0 budget for an operation that provably takes
// microseconds is never a calibration, only a mistake.
static const uint16_t MIN_US[N_ID] = {
  1,  // PRELOAD
  1,  // DACPROG1
  1,  // DACPROG2
  1,  // ADCREAD
  0,  // ADCSWITCH
  0,  // GUARD
  0,  // SETTLE
  1,  // STARTLAT
  0,  // TRIGCOMP
};

uint16_t minLatchUs(const Def& c, bool bothChannels) {
  return (uint16_t)(c.us[PRELOAD] + (bothChannels ? c.us[DACPROG2] : c.us[DACPROG1]) +
                    SJ_CAL_MIN_SLACK_US);
}

int8_t indexOf(const char* name) {
  for (uint8_t i = 0; i < N_ID; i++)
    if (!strcmp(name, NAME[i])) return (int8_t)i;
  return -1;
}

const char* validate(const Def& c) {
  for (uint8_t i = 0; i < N_ID; i++) {
    if (c.us[i] > SJ_CAL_MAX_US) return "value exceeds 1000 us";
    if (c.us[i] < MIN_US[i])     return "value below the parameter's floor (see HELP)";
  }
  if (c.us[DACPROG2] < c.us[DACPROG1])
    return "DACPROG2 cannot be smaller than DACPROG1";
  // The first latch is programmed one preload window before t0, and t0 is
  // START_LATENCY after the arm; TRIGCOMP moves t0 earlier still. What is left
  // has to cover the programming itself plus the scheduler's own minimum.
  const uint32_t need = (uint32_t)c.us[PRELOAD] + c.us[DACPROG2] + SJ_CAL_MIN_SLACK_US;
  if ((uint32_t)c.us[STARTLAT] < need + c.us[TRIGCOMP])
    return "STARTLAT must cover PRELOAD + DACPROG2 + 3 us beyond TRIGCOMP";
  return nullptr;
}

const char* apply(const Def& base, uint8_t id, long value, Def& out) {
  if (id >= N_ID)                          return "unknown parameter";
  if (value < 0)                           return "value must be >= 0";
  if (value > SJ_CAL_MAX_US)               return "value exceeds 1000 us";
  out = base;
  out.us[id] = (uint16_t)value;
  return validate(out);
}

// ------------------------------------------------------------- device section
#ifdef ARDUINO

} // namespace Cal

#include "Config.h"
#include "FastIO.h"

namespace Cal {

static_assert(SJ_CAL_MIN_SLACK_US == SJ_MIN_SCHEDULE_US,
              "SJ_CAL_MIN_SLACK_US must track SJ_MIN_SCHEDULE_US");

static Def liveDef, buildDef;

void begin() {
  buildDef.us[PRELOAD]   = SJ_PRELOAD_US;
  buildDef.us[DACPROG1]  = SJ_DAC_PROG1_US;
  buildDef.us[DACPROG2]  = SJ_DAC_PROG2_US;
  buildDef.us[ADCREAD]   = SJ_ADC_READ_US;
  buildDef.us[ADCSWITCH] = SJ_ADC_SWITCH_US;
  buildDef.us[GUARD]     = SJ_MEAS_GUARD_US;
  buildDef.us[SETTLE]    = SJ_DAC_SETTLE_US;
  buildDef.us[STARTLAT]  = SJ_START_LATENCY_US;
  buildDef.us[TRIGCOMP]  = 0;      // unmeasured: no compensation until someone measures it
  liveDef = buildDef;
}

const Def& live()     { return liveDef; }
const Def& defaults() { return buildDef; }
bool       isDefault() { return memcmp(&liveDef, &buildDef, sizeof liveDef) == 0; }

static uint32_t liveEpoch = 1;   // 0 is reserved for "nothing cached yet"

uint32_t epoch() { return liveEpoch; }

void set(const Def& c) {
  // A trigger edge can arm a train between two words of this assignment, and
  // Engine::startTrain would then copy half of one budget and half of another.
  // The bus lock masks the trigger and player ISRs for the few cycles it takes
  // — the same reason Triggers::setRoute takes it. The epoch is bumped inside
  // the same lock so no arm can read a new budget with an old epoch and keep a
  // plan compiled against the old one.
  FastIO::busLock();
  liveDef = c;
  liveEpoch++;
  FastIO::busUnlock();
}

#endif // ARDUINO

} // namespace Cal
