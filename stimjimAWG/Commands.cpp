//    stimjimAWG — command handlers: S/L/W definition with atomic staging, `?`
//    queries + round-trip serializers, DELAY/DT/ENV/MEAS, the CAL timing budget,
//    T/U start-stop with the
//    completion-ring drain, TRIG/R routing, byte-exact M/V/A/E (BIST contract),
//    READ, B/C/D/P, STAT, SCREEN, DUMP, LOG, the SD file group and the BENCH
//    group. GPL-3.0-or-later.

#include "Protocol.h"
#include "Config.h"
#include "FastIO.h"
#include "Engine.h"
#include "TrainStore.h"
#include "Triggers.h"
#include "UiMenu.h"
#include "Measure.h"
#include "SdLog.h"
#include "Cal.h"
#include "Clock.h"
#include <ctype.h>
#include <string.h>
#include <stdlib.h>

namespace Commands {

// ------------------------------------------------------------------ helpers

// Parse up to `maxN` comma-separated integers after the command word.
// Accepts an optional leading comma before each value. Returns count parsed.
// (Lenient variant used by BENCH; the protocol handlers use parseFields.)
static uint8_t parseLongs(const char* p, long* out, uint8_t maxN) {
  uint8_t n = 0;
  while (n < maxN) {
    while (*p == ',' || *p == ' ') p++;
    if (!*p) break;
    char* end;
    long v = strtol(p, &end, 10);
    if (end == p) break;
    out[n++] = v;
    p = end;
  }
  return n;
}

// Strict comma-separated integer list: returns the field count (0..maxN),
// or -1 on malformed input, trailing garbage, or more than maxN fields —
// the fail-loudly counterpart of the legacy sscanf handlers.
static int8_t parseFields(const char* p, long* out, int8_t maxN) {
  int8_t n = 0;
  while (*p == ' ') p++;
  if (!*p) return 0;
  for (;;) {
    char* end;
    long v = strtol(p, &end, 10);
    if (end == p || n == maxN) return -1;
    out[n++] = v;
    p = end;
    while (*p == ' ') p++;
    if (!*p) return n;
    if (*p != ',') return -1;
    p++;
    while (*p == ' ') p++;
  }
}

// The separating comma of a long-form command's argument list. parseFields is
// deliberately strict about it and every call site that sits behind one skips
// it; this is that skip, written so an empty argument list stays empty rather
// than walking off the end.
static inline const char* skipComma(const char* p) { return (*p == ',') ? p + 1 : p; }

// Clock-independent: 1000 ns / cycles-per-us. 8.33 ns at 120 MHz (Teensy 3.5),
// 1.67 ns at 600 MHz (Teensy 4.x). uint64 intermediate so long BENCH intervals
// cannot overflow the multiply.
static inline uint32_t cycToNs(uint32_t cyc) {
  return (uint32_t)(((uint64_t)cyc * 1000u + SJ_CYC_PER_US / 2) / SJ_CYC_PER_US);
}

static void printStat(const char* name, uint32_t n, uint32_t mn, uint64_t sum, uint32_t mx) {
  uint32_t avg = (uint32_t)(sum / n);
  Serial.printf("BENCH,%s,n=%lu,cycles(min/avg/max)=%lu/%lu/%lu,ns=%lu/%lu/%lu\n",
                name, (unsigned long)n,
                (unsigned long)mn, (unsigned long)avg, (unsigned long)mx,
                (unsigned long)cycToNs(mn), (unsigned long)cycToNs(avg),
                (unsigned long)cycToNs(mx));
}

// Time n invocations of op(i); print min/avg/max in cycles and ns.
template <typename F>
static void benchRun(const char* name, uint32_t n, F op) {
  uint32_t mn = UINT32_MAX, mx = 0;
  uint64_t sum = 0;
  for (uint32_t i = 0; i < n; i++) {
    uint32_t t0 = ARM_DWT_CYCCNT;
    op(i);
    uint32_t dt = ARM_DWT_CYCCNT - t0;
    if (dt < mn) mn = dt;
    if (dt > mx) mx = dt;
    sum += dt;
  }
  printStat(name, n, mn, sum, mx);
}

static void restoreDacOffsets(uint8_t mask) {
  if (mask & 1) FastIO::dacProgram(0, (int16_t)Stimjim.currentOffsets[0]);
  if (mask & 2) FastIO::dacProgram(1, (int16_t)Stimjim.currentOffsets[1]);
  FastIO::dacLatch(mask);
}

static void spinUntil(uint64_t deadlineCyc) {
  while ((int64_t)(FastIO::cycles64() - deadlineCyc) < 0) ;
}

static void ok()                                   { Serial.println("OK"); }
static void err(const char* cmd, const char* msg)  { Serial.printf("ERR %s: %s\n", cmd, msg); }
static void warn(const char* cmd, const char* msg) { Serial.printf("WARN %s: %s\n", cmd, msg); }

// Last mode commanded per channel (`M` shadow state, protocol §3; original
// numbering 0-3). Boot state is grounded: Stimjim.begin() ends inside
// getCurrentOffsets with setOutputMode(ch,3). Trains switch the OE pins
// autonomously and leave driven channels grounded — kept in sync at start/
// stop/completion below.
static uint8_t modeShadow[2] = {3, 3};

// -------------------------------------------------- S / L / W (slots 0-99)

static void dumpSlot(uint8_t idx) {
  const TrainDef& t = TrainStore::slotConst(idx);
  static const char* modeStr[4] = {
    "Voltage output", "Current output",
    "Not driven (hi-Z)", "Not driven (grounded)"
  };
  static const char* typeStr[3] = {"piecewise-constant (S)", "piecewise-ramp (L)", "sine (W)"};

  Serial.println("----------------------------------");
  Serial.printf("Parameters for PulseTrain[%u] — %s\r\n", idx, typeStr[t.type]);
  const char* nm0 = (t.mode0 <= 1 && t.meas.what0 == 0) ? " (no measurement)" : "";
  const char* nm1 = (t.mode1 <= 1 && t.meas.what1 == 0) ? " (no measurement)" : "";
  Serial.printf("  mode[ch0]: %u (%s%s)\r\n  mode[ch1]: %u (%s%s)\r\n",
                t.mode0, modeStr[t.mode0], nm0, t.mode1, modeStr[t.mode1], nm1);
  Serial.printf("  period:    %lu usec (%0.3f sec, %0.3f Hz)\r\n",
                (unsigned long)t.period_us, 0.000001 * t.period_us, 1000000.0 / t.period_us);
  Serial.printf("  duration:  %lu usec (%0.3f sec)\r\n",
                (unsigned long)t.duration_us, 0.000001 * t.duration_us);
  Serial.printf("  delay:     %lu usec after the start request (%0.3f sec)\r\n",
                (unsigned long)t.delay_us, 0.000001 * t.delay_us);
  if (t.type == PIECEWISE_RAMP) {
    if (t.dt_us) Serial.printf("  interval:  %lu usec per ramp sample\r\n",
                               (unsigned long)t.dt_us);
    else         Serial.printf("  interval:  %u usec per ramp sample (build default)\r\n",
                               SJ_TARGET_DT_US);
  }
  const char* u0 = (t.mode0 == 1) ? "uA" : "mV";
  const char* u1 = (t.mode1 == 1) ? "uA" : "mV";
  if (t.type == SINE) {
    char f0[16], f1[16], p0[16], p1[16];
    TrainStore::milliToStr(t.sine.freq0_mHz, f0);
    TrainStore::milliToStr(t.sine.freq1_mHz, f1);
    TrainStore::milliToStr(t.sine.phase0_mdeg, p0);
    TrainStore::milliToStr(t.sine.phase1_mdeg, p1);
    Serial.printf("  burst:     %lu usec per period\r\n", (unsigned long)t.sine.burst_us);
    Serial.printf("  amplitude: %ld%s, %ld%s\r\n", (long)t.sine.amp0, u0, (long)t.sine.amp1, u1);
    Serial.printf("  frequency: %sHz, %sHz\r\n", f0, f1);
    Serial.printf("  phase:     %sdeg, %sdeg\r\n", p0, p1);
  } else {
    Serial.println("\r\n  stage    duration     output0   output1");
    for (uint8_t j = 0; j < t.nStages; j++)
      Serial.printf("   %2u  %7lu usec %8ld%s %8ld%s\r\n", j,
                    (unsigned long)t.stages[j].dur_us,
                    (long)t.stages[j].a0, u0, (long)t.stages[j].a1, u1);
    if (t.nStages == 0)
      Serial.println("   (no stages)");
  }
  Serial.printf("  env:  rampIn %lu usec, rampOut %lu usec, shape %u\r\n",
                (unsigned long)t.env.rampIn_us, (unsigned long)t.env.rampOut_us, t.env.shape);
  Serial.printf("  meas: what0 %u, what1 %u, when %u, stage %d, report %u, fit %u (%s)\r\n",
                t.meas.what0, t.meas.what1, t.meas.when, t.meas.stage, t.meas.report,
                t.meas.fit, t.meas.fit ? "rotate over repetitions" : "refuse if it does not fit");
  Serial.println("----------------------------------");
  ok();
}

// Parse "<idx>" and classify what follows: query dump (end), canonical query
// ('?'), or set (','). Returns false after printing an ERR.
static bool parseSlotIdx(const char* cmd, const char*& p, uint8_t& idx) {
  char* end;
  long v = strtol(p, &end, 10);
  if (end == p) { err(cmd, "need a slot index 0-99"); return false; }
  if (v < 0 || v >= SJ_NUM_SLOTS) { err(cmd, "slot index out of range 0-99"); return false; }
  idx = (uint8_t)v;
  p = end;
  while (*p == ' ') p++;
  return true;
}

static void handleTrain(char letter, const char* args) {
  const char cmd[2] = {letter, '\0'};
  const char* p = args;
  uint8_t idx;
  if (!parseSlotIdx(cmd, p, idx)) return;

  if (*p == '\0') { dumpSlot(idx); return; }                 // bare S<idx> — legacy dump
  if (*p == '?') {                                           // canonical one-line form,
    char line[SJ_SERIALIZE_MAX];                             // letter matches actual type
    TrainStore::serializeTrain(idx, TrainStore::slotConst(idx), line, sizeof line);
    Serial.println(line);
    return;
  }
  if (*p != ',') { err(cmd, "expected ',' , '?' or end of line after the index"); return; }

  if (Engine::activeSlot(0) == idx || Engine::activeSlot(1) == idx) {
    err(cmd, "slot is attached to a running train — stop first (T-1 / U-1)");
    return;
  }

  TrainDef staged;
  char errbuf[SJ_MSG_MAX], warnbuf[SJ_MSG_MAX];
  if (!TrainStore::parseTrainBody(letter, p, TrainStore::slotConst(idx), staged,
                                  errbuf, sizeof errbuf, warnbuf, sizeof warnbuf)) {
    err(cmd, errbuf);   // slot untouched — atomic staging
    return;
  }
  if (warnbuf[0]) warn(cmd, warnbuf);
  // A caveat the parser cannot make, because what it would have to compare
  // against is runtime state: a stage boundary closer to its predecessor than
  // one latch costs schedules a latch that cannot be on time. Every such latch
  // shows up in the train's own deadline counters at the end, but by then the
  // waveform has already played, so say it here. A warning and not a refusal --
  // the figure moves with `CAL`, and nothing under about 10 us settles at the
  // output anyway (`CAL SETTLE` is 9).
  {
    const uint32_t dtUs = staged.dt_us ? staged.dt_us : (uint32_t)SJ_TARGET_DT_US;
    const uint32_t gap  = TrainStore::minLatchGapUs(staged, dtUs);
    const uint16_t need = Cal::minLatchUs(Cal::live(), staged.mode0 < 2 && staged.mode1 < 2);
    if (gap < need) {
      char note[SJ_MSG_MAX];
      snprintf(note, sizeof note,
               "%lu us between two latches is under the %u us one latch costs on this "
               "board — those latches will be late",
               (unsigned long)gap, (unsigned)need);
      warn(cmd, note);
    }
  }
  TrainStore::commit(idx, staged);
  // Precompute what this definition alone decides, here in command context
  // where microseconds are free, rather than inside the start latency.
  Engine::deriveSine(idx);

  char line[SJ_SERIALIZE_MAX];
  TrainStore::serializeTrain(idx, staged, line, sizeof line);
  Serial.println(line);   // echo the canonical round-trip line of what was stored
}

// ----------------------------------------------------------------- DELAY
//
// Convenience setter for the slot's post-trigger delay, which is also the
// optional 6th field of the S/L/W header. Note the asymmetry that follows from
// "a waveform line fully defines its header": a later S/L/W line without that
// field resets the delay to 0, so set DELAY *after* defining the waveform.
static void handleDelay(const char* args) {
  const char* p = args;
  uint8_t idx;
  if (!parseSlotIdx("DELAY", p, idx)) return;

  char line[48];
  if (*p == '?') {
    TrainStore::serializeDelay(idx, TrainStore::slotConst(idx).delay_us, line, sizeof line);
    Serial.println(line);
    return;
  }
  if (*p != ',') { err("DELAY", "need DELAY<idx>,<delay_us> or DELAY<idx>?"); return; }

  long v[1];
  if (parseFields(p + 1, v, 1) != 1) { err("DELAY", "need <delay_us>"); return; }
  if (v[0] < 0)                      { err("DELAY", "delay_us must be >= 0"); return; }
  if ((unsigned long)v[0] > SJ_MAX_DELAY_US) { err("DELAY", "delay_us exceeds 2000000000 us (2000 s)"); return; }

  if (Engine::activeSlot(0) == idx || Engine::activeSlot(1) == idx) {
    err("DELAY", "slot is attached to a running train — stop first (T-1 / U-1)");
    return;
  }

  TrainDef staged = TrainStore::slotConst(idx);
  staged.delay_us = (uint32_t)v[0];
  TrainStore::commit(idx, staged);
  TrainStore::serializeDelay(idx, staged.delay_us, line, sizeof line);
  Serial.println(line);
}

// -------------------------------------------------------------------- DT
//
// The slot's ramp sample interval, also the optional 7th field of an `L`
// header. Same asymmetry as DELAY: a later L line without the field resets the
// interval to the build default, so set DT *after* defining the waveform.
// A coarser interval is what makes room for a measurement point on a ramp
// (protocol §4); the value the player can actually keep up with depends on the
// CAL budgets and is checked at start, not here.
static void handleDt(const char* args) {
  const char* p = args;
  uint8_t idx;
  if (!parseSlotIdx("DT", p, idx)) return;

  char line[48];
  if (*p == '?') {
    TrainStore::serializeDt(idx, TrainStore::slotConst(idx).dt_us, line, sizeof line);
    Serial.println(line);
    return;
  }
  if (*p != ',') { err("DT", "need DT<idx>,<dt_us> or DT<idx>?"); return; }

  long v[1];
  if (parseFields(p + 1, v, 1) != 1) { err("DT", "need <dt_us>"); return; }
  if (v[0] != 0 && ((unsigned long)v[0] < SJ_MIN_DT_US || (unsigned long)v[0] > SJ_MAX_DT_US)) {
    err("DT", "dt_us must be 0 (build default) or 2..1000000 us");
    return;
  }
  if (TrainStore::slotConst(idx).type != PIECEWISE_RAMP) {
    err("DT", "only an L (ramp) slot has a sample interval");
    return;
  }
  if (Engine::activeSlot(0) == idx || Engine::activeSlot(1) == idx) {
    err("DT", "slot is attached to a running train — stop first (T-1 / U-1)");
    return;
  }

  TrainDef staged = TrainStore::slotConst(idx);
  staged.dt_us = (uint32_t)v[0];
  TrainStore::commit(idx, staged);
  TrainStore::serializeDt(idx, staged.dt_us, line, sizeof line);
  Serial.println(line);
}

// ------------------------------------------------------------------- CAL
//
// The hardware timing budget (Cal.h) as set-commands. `CAL?` prints the whole
// set, `CAL,<name>?` one parameter, `CAL,<name>,<us>` sets one and `CALDEF`
// restores the compiled defaults. Sets are refused while a train runs: the
// engine takes its copy at arm time, so a mid-train change would describe a
// board the running waveform is not using.
static void printCalLine(uint8_t id) {
  Serial.printf("CAL,%s,%u\n", Cal::NAME[id], Cal::live().us[id]);
}

static void handleCal(const char* args) {
  const char* p = args;
  while (*p == ' ') p++;
  if (*p == '\0' || *p == '?') {
    for (uint8_t i = 0; i < Cal::N_ID; i++) printCalLine(i);
    ok();
    return;
  }
  if (*p != ',') { err("CAL", "need CAL? , CAL,<name>? or CAL,<name>,<us>"); return; }
  p++;
  while (*p == ' ') p++;

  char name[16];
  size_t n = 0;
  while (isalnum((unsigned char)*p) && n + 1 < sizeof name) name[n++] = *p++;
  name[n] = '\0';
  const int8_t id = Cal::indexOf(name);
  if (id < 0) {
    Serial.printf("ERR CAL: unknown parameter '%s' — CAL? lists them\n", name);
    return;
  }
  while (*p == ' ') p++;
  if (*p == '?' || *p == '\0') { printCalLine((uint8_t)id); return; }
  if (*p != ',') { err("CAL", "expected ',' , '?' or end of line after the name"); return; }

  long v[1];
  if (parseFields(p + 1, v, 1) != 1) { err("CAL", "need <us>"); return; }
  if (Engine::anyActive()) {
    err("CAL", "a train is running — stop first (T-1 / U-1)");
    return;
  }
  Cal::Def staged;
  const char* msg = Cal::apply(Cal::live(), (uint8_t)id, v[0], staged);
  if (msg) { err("CAL", msg); return; }
  Cal::set(staged);
  printCalLine((uint8_t)id);
}

static void handleCalDef() {
  if (Engine::anyActive()) {
    err("CALDEF", "a train is running — stop first (T-1 / U-1)");
    return;
  }
  Cal::set(Cal::defaults());
  for (uint8_t i = 0; i < Cal::N_ID; i++) printCalLine(i);
  ok();
}

// --------------------------------------------------------------------- CLK
//
// The wall clock is a label and never an authority (Clock.h). The reply pairs
// it with the microsecond count every log row is stamped with, because that
// pair -- not the RTC -- is what anchors a log file to a computer's log: a host
// subtracts its own clock from it and bounds the error by timing the round
// trip. A host that never sets the clock still gets a usable anchor this way.

static void clkStatus() {
  uint32_t s;
  uint16_t ms;
  uint64_t us;
  const bool ok_ = Clock::anchor(s, ms, us);
  char usStr[24];
  Protocol::u64str(us, usStr);
  const Clock::Source src = Clock::source();
  Serial.printf("CLK,%lu,%u,%s,%s\n", (unsigned long)(ok_ ? s : 0),
                (unsigned)(ok_ ? ms : 0), usStr, Clock::sourceName(src));
  if (!ok_) { Serial.println("# clock: this board has no running RTC"); return; }
  // What the source means matters more than the reading: a `build` time is the
  // build host's *local* time at compile, so it is off by however long the
  // binary has been in service and by the zone offset on top of that.
  static const char* what[3] = {
    "the binary's compile time in the BUILD HOST's local zone — a label, not a time",
    "running from VBAT since an earlier upload — the epoch is unverified",
    "UTC, set by a host this session — good to about a millisecond",
  };
  char iso[SJ_ISO_MAX];
  Clock::isoString(iso, sizeof iso, s, ms);
  Serial.printf("# clock: %s src=%s (%s)\n", iso, Clock::sourceName(src), what[src]);
}

static void handleClk(const char* args) {
  const char* p = args;
  while (*p == ',' || *p == ' ') p++;
  if (!*p || *p == '?') { clkStatus(); return; }

  // strtoul, not the shared parseFields: a Unix second count passes 2^31 in
  // 2038 and the strict parser's strtol would saturate there.
  char* end;
  const unsigned long secs = strtoul(p, &end, 10);
  if (end == p) { err("CLK", "need CLK,<unix_seconds>[,<ms>] or CLK?"); return; }
  p = end;
  unsigned long ms = 0;
  if (*p == ',') {
    p++;
    ms = strtoul(p, &end, 10);
    if (end == p) { err("CLK", "malformed millisecond field"); return; }
    p = end;
  }
  while (*p == ' ') p++;
  if (*p)      { err("CLK", "trailing characters after CLK,<unix>[,<ms>]"); return; }
  if (ms > 999) { err("CLK", "ms must be 0-999"); return; }
  Clock::set((uint32_t)secs, (uint16_t)ms);
  clkStatus();
}

// -------------------------------------------------------------------- PAGE

static void handlePage(const char* args) {
  const char* p = args;
  while (*p == ',' || *p == ' ') p++;
  if (!*p)        { UiMenu::pageNext();   return; }   // same action as the page button
  if (*p == '?')  { UiMenu::pageStatus(); return; }
  long v;
  if (parseFields(p, &v, 1) != 1 || v < 0 || v > 255) {
    err("PAGE", "need PAGE (advance), PAGE,<n> (select) or PAGE? (query)");
    return;
  }
  UiMenu::pageSelect((uint8_t)v);
}

// ------------------------------------------------------------- ENV / MEAS

static void handleEnv(const char* args) {
  const char* p = args;
  uint8_t idx;
  if (!parseSlotIdx("ENV", p, idx)) return;

  char line[64];
  if (*p == '?') {
    TrainStore::serializeEnv(idx, TrainStore::slotConst(idx).env, line, sizeof line);
    Serial.println(line);
    return;
  }
  if (*p != ',') { err("ENV", "need ENV<idx>,<rampIn_us>,<rampOut_us>[,<shape>] or ENV<idx>?"); return; }

  long v[3];
  int8_t n = parseFields(p + 1, v, 3);
  if (n < 2) { err("ENV", "need <rampIn_us>,<rampOut_us>[,<shape>]"); return; }
  if (v[0] < 0 || v[1] < 0 || (n == 3 && v[2] < 0)) { err("ENV", "fields must be non-negative"); return; }

  if (Engine::activeSlot(0) == idx || Engine::activeSlot(1) == idx) {
    err("ENV", "slot is attached to a running train — stop first (T-1 / U-1)");
    return;
  }

  EnvDef e;
  e.rampIn_us  = (uint32_t)v[0];
  e.rampOut_us = (uint32_t)v[1];
  e.shape      = (n == 3) ? (uint8_t)v[2] : 0;
  const char* msg = TrainStore::validateEnv(TrainStore::slotConst(idx), e);
  if (msg) { err("ENV", msg); return; }

  TrainDef staged = TrainStore::slotConst(idx);
  staged.env = e;
  TrainStore::commit(idx, staged);
  TrainStore::serializeEnv(idx, e, line, sizeof line);
  Serial.println(line);
}

static void handleMeas(const char* args) {
  const char* p = args;
  uint8_t idx;
  if (!parseSlotIdx("MEAS", p, idx)) return;

  char line[64];
  if (*p == '?') {
    TrainStore::serializeMeas(idx, TrainStore::slotConst(idx).meas, line, sizeof line);
    Serial.println(line);
    return;
  }
  if (*p != ',') { err("MEAS", "need MEAS<idx>,<what0>,<what1>,<when>,<stage>[,<report>[,<fit>]] or MEAS<idx>?"); return; }

  long v[6];
  int8_t n = parseFields(p + 1, v, 6);
  if (n < 4) { err("MEAS", "need <what0>,<what1>,<when>,<stage>[,<report>[,<fit>]]"); return; }
  if (v[0] < 0 || v[1] < 0 || v[2] < 0 || v[3] < -1 || (n >= 5 && v[4] < 0) ||
      (n >= 6 && v[5] < 0)) {
    err("MEAS", "fields must be non-negative (stage may be -1 = all)");
    return;
  }
  // keep the narrow casts below honest (detailed validation in validateMeas)
  if (v[0] > 3 || v[1] > 3 || v[2] > 3 || v[3] >= SJ_MAX_STAGES ||
      (n >= 5 && v[4] > 3) || (n >= 6 && v[5] > SJ_FIT_ROTATE)) {
    err("MEAS", "field out of range");
    return;
  }

  if (Engine::activeSlot(0) == idx || Engine::activeSlot(1) == idx) {
    err("MEAS", "slot is attached to a running train — stop first (T-1 / U-1)");
    return;
  }

  MeasDef m;
  m.what0  = (uint8_t)v[0];
  m.what1  = (uint8_t)v[1];
  m.when   = (uint8_t)v[2];
  m.stage  = (int8_t)v[3];
  // Omitted optional fields take the boot default, not the stored value: a
  // MEAS line fully defines the slot's measurement configuration, the same way
  // an S/L/W line fully defines its header.
  m.report = (n >= 5) ? (uint8_t)v[4] : 0;
  m.fit    = (n >= 6) ? (uint8_t)v[5] : (uint8_t)SJ_FIT_ROTATE;
  char warnbuf[SJ_MSG_MAX];
  const char* msg = TrainStore::validateMeas(TrainStore::slotConst(idx), m, warnbuf, sizeof warnbuf);
  if (msg) { err("MEAS", msg); return; }
  if (warnbuf[0]) warn("MEAS", warnbuf);

  TrainDef staged = TrainStore::slotConst(idx);
  staged.meas = m;
  TrainStore::commit(idx, staged);
  TrainStore::serializeMeas(idx, m, line, sizeof line);
  Serial.println(line);
}

// ------------------------------------------------------- T / U (start/stop)

// Completed trains per boot — the legacy `Train #<n> complete.` counter.
static uint32_t trainCount = 0;

// Silent when the train met every deadline, which is the expected case.
// The two faults have different fixes, so they are reported separately: a late
// latch is a budget to recalibrate, an overdue event is a waveform whose own
// deadlines collide. Only the first is a defect.
static void printTimingFaults(const Engine::Completion& c) {
  if (c.lateEvents)
    Serial.printf("WARN engine: %lu latch(es) overran their deadline by up to %lu ns — "
                  "a timing budget in Config.h is too small here (recalibrate with BENCH)\n",
                  (unsigned long)c.lateEvents, (unsigned long)cycToNs(c.maxLateCyc));
  if (c.startNeedUs)
    Serial.printf("WARN engine: arming this train took longer than CAL STARTLAT (%u us), "
                  "so its first latch could not be on time — set CAL STARTLAT >= %u us "
                  "(BENCHARM,<slot> measures the arm; a TRIG independent route arms twice)\n",
                  (unsigned)Cal::live().us[Cal::STARTLAT], (unsigned)c.startNeedUs);
  if (c.overdueEvents)
    Serial.printf("# engine: %lu event(s) were already due when the player reached them, "
                  "worst by %lu ns — an earlier event ran long (events that share a "
                  "deadline by definition are not counted)\n",
                  (unsigned long)c.overdueEvents, (unsigned long)cycToNs(c.maxOverdueCyc));
}

static void handleStart(char letter, const char* args) {
  const char cmd[2] = {letter, '\0'};
  uint8_t eng = (letter == 'U') ? 1 : 0;

  long v;
  if (parseFields(args, &v, 1) != 1) {   // strict: legacy atoi turned "Tfoo" into "T0"
    err(cmd, "need a slot index (or -1 to stop)");
    return;
  }
  if (v < 0) {
    uint8_t mask = Engine::claimedMask(eng);
    int16_t stopped = Engine::activeSlot(eng);
    Engine::stopTrain(eng);
    if (mask & 1) modeShadow[0] = 3;
    if (mask & 2) modeShadow[1] = 3;
    Serial.print("Forcing "); Serial.print(letter); Serial.println(" train to stop");
    // A stop pushes no completion record (legacy `T-1` printed only its own
    // line), so whatever the plan accumulated is summarized here instead of
    // being discarded.
    if (stopped >= 0) {
      Measure::poll();                     // stream what is still in the ring first
      Engine::Completion f;
      Engine::timingFaults(eng, f);
      printTimingFaults(f);
      UiMenu::noteResult(trainCount, eng, (uint8_t)stopped);
      Measure::printSummary(eng, (uint8_t)stopped);
      SdLog::flushNow();
    }
    return;
  }
  if (v >= SJ_NUM_SLOTS) {
    Serial.println("Invalid PulseTrain index.");   // legacy byte-exact
    return;
  }

  char errbuf[SJ_MSG_MAX];
  // A trigger edge can preempt loop() and call startTrain for the same engine,
  // and the busy check plus the arm-time writes are not atomic. The bus lock
  // masks the trigger ISRs (priority 80) for the duration. Trigger ISRs need no
  // lock themselves: they cannot preempt each other, and the players -- the
  // only thing above them -- never start trains.
  FastIO::busLock();
  bool started = Engine::startTrain(eng, (uint8_t)v, TrainStore::slotConst((uint8_t)v),
                                    errbuf, sizeof errbuf);
  FastIO::busUnlock();
  if (!started) {
    warn(cmd, errbuf);                   // ignore-and-warn policy (plan §2.5)
    return;
  }
  uint8_t mask = Engine::claimedMask(eng);
  if (mask & 1) modeShadow[0] = 3;       // trains leave driven channels grounded
  if (mask & 2) modeShadow[1] = 3;
  // legacy byte-exact start line (leading blank line included)
  Serial.print("\r\nStarted "); Serial.print(letter);
  Serial.print(" train with parameters of PulseTrain "); Serial.println((int)v);
}

// Drain the completion ring — called from loop() (.ino); all result printing
// happens here, never in ISR context (plan §3.3).
void poll() {
  Engine::Completion c;
  while (Engine::popCompletion(c)) {
    trainCount++;
    // Drain the ring first: the train's last repetitions are still in it, and a
    // log or a streaming host should see them before the completion line rather
    // than after it (protocol §4).
    Measure::poll();
    Serial.print("Train #"); Serial.print(trainCount);
    Serial.print(" complete. Delivered "); Serial.print(c.nPulses);
    Serial.println(" pulses.");
    printTimingFaults(c);
    // Before printSummary, which is what releases the finished train's plan
    // buffer back to loop()'s housekeeping.
    UiMenu::noteResult(trainCount, c.eng, c.slot);
    if (!Measure::printSummary(c.eng, c.slot))
      Serial.println("Note: no measurement carried out.");
    SdLog::flushNow();
    if (c.chMask & 1) modeShadow[0] = 3;
    if (c.chMask & 2) modeShadow[1] = 3;
  }
}

// ------------------------------------------ READ (manual averaged measurement)

static void handleRead(const char* args) {
  long v[2];
  int8_t n = parseFields(args, v, 2);
  if (n < 1) { err("READ", "need READ<ch>[,<n>]"); return; }
  if (v[0] < 0 || v[0] > 1) { err("READ", "channel must be 0 or 1"); return; }
  long navg = (n == 2) ? v[1] : 16;
  if (navg < 1 || navg > 10000) { err("READ", "n must be 1-10000"); return; }
  if (Engine::anyActive()) {
    err("READ", "engine busy — READ would stall the players; use MEAS for in-train measurement");
    return;
  }

  uint8_t ch = (uint8_t)v[0];
  // Same calibrated path as `E` (legacy readAdc + offset + *_PER_ADC), n
  // samples per line; single-pass sums -> mean and sample std dev.
  double sum[2] = {0, 0}, sq[2] = {0, 0};
  for (uint8_t line = 0; line < 2; line++) {
    FastIO::busLock();
    for (long i = 0; i < navg; i++) {
      double u = (Stimjim.readAdc(ch, line) - Stimjim.adcOffset10[ch])
               * (line ? MICROAMPS_PER_ADC : MILLIVOLTS_PER_ADC);
      sum[line] += u;
      sq[line]  += u * u;
    }
    FastIO::acquireBus();               // legacy SPI transactions clobber the CTARs
    FastIO::busUnlock();
  }
  double mean[2], sd[2];
  for (uint8_t line = 0; line < 2; line++) {
    mean[line] = sum[line] / navg;
    double var = (navg > 1) ? (sq[line] - sum[line] * sum[line] / navg) / (navg - 1) : 0.0;
    sd[line]   = (var > 0) ? sqrt(var) : 0.0;
  }
  Serial.printf("READ,%u,%ld,%.2f,%.2f,%.2f,%.2f\n",
                ch, navg, mean[0], sd[0], mean[1], sd[1]);
}

// ------------------------------- M / V / A / E (byte-exact replies — BIST)

static void handleM(const char* args) {
  // query form: M<ch>? returns the shadow state as a canonical set-line
  const char* p = args;
  char* end;
  long ch = strtol(p, &end, 10);
  if (end != p) {
    const char* q = end;
    while (*q == ' ') q++;
    if (*q == '?') {
      if (ch < 0 || ch > 1) { err("M", "channel must be 0 or 1"); return; }
      Serial.printf("M%ld,%u\n", ch, modeShadow[ch]);
      return;
    }
  }

  long v[2];
  if (parseFields(args, v, 2) != 2) { err("M", "need <ch>,<mode>"); return; }
  if (v[0] < 0 || v[0] > 1) { err("M", "channel must be 0 or 1"); return; }
  if (v[1] < 0 || v[1] > 3) { err("M", "mode must be 0-3 (90/91 exist in train definitions only)"); return; }

  // Original numbering 0-3 maps 1:1 onto the OE decoder — same as the upstream
  // open-ephys firmware. (The lab firmware's documented 2-5 renumbering never
  // matched its own decoder; see protocol §6.8.)
  Stimjim.setOutputMode((byte)v[0], (byte)v[1]);   // two GPIO writes — ISR-safe, no bus lock needed
  modeShadow[v[0]] = (uint8_t)v[1];

  Serial.print("Set channel "); Serial.print((int)v[0]);
  Serial.print(" to mode ");    Serial.println((int)v[1]);
}

static void handleV(const char* args) {
  long v[2];
  if (parseFields(args, v, 2) != 2) { err("V", "need <ch>,<mV>"); return; }
  if (v[0] < 0 || v[0] > 1) { err("V", "channel must be 0 or 1"); return; }
  if (Engine::anyActive()) warn("V", "train running — executing under bus lock");

  int channel = (int)v[0], amp = (int)v[1];
  int dacVal = 1.0 * amp / MILLIVOLTS_PER_DAC + Stimjim.voltageOffsets[channel];   // legacy float math
  if (dacVal <= 32767 && dacVal >= -32768) {
    FastIO::busLock();
    Stimjim.writeToDac((byte)channel, (int16_t)dacVal);
    FastIO::acquireBus();          // legacy SPI transaction clobbered the CTARs
    FastIO::busUnlock();
    Serial.print("Set channel "); Serial.print(channel);
    Serial.print(" to amplitude "); Serial.print(amp);
    Serial.print(" mV (dac value "); Serial.print(dacVal); Serial.println(").");
  } else {
    Serial.print(dacVal); Serial.println(" is out of range.");
  }
}

static void handleA(const char* args) {
  long v[2];
  if (parseFields(args, v, 2) != 2) { err("A", "need <ch>,<dac>"); return; }
  if (v[0] < 0 || v[0] > 1) { err("A", "channel must be 0 or 1"); return; }
  if (Engine::anyActive()) warn("A", "train running — executing under bus lock");

  int channel = (int)v[0];
  long amp = v[1];
  if (amp <= 32767 && amp >= -32768) {
    FastIO::busLock();
    Stimjim.writeToDac((byte)channel, (int16_t)amp);
    FastIO::acquireBus();
    FastIO::busUnlock();
    Serial.print("Set channel "); Serial.print(channel);
    Serial.print(" to amplitude "); Serial.println((int)amp);
  } else {
    Serial.print((int)amp); Serial.println(" is out of range.");
  }
}

static void handleE(const char* args) {
  long v[2];
  if (parseFields(args, v, 2) != 2) { err("E", "need <ch>,<line>"); return; }
  if (v[0] < 0 || v[0] > 1) { err("E", "channel must be 0 or 1"); return; }
  if (v[1] < 0 || v[1] > 1) { err("E", "line must be 0 (voltage) or 1 (current)"); return; }
  if (Engine::anyActive()) warn("E", "train running — executing under bus lock");

  int channel = (int)v[0], line = (int)v[1];
  FastIO::busLock();
  int val = Stimjim.readAdc((byte)channel, (byte)line);
  FastIO::acquireBus();
  FastIO::busUnlock();
  // identical expression to the legacy handler: float math, truncation to int
  int valRealUnits = (val - Stimjim.adcOffset10[channel]) * (line ? MICROAMPS_PER_ADC : MILLIVOLTS_PER_ADC);
  Serial.print("Read value: "); Serial.print(val);
  Serial.print(" (");            Serial.print(valRealUnits);
  Serial.println(line ? "uA)" : "mV)");
}

// -------------------------------------------------- B / C / D / P (offsets)

static void handleB() {
  if (Engine::anyActive()) { err("B", "calibration needs an idle engine — stop trains first"); return; }
  Stimjim.getAdcOffsets();         // grounds the outputs, ~2000 ADC reads
  FastIO::acquireBus();
  Measure::noteOffsets();          // the row formatting works from a fixed-point copy
  modeShadow[0] = modeShadow[1] = 3;
  ok();                            // legacy printed nothing — see protocol §6
}

static void handleC() {
  if (Engine::anyActive()) { err("C", "calibration needs an idle engine — stop trains first"); return; }
  warn("C", "output will ramp");   // getVoltageOffsets sweeps a DAC ramp on the outputs
  Stimjim.getCurrentOffsets();
  Stimjim.getVoltageOffsets();
  FastIO::acquireBus();
  Measure::noteOffsets();
  modeShadow[0] = modeShadow[1] = 3;
  ok();
}

static void handleD(const char* args) {
  while (*args == ' ') args++;
  if (*args == '?') {              // machine CSV (protocol §3)
    Serial.printf("D,%.4f,%.4f,%.4f,%.4f,%d,%d,%d,%d\n",
                  Stimjim.adcOffset25[0], Stimjim.adcOffset25[1],
                  Stimjim.adcOffset10[0], Stimjim.adcOffset10[1],
                  Stimjim.currentOffsets[0], Stimjim.currentOffsets[1],
                  Stimjim.voltageOffsets[0], Stimjim.voltageOffsets[1]);
    return;
  }
  // legacy human dump, byte-identical text + trailing blank line, then OK
  char str[240];
  sprintf(str, "ADC offsets (+-2.5V): %f, %f\r\nADC offsets (+-10V): %f, %f\r\ncurrent offsets: %d, %d\r\nvoltage offsets: %d, %d\r\n",
          Stimjim.adcOffset25[0], Stimjim.adcOffset25[1],
          Stimjim.adcOffset10[0], Stimjim.adcOffset10[1],
          Stimjim.currentOffsets[0], Stimjim.currentOffsets[1],
          Stimjim.voltageOffsets[0], Stimjim.voltageOffsets[1]);
  Serial.println(str);
  ok();
}

static void handleP() {
  TriggerRoute trig[2] = {Triggers::route(0), Triggers::route(1)};
  TrainStore::eepromSave(trig);
  Serial.println("First 10 slot definitions and the trigger table saved to EEPROM.");
}

// ------------------------------------------------- TRIG / R (queries only)

static void serializeTrig(uint8_t t, char* buf, size_t n) {
  const TriggerRoute& r = Triggers::route(t);
  snprintf(buf, n, "TRIG%u,%u,%d,%d,%u", t, r.mode, r.slot0, r.slot1, r.edge);
}

static void handleTrig(const char* args) {
  const char* p = args;
  char* end;
  long t = strtol(p, &end, 10);
  if (end == p || t < 0 || t > 1) { err("TRIG", "need trigger input 0 or 1"); return; }
  p = end;
  while (*p == ' ') p++;
  char line[48];
  if (*p == '?') {
    serializeTrig((uint8_t)t, line, sizeof line);
    Serial.println(line);
    return;
  }
  if (*p != ',') { err("TRIG", "need TRIG<t>,<mode>,<slot0>,<slot1>,<edge> or TRIG<t>?"); return; }

  long v[4];
  if (parseFields(p + 1, v, 4) != 4) {
    err("TRIG", "need <mode>,<slot0>,<slot1>,<edge> (slots -1 for none)");
    return;
  }
  if (v[0] < 0 || v[0] > 3 || v[3] < 0 || v[3] > 1 ||
      v[1] < -128 || v[1] > 127 || v[2] < -128 || v[2] > 127) {
    err("TRIG", "field out of range");
    return;
  }
  TriggerRoute r = {(uint8_t)v[0], (int8_t)v[1], (int8_t)v[2], (uint8_t)v[3]};
  const char* msg = Triggers::validateRoute(r);
  if (msg) { err("TRIG", msg); return; }

  Triggers::setRoute((uint8_t)t, r);
  serializeTrig((uint8_t)t, line, sizeof line);
  Serial.println(line);
}

static void handleR(const char* args) {
  const char* p = args;
  char* end;
  long t = strtol(p, &end, 10);
  if (end != p) {
    const char* q = end;
    while (*q == ' ') q++;
    if (*q == '?' && t >= 0 && t <= 1) {
      // legacy view of the TRIG table: R<t>,<slot>,<outputFlag>
      const TriggerRoute& r = Triggers::route((uint8_t)t);
      Serial.printf("R%ld,%d,%d\n", t, (r.mode == 1) ? r.slot0 : -1, (r.mode == 3) ? 1 : 0);
      return;
    }
  }

  // Legacy setter: R<trig>,<slot>[,<outputFlag>]. It writes the same table the
  // TRIG command owns — outputFlag != 0 makes the pin a stimulus marker,
  // otherwise the edge starts <slot> jointly on rising edges (protocol §3).
  long v[3];
  int8_t n = parseFields(args, v, 3);
  if (n < 2) { err("R", "need R<trig>,<slot>[,<output>] or R<t>?"); return; }
  if (v[0] < 0 || v[0] > 1) { err("R", "trigger input must be 0 or 1"); return; }
  if (v[1] < -1 || v[1] >= SJ_NUM_SLOTS) { err("R", "slot must be -1 or 0-99"); return; }

  TriggerRoute r;
  if (n == 3 && v[2] != 0)   r = {3, -1, -1, 0};                     // marker output
  else if (v[1] < 0)         r = {0, -1, -1, 0};                     // disabled
  else                       r = {1, (int8_t)v[1], -1, 0};           // joint, rising

  const char* msg = Triggers::validateRoute(r);
  if (msg) { err("R", msg); return; }
  Triggers::setRoute((uint8_t)v[0], r);

  char line[48];
  serializeTrig((uint8_t)v[0], line, sizeof line);
  Serial.println(line);
}

// --------------------------------------------------------------------- DUMP

// The sink is explicit because the SD log header carries the same block: a log
// must identify the waveforms that produced it even when the train was started
// by a trigger edge with no host listening.
void writeDump(Print& out) {
  out.printf("# %s fw=%s proto=%d session dump — paste back to restore\n",
             SJ_FW_NAME, SJ_FW_VERSION, SJ_PROTO_VERSION);
  char line[SJ_SERIALIZE_MAX];
  for (uint8_t idx = 0; idx < SJ_NUM_SLOTS; idx++) {
    const TrainDef& t = TrainStore::slotConst(idx);
    if (!TrainStore::isDefaultTrain(t)) {
      TrainStore::serializeTrain(idx, t, line, sizeof line);
      out.println(line);
    }
    // a W line already carries its envelope; S/L need a separate ENV line
    if (t.type != SINE && !TrainStore::isDefaultEnv(t.env)) {
      TrainStore::serializeEnv(idx, t.env, line, sizeof line);
      out.println(line);
    }
    if (!TrainStore::isDefaultMeas(t)) {
      TrainStore::serializeMeas(idx, t.meas, line, sizeof line);
      out.println(line);
    }
  }
  // Real lines now that TRIG is a setter: a dump pastes back complete.
  for (uint8_t t = 0; t < 2; t++) {
    serializeTrig(t, line, sizeof line);
    out.println(line);
  }
  // Only the timing budgets that differ from this build's defaults, so a dump
  // pasted into an identical firmware reproduces the state and a dump read by
  // a human shows what was calibrated by hand.
  for (uint8_t i = 0; i < Cal::N_ID; i++)
    if (Cal::live().us[i] != Cal::defaults().us[i])
      out.printf("CAL,%s,%u\n", Cal::NAME[i], Cal::live().us[i]);
}

static void handleDump() {
  writeDump(Serial);
  ok();
}

// ------------------------------------------------------- LOG and the SD group

// Copy the comma-delimited token at *p (skipping one leading comma) into buf
// and leave p on the delimiter. File names carry dots and digits, so they
// cannot go through parseFields.
static bool nextToken(const char*& p, char* buf, size_t n) {
  while (*p == ' ') p++;
  if (*p == ',') p++;
  while (*p == ' ') p++;
  size_t i = 0;
  while (*p && *p != ',' && i + 1 < n) buf[i++] = *p++;
  while (i && buf[i - 1] == ' ') i--;
  buf[i] = '\0';
  return i != 0;
}

static void handleLog(const char* args) {
  const char* p = args;
  while (*p == ' ') p++;
  if (*p == '\0' || *p == '?') { SdLog::status(); return; }
  if (*p == '0' && p[1] == '\0') { SdLog::closeLog(); return; }
  if (*p == '1') {
    p++;
    while (*p == ' ') p++;
    if (*p == '\0') { SdLog::openLog(nullptr); return; }
    char name[80];
    if (*p == ',' && nextToken(p, name, sizeof name)) { SdLog::openLog(name); return; }
    err("LOG", "expected LOG1[,<name>]");
    return;
  }
  err("LOG", "need LOG? (status), LOG1[,<name>] (open) or LOG0 (close)");
}

static void sdGroupList() {
  Serial.println("# SD group — the socket is under the cover, so the card is served over serial");
  Serial.println("# SDINFO[,1]                   card present, size in KiB, open log; 1 also");
  Serial.println("#                                scans for used space (slow on a big card)");
  Serial.println("# SDLIST[,<dir>]               one SDLIST,<name>,<size> line per entry");
  Serial.println("# SDGET,<name>[,<off>[,<len>]] header line, exactly <len> bytes, then crc32");
  Serial.println("# SDDEL,<name>                 delete one file (not the open log)");
  Serial.println("# LOG? / LOG1[,<name>] / LOG0  status / open / close the measurement log");
  ok();
}

static void handleSdGet(const char* args) {
  const char* p = args;
  char name[80];
  if (!nextToken(p, name, sizeof name)) {
    err("SDGET", "need SDGET,<name>[,<offset>[,<len>]]");
    return;
  }
  if (*p == ',') p++;
  long v[2] = {0, 0};
  int8_t n = parseFields(p, v, 2);
  if (n < 0) { err("SDGET", "offset and length must be integers"); return; }
  if ((n >= 1 && v[0] < 0) || (n >= 2 && v[1] < 0)) {
    err("SDGET", "offset and length must not be negative");
    return;
  }
  SdLog::get(name, (uint64_t)v[0], (uint64_t)v[1]);   // length 0 = to end of file
}

static void handleSdDel(const char* args) {
  const char* p = args;
  char name[80];
  if (!nextToken(p, name, sizeof name)) { err("SDDEL", "need SDDEL,<name>"); return; }
  SdLog::del(name);
}

// -------------------------------------------------------------------- BENCH

// Longest delay BENCHSETTLE will sweep. It sizes a stack array of means, and
// nothing on this hardware settles anywhere near 64 us.
#define SJ_SETTLE_DMAX 64
#define STR_(x) #x
#define STR(x) STR_(x)


static void benchList() {
  Serial.printf("# BENCH group — timing in CPU cycles (%lu/us) and ns\n",
                (unsigned long)SJ_CYC_PER_US);
  Serial.println("# BENCHDAC[,n]                 dacProgram single channel (no latch)");
  Serial.println("# BENCHDAC2[,n]                dacProgramBoth (no latch)");
  Serial.println("# BENCHLATCH[,n]               dacLatch(0b11) pulse");
  Serial.println("# BENCHADC[,ch,line,n]         adcRead, line pre-selected");
  Serial.println("# BENCHSW[,ch,n]               adcSelectLine+adcRead pair (line-switch cost)");
  Serial.println("# BENCHMISO[,n]                alternating ch0/ch1 reads (MISO mux swap)");
  Serial.println("# BENCHCYC[,n]                 cycles64() overhead");
  Serial.println("# BENCHK[,n]                   K_RELOAD recalibration, residual stats");
  Serial.println("# BENCHFMT[,n]                 one MDATA/log row's number formatting (row rate)");
  Serial.println("# BENCHSD[,rows]               the card write after it: buffered println + flush policy");
  Serial.println("# BENCHARM,slot[,n]            Engine::startTrain cost (what CAL STARTLAT must cover)");
  Serial.println("# BENCHSETTLE,ch,code,dmax_us[,n]   ADC reading vs delay after a latch (CAL SETTLE)");
  Serial.println("# BENCHPIT,period_us,n[,preload_us]  PIT wake/latch jitter vs deadline");
  Serial.println("# BENCHSQ,ch,code,half_us,n    square wave, FastIO path (scope A/B)");
  Serial.println("# BENCHSQL,ch,code,half_us,n   square wave, legacy Stimjim.writeToDac path");
  ok();
}

// BENCHARM,<slot>[,n] -- the cost of Engine::startTrain: every microsecond of
// precomputation that happens before t0 is taken, which is exactly what
// CAL STARTLAT has to cover for the first latch to land on time. A trigger
// edge routed in independent mode pays it *twice* before the second engine's
// first latch is due, so the two-engine case needs twice this number.
//
// The dominant term is Measure::armPlan, so bench a measured slot and an
// unmeasured one (mode 90/91) to see what in-train measurement costs an arm.
//
// Nothing plays: the train is stopped again inside the same bus lock, which
// also keeps a player ISR from landing in the middle of a measured interval.
// The outputs stay parked at the calibration offsets throughout.
// BENCHSETTLE,<ch>,<code>,<dmax_us>[,n] -- how long after a latch a reading
// means anything, which is what CAL SETTLE budgets. Measured with the board's
// own ADC rather than an oscilloscope: for each delay 0..dmax the same step is
// latched n times and read that many microseconds later, so the delay at which
// the readings stop moving IS the settling time as the instrument sees it --
// DAC output settling and ADC aperture together, which is the quantity the
// measurement engine actually depends on.
//
// The step is driven on the output, so the channel's mode has to be one the
// bench is safe with; the outputs are restored to the calibration offsets at
// the end. Reported in raw ADC codes: 2.4 mV or 1.6 uA per code.
static void benchSettle(const char* args) {
  long v[4];
  uint8_t np = parseLongs(args, v, 4);
  if (np < 3) { err("BENCHSETTLE", "need ch,code,dmax_us[,n]"); return; }
  const uint8_t ch = (uint8_t)v[0];
  const int16_t code = (int16_t)v[1];
  const uint32_t dmax = (uint32_t)v[2];
  const uint32_t n = (np >= 4 && v[3] > 0) ? (uint32_t)v[3] : 32;
  if (ch > 1) { err("BENCHSETTLE", "ch must be 0/1"); return; }
  if (dmax < 1 || dmax > SJ_SETTLE_DMAX) {
    err("BENCHSETTLE", "dmax_us must be 1.." STR(SJ_SETTLE_DMAX));
    return;
  }
  Serial.println("WARN BENCHSETTLE: drives the DAC - make sure the output mode is safe");

  const uint8_t line = 0;                 // 0 = output voltage sense
  const int16_t park = (int16_t)Stimjim.currentOffsets[ch];
  int32_t mean[SJ_SETTLE_DMAX + 1];
  FastIO::adcSelectLine(ch, line);        // pre-selected, as the player leaves it
  for (uint32_t d = 0; d <= dmax; d++) {
    int32_t sum = 0;
    for (uint32_t i = 0; i < n; i++) {
      // Return to the parked level and let it settle fully, so every
      // repetition of every delay starts from the same place.
      FastIO::dacProgram(ch, park);
      FastIO::dacLatch((uint8_t)(1u << ch));
      delayMicroseconds(200);
      FastIO::dacProgram(ch, code);       // programmed early, exactly as a latch event is
      uint64_t t = FastIO::cycles64();
      FastIO::dacLatch((uint8_t)(1u << ch));
      spinUntil(t + SJ_US_TO_CYC(d));
      sum += FastIO::adcRead(ch, line);
    }
    mean[d] = sum / (int32_t)n;
    Serial.printf("BENCH,SETTLE,ch=%u,delay_us=%lu,raw=%ld\n",
                  (unsigned)ch, (unsigned long)d, (long)mean[d]);
  }
  restoreDacOffsets((uint8_t)(1u << ch));

  // The answer: the earliest delay from which every later reading stays within
  // two ADC codes of the final one. Two codes rather than one because the ADC
  // itself is not noise-free at this averaging depth.
  uint32_t first = dmax;
  for (uint32_t d = 0; d <= dmax; d++) {
    bool flat = true;
    for (uint32_t e = d; e <= dmax; e++)
      if (mean[e] - mean[dmax] > 2 || mean[dmax] - mean[e] > 2) { flat = false; break; }
    if (flat) { first = d; break; }
  }
  Serial.printf("# settled to within 2 codes of %ld after %lu us "
                "(CAL SETTLE is %u us)\n",
                (long)mean[dmax], (unsigned long)first,
                (unsigned)Cal::live().us[Cal::SETTLE]);
  ok();
}

static void benchArm(const char* args) {
  long v[2];
  uint8_t np = parseLongs(args, v, 2);
  if (np < 1) { err("BENCHARM", "need slot[,n]"); return; }
  if (v[0] < 0 || v[0] >= (long)SJ_NUM_SLOTS) { err("BENCHARM", "slot out of range"); return; }
  const uint8_t slot = (uint8_t)v[0];
  const uint32_t n = (np >= 2 && v[1] > 0) ? (uint32_t)v[1] : 200;
  const TrainDef& def = TrainStore::slotConst(slot);
  char scratch[SJ_MSG_MAX];
  uint32_t mn = UINT32_MAX, mx = 0;
  uint64_t sum = 0;
  for (uint32_t i = 0; i < n; i++) {
    FastIO::busLock();
    uint32_t t0 = ARM_DWT_CYCCNT;
    bool started = Engine::startTrain(0, slot, def, scratch, sizeof scratch, 0);
    uint32_t dt = ARM_DWT_CYCCNT - t0;
    Engine::stopTrain(0);
    FastIO::busUnlock();
    if (!started) { err("BENCHARM", scratch); return; }
    if (dt < mn) mn = dt;
    if (dt > mx) mx = dt;
    sum += dt;
  }
  printStat("ARM", n, mn, sum, mx);
  Serial.printf("# CAL STARTLAT is %u us; one engine needs the max above, "
                "a TRIG independent route twice it\n",
                (unsigned)Cal::live().us[Cal::STARTLAT]);
  ok();
}

static void benchPit(const char* args) {
  long v[3] = {0, 0, 0};
  uint8_t n = parseLongs(args, v, 3);
  if (n < 2) { err("BENCHPIT", "need period_us,n[,preload_us]"); return; }
  uint32_t period = (uint32_t)v[0], reps = (uint32_t)v[1];
  uint32_t preload = (n >= 3) ? (uint32_t)v[2] : 0;
  if (period < 20 || preload >= period) { err("BENCHPIT", "period >= 20 us and > preload"); return; }
  if (reps < 1 || (uint64_t)period * reps > 10000000ull) { err("BENCHPIT", "total run must be <= 10 s"); return; }

  Engine::PitBenchResult r;
  if (!Engine::benchPitLatency(period, reps, preload, &r)) {
    err("BENCHPIT", "bench already active or timed out");
    return;
  }
  int32_t avg = (int32_t)(r.sumErr / (int64_t)r.n);
  Serial.printf("BENCH,PIT,n=%lu,period=%luus,preload=%luus,err_cycles(min/avg/max)=%ld/%ld/%ld\n",
                (unsigned long)r.n, (unsigned long)period, (unsigned long)preload,
                (long)r.minErr, (long)avg, (long)r.maxErr);
  Serial.println("# histogram: bin0=early, then 0.5us bins of lateness");
  for (uint32_t b = 0; b < SJ_BENCH_BINS; b++)
    if (r.hist[b])
      Serial.printf("#   bin%lu: %lu\n", (unsigned long)b, (unsigned long)r.hist[b]);
  ok();
}

static void benchSquare(const char* args, bool legacyPath) {
  long v[4];
  if (parseLongs(args, v, 4) < 4) { err("BENCHSQ", "need ch,code,half_us,n"); return; }
  uint8_t  ch = (uint8_t)v[0];
  int16_t  code = (int16_t)v[1];
  uint32_t half = (uint32_t)v[2], n = (uint32_t)v[3];
  if (ch > 1) { err("BENCHSQ", "ch must be 0/1"); return; }
  if (half < 5 || (uint64_t)half * n * 2 > 10000000ull) { err("BENCHSQ", "5 us <= half_us, total <= 10 s"); return; }
  Serial.println("WARN BENCHSQ: drives the DAC — make sure the output mode is safe (grounded)");

  if (legacyPath) {
    for (uint32_t i = 0; i < n; i++) {
      Stimjim.writeToDac(ch, code);
      delayMicroseconds(half);
      Stimjim.writeToDac(ch, (int16_t)-code);
      delayMicroseconds(half);
    }
    FastIO::acquireBus();               // legacy SPI transactions clobbered the CTARs
    restoreDacOffsets(1 << ch);
  } else {
    uint64_t t = FastIO::cycles64();
    for (uint32_t i = 0; i < n; i++) {
      FastIO::dacProgram(ch, code);
      FastIO::dacLatch(1 << ch);
      t += SJ_US_TO_CYC(half);
      spinUntil(t);
      FastIO::dacProgram(ch, (int16_t)-code);
      FastIO::dacLatch(1 << ch);
      t += SJ_US_TO_CYC(half);
      spinUntil(t);
    }
    restoreDacOffsets(1 << ch);
  }
  ok();
}

// sub = command word after "BENCH", args = rest of line
// BENCHSD,<rows> -- what a log row costs *after* the formatting: SdFat's
// buffered println plus the flush policy SdLog::poll() applies, driven exactly
// as loop() drives it. Together with BENCHFMT this accounts for the whole write
// path, which docs/timing.md section 6 had never timed.
//
// It writes to the card and then deletes its own file, so it refuses to run
// while a log is open rather than appending benchmark rows to real data.
static void benchSdWrite(const char* args) {
#if SJ_USE_SD
  long v[2];
  uint32_t rows = parseLongs(args, v, 1) ? (uint32_t)v[0] : 2000;
  if (rows < 1 || rows > 100000) { err("BENCHSD", "rows must be 1-100000"); return; }
  if (SdLog::isOpen()) {
    err("BENCHSD", "a log is open -- close it first (LOG0); this bench writes its own file");
    return;
  }
  warn("BENCHSD", "writing and then deleting BENCHSD.CSV on the card");
  SdLog::openLog("BENCHSD.CSV");
  if (!SdLog::isOpen()) return;                 // openLog printed the reason

  // A representative row: the same shape and length Measure::poll() produces.
  char row[144];
  Measure::benchFormatRow(row, sizeof row, 1);
  const uint32_t t0 = ARM_DWT_CYCCNT;
  uint32_t mn = UINT32_MAX, mx = 0;
  uint64_t sum = 0;
  for (uint32_t i = 0; i < rows; i++) {
    const uint32_t a = ARM_DWT_CYCCNT;
    SdLog::writeRow(row);
    SdLog::poll();                              // the flush policy, as loop() applies it
    const uint32_t dt = ARM_DWT_CYCCNT - a;
    if (dt < mn) mn = dt;
    if (dt > mx) mx = dt;
    sum += dt;
    FastIO::cycles64();                         // a long run must not starve the timebase
  }
  SdLog::flushNow();
  const uint32_t total = ARM_DWT_CYCCNT - t0;
  printStat("SD", rows, mn, sum, mx);
  // The maximum is the number that matters: a card's internal housekeeping
  // stall is what empties the MDATA ring, and an average hides it.
  Serial.printf("# BENCHSD: %lu rows of %u bytes in %lu us -- %lu rows/s sustained, "
                "worst single row %lu us\n",
                (unsigned long)rows, (unsigned)strlen(row),
                (unsigned long)(total / SJ_CYC_PER_US),
                (unsigned long)((uint64_t)rows * 1000000ull / (total / SJ_CYC_PER_US)),
                (unsigned long)(mx / SJ_CYC_PER_US));
  SdLog::closeLog();
  SdLog::del("BENCHSD.CSV");
#else
  (void)args;
  err("BENCHSD", "no SD support in this build");
#endif
}

static void benchDispatch(const char* sub, const char* args) {
  if (!*sub) { benchList(); return; }   // "BENCH" and "BENCH?" both list
  if (Engine::anyActive()) {
    err("BENCH", "benchmarks need an idle engine — stop trains first (T-1 / U-1)");
    return;
  }
  FastIO::acquireBus();   // benches assume our CTAR config regardless of history

  long v[4];
  if (!strcmp(sub, "DAC")) {
    uint32_t n = parseLongs(args, v, 1) ? (uint32_t)v[0] : 1000;
    int16_t c = (int16_t)Stimjim.currentOffsets[0];
    benchRun("DAC", n, [c](uint32_t) { FastIO::dacProgram(0, c); });   // no latch: output unchanged
    ok();
  } else if (!strcmp(sub, "DAC2")) {
    uint32_t n = parseLongs(args, v, 1) ? (uint32_t)v[0] : 1000;
    int16_t c0 = (int16_t)Stimjim.currentOffsets[0];
    int16_t c1 = (int16_t)Stimjim.currentOffsets[1];
    benchRun("DAC2", n, [c0, c1](uint32_t) { FastIO::dacProgramBoth(c0, c1); });
    ok();
  } else if (!strcmp(sub, "LATCH")) {
    uint32_t n = parseLongs(args, v, 1) ? (uint32_t)v[0] : 1000;
    restoreDacOffsets(0b11);            // make sure latching only re-asserts the offsets
    benchRun("LATCH", n, [](uint32_t) { FastIO::dacLatch(0b11); });
    ok();
  } else if (!strcmp(sub, "ADC")) {
    uint8_t np = parseLongs(args, v, 3);
    uint8_t ch = np >= 1 ? (uint8_t)v[0] : 0;
    uint8_t line = np >= 2 ? (uint8_t)v[1] : 0;
    uint32_t n = np >= 3 ? (uint32_t)v[2] : 1000;
    if (ch > 1 || line > 1) { err("BENCHADC", "ch,line must be 0/1"); return; }
    FastIO::adcSelectLine(ch, line);
    benchRun("ADC", n, [ch, line](uint32_t) { (void)FastIO::adcRead(ch, line); });
    ok();
  } else if (!strcmp(sub, "SW")) {
    uint8_t np = parseLongs(args, v, 2);
    uint8_t ch = np >= 1 ? (uint8_t)v[0] : 0;
    uint32_t n = np >= 2 ? (uint32_t)v[1] : 500;
    if (ch > 1) { err("BENCHSW", "ch must be 0/1"); return; }
    // alternate lines so every read pays the control-register switch
    benchRun("SW", n, [ch](uint32_t i) {
      uint8_t line = i & 1;
      FastIO::adcSelectLine(ch, line);
      (void)FastIO::adcRead(ch, line);
    });
    FastIO::adcSelectLine(ch, 0);
    ok();
  } else if (!strcmp(sub, "MISO")) {
    uint32_t n = parseLongs(args, v, 1) ? (uint32_t)v[0] : 1000;
    FastIO::adcSelectLine(0, 0);
    FastIO::adcSelectLine(1, 0);
    benchRun("MISO", n, [](uint32_t i) { (void)FastIO::adcRead(i & 1, 0); });
    ok();
  } else if (!strcmp(sub, "CYC")) {
    uint32_t n = parseLongs(args, v, 1) ? (uint32_t)v[0] : 10000;
    benchRun("CYC", n, [](uint32_t) { (void)FastIO::cycles64(); });
    ok();
  } else if (!strcmp(sub, "K")) {
    uint32_t n = parseLongs(args, v, 1) ? (uint32_t)v[0] : 65;
    int32_t mn, med, mx;
    Engine::calibrateKReload((uint16_t)n, &mn, &med, &mx);
    Serial.printf("BENCH,K,n=%lu,residual_cycles(min/med/max)=%ld/%ld/%ld,K_RELOAD=%lu\n",
                  (unsigned long)n, (long)mn, (long)med, (long)mx,
                  (unsigned long)Engine::kReloadCycles());
    ok();
  } else if (!strcmp(sub, "FMT")) {
    // What one MDATA/log row costs to render: four field conversions plus the
    // row itself, and nothing else -- no serial write, no card. It runs in
    // loop() context like the real thing, so no waveform can see it; what it
    // bounds is the sustainable row rate and through that the headroom of the
    // 128-entry MDATA ring (docs/timing.md section 6).
    uint32_t n = parseLongs(args, v, 1) ? (uint32_t)v[0] : 2000;
    char row[144];
    benchRun("FMT", n, [&row](uint32_t i) { (void)Measure::benchFormatRow(row, sizeof row, i); });
    Serial.printf("# BENCHFMT: last row was %u bytes: %s\n", (unsigned)strlen(row), row);
    ok();
  } else if (!strcmp(sub, "SD")) {
    benchSdWrite(args);
  } else if (!strcmp(sub, "ARM")) {
    benchArm(args);
  } else if (!strcmp(sub, "SETTLE")) {
    benchSettle(args);
  } else if (!strcmp(sub, "PIT")) {
    benchPit(args);
  } else if (!strcmp(sub, "SQ")) {
    benchSquare(args, false);
  } else if (!strcmp(sub, "SQL")) {
    benchSquare(args, true);
  } else {
    err("BENCH", "unknown benchmark — BENCH? lists them");
  }
}

// --------------------------------------------------------------------- HELP

static void help() {
  Serial.println("# stimjimAWG commands — details: docs/serial-protocol.md");
  Serial.println("#   S<i>,m0,m1,per,dur[,delay];a0,a1,d;...  rectangular train (slots 0-99, <=10 stages)");
  Serial.println("#   L<i>,m0,m1,per,dur[,delay[,dt]];...  same syntax, linear ramps (0-dur stage = jump)");
  Serial.println("#   W<i>,m0,m1,per,dur[,delay];amp;freq;phase[;env]  sine train (decimals in Hz ok)");
  Serial.println("#   modes: 0 V, 1 I, 2/3 channel not driven; 90/91 V/I without measurement");
  Serial.println("#   S<i> / L<i> / W<i>              human parameter dump; append ? for the canonical line");
  Serial.println("#   T<i> / T-1, U<i> / U-1          start/stop engine 0 / 1");
  Serial.println("#   DELAY<i>,<us>                   delay from start request to first sample; DELAY<i>?");
  Serial.println("#   ENV<i>,in,out[,shape]           amplitude envelope; ENV<i>?");
  Serial.println("#   DT<i>,<us>                      L ramp sample interval, 0 = build default; DT<i>?");
  Serial.println("#   MEAS<i>,w0,w1,when,stage[,rep[,fit]]  measurement config; MEAS<i>?");
  Serial.println("#     fit: 0 refuse a point that does not fit its gap, 1 rotate its reads (default)");
  Serial.println("#   CAL? / CAL,<name>,<us> / CALDEF timing budget: PRELOAD DACPROG1 DACPROG2 ADCREAD");
  Serial.println("#                                   ADCSWITCH GUARD SETTLE STARTLAT TRIGCOMP (us)");
  Serial.println("#   CLK? / CLK,<unix>[,<ms>]        wall clock: read (with the us timebase) / set");
  Serial.println("#   READ<ch>[,n]                    manual averaged V+I read (mean and std dev)");
  Serial.println("#   M<ch>,<mode>  V<ch>,<mV>  A<ch>,<dac>  E<ch>,<line>   immediate (legacy replies)");
  Serial.println("#   B / C                           recalibrate ADC / current+voltage offsets");
  Serial.println("#   D / D?                          print offsets (human / CSV)");
  Serial.println("#   P                               save slots 0-9 + triggers to EEPROM");
  Serial.println("#   DUMP / STAT / IDN               session export / engine status / identity");
  Serial.println("#   SCREEN / SCREEN,1               what the panel shows: text / text + pixel art");
  Serial.println("#   PAGE / PAGE,<n> / PAGE?         display page: advance / select / query");
  Serial.println("#   TRIG<t>,<mode>,<s0>,<s1>,<edge> route input 0/1: 0 off, 1 joint, 2 independent,");
  Serial.println("#                                   3 stimulus marker out; edge 0 rising, 1 falling");
  Serial.println("#   R<t>,<slot>[,<out>]             legacy alias of TRIG; TRIG<t>? / R<t>? query");
  Serial.println("#   LOG? / LOG1[,<name>] / LOG0     SD measurement log: status / open / close");
  Serial.println("#   SD?                             SD file access over serial (SDLIST, SDGET, ...)");
  Serial.println("#   BENCH?                          hardware benchmarks (BENCHDAC, BENCHPIT, ...)");
  Serial.println("# Buttons: Btn0 next page, Btn1 fires IN0's route, Btn2 fires IN1's");
  ok();
}

// ------------------------------------------------------------------ dispatch

void handleLine(const char* line) {
  if (!strcmp(line, "?")) { help(); return; }

  size_t wl = 0;
  while (isalpha((unsigned char)line[wl])) wl++;
  if (wl == 0) { err("parse", "expected command word"); return; }
  const char* args = line + wl;

  if (wl == 1) {
    switch (line[0]) {
      case 'S': case 'L': case 'W': handleTrain(line[0], args); return;
      case 'M': handleM(args); return;
      case 'V': handleV(args); return;
      case 'A': handleA(args); return;
      case 'E': handleE(args); return;
      case 'B': handleB(); return;
      case 'C': handleC(); return;
      case 'D': handleD(args); return;
      case 'P': handleP(); return;
      case 'R': handleR(args); return;
      case 'T': case 'U': handleStart(line[0], args); return;
      default:
        Serial.printf("ERR %c: unknown command\n", line[0]);
        return;
    }
  }

  char word[16];
  if (wl >= sizeof(word)) { err("parse", "unknown command"); return; }
  memcpy(word, line, wl);
  word[wl] = '\0';

  if (!strcmp(word, "IDN")) {
    Protocol::printIdentity(Serial);
  } else if (!strcmp(word, "HELP")) {
    help();
  } else if (!strcmp(word, "STAT")) {
    Engine::EngineStatus s[2];
    Engine::status(0, s[0]);
    Engine::status(1, s[1]);
    Serial.printf("STAT,%d,%lu,%lu,%lu,%d,%lu,%lu,%lu\n",
                  s[0].slot, (unsigned long)s[0].nPulses,
                  (unsigned long)s[0].elapsed_us, (unsigned long)s[0].duration_us,
                  s[1].slot, (unsigned long)s[1].nPulses,
                  (unsigned long)s[1].elapsed_us, (unsigned long)s[1].duration_us);
  } else if (!strcmp(word, "READ")) {
    handleRead(args);
  } else if (!strcmp(word, "DELAY")) {
    handleDelay(args);
  } else if (!strcmp(word, "DT")) {
    handleDt(args);
  } else if (!strcmp(word, "CAL")) {
    handleCal(args);
  } else if (!strcmp(word, "CALDEF")) {
    handleCalDef();
  } else if (!strcmp(word, "CLK")) {
    handleClk(args);
  } else if (!strcmp(word, "SCREEN")) {
    // `SCREEN` is the composed text; `SCREEN,1` adds the 4 KB pixel block.
    // parseFields is strict about the separating comma, which every other call
    // site skips with a `p + 1`; there is nothing to skip when there are no
    // arguments at all.
    long v;
    UiMenu::dumpScreen((parseFields(skipComma(args), &v, 1) == 1) && v != 0);
  } else if (!strcmp(word, "PAGE")) {
    handlePage(args);
  } else if (!strcmp(word, "ENV")) {
    handleEnv(args);
  } else if (!strcmp(word, "MEAS")) {
    handleMeas(args);
  } else if (!strcmp(word, "TRIG")) {
    handleTrig(args);
  } else if (!strcmp(word, "DUMP")) {
    handleDump();
  } else if (!strcmp(word, "LOG")) {
    handleLog(args);
  } else if (!strcmp(word, "SD")) {
    sdGroupList();
  } else if (!strcmp(word, "SDINFO")) {
    long v;
    // Was `parseFields(args, ...)`, which never matched: `args` still carries
    // the comma and parseFields refuses one, so `SDINFO,1` silently behaved
    // exactly like a bare `SDINFO` and never scanned the used space.
    bool withUsed = (parseFields(skipComma(args), &v, 1) == 1) && v != 0;
    if (withUsed && Engine::anyActive())
      err("SDINFO", "the used-space scan takes seconds — not while a train runs");
    else
      SdLog::info(withUsed);
  } else if (!strcmp(word, "SDLIST")) {
    const char* p = args;
    char dir[80];
    SdLog::list(nextToken(p, dir, sizeof dir) ? dir : nullptr);
  } else if (!strcmp(word, "SDGET")) {
    handleSdGet(args);
  } else if (!strcmp(word, "SDDEL")) {
    handleSdDel(args);
  } else if (!strncmp(word, "BENCH", 5)) {
    // `BENCHDAC2` is the one command word carrying a digit, and the leading
    // alphabetic run above stopped in front of it — so the sub-command is
    // re-scanned over alphanumerics. Without this, BENCHDAC2,2000 ran BENCHDAC
    // with a repetition count of 2 and said so only in its own reply.
    // "BENCH" and "BENCH?" leave sub empty, which lists the group.
    char sub[16];
    size_t sl = 0;
    const char* q = line + 5;
    while (isalnum((unsigned char)*q) && sl + 1 < sizeof sub) sub[sl++] = *q++;
    sub[sl] = '\0';
    benchDispatch(sub, q);
  } else {
    Serial.printf("ERR %s: unknown command\n", word);
  }
}

} // namespace Commands
