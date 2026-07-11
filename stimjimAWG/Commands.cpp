//    stimjimAWG — command handlers. Phase 2: S/L/W with atomic staging, `?`
//    queries + round-trip serializers, byte-exact M/V/A/E (BIST contract),
//    B/C/D/P, ENV/MEAS, DUMP; plus the Phase-1 BENCH group. Phase 3: T/U
//    start/stop (legacy reply lines), completion-ring drain + result summary,
//    READ manual measurement, live STAT. TRIG/R setters arrive in Phase 8,
//    LOG in Phase 7. GPL-3.0-or-later.

#include "Protocol.h"
#include "Config.h"
#include "FastIO.h"
#include "Engine.h"
#include "TrainStore.h"
#include "Triggers.h"
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

static inline uint32_t cycToNs(uint32_t cyc) {          // 1 cycle = 25/3 ns
  return (cyc * 25u + 1u) / 3u;
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
  Serial.printf("  meas: what0 %u, what1 %u, when %u, stage %d, report %u\r\n",
                t.meas.what0, t.meas.what1, t.meas.when, t.meas.stage, t.meas.report);
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
  TrainStore::commit(idx, staged);

  char line[SJ_SERIALIZE_MAX];
  TrainStore::serializeTrain(idx, staged, line, sizeof line);
  Serial.println(line);   // echo the canonical round-trip line of what was stored
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
  if (*p != ',') { err("MEAS", "need MEAS<idx>,<what0>,<what1>,<when>,<stage>[,<report>] or MEAS<idx>?"); return; }

  long v[5];
  int8_t n = parseFields(p + 1, v, 5);
  if (n < 4) { err("MEAS", "need <what0>,<what1>,<when>,<stage>[,<report>]"); return; }
  if (v[0] < 0 || v[1] < 0 || v[2] < 0 || v[3] < -1 || (n == 5 && v[4] < 0)) {
    err("MEAS", "fields must be non-negative (stage may be -1 = all)");
    return;
  }
  // keep the narrow casts below honest (detailed validation in validateMeas)
  if (v[0] > 3 || v[1] > 3 || v[2] > 3 || v[3] >= SJ_MAX_STAGES || (n == 5 && v[4] > 3)) {
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
  m.report = (n == 5) ? (uint8_t)v[4] : 0;
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

// ------------------------------------------------- T / U (start/stop, Phase 3)

// Completed trains per boot — the legacy `Train #<n> complete.` counter.
static uint32_t trainCount = 0;

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
    Engine::stopTrain(eng);
    if (mask & 1) modeShadow[0] = 3;
    if (mask & 2) modeShadow[1] = 3;
    Serial.print("Forcing "); Serial.print(letter); Serial.println(" train to stop");
    return;
  }
  if (v >= SJ_NUM_SLOTS) {
    Serial.println("Invalid PulseTrain index.");   // legacy byte-exact
    return;
  }

  char errbuf[SJ_MSG_MAX];
  if (!Engine::startTrain(eng, (uint8_t)v, TrainStore::slotConst((uint8_t)v),
                          errbuf, sizeof errbuf)) {
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
    Serial.print("Train #"); Serial.print(trainCount);
    Serial.print(" complete. Delivered "); Serial.print(c.nPulses);
    Serial.println(" pulses.");
    // Phase 7 replaces this note with MSUM summary lines (protocol §4)
    Serial.println("Note: no measurement carried out.");
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
  modeShadow[0] = modeShadow[1] = 3;
  ok();                            // legacy printed nothing — see protocol §6
}

static void handleC() {
  if (Engine::anyActive()) { err("C", "calibration needs an idle engine — stop trains first"); return; }
  warn("C", "output will ramp");   // getVoltageOffsets sweeps a DAC ramp on the outputs
  Stimjim.getCurrentOffsets();
  Stimjim.getVoltageOffsets();
  FastIO::acquireBus();
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
  if (*p == '?') {
    char line[48];
    serializeTrig((uint8_t)t, line, sizeof line);
    Serial.println(line);
    return;
  }
  err("TRIG", "setter not implemented yet (Phase 8) — query with TRIG<t>?");
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
  err("R", "setter not implemented yet (Phase 8) — query with R<t>?");
}

// --------------------------------------------------------------------- DUMP

static void handleDump() {
  Serial.printf("# %s fw=%s proto=%d session dump — paste back to restore\n",
                SJ_FW_NAME, SJ_FW_VERSION, SJ_PROTO_VERSION);
  char line[SJ_SERIALIZE_MAX];
  for (uint8_t idx = 0; idx < SJ_NUM_SLOTS; idx++) {
    const TrainDef& t = TrainStore::slotConst(idx);
    if (!TrainStore::isDefaultTrain(t)) {
      TrainStore::serializeTrain(idx, t, line, sizeof line);
      Serial.println(line);
    }
    // a W line already carries its envelope; S/L need a separate ENV line
    if (t.type != SINE && !TrainStore::isDefaultEnv(t.env)) {
      TrainStore::serializeEnv(idx, t.env, line, sizeof line);
      Serial.println(line);
    }
    if (!TrainStore::isDefaultMeas(t)) {
      TrainStore::serializeMeas(idx, t.meas, line, sizeof line);
      Serial.println(line);
    }
  }
  // commented until the TRIG setter exists (Phase 8) so paste-back stays clean
  for (uint8_t t = 0; t < 2; t++) {
    serializeTrig(t, line, sizeof line);
    Serial.printf("# %s\n", line);
  }
  ok();
}

// -------------------------------------------------------------------- BENCH

static void benchList() {
  Serial.println("# BENCH group (Phase 1) — timing in CPU cycles (120/us) and ns");
  Serial.println("# BENCHDAC[,n]                 dacProgram single channel (no latch)");
  Serial.println("# BENCHDAC2[,n]                dacProgramBoth (no latch)");
  Serial.println("# BENCHLATCH[,n]               dacLatch(0b11) pulse");
  Serial.println("# BENCHADC[,ch,line,n]         adcRead, line pre-selected");
  Serial.println("# BENCHSW[,ch,n]               adcSelectLine+adcRead pair (line-switch cost)");
  Serial.println("# BENCHMISO[,n]                alternating ch0/ch1 reads (MISO mux swap)");
  Serial.println("# BENCHCYC[,n]                 cycles64() overhead");
  Serial.println("# BENCHK[,n]                   K_RELOAD recalibration, residual stats");
  Serial.println("# BENCHPIT,period_us,n[,preload_us]  PIT wake/latch jitter vs deadline");
  Serial.println("# BENCHSQ,ch,code,half_us,n    square wave, FastIO path (scope A/B)");
  Serial.println("# BENCHSQL,ch,code,half_us,n   square wave, legacy Stimjim.writeToDac path");
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
  Serial.println("# stimjimAWG commands (Phase 4) — details: docs/serial-protocol.md");
  Serial.println("#   S<i>,m0,m1,per,dur;a0,a1,d;...  rectangular train (slots 0-99, <=10 stages)");
  Serial.println("#   L<i>,...                        same syntax, linear ramps (0-dur stage = jump)");
  Serial.println("#   W<i>,m0,m1,per,dur;amp;freq;phase[;env]  sine train (triplets, decimals in Hz ok)");
  Serial.println("#   modes: 0 V, 1 I, 2/3 channel not driven; 90/91 V/I without measurement");
  Serial.println("#   S<i> / L<i> / W<i>              human parameter dump; append ? for the canonical line");
  Serial.println("#   T<i> / T-1, U<i> / U-1          start/stop engine 0 / 1 (S/L slots; W in Phase 5)");
  Serial.println("#   ENV<i>,in,out[,shape]           amplitude envelope; ENV<i>?");
  Serial.println("#   MEAS<i>,w0,w1,when,stage[,rep]  measurement config; MEAS<i>?");
  Serial.println("#   READ<ch>[,n]                    manual averaged V+I read (mean and std dev)");
  Serial.println("#   M<ch>,<mode>  V<ch>,<mV>  A<ch>,<dac>  E<ch>,<line>   immediate (legacy replies)");
  Serial.println("#   B / C                           recalibrate ADC / current+voltage offsets");
  Serial.println("#   D / D?                          print offsets (human / CSV)");
  Serial.println("#   P                               save slots 0-9 + triggers to EEPROM");
  Serial.println("#   DUMP / STAT / IDN               session export / engine status / identity");
  Serial.println("#   TRIG<t>? / R<t>?                trigger routing queries (setters in Phase 8)");
  Serial.println("#   BENCH?                          hardware benchmarks (BENCHDAC, BENCHPIT, ...)");
  Serial.println("# Not yet available: W playback (Phase 5), MEAS execution + LOG (Phase 7), TRIG/R setters (Phase 8)");
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
    Serial.printf("IDN,%s,%s,fw=%s,proto=%d\n",
                  SJ_FW_NAME, SJ_HW_NAME, SJ_FW_VERSION, SJ_PROTO_VERSION);
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
  } else if (!strcmp(word, "ENV")) {
    handleEnv(args);
  } else if (!strcmp(word, "MEAS")) {
    handleMeas(args);
  } else if (!strcmp(word, "TRIG")) {
    handleTrig(args);
  } else if (!strcmp(word, "DUMP")) {
    handleDump();
  } else if (!strcmp(word, "LOG")) {
    err("LOG", "SD logging arrives in Phase 7");
  } else if (!strncmp(word, "BENCH", 5)) {
    const char* sub = word + 5;
    // allow "BENCH?" — the '?' lands in args, sub is empty
    benchDispatch(sub, args);
  } else {
    Serial.printf("ERR %s: unknown command\n", word);
  }
}

} // namespace Commands
