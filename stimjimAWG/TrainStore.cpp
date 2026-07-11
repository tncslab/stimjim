//    stimjimAWG — TrainStore implementation: slot storage, S/L/W body parser
//    with staging (atomic), ENV/MEAS validation, round-trip serializers,
//    EEPROM v2. Host-testable except the #ifdef ARDUINO block at the end.
//    GPL-3.0-or-later; see Config.h header.

#include "TrainStore.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#ifdef ARDUINO
#include <EEPROM.h>
#endif

namespace TrainStore {

static TrainDef slots[SJ_NUM_SLOTS];

// DAC full-scale in engineering units (from Stimjim.h conversion factors,
// duplicated as plain numbers so this file stays host-compilable):
// 32767 * 0.4574 mV/DAC ~= 14988 mV; 32767 * 0.1017 uA/DAC ~= 3332 uA.
// The 3000 uA warning threshold is the known-bad conversion limit
// (protocol §2 / stimjimPulser header warning), reached before full scale.
static const long DAC_FULLSCALE_MV = 14988;
static const long WARN_LIMIT_UA    = 3000;

void slotDefault(TrainDef& t) {
  memset(&t, 0, sizeof(t));   // also zeroes union padding => deterministic EEPROM images
  t.type        = PIECEWISE_HOLD;
  t.mode0       = 5;          // grounded
  t.mode1       = 5;
  t.period_us   = 10000;
  t.duration_us = 500000;
  t.nStages     = 0;
  t.meas.what0  = 3;          // both V and I
  t.meas.what1  = 3;
  t.meas.when   = 1;          // auto default for S/L (2 for SINE), see defaultWhen()
  t.meas.report = 0;          // end-of-train summary only
}

void begin() {
  for (auto& s : slots) slotDefault(s);
}

TrainDef&       slot(uint8_t idx)      { return slots[idx]; }
const TrainDef& slotConst(uint8_t idx) { return slots[idx]; }

void commit(uint8_t idx, const TrainDef& staged) { slots[idx] = staged; }

uint8_t defaultWhen(uint8_t type) { return type == SINE ? 2 : 1; }

bool isDefaultTrain(const TrainDef& t) {
  return t.type == PIECEWISE_HOLD && t.mode0 == 5 && t.mode1 == 5 &&
         t.period_us == 10000 && t.duration_us == 500000 && t.nStages == 0;
}
bool isDefaultEnv(const EnvDef& e) {
  return e.rampIn_us == 0 && e.rampOut_us == 0 && e.shape == 0;
}
bool isDefaultMeas(const TrainDef& t) {
  return t.meas.what0 == 3 && t.meas.what1 == 3 &&
         t.meas.when == defaultWhen(t.type) && t.meas.report == 0;
}

// -------------------------------------------------------------- scan helpers

static const char* sksp(const char* p) {
  while (*p == ' ' || *p == '\t') p++;
  return p;
}

static bool isDigitCh(char c) { return c >= '0' && c <= '9'; }

// Signed integer (amplitudes). strtol handles sign; strict about presence.
static bool scanLong(const char*& p, long& v) {
  p = sksp(p);
  char* end;
  v = strtol(p, &end, 10);
  if (end == p) return false;
  p = end;
  return true;
}

// Unsigned integer (periods/durations, full uint32 range) — a leading '-'
// must fail loudly instead of wrapping through strtoul.
static bool scanULong(const char*& p, unsigned long& v) {
  p = sksp(p);
  if (*p == '-') return false;
  char* end;
  v = strtoul(p, &end, 10);
  if (end == p) return false;
  p = end;
  return true;
}

// Decimal with up to 3 fraction digits -> milli-units (W frequencies/phases;
// protocol §2: "decimals accepted, stored as mHz"). A nonzero 4th decimal
// digit fails: silently rounding sub-mHz requests would violate round-trip
// exactness and the no-silent-coercion rule.
static bool scanMilli(const char*& p, long long& mv) {
  p = sksp(p);
  const char* q = p;
  bool neg = false;
  if (*q == '+' || *q == '-') { neg = (*q == '-'); q++; }
  long long ip = 0;
  bool any = false;
  while (isDigitCh(*q)) {
    ip = ip * 10 + (*q - '0');
    if (ip > 4400000000LL) return false;   // > uint32 mHz range, reject early
    q++; any = true;
  }
  long long frac = 0;
  int nd = 0;
  if (*q == '.') {
    q++;
    while (isDigitCh(*q)) {
      if (nd < 3)          { frac = frac * 10 + (*q - '0'); nd++; }
      else if (*q != '0')  return false;   // sub-milli resolution requested
      q++; any = true;
    }
  }
  if (!any) return false;
  while (nd < 3) { frac *= 10; nd++; }
  mv = ip * 1000 + frac;
  if (neg) mv = -mv;
  p = q;
  return true;
}

static bool expect(const char*& p, char c) {
  p = sksp(p);
  if (*p != c) return false;
  p++;
  return true;
}

static void setMsg(char* buf, size_t n, const char* msg) {
  if (buf && n) snprintf(buf, n, "%s", msg);
}

// Append an accepted-with-caveat note; multiple warnings are "; "-joined.
// A message already present is not repeated (e.g. several stages over limit).
static void addWarn(char* warn, size_t n, const char* msg) {
  if (!warn || !n) return;
  if (strstr(warn, msg)) return;
  size_t len = strlen(warn);
  snprintf(warn + len, n - len, "%s%s", len ? "; " : "", msg);
}

// -------------------------------------------------------------- train parser

// Warn about amplitudes the DAC cannot faithfully produce in the channel's
// mode (per-channel modes; a0 belongs to mode0, a1 to mode1).
static void checkAmplitude(long a, uint8_t mode, char* warn, size_t warnsz) {
  long mag = a < 0 ? -a : a;
  bool current = (mode == 1 || mode == 3);
  if (current && mag > WARN_LIMIT_UA)
    addWarn(warn, warnsz, "amplitudes above 3000 uA convert incorrectly on the DAC");
  if (!current && mode != 4 && mode != 5 && mag > DAC_FULLSCALE_MV)
    addWarn(warn, warnsz, "voltage amplitude exceeds DAC full scale (~14988 mV), will clip");
}

bool parseTrainBody(char letter, const char* body, const TrainDef& current,
                    TrainDef& staged, char* err, size_t errsz,
                    char* warn, size_t warnsz) {
  if (warn && warnsz) warn[0] = '\0';

  slotDefault(staged);
  staged.type = (letter == 'W') ? SINE
              : (letter == 'L') ? PIECEWISE_RAMP : PIECEWISE_HOLD;
  // Preserve per-slot config not carried by the line (protocol §2): ENV for
  // S/L (a W line always defines/resets its own), MEAS for all types.
  staged.env  = current.env;
  staged.meas = current.meas;

  const char* p = body;

  // ---- common header: ,mode0,mode1,period_us,duration_us
  long m0, m1;
  unsigned long period, duration;
  if (!expect(p, ',') || !scanLong(p, m0)) { setMsg(err, errsz, "bad or missing mode0"); return false; }
  if (!expect(p, ',') || !scanLong(p, m1)) { setMsg(err, errsz, "bad or missing mode1"); return false; }
  if (m0 < 0 || m0 > 5 || m1 < 0 || m1 > 5) { setMsg(err, errsz, "mode out of range 0-5"); return false; }
  if (!expect(p, ',') || !scanULong(p, period))   { setMsg(err, errsz, "bad or missing period_us"); return false; }
  if (!expect(p, ',') || !scanULong(p, duration)) { setMsg(err, errsz, "bad or missing duration_us"); return false; }
  if (period == 0) { setMsg(err, errsz, "period_us must be > 0"); return false; }
  staged.mode0       = (uint8_t)m0;
  staged.mode1       = (uint8_t)m1;
  staged.period_us   = (uint32_t)period;
  staged.duration_us = (uint32_t)duration;

  if (staged.type != SINE) {
    // ---- S/L stage triplets: ;a0,a1,dur_us  (0-10 of them; 0 = legacy "empty train")
    unsigned long durSum = 0;
    for (;;) {
      p = sksp(p);
      if (!*p) break;
      if (*p != ';') { setMsg(err, errsz, "expected ';' before stage triplet"); return false; }
      p++;
      p = sksp(p);
      if (!*p) break;   // tolerate a trailing ';' like the legacy strtok parser
      if (staged.nStages >= SJ_MAX_STAGES) {
        setMsg(err, errsz, "too many stages (max 10)");   // legacy overflowed silently here
        return false;
      }
      StageDef& st = staged.stages[staged.nStages];
      long a0, a1;
      unsigned long d;
      if (!scanLong(p, a0))                    { setMsg(err, errsz, "bad stage amplitude a0"); return false; }
      if (!expect(p, ',') || !scanLong(p, a1)) { setMsg(err, errsz, "bad stage amplitude a1"); return false; }
      if (!expect(p, ',') || !scanULong(p, d)) { setMsg(err, errsz, "bad stage dur_us"); return false; }
      st.a0 = a0;
      st.a1 = a1;
      st.dur_us = (uint32_t)d;
      durSum += d;
      checkAmplitude(a0, staged.mode0, warn, warnsz);
      checkAmplitude(a1, staged.mode1, warn, warnsz);
      staged.nStages++;
    }
    if (durSum > staged.period_us)
      addWarn(warn, warnsz, "stage durations exceed period_us");
  } else {
    // ---- W triplets: ;amp0,amp1,burst ;f0,f1,0 ;ph0,ph1,0 [;rampIn,rampOut,shape]
    long amp0, amp1, resv;
    unsigned long burst, rampIn, rampOut, shape;
    long long f0, f1, ph0, ph1;

    if (!expect(p, ';') || !scanLong(p, amp0))   { setMsg(err, errsz, "bad amplitude triplet (amp0)"); return false; }
    if (!expect(p, ',') || !scanLong(p, amp1))   { setMsg(err, errsz, "bad amplitude triplet (amp1)"); return false; }
    if (!expect(p, ',') || !scanULong(p, burst)) { setMsg(err, errsz, "bad amplitude triplet (burst_us)"); return false; }

    if (!expect(p, ';') || !scanMilli(p, f0))    { setMsg(err, errsz, "bad frequency triplet (freq0)"); return false; }
    if (!expect(p, ',') || !scanMilli(p, f1))    { setMsg(err, errsz, "bad frequency triplet (freq1)"); return false; }
    if (!expect(p, ',') || !scanLong(p, resv))   { setMsg(err, errsz, "frequency triplet needs its 3rd (reserved) field"); return false; }

    if (!expect(p, ';') || !scanMilli(p, ph0))   { setMsg(err, errsz, "bad phase triplet (phase0)"); return false; }
    if (!expect(p, ',') || !scanMilli(p, ph1))   { setMsg(err, errsz, "bad phase triplet (phase1)"); return false; }
    if (!expect(p, ',') || !scanLong(p, resv))   { setMsg(err, errsz, "phase triplet needs its 3rd (reserved) field"); return false; }

    p = sksp(p);
    if (*p == ';') {
      p++;
      if (!scanULong(p, rampIn))                     { setMsg(err, errsz, "bad envelope triplet (rampIn_us)"); return false; }
      if (!expect(p, ',') || !scanULong(p, rampOut)) { setMsg(err, errsz, "bad envelope triplet (rampOut_us)"); return false; }
      if (!expect(p, ',') || !scanULong(p, shape))   { setMsg(err, errsz, "bad envelope triplet (shape)"); return false; }
    } else {
      rampIn = rampOut = shape = 0;   // omitted 5th triplet resets the envelope (protocol §2)
    }

    if (f0 < 0 || f1 < 0) {
      // The legacy firmware accepted negative Hz (yielding a time-reversed
      // sine through table-index wraparound); rejected here — protocol §6.
      setMsg(err, errsz, "negative frequency not supported");
      return false;
    }
    if (f0 > 0xFFFFFFFFLL || f1 > 0xFFFFFFFFLL) { setMsg(err, errsz, "frequency out of range"); return false; }
    if (ph0 < -2000000000LL || ph0 > 2000000000LL ||
        ph1 < -2000000000LL || ph1 > 2000000000LL) { setMsg(err, errsz, "phase out of range"); return false; }
    if (shape > 0) { setMsg(err, errsz, "envelope shape 1 (raised-cosine) is reserved, use 0"); return false; }

    staged.sine.amp0       = amp0;
    staged.sine.amp1       = amp1;
    staged.sine.burst_us   = (uint32_t)burst;
    staged.sine.freq0_mHz  = (uint32_t)f0;
    staged.sine.freq1_mHz  = (uint32_t)f1;
    staged.sine.phase0_mdeg = (int32_t)ph0;
    staged.sine.phase1_mdeg = (int32_t)ph1;
    staged.env.rampIn_us   = (uint32_t)rampIn;
    staged.env.rampOut_us  = (uint32_t)rampOut;
    staged.env.shape       = 0;

    checkAmplitude(amp0, staged.mode0, warn, warnsz);
    checkAmplitude(amp1, staged.mode1, warn, warnsz);
    if (burst > staged.period_us)
      addWarn(warn, warnsz, "burst_us exceeds period_us");
  }

  p = sksp(p);
  if (*p) { setMsg(err, errsz, "unexpected text after last field"); return false; }

  // Envelope must fit the (possibly new) duration — for S/L this also guards
  // the ENV preserved from the previous definition (fail loudly, don't clamp).
  if ((uint64_t)staged.env.rampIn_us + staged.env.rampOut_us > staged.duration_us) {
    setMsg(err, errsz, staged.type == SINE
           ? "envelope rampIn+rampOut exceeds duration_us"
           : "stored ENV rampIn+rampOut exceeds new duration_us (reset ENV first)");
    return false;
  }

  // MEAS `when` follows the waveform family on a type change (protocol §5
  // auto-when); an explicit first-stage-only choice (0) stays valid for S/L.
  if (staged.type == SINE && staged.meas.when != 2)
    staged.meas.when = 2;
  else if (staged.type != SINE && staged.meas.when == 2)
    staged.meas.when = 1;

  return true;
}

// --------------------------------------------------------- ENV/MEAS validation

const char* validateEnv(const TrainDef& t, const EnvDef& e) {
  if (e.shape > 0)
    return "shape 1 (raised-cosine) is reserved, use 0";
  if ((uint64_t)e.rampIn_us + e.rampOut_us > t.duration_us)
    return "rampIn+rampOut exceeds the slot's duration_us";
  return nullptr;
}

const char* validateMeas(const TrainDef& t, const MeasDef& m, char* warn, size_t warnsz) {
  if (warn && warnsz) warn[0] = '\0';
  if (m.what0 > 3 || m.what1 > 3) return "what must be 0-3";
  if (m.when > 2)                 return "when must be 0-2";
  if (m.report > 3)               return "report must be 0-3 (+1 stream, +2 SD)";
  if (t.type == SINE && m.when != 2) return "sine slots require when=2 (sine peak)";
  if (t.type != SINE && m.when == 2) return "when=2 (sine peak) requires a W slot";
  // Modes 2/3 suppress measurement at plan-compile time (documented); 4/5 make
  // the request meaningless — accepted with WARN per protocol §4.
  if ((t.mode0 >= 4 && m.what0 != 0) || (t.mode1 >= 4 && m.what1 != 0))
    addWarn(warn, warnsz, "channel mode 4/5 delivers nothing to measure");
  if (m.report & 1)
    addWarn(warn, warnsz, "MDATA streaming is deferred (format frozen); summary/SD only in v1");
  return nullptr;
}

// ------------------------------------------------------------------ serializers

void milliToStr(long long milli, char* buf) {
  // avoid %lld — not guaranteed in newlib-nano printf; |milli| <= ~4.3e9 so
  // the integer part always fits an unsigned long
  unsigned long long a = (milli < 0) ? (unsigned long long)(-milli) : (unsigned long long)milli;
  unsigned long ip = (unsigned long)(a / 1000);
  unsigned fr = (unsigned)(a % 1000);
  int o = sprintf(buf, "%s%lu", (milli < 0) ? "-" : "", ip);
  if (fr) {
    sprintf(buf + o, ".%03u", fr);
    // trim trailing zeros of the fraction ("0.250" -> "0.25")
    char* e = buf + strlen(buf) - 1;
    while (*e == '0') *e-- = '\0';
    if (*e == '.') *e = '\0';
  }
}

void serializeTrain(uint8_t idx, const TrainDef& t, char* buf, size_t n) {
  char letter = (t.type == SINE) ? 'W' : (t.type == PIECEWISE_RAMP) ? 'L' : 'S';
  size_t o = snprintf(buf, n, "%c%u,%u,%u,%lu,%lu", letter, idx, t.mode0, t.mode1,
                      (unsigned long)t.period_us, (unsigned long)t.duration_us);
  if (t.type == SINE) {
    char f0[16], f1[16], p0[16], p1[16];
    milliToStr(t.sine.freq0_mHz, f0);
    milliToStr(t.sine.freq1_mHz, f1);
    milliToStr(t.sine.phase0_mdeg, p0);
    milliToStr(t.sine.phase1_mdeg, p1);
    // canonical form always carries the envelope triplet: parsing it back
    // (even 0,0,0) restores exactly this state — round-trip stable
    snprintf(buf + o, n - o, ";%ld,%ld,%lu;%s,%s,0;%s,%s,0;%lu,%lu,%u",
             (long)t.sine.amp0, (long)t.sine.amp1, (unsigned long)t.sine.burst_us,
             f0, f1, p0, p1,
             (unsigned long)t.env.rampIn_us, (unsigned long)t.env.rampOut_us, t.env.shape);
  } else {
    for (uint8_t i = 0; i < t.nStages && o < n; i++)
      o += snprintf(buf + o, n - o, ";%ld,%ld,%lu",
                    (long)t.stages[i].a0, (long)t.stages[i].a1,
                    (unsigned long)t.stages[i].dur_us);
  }
}

void serializeEnv(uint8_t idx, const EnvDef& e, char* buf, size_t n) {
  snprintf(buf, n, "ENV%u,%lu,%lu,%u", idx,
           (unsigned long)e.rampIn_us, (unsigned long)e.rampOut_us, e.shape);
}

void serializeMeas(uint8_t idx, const MeasDef& m, char* buf, size_t n) {
  snprintf(buf, n, "MEAS%u,%u,%u,%u,%u", idx, m.what0, m.what1, m.when, m.report);
}

// --------------------------------------------------------------------- EEPROM

#ifdef ARDUINO
static_assert(sizeof(EepromImageV2) <= 4096, "EepromImageV2 exceeds Teensy 3.5 EEPROM");

// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over everything after the
// header's crc field, i.e. slots + trig (WaveformDef.h contract).
static uint16_t crc16(const uint8_t* d, size_t n) {
  uint16_t crc = 0xFFFF;
  while (n--) {
    crc ^= (uint16_t)(*d++) << 8;
    for (int b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

// One static image buffer (~1.6 KB) shared by save/restore — both run from
// command/boot context only, never concurrently.
static EepromImageV2 eeImg;
static const size_t EE_CRC_SPAN = sizeof(EepromImageV2) - offsetof(EepromImageV2, slots);

void eepromSave(const TriggerRoute trig[2]) {
  eeImg.magic   = SJ_EEPROM_MAGIC;
  eeImg.version = SJ_EEPROM_VERSION;
  memcpy(eeImg.slots, slots, sizeof(eeImg.slots));
  eeImg.trig[0] = trig[0];
  eeImg.trig[1] = trig[1];
  eeImg.crc = crc16((const uint8_t*)&eeImg + offsetof(EepromImageV2, slots), EE_CRC_SPAN);
  EEPROM.put(0, eeImg);
}

bool eepromRestore(TriggerRoute trigOut[2]) {
  EEPROM.get(0, eeImg);
  if (eeImg.magic != SJ_EEPROM_MAGIC || eeImg.version != SJ_EEPROM_VERSION)
    return false;
  if (eeImg.crc != crc16((const uint8_t*)&eeImg + offsetof(EepromImageV2, slots), EE_CRC_SPAN))
    return false;
  memcpy(slots, eeImg.slots, sizeof(eeImg.slots));
  trigOut[0] = eeImg.trig[0];
  trigOut[1] = eeImg.trig[1];
  return true;
}
#endif // ARDUINO

} // namespace TrainStore
