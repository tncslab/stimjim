//    stimjimAWG — TrainStore implementation: slot storage, S/L/W body parser
//    with staging (atomic), ENV/MEAS validation, round-trip serializers,
//    versioned EEPROM. Host-testable except the #ifdef ARDUINO block at the end.
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
  t.mode0       = 3;          // grounded = not driven (original numbering)
  t.mode1       = 3;
  t.period_us   = 10000;
  t.duration_us = 500000;
  t.delay_us    = 0;          // fire immediately on the start request
  t.dt_us       = 0;          // ramp sample interval: use the build default
  t.nStages     = 0;
  t.meas.what0  = 3;          // both V and I
  t.meas.what1  = 3;
  t.meas.when   = 0;          // auto default for S/L (3 = both peaks for SINE), see defaultWhen()
  t.meas.stage  = -1;         // all stages
  t.meas.report = 0;          // end-of-train summary only
  t.meas.fit    = SJ_FIT_ROTATE;  // rotate reads over repetitions rather than refuse a point
}

void begin() {
  for (auto& s : slots) slotDefault(s);
}

TrainDef&       slot(uint8_t idx)      { return slots[idx]; }
const TrainDef& slotConst(uint8_t idx) { return slots[idx]; }

void commit(uint8_t idx, const TrainDef& staged) { slots[idx] = staged; }

uint8_t defaultWhen(uint8_t type) { return type == SINE ? 3 : 0; }

bool isDefaultTrain(const TrainDef& t) {
  return t.type == PIECEWISE_HOLD && t.mode0 == 3 && t.mode1 == 3 &&
         t.period_us == 10000 && t.duration_us == 500000 &&
         t.delay_us == 0 && t.dt_us == 0 && t.nStages == 0;
}
bool isDefaultEnv(const EnvDef& e) {
  return e.rampIn_us == 0 && e.rampOut_us == 0 && e.shape == 0;
}
bool isDefaultMeas(const TrainDef& t) {
  return t.meas.what0 == 3 && t.meas.what1 == 3 &&
         t.meas.when == defaultWhen(t.type) && t.meas.stage == -1 &&
         t.meas.report == 0 && t.meas.fit == SJ_FIT_ROTATE;
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
// mode (per-channel modes, already normalized to 0-3; a0 belongs to mode0,
// a1 to mode1). Undriven channels (2/3) carry no output — nothing to check.
static void checkAmplitude(long a, uint8_t mode, char* warn, size_t warnsz) {
  long mag = a < 0 ? -a : a;
  if (mode == 1 && mag > WARN_LIMIT_UA)
    addWarn(warn, warnsz, "amplitudes above 3000 uA convert incorrectly on the DAC");
  if (mode == 0 && mag > DAC_FULLSCALE_MV)
    addWarn(warn, warnsz, "voltage amplitude exceeds DAC full scale (~14988 mV), will clip");
}

// Normalize one train-line mode field (protocol §2): 0-3 pass through, 90/91
// mean "voltage/current, measurement disabled" and are stored as 0/1 with the
// channel's meas.what forced to 0; a plain 0/1 promotes a stored what of 0
// back to the default 3 (explicit 1/2 refinements are preserved).
static bool normalizeMode(long m, uint8_t& mode, uint8_t& what) {
  if (m == 90 || m == 91) { mode = (uint8_t)(m - 90); what = 0; return true; }
  if (m < 0 || m > 3) return false;
  mode = (uint8_t)m;
  if (m <= 1 && what == 0) what = 3;
  return true;
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
  if (!normalizeMode(m0, staged.mode0, staged.meas.what0) ||
      !normalizeMode(m1, staged.mode1, staged.meas.what1)) {
    setMsg(err, errsz, "mode must be 0-3 or 90/91 (V/I without measurement)");
    return false;
  }
  if (!expect(p, ',') || !scanULong(p, period))   { setMsg(err, errsz, "bad or missing period_us"); return false; }
  if (!expect(p, ',') || !scanULong(p, duration)) { setMsg(err, errsz, "bad or missing duration_us"); return false; }
  if (period == 0) { setMsg(err, errsz, "period_us must be > 0"); return false; }
  staged.period_us   = (uint32_t)period;
  staged.duration_us = (uint32_t)duration;

  // ---- optional 6th header field: post-trigger delay (protocol §2).
  // A legacy line ends the header here (next character is ';' or end of line),
  // so a ',' at this point can only be the new field — the syntax extension is
  // unambiguous. Like the W envelope triplet, omitting it *resets* the delay
  // to 0: an S/L/W line always fully defines its own header, so replaying an
  // old script reproduces the old behaviour exactly.
  p = sksp(p);
  if (*p == ',') {
    p++;
    unsigned long delay;
    if (!scanULong(p, delay)) { setMsg(err, errsz, "bad delay_us"); return false; }
    if (delay > SJ_MAX_DELAY_US) { setMsg(err, errsz, "delay_us exceeds 2000000000 us (2000 s)"); return false; }
    staged.delay_us = (uint32_t)delay;
  } else {
    staged.delay_us = 0;
  }

  // ---- optional 7th header field: the ramp sample interval, `L` only. It is
  // positional, so a slot that wants a custom interval and no delay writes the
  // delay as 0. Omitting it resets the interval to the build default, for the
  // same reason omitting the delay resets that: a waveform line fully defines
  // its own header.
  staged.dt_us = 0;
  p = sksp(p);
  if (*p == ',') {
    p++;
    unsigned long dt;
    if (staged.type != PIECEWISE_RAMP) {
      setMsg(err, errsz, "only an L line takes a 7th header field (ramp sample interval)");
      return false;
    }
    if (!scanULong(p, dt)) { setMsg(err, errsz, "bad dt_us"); return false; }
    if (dt < SJ_MIN_DT_US || dt > SJ_MAX_DT_US) {
      setMsg(err, errsz, "dt_us must be 2..1000000 us (0 = the build default, omit the field)");
      return false;
    }
    staged.dt_us = (uint32_t)dt;
  }

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

  // MEAS `when`/`stage` follow the waveform family on a type change (protocol
  // §4 auto-coercion); a same-family redefinition keeps explicit choices.
  if (staged.type == SINE) {
    if (staged.meas.when < 1 || staged.meas.when > 3) staged.meas.when = 3;
    staged.meas.stage = -1;
  } else {
    staged.meas.when = 0;
    // a preserved per-stage selection must fit the (possibly shorter) new train
    if (staged.meas.stage >= (int8_t)staged.nStages) {
      setMsg(err, errsz, "stored MEAS stage exceeds the new stage count (reset MEAS first)");
      return false;
    }
  }

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
  if (m.report > 3)               return "report must be 0-3 (+1 stream, +2 SD)";
  if (m.fit > SJ_FIT_ROTATE)
    return "fit must be 0 (refuse a point that does not fit) or 1 (rotate over repetitions)";
  if (t.type == SINE) {
    if (m.when < 1 || m.when > 3) return "sine slots require when 1 (+peak), 2 (-peak) or 3 (both)";
    if (m.stage != -1)            return "sine slots have no stages — stage must be -1";
  } else {
    if (m.when != 0)              return "S/L slots require when=0 (near stage end)";
    if (m.stage < -1 || m.stage >= (int8_t)t.nStages)
      return "stage must be -1 (all) or a valid stage index";
  }
  // Channels the train does not drive (mode 2/3) are never measured — a
  // non-zero what there is meaningless: accepted with WARN per protocol §4.
  if ((t.mode0 >= 2 && m.what0 != 0) || (t.mode1 >= 2 && m.what1 != 0))
    addWarn(warn, warnsz, "channel not driven (mode 2/3) — nothing to measure");
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

// Canonical mode rendering (protocol §2): a driven channel with measurement
// disabled round-trips as 90/91 so the flag survives S<idx>?/DUMP replay.
static unsigned modeOut(uint8_t mode, uint8_t what) {
  return (mode <= 1 && what == 0) ? 90u + mode : mode;
}

void serializeTrain(uint8_t idx, const TrainDef& t, char* buf, size_t n) {
  char letter = (t.type == SINE) ? 'W' : (t.type == PIECEWISE_RAMP) ? 'L' : 'S';
  size_t o = snprintf(buf, n, "%c%u,%u,%u,%lu,%lu", letter, idx,
                      modeOut(t.mode0, t.meas.what0), modeOut(t.mode1, t.meas.what1),
                      (unsigned long)t.period_us, (unsigned long)t.duration_us);
  // The optional delay and ramp-interval fields are emitted only when set, so
  // a slot that uses neither still serializes to a line an older firmware
  // would accept. Round-trip stays exact: an absent field parses back as 0.
  // They are positional, so a set interval forces the delay field out too.
  if (t.delay_us || t.dt_us)
    o += snprintf(buf + o, n - o, ",%lu", (unsigned long)t.delay_us);
  if (t.dt_us)
    o += snprintf(buf + o, n - o, ",%lu", (unsigned long)t.dt_us);
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

void serializeDelay(uint8_t idx, uint32_t delay_us, char* buf, size_t n) {
  snprintf(buf, n, "DELAY%u,%lu", idx, (unsigned long)delay_us);
}

void serializeDt(uint8_t idx, uint32_t dt_us, char* buf, size_t n) {
  snprintf(buf, n, "DT%u,%lu", idx, (unsigned long)dt_us);
}

void serializeEnv(uint8_t idx, const EnvDef& e, char* buf, size_t n) {
  snprintf(buf, n, "ENV%u,%lu,%lu,%u", idx,
           (unsigned long)e.rampIn_us, (unsigned long)e.rampOut_us, e.shape);
}

void serializeMeas(uint8_t idx, const MeasDef& m, char* buf, size_t n) {
  snprintf(buf, n, "MEAS%u,%u,%u,%u,%d,%u,%u", idx, m.what0, m.what1, m.when,
           m.stage, m.report, m.fit);
}

// --------------------------------------------------------------------- EEPROM

#ifdef ARDUINO
// EEPROM.put() beyond E2END is a silent no-op in the Teensy core, so an image
// that does not fit would "save" and then restore as garbage. Catch it at
// compile time on whatever board is selected: 4096 B on Teensy 3.5 and 4.1,
// but only 1080 B on a Teensy 4.0, which cannot hold ten slots.
static_assert(sizeof(EepromImage) <= (size_t)E2END + 1,
              "EepromImage exceeds this board's EEPROM — rebuild with a smaller "
              "-DSJ_EEPROM_SLOTS (a Teensy 4.0 fits about 6)");

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
static EepromImage eeImg;
static const size_t EE_CRC_SPAN = sizeof(EepromImage) - offsetof(EepromImage, slots);

void eepromSave(const TriggerRoute trig[2]) {
  eeImg.magic   = SJ_EEPROM_MAGIC;
  eeImg.version = SJ_EEPROM_VERSION;
  memcpy(eeImg.slots, slots, sizeof(eeImg.slots));
  eeImg.trig[0] = trig[0];
  eeImg.trig[1] = trig[1];
  eeImg.cal     = Cal::live();
  eeImg.crc = crc16((const uint8_t*)&eeImg + offsetof(EepromImage, slots), EE_CRC_SPAN);
  EEPROM.put(0, eeImg);
}

bool eepromRestore(TriggerRoute trigOut[2], bool* calRejected) {
  if (calRejected) *calRejected = false;
  EEPROM.get(0, eeImg);
  if (eeImg.magic != SJ_EEPROM_MAGIC || eeImg.version != SJ_EEPROM_VERSION)
    return false;
  if (eeImg.crc != crc16((const uint8_t*)&eeImg + offsetof(EepromImage, slots), EE_CRC_SPAN))
    return false;
  memcpy(slots, eeImg.slots, sizeof(eeImg.slots));
  trigOut[0] = eeImg.trig[0];
  trigOut[1] = eeImg.trig[1];
  // A stored budget that no longer validates is dropped and the build defaults
  // stay in force: the image may predate a Config.h change that moved a floor,
  // and arming from an inconsistent budget is worse than losing a calibration.
  // The slots are kept either way — they are checked by the same CRC. Nothing
  // in this module prints, so the caller reports it.
  if (Cal::validate(eeImg.cal) == nullptr) Cal::set(eeImg.cal);
  else if (calRejected) *calRejected = true;
  return true;
}
#endif // ARDUINO

} // namespace TrainStore
