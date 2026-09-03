//    stimjimAWG — Measure implementation: arm-time plan compilation (pure), the
//    player-ISR read/accumulate path, the MDATA ring and the MSUM summary.
//    GPL-3.0-or-later; see Config.h header.

#include "Measure.h"
#include <math.h>
#include <string.h>

namespace Measure {

// --------------------------------------------------------------- pure section

static inline uint8_t bitCount2(uint8_t v) { return (uint8_t)((v & 1) + ((v >> 1) & 1)); }

uint32_t peakSampleIndex(uint32_t phaseInit, uint32_t phaseInc, uint32_t target) {
  // Unsigned subtraction is the mod-2^32 phase distance to the target; the
  // first sample at or after the crossing is ceil(distance / increment).
  uint32_t d = target - phaseInit;
  return (uint32_t)(((uint64_t)d + phaseInc - 1) / phaseInc);
}

bool accumStats(const Accum& a, double& mean, double& sd) {
  if (a.n == 0) return false;
  mean = (double)a.sum / (double)a.n;
  if (a.n < 2) { sd = 0.0; return true; }
  // Single-pass estimator (protocol §4): exact in the integer sums, so the
  // usual catastrophic-cancellation objection applies only to the final
  // subtraction — harmless at 13-bit codes and realistic repetition counts.
  double var = ((double)a.sumsq - (double)a.sum * (double)a.sum / (double)a.n)
             / (double)(a.n - 1);
  sd = (var > 0.0) ? sqrt(var) : 0.0;
  return true;
}

void planBuild(Plan& pl, uint8_t slot, const TrainDef& def, const Geometry& g) {
  memset(&pl, 0, sizeof pl);
  pl.slot   = slot;
  pl.type   = g.type;
  pl.report = def.meas.report;

  // `what` (0 none, 1 V, 2 I, 3 both) is already a line bitmask: bit0 = line 0
  // (output voltage), bit1 = line 1 (current sense). A channel the train does
  // not drive is never measured, whatever MEAS says (protocol §4).
  pl.lines[0] = (g.chMask & 1) ? (uint8_t)(def.meas.what0 & 3) : 0;
  pl.lines[1] = (g.chMask & 2) ? (uint8_t)(def.meas.what1 & 3) : 0;
  uint8_t nReads = (uint8_t)(bitCount2(pl.lines[0]) + bitCount2(pl.lines[1]));
  if (nReads == 0) return;                 // on stays false

  pl.budgetCyc = nReads * (g.adcReadCyc + g.adcSwitchCyc) + g.guardCyc;
  // Room a point needs inside its free gap: the reads themselves, the settling
  // time before them and the next latch's programming window after them.
  const uint32_t room = g.preloadCyc + pl.budgetCyc + g.settleCyc;
  pl.envGate = (def.env.rampIn_us != 0 || def.env.rampOut_us != 0);
  const uint8_t chm = (uint8_t)((pl.lines[0] ? 1 : 0) | (pl.lines[1] ? 2 : 0));

  if (g.type != SINE) {
    for (uint8_t i = 0; i < g.nStages && pl.nPoints < SJ_MEAS_POINTS; i++) {
      if (def.meas.stage >= 0 && (uint8_t)def.meas.stage != i) continue;
      const uint64_t stageEnd = g.cum[i + 1];
      const uint64_t stageDur = g.cum[i + 1] - g.cum[i];
      // The free gap is the stage for `S`, but only one ramp sample interval
      // for `L` — its samples latch every stageDur/N cycles.
      uint64_t gap = stageDur;
      if (g.type == PIECEWISE_RAMP) {
        uint32_t N = (g.stageN && g.stageN[i]) ? g.stageN[i] : 1;
        gap = stageDur / N;
      }
      const uint8_t p = pl.nPoints++;
      pl.label[p]  = i;
      pl.chMask[p] = chm;
      if (gap < room) {
        pl.skipMask |= (uint16_t)(1u << p);
        if (pl.needCyc == 0) { pl.needCyc = room; pl.roomCyc = (uint32_t)gap; }
        pl.atCyc[p] = stageEnd;            // never fires; keeps the list sorted
      } else {
        pl.atCyc[p] = stageEnd - g.preloadCyc - pl.budgetCyc;
      }
    }
    pl.on = (pl.nPoints != 0);
    return;
  }

  // ---- SINE: peaks of the lower-numbered measured channel
  const uint8_t pc = pl.lines[0] ? 0 : 1;
  const uint8_t oc = (uint8_t)(pc ^ 1);
  pl.peakChannel   = pc;
  pl.peakMismatch  = (pl.lines[oc] != 0) &&
                     (g.phaseInc[oc] != g.phaseInc[pc] ||
                      g.phaseInit[oc] != g.phaseInit[pc]);
  const uint32_t inc = g.phaseInc[pc];
  if (inc == 0) return;                    // 0 Hz: there is no peak to solve

  // when: bit0 (=1) positive peak at 90 deg, bit1 (=2) negative peak at 270 deg
  static const uint32_t target[2] = {0x40000000u, 0xC0000000u};
  static const uint16_t degrees[2] = {90, 270};
  uint64_t at[2] = {0, 0};
  uint16_t deg[2] = {0, 0};
  bool     ok[2] = {false, false};
  uint8_t  np = 0;
  for (uint8_t j = 0; j < 2; j++) {
    if (!(def.meas.when & (1u << j))) continue;
    // The reads go *after* the peak sample latches — that sample is what we
    // want to read — and must finish before the next sample's programming.
    const uint64_t a = (uint64_t)peakSampleIndex(g.phaseInit[pc], inc, target[j])
                     * g.sampleCyc + g.settleCyc;
    at[np]  = a;
    deg[np] = degrees[j];
    ok[np]  = (g.sampleCyc >= room) &&
              (a + pl.budgetCyc + g.preloadCyc <= g.burstCyc);
    np++;
  }
  if (np == 2 && at[1] < at[0]) {          // keep the point list ascending
    uint64_t ta = at[0]; at[0] = at[1]; at[1] = ta;
    uint16_t td = deg[0]; deg[0] = deg[1]; deg[1] = td;
    bool     to = ok[0];  ok[0] = ok[1];   ok[1] = to;
  }
  for (uint8_t j = 0; j < np; j++) {
    pl.atCyc[j]  = at[j];
    pl.label[j]  = deg[j];
    pl.chMask[j] = chm;
    if (!ok[j]) {
      pl.skipMask |= (uint16_t)(1u << j);
      if (pl.needCyc == 0) { pl.needCyc = room; pl.roomCyc = g.sampleCyc; }
    }
  }
  pl.nPoints = np;
  pl.on      = (np != 0);
}

// ------------------------------------------------------------- device section
#ifdef ARDUINO

} // namespace Measure

#include "Config.h"
#include "FastIO.h"
#include "SdLog.h"
#include "Protocol.h"
#include <stdio.h>

namespace Measure {

static Plan plan_[2];
static volatile bool notePending[2] = {false, false};

// MDATA ring — SPSC: the player ISRs produce, poll() is the *only* consumer
// (it both prints and hands rows to SdLog; two independent drains would each
// see half the records).
struct Rec {
  uint64_t tCyc;
  uint32_t pulse;
  uint16_t label;
  uint8_t  slot, report, valid;    // valid: bit (ch*2 + line)
  int16_t  raw[2][2];
};
#define SJ_MDATA_RING 128
static Rec ring[SJ_MDATA_RING];
static volatile uint16_t rHead = 0, rTail = 0;
static volatile uint32_t rDropped = 0;

void begin() {
  memset(plan_, 0, sizeof plan_);
}

bool hasPlan(uint8_t eng) { return plan_[eng].on; }

void armPlan(uint8_t eng, uint8_t slot, const TrainDef& def, const Geometry& g) {
  planBuild(plan_[eng], slot, def, g);
  notePending[eng] = plan_[eng].on;
}

uint64_t nextDeadline(uint8_t eng, uint64_t pulseStart) {
  Plan& p = plan_[eng];
  if (!p.on) return UINT64_MAX;
  while (p.next < p.nPoints && (p.skipMask & (1u << p.next))) p.next++;
  if (p.next >= p.nPoints) return UINT64_MAX;
  return pulseStart + p.atCyc[p.next];
}

void pulseDone(uint8_t eng) { plan_[eng].next = 0; }

void fire(uint8_t eng, uint32_t pulseIdx, uint64_t atCyc, int32_t envQ15) {
  Plan& p = plan_[eng];
  const uint8_t i = p.next;
  if (i >= p.nPoints) return;
  p.next = (uint8_t)(i + 1);
  // A reading taken while the envelope ramps describes an attenuated waveform;
  // averaging it with full-amplitude repetitions would produce a mean that
  // describes neither, so those repetitions are counted and dropped.
  if (p.envGate && envQ15 < 32768) { p.envSkipped++; return; }

  uint8_t valid = 0;
  int16_t raw[2][2] = {{0, 0}, {0, 0}};
  for (uint8_t ch = 0; ch < 2; ch++) {
    if (!(p.chMask[i] & (1u << ch))) continue;
    for (uint8_t ln = 0; ln < 2; ln++) {
      if (!(p.lines[ch] & (1u << ln))) continue;
      FastIO::adcSelectLine(ch, ln);
      const int16_t v = FastIO::adcRead(ch, ln);
      Accum& a = p.acc[i][ch][ln];
      a.n++;
      a.sum   += v;
      a.sumsq += (int32_t)v * v;           // int16 squared fits int32
      raw[ch][ln] = v;
      valid |= (uint8_t)(1u << (ch * 2 + ln));
    }
  }

  if (!(p.report & 3)) return;             // summary only: nothing to stream
  const uint16_t h = rHead, next = (uint16_t)((h + 1) & (SJ_MDATA_RING - 1));
  if (next == rTail) { rDropped++; return; }
  Rec& r = ring[h];
  r.tCyc   = atCyc;
  r.pulse  = pulseIdx;
  r.label  = p.label[i];
  r.slot   = p.slot;
  r.report = p.report;
  r.valid  = valid;
  memcpy(r.raw, raw, sizeof raw);
  rHead = next;
}

// ------------------------------------------------------- unit conversion (loop)

// Raw ADC code -> mV (line 0) or uA (line 1) through the same calibrated path
// as `E` and `READ`: offset-corrected, then the per-line scale.
static inline double toPhysD(double raw, uint8_t ch, uint8_t line) {
  return (raw - Stimjim.adcOffset10[ch]) * (line ? MICROAMPS_PER_ADC : MILLIVOLTS_PER_ADC);
}
static inline double toPhys(int16_t raw, uint8_t ch, uint8_t line) {
  return toPhysD((double)raw, ch, line);
}
// The offset cancels in a difference, so a spread converts with the scale alone.
static inline double sdPhys(double sdRaw, uint8_t line) {
  return sdRaw * (line ? MICROAMPS_PER_ADC : MILLIVOLTS_PER_ADC);
}
// Cycles to whole microseconds, rounded up — a budget must never read short.
static inline uint32_t cycToUsUp(uint32_t cyc) {
  return (uint32_t)((cyc + SJ_CYC_PER_US - 1) / SJ_CYC_PER_US);
}

// One numeric field of an MSUM/MDATA record, empty where nothing was measured.
static inline void field(char* b, size_t n, bool have, double v) {
  if (have) snprintf(b, n, "%.2f", v);
  else      b[0] = '\0';
}

// Runs once per armed plan, before the ring is drained in the same poll() call
// — so the log block and the warnings always precede that train's rows.
static void printNote(uint8_t eng) {
  const Plan& p = plan_[eng];
  if (p.report & 2) SdLog::noteTrain(p.slot);
  if (p.skipMask) {
    for (uint8_t i = 0; i < p.nPoints; i++) {
      if (!(p.skipMask & (1u << i))) continue;
      Serial.printf("WARN MEAS: slot %u point %u needs %lu us but only %lu us is free "
                    "— not measured\n", p.slot, p.label[i],
                    (unsigned long)cycToUsUp(p.needCyc),
                    (unsigned long)SJ_CYC_TO_US(p.roomCyc));
    }
  }
  if (p.peakMismatch)
    Serial.printf("WARN MEAS: slot %u sine peaks follow channel %u; the other measured "
                  "channel has a different frequency or phase\n", p.slot, p.peakChannel);
}

void poll() {
  for (uint8_t e = 0; e < 2; e++)
    if (notePending[e]) { notePending[e] = false; printNote(e); }

  while (rTail != rHead) {
    const Rec& r = ring[rTail];
    char f[4][16];
    for (uint8_t ch = 0; ch < 2; ch++)
      for (uint8_t ln = 0; ln < 2; ln++) {
        const bool have = (r.valid >> (ch * 2 + ln)) & 1;
        field(f[ch * 2 + ln], sizeof f[0], have, have ? toPhys(r.raw[ch][ln], ch, ln) : 0.0);
      }
    if (r.report & 1)
      Serial.printf("MDATA,%u,%lu,%u,%s,%s,%s,%s\n", r.slot, (unsigned long)r.pulse,
                    r.label, f[0], f[1], f[2], f[3]);
    if (r.report & 2) {
      // Microseconds since boot outgrow 32 bits after 71 minutes, so the
      // timestamp is rendered by hand rather than trusting printf's %llu.
      char ts[24], row[144];
      Protocol::u64str(SJ_CYC_TO_US(r.tCyc), ts);
      snprintf(row, sizeof row, "%s,%u,%lu,%u,%s,%s,%s,%s",
               ts, r.slot, (unsigned long)r.pulse, r.label, f[0], f[1], f[2], f[3]);
      SdLog::writeRow(row);
    }
    rTail = (uint16_t)((rTail + 1) & (SJ_MDATA_RING - 1));
    // A long drain must not starve the 64-bit timebase extension.
    FastIO::cycles64();
  }

  if (rDropped) {
    Serial.printf("WARN MEAS: MDATA ring overflowed — %lu records dropped "
                  "(the host is not reading fast enough)\n", (unsigned long)rDropped);
    rDropped = 0;
  }
}

void printSummary(uint8_t eng, uint8_t slot) {
  Plan& p = plan_[eng];
  if (!p.on) return;
  if (p.slot != slot) {
    Serial.printf("# MSUM: engine %u was re-armed before its summary printed — dropped\n", eng);
    return;
  }
  for (uint8_t i = 0; i < p.nPoints; i++) {
    char f[8][16];
    uint32_t n = 0;
    for (uint8_t ch = 0; ch < 2; ch++)
      for (uint8_t ln = 0; ln < 2; ln++) {
        const Accum& a = p.acc[i][ch][ln];
        double meanRaw, sdRaw;
        const bool have = accumStats(a, meanRaw, sdRaw);
        // mean carries the channel offset; the spread does not (it cancels)
        field(f[ch * 4 + ln * 2],     sizeof f[0], have,
              have ? toPhysD(meanRaw, ch, ln) : 0.0);
        field(f[ch * 4 + ln * 2 + 1], sizeof f[0], have && a.n >= 2, sdPhys(sdRaw, ln));
        if (have && a.n > n) n = a.n;
      }
    Serial.printf("MSUM,%u,%lu,%u,%s,%s,%s,%s,%s,%s,%s,%s\n",
                  p.slot, (unsigned long)n, p.label[i],
                  f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7]);
    if (p.skipMask & (1u << i))
      Serial.printf("# MSUM point %u: needs %lu us, only %lu us free — not measured\n",
                    p.label[i],
                    (unsigned long)cycToUsUp(p.needCyc),
                    (unsigned long)SJ_CYC_TO_US(p.roomCyc));
  }
  if (p.envSkipped)
    Serial.printf("# MSUM: %lu repetitions skipped inside the ENV ramps\n",
                  (unsigned long)p.envSkipped);
}

#endif // ARDUINO

} // namespace Measure
