//    stimjimAWG — Measure implementation: arm-time plan compilation (pure), the
//    player-ISR read/accumulate path, the MDATA ring and the MSUM summary.
//    GPL-3.0-or-later; see Config.h header.

#include "Measure.h"
#include <math.h>
#include <string.h>
#include <stddef.h>   // offsetof: the Plan splits into a compiled and a result region

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

// ADC window `nr` reads need: the conversions themselves plus the margin
// before the next latch starts programming the DAC.
static inline uint32_t windowCyc(uint8_t nr, const Geometry& g) {
  return nr * (g.adcReadCyc + g.adcSwitchCyc) + g.guardCyc;
}

// Free gap `nr` reads need: the window, the settling time that has to pass
// after the latch before the first read, and the next latch's programming.
static inline uint32_t roomCyc(uint8_t nr, const Geometry& g) {
  return g.preloadCyc + windowCyc(nr, g) + g.settleCyc;
}

// Most reads (1..nReads) that fit one `gap`; 0 when not even one does.
static uint8_t readsPerGap(uint8_t nReads, uint64_t gap, const Geometry& g) {
  for (uint8_t r = nReads; r >= 1; r--)
    if (roomCyc(r, g) <= gap) return r;
  return 0;
}

// Decide how point p reads its lines inside a free gap of `gap` cycles: all of
// them in one gap, in `nGrp` groups one per repetition (fit = rotate), or not
// at all. Fills nGrp/grpMask/budCyc and returns false when the point has to be
// refused, having recorded the arithmetic of the first such point.
static bool groupPoint(Plan& pl, uint8_t p, uint64_t gap, const Geometry& g, uint8_t fit) {
  const uint8_t rpg = readsPerGap(pl.nReads, gap, g);
  const bool rotate = (fit == SJ_FIT_ROTATE);
  if (rpg == 0 || (rpg < pl.nReads && !rotate)) {
    if (pl.needCyc == 0) {
      // What to report as "needed" is what this `fit` would have asked for:
      // the full window in strict mode, a single read when rotating (the only
      // way rotation fails is that not even one read fits).
      pl.needCyc = roomCyc(rotate ? 1 : pl.nReads, g);
      pl.roomCyc = (uint32_t)gap;
    }
    return false;
  }
  const uint8_t nGrp = (uint8_t)((pl.nReads + rpg - 1) / rpg);
  // Balanced groups: ceil(nReads / nGrp) <= rpg reads each, so the placement
  // budget below covers every group and the last one may be smaller.
  const uint8_t per = (uint8_t)((pl.nReads + nGrp - 1) / nGrp);
  pl.nGrp[p]   = nGrp;
  pl.budCyc[p] = windowCyc(per, g);
  uint8_t gi = 0, cnt = 0;
  for (uint8_t ch = 0; ch < 2; ch++)
    for (uint8_t ln = 0; ln < 2; ln++) {
      if (!(pl.lines[ch] & (1u << ln))) continue;
      pl.grpMask[p][gi] |= (uint8_t)(1u << (ch * 2 + ln));
      if (++cnt == per) { cnt = 0; gi++; }
    }
  if (nGrp > 1) pl.rotMask |= (uint16_t)(1u << p);
  return true;
}

// True when a rotating plan will not deliver enough repetitions for every
// group to fire once — some lines then have no reading at all.
static bool rotationShort(const Plan& pl) {
  if (!pl.rotMask) return false;
  uint8_t maxGrp = 1;
  for (uint8_t p = 0; p < pl.nPoints; p++)
    if (pl.nGrp[p] > maxGrp) maxGrp = pl.nGrp[p];
  return pl.reps < maxGrp;
}

// The compiled region runs from the start of the Plan up to the tag; the
// result state follows it. Splitting the zeroing at that boundary is what took
// the arm's memset from 1224 bytes to 242 plus 96 per point (Measure.h).
#define SJ_PLAN_COMPILED_BYTES offsetof(Plan, tagValid)

static void planCompile(Plan& pl, uint8_t slot, const TrainDef& def, const Geometry& g) {
  memset(&pl, 0, SJ_PLAN_COMPILED_BYTES);
  pl.slot   = slot;
  pl.type   = g.type;
  pl.report = def.meas.report;

  // `what` (0 none, 1 V, 2 I, 3 both) is already a line bitmask: bit0 = line 0
  // (output voltage), bit1 = line 1 (current sense). A channel the train does
  // not drive is never measured, whatever MEAS says (protocol §4).
  pl.lines[0] = (g.chMask & 1) ? (uint8_t)(def.meas.what0 & 3) : 0;
  pl.lines[1] = (g.chMask & 2) ? (uint8_t)(def.meas.what1 & 3) : 0;
  pl.nReads = (uint8_t)(bitCount2(pl.lines[0]) + bitCount2(pl.lines[1]));
  if (pl.nReads == 0) return;              // on stays false

  pl.budgetCyc = windowCyc(pl.nReads, g);  // every read in one gap
  pl.envGate = (def.env.rampIn_us != 0 || def.env.rampOut_us != 0);
  // Repetitions the train will deliver — rotation needs at least one per group
  // to cover every line, and this is where that is known.
  pl.reps = def.period_us
          ? (uint32_t)(((uint64_t)def.duration_us + def.period_us - 1) / def.period_us)
          : 0;
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
      if (!groupPoint(pl, p, gap, g, def.meas.fit)) {
        pl.skipMask |= (uint16_t)(1u << p);
        pl.atCyc[p] = stageEnd;            // never fires; keeps the list sorted
      } else {
        pl.atCyc[p] = stageEnd - g.preloadCyc - pl.budCyc[p];
      }
    }
    pl.on = (pl.nPoints != 0);
    pl.rotShort = rotationShort(pl);
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
  uint8_t  np = 0;
  for (uint8_t j = 0; j < 2; j++) {
    if (!(def.meas.when & (1u << j))) continue;
    // The reads go *after* the peak sample latches — that sample is what we
    // want to read — and must finish before the next sample's programming.
    at[np]  = (uint64_t)peakSampleIndex(g.phaseInit[pc], inc, target[j])
            * g.sampleCyc + g.settleCyc;
    deg[np] = degrees[j];
    np++;
  }
  if (np == 2 && at[1] < at[0]) {          // keep the point list ascending
    uint64_t ta = at[0]; at[0] = at[1]; at[1] = ta;
    uint16_t td = deg[0]; deg[0] = deg[1]; deg[1] = td;
  }
  pl.nPoints = np;
  for (uint8_t j = 0; j < np; j++) {
    pl.atCyc[j]  = at[j];
    pl.label[j]  = deg[j];
    pl.chMask[j] = chm;
    // The free gap of a sine point is one sample interval; the reads must also
    // finish inside the burst, before the off event's programming window.
    // The two failures report different arithmetic: the sample interval is too
    // short, or the peak sits too close to the end of the burst.
    if (!groupPoint(pl, j, g.sampleCyc, g, def.meas.fit)) {
      pl.skipMask |= (uint16_t)(1u << j);
    } else if (at[j] + pl.budCyc[j] + g.preloadCyc > g.burstCyc) {
      pl.skipMask |= (uint16_t)(1u << j);
      pl.rotMask &= (uint16_t)~(1u << j);
      if (pl.needCyc == 0) {
        pl.needCyc = (uint32_t)(at[j] + pl.budCyc[j] + g.preloadCyc);
        pl.roomCyc = (uint32_t)g.burstCyc;
      }
    }
  }
  pl.on       = (np != 0);
  pl.rotShort = rotationShort(pl);
}

// Clear the result state: the accumulators of the points that exist, the
// envelope-skip count and the point cursor. 96 bytes per point, so a one- or
// two-point plan costs a fraction of what zeroing all ten did.
static inline void planResetResults(Plan& pl) {
  memset(pl.acc, 0, (size_t)pl.nPoints * sizeof pl.acc[0]);
  pl.next       = 0;
  pl.envSkipped = 0;
}

void planBuild(Plan& pl, uint8_t slot, const TrainDef& def, const Geometry& g) {
  planCompile(pl, slot, def, g);
  pl.tagValid = false;          // the caller owns the tag
  planResetResults(pl);
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

// Two plans per engine, and one invariant that everything below rests on:
//
//   plan_[eng][live[eng]] is the ONLY plan the player ISR of engine eng ever
//   touches. The other one belongs to loop(), which may compile into it and
//   clear it whenever it likes.
//
// live[eng] moves only inside armPlan, which runs with that engine's player
// masked or stopped, so no reader ever sees it change under itself. What the
// second buffer buys (docs/PLAN_plan-out-of-arm.md):
//
//   - the 3.1 us/point compile and the 0.55 us/point zeroing leave the arm,
//     because loop() prepares the buffer the next arm will take;
//   - a train re-armed before loop() drained its completion keeps its summary,
//     which a single in-place plan could not.
static Plan plan_[2][2];
static volatile uint8_t live[2] = {0, 0};
// Which buffer holds the train that just finished, and whether its summary is
// still unprinted. Set in the player ISR at completion, cleared by
// printSummary. loop() must not reuse or clear the buffer it names.
static volatile uint8_t summaryIdx[2]     = {0, 0};
static volatile bool    summaryPending[2] = {false, false};
// The buffer holds accumulated results that have not been cleared yet.
static volatile bool    dirty[2][2]       = {{false, false}, {false, false}};
// loop() is writing the spare buffer of this engine right now. An arm that
// lands in that window (a trigger ISR preempting loop()) reuses the live buffer
// in place instead of taking the spare -- which is exactly what the
// single-buffer firmware always did, so the fallback is a known-good path and
// costs what it used to cost. Plain volatile is enough for the handshake: one
// core, and ISR entry and exit serialize.
static volatile bool    loopBusy[2]       = {false, false};
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

bool hasPlan(uint8_t eng) { return plan_[eng][live[eng]].on; }

// True when this buffer does not already describe that (slot, definition, CAL)
// triple. Four loads, so both the arm and loop()'s warm can ask before doing
// anything expensive.
static inline bool tagMisses(const Plan& p, uint8_t slot,
                             uint32_t defEpoch, uint32_t calEpoch) {
  return !p.tagValid || p.tagSlot != slot ||
         p.tagDefEpoch != defEpoch || p.tagCalEpoch != calEpoch;
}

static void planCompileTagged(Plan& p, uint8_t slot, const TrainDef& def,
                              const Geometry& g, uint32_t defEpoch, uint32_t calEpoch) {
  planCompile(p, slot, def, g);
  p.tagValid    = true;
  p.tagSlot     = slot;
  p.tagDefEpoch = defEpoch;
  p.tagCalEpoch = calEpoch;
}

// Which buffer the next arm of this engine will take. Not a claim -- callers
// that intend to write it must go through claimSpare.
static inline uint8_t spareIdx(uint8_t eng) { return (uint8_t)(live[eng] ^ 1); }

// Claim the spare buffer for loop(). Returns SJ_NO_BUFFER when it must be left
// alone: it still holds an unprinted summary, or an arm took it between reading
// live[] and publishing the claim. The re-read of live[] after setting loopBusy
// is what closes that second window -- an arm that beat us moved live, and an
// arm that comes after us sees the flag and stays on its own buffer, so exactly
// one of the two happens.
#define SJ_NO_BUFFER 0xFF
static uint8_t claimSpare(uint8_t eng) {
  const uint8_t n = spareIdx(eng);
  if (summaryPending[eng] && summaryIdx[eng] == n) return SJ_NO_BUFFER;
  loopBusy[eng] = true;
  if (live[eng] != (uint8_t)(n ^ 1)) { loopBusy[eng] = false; return SJ_NO_BUFFER; }
  return n;
}
static inline void releaseSpare(uint8_t eng) { loopBusy[eng] = false; }

bool planReady(uint8_t eng, uint8_t slot, uint32_t defEpoch, uint32_t calEpoch) {
  const uint8_t n = spareIdx(eng);
  if (summaryPending[eng] && summaryIdx[eng] == n) return false;
  return !tagMisses(plan_[eng][n], slot, defEpoch, calEpoch);
}

void warmPlan(uint8_t eng, uint8_t slot, const TrainDef& def, const Geometry& g,
              uint32_t defEpoch, uint32_t calEpoch) {
  const uint8_t n = claimSpare(eng);
  if (n == SJ_NO_BUFFER) return;
  Plan& p = plan_[eng][n];
  if (tagMisses(p, slot, defEpoch, calEpoch)) {
    planCompileTagged(p, slot, def, g, defEpoch, calEpoch);
    // Always reset after a compile, never only when dirty: the new plan may
    // have more points than the one whose clear this buffer last got.
    planResetResults(p);
    dirty[eng][n] = false;
  }
  releaseSpare(eng);
}

void housekeep(uint8_t eng) {
  if (!dirty[eng][spareIdx(eng)]) return;      // nothing to do, and no claim needed
  const uint8_t n = claimSpare(eng);
  if (n == SJ_NO_BUFFER) return;
  if (dirty[eng][n]) {                          // re-read under the claim
    planResetResults(plan_[eng][n]);
    dirty[eng][n] = false;
  }
  releaseSpare(eng);
}

void armPlan(uint8_t eng, uint8_t slot, const TrainDef& def, const Geometry& g,
             uint32_t defEpoch, uint32_t calEpoch) {
  // Take the buffer the player is not using -- unless loop() is writing it, in
  // which case stay on the live one and rewrite it in place. That fallback is
  // what the single-buffer firmware always did: it costs the compile and the
  // clear, and it can lose an unprinted summary, but it is never wrong. It
  // needs a trigger edge to land inside loop()'s compile window to happen at
  // all.
  uint8_t n = loopBusy[eng] ? live[eng] : (uint8_t)(live[eng] ^ 1);
  // The compiled region depends on the slot, its definition and the CAL set --
  // never on the calibration offsets, which is why an offset recalibration is
  // not in the tag. loop() has normally compiled the spare already (warmPlan),
  // so this misses only when loop() has not run since the last arm or the edit.
  bool compiled = tagMisses(plan_[eng][n], slot, defEpoch, calEpoch);
  if (compiled && n != live[eng]) {
    // The spare describes some other slot, but the live buffer may still
    // describe this one -- a repeated start of the same slot with loop()
    // starved. Reusing it in place is free where compiling is not, and the
    // only thing it gives up is the second buffer's other benefit: it must not
    // take a buffer whose summary nobody has printed.
    const bool liveHoldsSummary = summaryPending[eng] && summaryIdx[eng] == live[eng];
    if (!liveHoldsSummary && !tagMisses(plan_[eng][live[eng]], slot, defEpoch, calEpoch)) {
      n = live[eng];
      compiled = false;
    }
  }
  Plan& p = plan_[eng][n];
  if (compiled) planCompileTagged(p, slot, def, g, defEpoch, calEpoch);
  // An armed train starts from zeroed accumulators. loop() normally cleared
  // them at housekeep time, so the usual case is neither branch. The dirty
  // flag covers a buffer loop() has not reached; the compile covers a subtler
  // one -- a plan cleared while it had fewer points than this one has leaves
  // the points beyond that count holding an older train's sums, and only a
  // reset taken *after* the compile knows how many points to clear.
  if (compiled || dirty[eng][n]) planResetResults(p);
  dirty[eng][n] = p.on;
  live[eng]     = n;
  notePending[eng] = p.on;
}

// Called by the player ISR where it pushes the completion record: it freezes
// the finished train's buffer so neither the next arm nor loop()'s housekeeping
// takes it before printSummary has read it.
void trainDone(uint8_t eng) {
  summaryIdx[eng]     = live[eng];
  summaryPending[eng] = true;
}

uint64_t nextDeadline(uint8_t eng, uint64_t pulseStart) {
  Plan& p = plan_[eng][live[eng]];
  if (!p.on) return UINT64_MAX;
  while (p.next < p.nPoints && (p.skipMask & (1u << p.next))) p.next++;
  if (p.next >= p.nPoints) return UINT64_MAX;
  return pulseStart + p.atCyc[p.next];
}

void pulseDone(uint8_t eng) { plan_[eng][live[eng]].next = 0; }

void fire(uint8_t eng, uint32_t pulseIdx, uint64_t atCyc, int32_t envQ15) {
  Plan& p = plan_[eng][live[eng]];
  const uint8_t i = p.next;
  if (i >= p.nPoints) return;
  p.next = (uint8_t)(i + 1);
  // A reading taken while the envelope ramps describes an attenuated waveform;
  // averaging it with full-amplitude repetitions would produce a mean that
  // describes neither, so those repetitions are counted and dropped.
  if (p.envGate && envQ15 < 32768) { p.envSkipped++; return; }

  // Which lines this repetition reads. Without rotation that is every line of
  // the point (group 0 holds them all); with it, one group per pulse in
  // rotation — the reads stay at this instant, only their number per pulse
  // drops. The modulo keeps the rotation aligned to the pulse grid even when
  // the envelope gate has dropped repetitions.
  const uint8_t rmask = p.grpMask[i][p.nGrp[i] > 1 ? (uint8_t)(pulseIdx % p.nGrp[i]) : 0];

  uint8_t valid = 0;
  int16_t raw[2][2] = {{0, 0}, {0, 0}};
  for (uint8_t ch = 0; ch < 2; ch++) {
    for (uint8_t ln = 0; ln < 2; ln++) {
      if (!(rmask & (1u << (ch * 2 + ln)))) continue;
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
  const Plan& p = plan_[eng][live[eng]];
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
  if (p.rotMask) {
    // One line for the whole plan, not one per point: every rotating point of
    // a train normally rotates the same way, and the interesting numbers are
    // how thin the per-line count gets and how to get the full window back.
    uint8_t nRot = 0, maxGrp = 1;
    for (uint8_t i = 0; i < p.nPoints; i++) {
      if (!(p.rotMask & (1u << i))) continue;
      nRot++;
      if (p.nGrp[i] > maxGrp) maxGrp = p.nGrp[i];
    }
    Serial.printf("# MEAS: slot %u: %u read(s) do not fit one free gap — %u point(s) rotate "
                  "their reads over up to %u repetitions (each line gets about n/%u of them). "
                  "A wider gap: DT (L) or CAL; refuse instead: MEAS fit=0\n",
                  p.slot, p.nReads, nRot, maxGrp, maxGrp);
    if (p.rotShort)
      Serial.printf("WARN MEAS: slot %u delivers %lu repetition(s), fewer than the %u "
                    "rotation groups — some lines are never read\n",
                    p.slot, (unsigned long)p.reps, maxGrp);
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

void resultSnapshot(uint8_t eng, uint8_t slot, uint32_t trainNo, ResultSet& out) {
  // Same buffer choice as printSummary, but nothing is consumed: the engine may
  // already have been re-armed onto the other buffer, and the finished train's
  // one stays frozen until printSummary releases it.
  const uint8_t idx = summaryPending[eng] ? summaryIdx[eng] : live[eng];
  const Plan& p = plan_[eng][idx];
  out.valid   = false;
  out.eng     = eng;
  out.slot    = slot;
  out.trainNo = trainNo;
  out.nPoints = 0;
  out.nMax    = 0;
  if (!p.on || p.slot != slot) return;
  out.type    = p.type;
  out.nPoints = p.nPoints;
  for (uint8_t i = 0; i < p.nPoints; i++) {
    out.label[i] = p.label[i];
    for (uint8_t ch = 0; ch < 2; ch++) {
      ResultChan& r = out.ch[i][ch];
      memset(&r, 0, sizeof r);
      for (uint8_t ln = 0; ln < 2; ln++) {
        const Accum& a = p.acc[i][ch][ln];
        double meanRaw, sdRaw;
        if (!accumStats(a, meanRaw, sdRaw)) continue;
        // The calibrated path `MSUM` uses, then x1000 into the panel's unit:
        // millivolts become microvolts, microamps become nanoamps. The standard
        // error is sd/sqrt(n); the offset cancels in the spread, so only the
        // scale applies to it.
        const double mean = toPhysD(meanRaw, ch, ln) * 1000.0;
        const double se   = (a.n >= 2)
                          ? sdPhys(sdRaw, ln) * 1000.0 / sqrt((double)a.n) : 0.0;
        if (ln == 0) { r.nV = a.n; r.uV = (int32_t)llround(mean); r.seUV = (uint32_t)llround(se); }
        else         { r.nI = a.n; r.nA = (int32_t)llround(mean); r.seNA = (uint32_t)llround(se); }
        if (a.n > out.nMax) out.nMax = a.n;
      }
    }
  }
  out.valid = (p.nPoints != 0);
}

bool printSummary(uint8_t eng, uint8_t slot) {
  // The finished train's buffer, not the live one: the engine may already have
  // been re-armed onto the other buffer, and with two of them that no longer
  // costs the summary anything. It takes two re-arms with no loop() pass
  // between them to lose one now. Clearing summaryPending here is what releases
  // the buffer back to loop()'s housekeeping, so this must be called for every
  // completion, measured or not -- which is why the caller learns from the
  // return value rather than from a separate query.
  const uint8_t idx = summaryPending[eng] ? summaryIdx[eng] : live[eng];
  Plan& p = plan_[eng][idx];
  summaryPending[eng] = false;
  if (!p.on) return false;
  if (p.slot != slot) {
    Serial.printf("# MSUM: engine %u was re-armed twice before its summary printed — dropped\n", eng);
    return true;
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
    // With rotation the four lines are read on different repetitions, so their
    // counts differ by at most one and the n field above carries the largest.
    else if (p.rotMask & (1u << i))
      Serial.printf("# MSUM point %u: reads rotated over %u repetitions, n is the largest "
                    "of the lines' counts\n", p.label[i], p.nGrp[i]);
  }
  if (p.envSkipped)
    Serial.printf("# MSUM: %lu repetitions skipped inside the ENV ramps\n",
                  (unsigned long)p.envSkipped);
  return true;
}

#endif // ARDUINO

} // namespace Measure
