//    stimjimAWG — Measure: per-train measurement plans, accumulation, the MDATA
//    ring and summary formatting (plan §3.6, protocol §4).
//    GPL-3.0-or-later; see Config.h header.
//
//    A Plan is compiled at arm time into a short list of measurement *points*.
//    Point p starts its ADC reads at pulseStart + atCyc[p] and is placed so the
//    reads finish before the next latch event begins programming the DAC:
//
//        atCyc[p] = nextLatch - preloadCyc - budCyc[p]
//        budCyc[p] = readsPerGap * (ADC_READ + ADC_SWITCH) + GUARD
//
//    and the point is only accepted if preloadCyc + budCyc[p] + SETTLE fits the
//    free gap it lives in — the stage duration for `S`, the ramp *sample
//    interval* for `L`, the sine sample interval for `W`. The waveform is never
//    stretched to make room for the instrumentation, so a point whose reads do
//    not all fit is handled one of two ways (MeasDef.fit):
//
//      fit = 0 (strict)  the point is kept, flagged in skipMask, never fired,
//                        and reported with the summary.
//      fit = 1 (rotate)  the reads are split into nGrp[p] groups that each fit
//                        one gap, and one group fires per repetition, chosen by
//                        pulse index. Every read still happens at exactly the
//                        instant the point's label names; what shrinks is n —
//                        each line accumulates about nPulses/nGrp[p] samples.
//                        A point still refused this way is one where not even a
//                        single read fits the gap.
//
//    The ISR accumulates raw ADC codes only (n, sum, sum of squares — integer
//    adds). The channel offset and the unit scale are floats, so both are
//    applied in loop context when the summary is formatted.
//
//    planBuild/peakSampleIndex/accumStats carry no Arduino dependency and are
//    host-tested by tests/host/test_measure.cpp.

#ifndef STIMJIMAWG_MEASURE_H
#define STIMJIMAWG_MEASURE_H

#include <stdint.h>
#include "WaveformDef.h"

namespace Measure {

// Points per pulse: 10 stages (S/L) or 2 sine peaks — SJ_MAX_STAGES bounds it.
#define SJ_MEAS_POINTS SJ_MAX_STAGES
// Rotation groups per point: at most one per read, and there are at most four
// reads (two channels x two lines).
#define SJ_MEAS_GROUPS 4

// Raw-code accumulator for one (point, channel, line). sum/sumsq are int64:
// a 13-bit code squared is ~1.7e7, so even millions of repetitions cannot
// overflow, and the single-pass estimator stays exact in integers.
struct Accum {
  uint32_t n;
  int64_t  sum, sumsq;
};

// What the player knows about its own arm-time geometry, handed to planBuild so
// the plan arithmetic needs no access to Engine internals.
struct Geometry {
  uint8_t  type;                 // TrainType
  uint8_t  chMask;               // channels the train drives (bit0/bit1)
  uint8_t  nStages;
  const uint64_t* cum;           // nStages+1 stage boundaries from pulse start, cycles
  const uint32_t* stageN;        // RAMP: samples per stage; NULL for HOLD
  uint32_t sampleCyc;            // SINE: cycles per sample
  uint64_t burstCyc;             // SINE: burst length
  uint32_t phaseInit[2];         // SINE: Q32 start phase per channel
  uint32_t phaseInc[2];          // SINE: Q32 turns per sample per channel
  uint32_t preloadCyc;           // the player's program+spin budget before a latch
  uint32_t adcReadCyc;           // one conversion, line already selected
  uint32_t adcSwitchCyc;         // control-register write + first valid conversion
  uint32_t guardCyc;             // margin before the next preload window opens
  uint32_t settleCyc;            // AD5752 settling after a latch
};

struct Plan {
  bool     on;                   // false: this train measures nothing
  uint8_t  slot;
  uint8_t  type;                 // TrainType — decides how `label` reads
  uint8_t  report;               // MeasDef.report bitmask (+1 stream, +2 SD)
  uint8_t  lines[2];             // per channel: bit0 = line 0 (V), bit1 = line 1 (I)
  uint8_t  nReads;               // reads one point takes when nothing is rotated
  uint8_t  nPoints;
  uint16_t skipMask;             // bit p: point p does not fit and never fires
  uint16_t rotMask;              // bit p: point p rotates its reads (nGrp[p] > 1)
  bool     envGate;              // measure only where the envelope is fully on
  bool     rotShort;             // rotating, but the train has fewer repetitions
                                 // than groups: some lines never get read
  uint32_t reps;                 // repetitions the train will deliver (duration/period)
  uint8_t  peakChannel;          // SINE: channel whose phase the peaks follow
  bool     peakMismatch;         // SINE: the other measured channel differs from it
  uint32_t needCyc, roomCyc;     // fit arithmetic of the first refused point
  uint32_t budgetCyc;            // unsplit ADC window (all nReads in one gap)
  uint64_t atCyc[SJ_MEAS_POINTS];   // ascending offsets from pulseStart
  uint16_t label[SJ_MEAS_POINTS];   // stage index (S/L) or peak degrees (W)
  uint8_t  chMask[SJ_MEAS_POINTS];  // channels read at this point
  uint32_t budCyc[SJ_MEAS_POINTS];  // ADC window this point actually reserves
  uint8_t  nGrp[SJ_MEAS_POINTS];    // rotation groups (1 = every line every pulse)
  // Read mask of each group: bit (ch*2 + line). Group g fires on pulses where
  // pulseIdx % nGrp[p] == g; with nGrp[p] == 1, group 0 holds every read.
  uint8_t  grpMask[SJ_MEAS_POINTS][SJ_MEAS_GROUPS];
  // Everything above is compiled from (slot, definition, CAL) and nothing else,
  // so it survives an arm of the same unchanged slot: the tag below says which
  // one it describes and armPlan skips the compile when it matches. Everything
  // after the tag is per-train result state.
  //
  // planBuild zeroes the compiled region (242 bytes) and the accumulators of
  // the points that exist (96 bytes each) instead of the whole 1224-byte
  // struct, because that one memset cost 5-7 us inside the start latency
  // (docs/timing.md §7). Entries past nPoints keep whatever they held; nothing
  // reads them, every loop over points stops at nPoints.
  bool     tagValid;
  uint8_t  tagSlot;
  uint32_t tagDefEpoch, tagCalEpoch;
  Accum    acc[SJ_MEAS_POINTS][2][2];   // [point][channel][line]
  uint8_t  next;                 // live: next point to fire in the current pulse
  uint32_t envSkipped;           // repetitions dropped by the envelope gate
};

// What the OLED result pages show, taken from the same accumulators `MSUM`
// prints. Everything is integer so the display path stays float-free: the
// conversion runs once per completion, in loop() context, and the panel then
// only formats. Full scale is +-15 V and +-3.33 mA, so int32 has ample room in
// microvolts and nanoamps.
struct ResultChan {
  uint32_t nV, nI;       // repetitions behind each line; 0 = that line was not read
  int32_t  uV, nA;       // means
  uint32_t seUV, seNA;   // standard error of each mean, same units
};
struct ResultSet {
  bool     valid;        // false: the train measured nothing
  uint8_t  eng, slot, type, nPoints;
  uint32_t trainNo;      // the completion counter the `Train #n complete` line prints
  uint32_t nMax;         // largest n in the set
  uint16_t label[SJ_MEAS_POINTS];      // stage index (S/L) or peak degrees (W)
  ResultChan ch[SJ_MEAS_POINTS][2];
};

// ------------------------------------------------------------ pure plan math

// Compile `def`'s MeasDef against the geometry, and zero the result state of
// the points it produced (see the note in Plan). Leaves the tag fields cleared:
// the caller owns the tag, because only the caller knows what the definition
// and CAL epochs were when it read them.
void planBuild(Plan& pl, uint8_t slot, const TrainDef& def, const Geometry& g);

// First sample index k >= 0 whose Q32 phase has reached `target`:
// k = ceil(((target - phaseInit) mod 2^32) / phaseInc). phaseInc > 0 required.
uint32_t peakSampleIndex(uint32_t phaseInit, uint32_t phaseInc, uint32_t target);

// Mean and sample standard deviation of an accumulator, in raw ADC codes.
// Returns false when n == 0 (nothing measured); sd is 0 when n == 1.
bool accumStats(const Accum& a, double& mean, double& sd);

// ------------------------------------------------------------- device side
#ifdef ARDUINO

void begin();
void poll();   // deferred start notes, then the single drain of the MDATA ring

// Each engine holds two plans, and the arm swaps between them rather than
// rewriting one in place: the player ISR only ever reads the live buffer, so
// loop() owns the other and can prepare it without any lock at all. That is
// what takes the compile and the accumulator zeroing out of the start latency
// (docs/PLAN_plan-out-of-arm.md); it also means a train re-armed before its
// completion was drained keeps its summary.
//
// Make engine `eng`'s plan describe this arm, by taking the spare buffer. The
// compile is skipped when that buffer already describes the same slot at the
// same definition and CAL epochs -- which is what warmPlan arranges, and what
// every re-arm of an unedited slot gets anyway. An unmeasured train leaves a
// plan with on == false, so a stale plan can never fire. `defEpoch`/`calEpoch`
// are read by the caller under the same conditions as the rest of the arm.
void armPlan(uint8_t eng, uint8_t slot, const TrainDef& def, const Geometry& g,
             uint32_t defEpoch, uint32_t calEpoch);

// loop()-context preparation of the buffer the next arm will take. warmPlan
// compiles it and clears its accumulators; planReady reports whether that has
// already happened, so the caller can skip building a Geometry it would not
// use. Both are no-ops while the spare buffer holds an unprinted summary.
bool planReady(uint8_t eng, uint8_t slot, uint32_t defEpoch, uint32_t calEpoch);
void warmPlan(uint8_t eng, uint8_t slot, const TrainDef& def, const Geometry& g,
              uint32_t defEpoch, uint32_t calEpoch);
// Clear the spare buffer's accumulators if a finished train left results in
// them and the summary has been printed. This is the zeroing the arm used to
// do; calling it from loop() costs the arm nothing.
void housekeep(uint8_t eng);
// Player-ISR call at train end: freezes the finished train's buffer until
// printSummary has read it.
void trainDone(uint8_t eng);

// --- player-ISR interface -------------------------------------------------
// Absolute deadline of the next measurement point of the current pulse, or
// UINT64_MAX when there is none left. Skipped points are stepped over here.
uint64_t nextDeadline(uint8_t eng, uint64_t pulseStart);
// Take the reads of the point `nextDeadline` just reported and accumulate
// them. envQ15 is the envelope at the measurement instant (32768 = fully on);
// anything less is skipped when the plan gates on the envelope.
void fire(uint8_t eng, uint32_t pulseIdx, uint64_t atCyc, int32_t envQ15);
// Rewind to the first point of the next pulse.
void pulseDone(uint8_t eng);

// End-of-train MSUM block, printed from loop() after the completion line. It
// reads the buffer the *finished* train used, which is no longer the live one
// once the engine has been re-armed -- surviving that is what the second buffer
// is for. Returns false when the train measured nothing, so the caller can say
// so; call it for every completion either way, because it is also what hands
// the buffer back to loop()'s housekeeping.
bool printSummary(uint8_t eng, uint8_t slot);
// Whether the train the engine is playing *now* has measurement points.
bool hasPlan(uint8_t eng);

// The same buffer printSummary reads, converted to the integer units the panel
// works in and *not* consumed: it leaves summaryPending alone. Call it
// immediately before printSummary for the same completion, because that is what
// clears the flag naming the finished train's buffer. Sets out.valid = false
// for a train that measured nothing.
void resultSnapshot(uint8_t eng, uint8_t slot, uint32_t trainNo, ResultSet& out);

#endif // ARDUINO

} // namespace Measure

#endif // STIMJIMAWG_MEASURE_H
