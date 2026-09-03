//    stimjimAWG — Measure: per-train measurement plans, accumulation, the MDATA
//    ring and summary formatting (plan §3.6, protocol §4).
//    GPL-3.0-or-later; see Config.h header.
//
//    A Plan is compiled at arm time into a short list of measurement *points*.
//    Point p starts its ADC reads at pulseStart + atCyc[p] and is placed so the
//    reads finish before the next latch event begins programming the DAC:
//
//        atCyc[p] = nextLatch - preloadCyc - budget
//        budget   = nReads * (ADC_READ + ADC_SWITCH) + GUARD
//
//    and the point is only accepted if preloadCyc + budget + SETTLE fits the
//    free gap it lives in — the stage duration for `S`, the ramp *sample
//    interval* for `L`, the sine sample interval for `W`. A point that does not
//    fit is kept, flagged in skipMask and reported with the summary; the
//    waveform is never stretched to make room for the instrumentation.
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
  uint8_t  nPoints;
  uint16_t skipMask;             // bit p: point p does not fit and never fires
  bool     envGate;              // measure only where the envelope is fully on
  uint8_t  peakChannel;          // SINE: channel whose phase the peaks follow
  bool     peakMismatch;         // SINE: the other measured channel differs from it
  uint32_t needCyc, roomCyc;     // fit arithmetic of the first refused point
  uint32_t budgetCyc;            // ADC window width used by every point
  uint64_t atCyc[SJ_MEAS_POINTS];   // ascending offsets from pulseStart
  uint16_t label[SJ_MEAS_POINTS];   // stage index (S/L) or peak degrees (W)
  uint8_t  chMask[SJ_MEAS_POINTS];  // channels read at this point
  Accum    acc[SJ_MEAS_POINTS][2][2];   // [point][channel][line]
  uint8_t  next;                 // live: next point to fire in the current pulse
  uint32_t envSkipped;           // repetitions dropped by the envelope gate
};

// ------------------------------------------------------------ pure plan math

// Compile `def`'s MeasDef against the geometry. Zeroes the accumulators.
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

// Compile engine `eng`'s plan at arm time; always overwrites the previous one
// (an unmeasured train leaves a plan with on == false, so a stale plan can
// never fire). A finished or manually stopped train keeps its accumulators
// until the next arm, which is what lets `T-1` still print a summary.
void armPlan(uint8_t eng, uint8_t slot, const TrainDef& def, const Geometry& g);

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

// End-of-train MSUM block, printed from loop() after the completion line.
void printSummary(uint8_t eng, uint8_t slot);
bool hasPlan(uint8_t eng);

#endif // ARDUINO

} // namespace Measure

#endif // STIMJIMAWG_MEASURE_H
