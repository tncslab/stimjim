//    stimjimAWG — SampleGen: pure sample math, no Arduino dependencies so it is
//    host-testable (plan §3.5, tests/host/test_samplegen.cpp). Phase 4: ramp
//    Bresenham + envelope; Phase 5: sine table + phase accumulator.
//    GPL-3.0-or-later; see Config.h header.
//
//    Numeric design (plan §3.5, decided in Phase 4):
//    - Event *times* are 64-bit CPU-cycle counts, integer arithmetic only.
//      FP32 has a 24-bit mantissa (loses cycle exactness beyond 0.14 s) and
//      FP64 is software-emulated on the M4F — both unusable for deadlines.
//    - Instantaneous amplitudes are integer Q15 multiplies of DAC-code *deltas
//      relative to the channel offset* (so scaling never moves the baseline).
//    - The envelope needs one division (t / ramp length); it is replaced by an
//      arm-time FP32 reciprocal, leaving a single hardware-FPU multiply per
//      event (~20 cycles incl. lazy stacking). Worst-case FP32 error ~2^-24
//      is far below the 1/32768 Q15 quantum.
//    - Ramp interpolation is a Bresenham-style incremental division on both
//      the time and the code axis: no per-sample multiply/divide, no rounding
//      accumulation, and sample N lands *exactly* on the stage boundary and
//      end value (drift-free by construction).

#ifndef STIMJIMAWG_SAMPLEGEN_H
#define STIMJIMAWG_SAMPLEGEN_H

#include <stdint.h>

namespace SampleGen {

// ------------------------------------------------------------- Q15 helpers

// v * q15 / 32768, round-half-up (the +16384 bias before the arithmetic
// shift; for negative products this is round-toward-+inf by half an LSB —
// a sub-LSB asymmetry, irrelevant at DAC resolution). q15 = 32768 is the
// exact identity because (v<<15 + 16384)>>15 == v.
static inline int32_t scaleQ15(int32_t v, int32_t q15) {
  return (int32_t)(((int64_t)v * q15 + 16384) >> 15);
}

// ---------------------------------------------------- Phase 4: RAMP stages
//
// One `L` stage ramps from its entry value (previous stage's end, 0 for the
// first stage) to its programmed end value over dur_us, sampled at
// N = max(1, round(dur_us / targetDt_us)) points. Sample k (k = 1..N) lands at
//   t_k = stageStart + floor(k * durCyc / N)          [cycles, floor]
//   c_k = start + floor((k * dc + N/2) / N)           [codes, round-half-up]
// both realized incrementally: quotient step + remainder accumulator with
// carry (Bresenham), so the ISR does 3 additions and a compare per axis and
// k = N is exact on both axes. A 0-duration stage degenerates to N = 1,
// qt = rt = 0: a single sample at the stage start carrying the end value —
// the documented "instant jump".

struct RampStage {            // arm-time constants of one stage
  uint32_t N;                 // samples in this stage (>= 1)
  uint64_t qt;                // time-axis quotient step, cycles (= durCyc / N)
  uint32_t rt;                //   and remainder (= durCyc % N)
  int32_t  qc0, qc1;          // code-axis quotient steps (floor(dc / N), may be negative)
  uint32_t rc0, rc1;          //   and remainders in [0, N)
  int32_t  end0, end1;        // stage end values (DAC-code deltas rel. offset)
};

struct RampCursor {           // live state — holds the NEXT sample to latch
  uint32_t k;                 // 1..N: index of that sample within the stage
  uint64_t t;                 // its absolute deadline (cycles)
  int32_t  c0, c1;            // its values (DAC-code deltas rel. offset)
  uint32_t tAcc, cAcc0, cAcc1;// remainder accumulators
};

// Precompute a stage's Bresenham constants. start/end are DAC-code deltas at
// stage entry/exit; cycPerUs is exact (120 on Teensy 3.5).
void rampStageInit(RampStage& st, uint32_t dur_us, uint32_t cycPerUs,
                   uint32_t targetDt_us, int32_t start0, int32_t end0,
                   int32_t start1, int32_t end1);

// Enter a stage: reset the cursor to the stage start and advance it to the
// first sample (k = 1). start0/start1 must equal the previous stage's end
// (or 0 at pulse start) for value continuity.
void rampEnter(RampCursor& c, const RampStage& st, uint64_t stageStartCyc,
               int32_t start0, int32_t start1);

// Advance the cursor to the next sample (call after latching, while k < N).
void rampStep(RampCursor& c, const RampStage& st);

// ------------------------------------------------------ Phase 4: envelope
//
// env(t): 0 -> 1 linearly over rampIn from train start, 1 -> 0 ending exactly
// at duration (protocol §4). Evaluated per latch event against the event's
// absolute deadline, so L/W trains follow it per sample while S trains sample
// it at each stage latch (stair-step — documented; use L for smooth ramps).
// Events past tEnd (stages overrunning the train duration, legacy semantics)
// clamp to 0.

struct EnvCoef {
  bool     on;                // false = skip evaluation (env is identity)
  uint64_t t0, inEnd;         // ramp-in spans [t0, inEnd)
  uint64_t outStart, tEnd;    // ramp-out spans (outStart, tEnd]
  float    invIn, invOut;     // 1 / ramp length in cycles (arm-time reciprocals)
};

void envInit(EnvCoef& e, uint64_t t0, uint64_t durCyc,
             uint64_t rampInCyc, uint64_t rampOutCyc);

// Envelope value at absolute time t, Q15 in [0, 32768].
int32_t envQ15(const EnvCoef& e, uint64_t t);

// ---------------------------------------------------------- Phase 5: sine
//
// Q32 phase accumulator per channel: 2^32 = one turn, so wrap-around IS the
// 360 degree wrap — exact modular arithmetic, no drift. The only rounding is
// the one-time arm-time quantization of phaseInc (<= 0.5/2^32 turns/sample:
// a constant, deterministic frequency offset below 1e-7 relative, never an
// accumulating error). Samples come from a 1025-entry int16 Q15 full-wave
// table + linear interpolation (2 KB; max error ~1 LSB of the 16-bit DAC).

extern int16_t SINE_TAB[1025];    // [i] = round(32767 * sin(2*pi*i/1024)); [1024] = [0]
void sineTabInit();               // fill once at boot (double sin, loop context)

// Table lookup with linear interpolation. phase: Q32 turns. Result Q15.
int32_t sineQ15(uint32_t phase);

// Phase increment per sample for f (mHz) at one sample per sampleCyc CPU
// cycles. Caller must guarantee f < Fs/2 (result < 2^31). Computed in double
// (arm-time only): 53-bit precision, relative error 2^-53 — negligible next
// to the final integer rounding.
uint32_t sinePhaseInc(uint32_t f_mHz, uint32_t sampleCyc, uint32_t cycPerUs);

// Start phase: millidegrees (any sign) -> Q32 turns, exact integer math.
uint32_t sinePhaseInit(int32_t mdeg);

} // namespace SampleGen

#endif // STIMJIMAWG_SAMPLEGEN_H
