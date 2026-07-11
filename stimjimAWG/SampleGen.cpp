//    stimjimAWG — SampleGen implementation. Phase 4: ramp Bresenham +
//    envelope (sine follows in Phase 5). Pure math, host-testable.
//    GPL-3.0-or-later; see Config.h header.

#include "SampleGen.h"

namespace SampleGen {

// -------------------------------------------------------------------- ramps

void rampStageInit(RampStage& st, uint32_t dur_us, uint32_t cycPerUs,
                   uint32_t targetDt_us, int32_t start0, int32_t end0,
                   int32_t start1, int32_t end1) {
  // 64-bit sum: dur_us near UINT32_MAX must not wrap when the rounding bias
  // is added. N <= 2^32/targetDt so it fits int32 for targetDt >= 2.
  uint32_t N = (uint32_t)(((uint64_t)dur_us + targetDt_us / 2) / targetDt_us);
  if (N == 0) N = 1;
  st.N = N;

  uint64_t durCyc = (uint64_t)dur_us * cycPerUs;
  st.qt = durCyc / N;
  st.rt = (uint32_t)(durCyc % N);

  // Code axis: floor division so the remainder is always in [0, N) and the
  // carry test stays unsigned — works for down-ramps (dc < 0) too.
  int32_t dc0 = end0 - start0, dc1 = end1 - start1;
  int32_t q0 = dc0 / (int32_t)N, r0 = dc0 % (int32_t)N;
  if (r0 < 0) { q0--; r0 += (int32_t)N; }
  int32_t q1 = dc1 / (int32_t)N, r1 = dc1 % (int32_t)N;
  if (r1 < 0) { q1--; r1 += (int32_t)N; }
  st.qc0 = q0; st.rc0 = (uint32_t)r0;
  st.qc1 = q1; st.rc1 = (uint32_t)r1;
  st.end0 = end0;
  st.end1 = end1;
}

void rampStep(RampCursor& c, const RampStage& st) {
  c.k++;
  c.t += st.qt;
  c.tAcc += st.rt;
  if (c.tAcc >= st.N) { c.tAcc -= st.N; c.t++; }
  c.c0 += st.qc0;
  c.cAcc0 += st.rc0;
  if (c.cAcc0 >= st.N) { c.cAcc0 -= st.N; c.c0++; }
  c.c1 += st.qc1;
  c.cAcc1 += st.rc1;
  if (c.cAcc1 >= st.N) { c.cAcc1 -= st.N; c.c1++; }
}

void rampEnter(RampCursor& c, const RampStage& st, uint64_t stageStartCyc,
               int32_t start0, int32_t start1) {
  c.k = 0;
  c.t = stageStartCyc;
  c.tAcc = 0;                     // time axis: plain floor(k*durCyc/N)
  c.c0 = start0;
  c.c1 = start1;
  c.cAcc0 = c.cAcc1 = st.N >> 1;  // code axis: N/2 bias = round-half-up
  rampStep(c, st);                // cursor invariant: holds the next sample (k=1)
}

// ----------------------------------------------------------------- envelope

void envInit(EnvCoef& e, uint64_t t0, uint64_t durCyc,
             uint64_t rampInCyc, uint64_t rampOutCyc) {
  e.on       = (rampInCyc | rampOutCyc) != 0;
  e.t0       = t0;
  e.tEnd     = t0 + durCyc;
  e.inEnd    = t0 + rampInCyc;
  e.outStart = e.tEnd - rampOutCyc;
  // A 0-length ramp has an empty region (strict comparisons below), so its
  // reciprocal is never used — 0.0f keeps the float deterministic anyway.
  e.invIn  = rampInCyc  ? 1.0f / (float)rampInCyc  : 0.0f;
  e.invOut = rampOutCyc ? 1.0f / (float)rampOutCyc : 0.0f;
}

int32_t envQ15(const EnvCoef& e, uint64_t t) {
  if (t < e.inEnd)                              // ramp-in: [t0, inEnd)
    return (int32_t)((float)(t - e.t0) * e.invIn * 32768.0f + 0.5f);
  if (t > e.outStart) {                         // ramp-out: (outStart, tEnd]
    if (t >= e.tEnd) return 0;                  // events overrunning the train
    return (int32_t)((float)(e.tEnd - t) * e.invOut * 32768.0f + 0.5f);
  }
  return 32768;                                 // plateau (identity)
}

} // namespace SampleGen
