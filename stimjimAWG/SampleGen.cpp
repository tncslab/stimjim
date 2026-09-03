//    stimjimAWG — SampleGen implementation: ramp Bresenham, envelope, sine
//    table and phase-accumulator coefficients.
//    Pure math, host-testable. GPL-3.0-or-later; see Config.h header.

#include "SampleGen.h"
#include <math.h>

// rampStageInit and rampStep are the only SampleGen functions on a latency
// path — the first inside the start latency, once per ramp stage, the second
// inside the player ISR, once per sample. SJ_HOT places them in the same RAM
// section as Engine::startTrain and Engine::playerRun, so a call from RAM into
// flash does not give back what moving the callers there bought
// (docs/timing.md §7). Config.h pulls Arduino.h in, so the host tests — which
// compile this file on its own — take the empty definition instead.
#ifdef ARDUINO
  #include "Config.h"
#else
  #define SJ_HOT
#endif

namespace SampleGen {

// -------------------------------------------------------------------- ramps

SJ_HOT void rampStageInit(RampStage& st, uint32_t dur_us, uint32_t cycPerUs,
                   uint32_t targetDt_us, int32_t start0, int32_t end0,
                   int32_t start1, int32_t end1) {
  // N <= 2^32/targetDt so it fits int32 for targetDt >= 2. Both operands are
  // 32-bit, which is one hardware UDIV instead of a call into
  // __aeabi_uldivmod -- but dur_us is only bounded by strtoul, so a duration
  // within targetDt/2 of UINT32_MAX would wrap the rounding bias. That case
  // keeps the 64-bit sum.
  const uint32_t bias = targetDt_us / 2;
  uint32_t N = (dur_us <= UINT32_MAX - bias)
             ? (dur_us + bias) / targetDt_us
             : (uint32_t)(((uint64_t)dur_us + bias) / targetDt_us);
  if (N == 0) N = 1;
  st.N = N;

  // durCyc = dur_us * cycPerUs reaches 2.4e11, so durCyc/N and durCyc%N are a
  // 64-bit division as written -- ~100-150 cycles in __aeabi_uldivmod, twice,
  // and this runs per stage inside the start latency. Split it exactly instead:
  // with a = dur_us/N and b = dur_us%N (so dur_us = a*N + b, b < N),
  //   durCyc = a*N*cycPerUs + b*cycPerUs
  //   qt = durCyc/N = a*cycPerUs + (b*cycPerUs)/N      (a*N*cycPerUs divides by N)
  //   rt = durCyc%N =              (b*cycPerUs)%N
  // Same integer quotient and remainder, no approximation, four 32-bit UDIVs.
  // Valid while b*cycPerUs fits 32 bits (b < 2^32/120 ~ 35.8e6, i.e. any stage
  // shorter than ~35 s); the 64-bit form stays as the fallback above that.
  const uint32_t a = dur_us / N, b = dur_us % N;
  if (b <= UINT32_MAX / cycPerUs) {
    const uint32_t bc = b * cycPerUs;
    st.qt = (uint64_t)a * cycPerUs + bc / N;
    st.rt = bc % N;
  } else {
    const uint64_t durCyc = (uint64_t)dur_us * cycPerUs;
    st.qt = durCyc / N;
    st.rt = (uint32_t)(durCyc % N);
  }

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

SJ_HOT void rampStep(RampCursor& c, const RampStage& st) {
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

// --------------------------------------------------------------------- sine

int16_t SINE_TAB[1025];

void sineTabInit() {
  // double sin() once at boot (loop context — FP64 is fine there). [1024]
  // duplicates [0] so the interpolation never needs an index wrap.
  for (int i = 0; i <= 1024; i++)
    SINE_TAB[i] = (int16_t)lround(32767.0 * sin(6.283185307179586 * i / 1024.0));
}

int32_t sineQ15(uint32_t phase) {
  // top 10 bits: table interval; next 16 bits: Q16 interpolation fraction
  // (the bottom 6 phase bits are below the table's resolution — dropped)
  uint32_t idx  = phase >> 22;
  int32_t  frac = (int32_t)((phase >> 6) & 0xFFFF);
  int32_t  a = SINE_TAB[idx];
  return a + (((SINE_TAB[idx + 1] - a) * frac + 0x8000) >> 16);
}

uint32_t sinePhaseInc(uint32_t f_mHz, uint32_t sampleCyc, uint32_t cycPerUs) {
  // turns/sample = (f_mHz/1000) * sampleCyc / (cycPerUs*1e6); double keeps
  // 53-bit precision — the final integer rounding dominates (<= 0.5/2^32
  // turn/sample: a deterministic sub-1e-7 relative frequency offset, not an
  // accumulating error). Caller guarantees f < Fs/2 so the result fits.
  double turns = (double)f_mHz * (double)sampleCyc / ((double)cycPerUs * 1e9);
  return (uint32_t)(turns * 4294967296.0 + 0.5);
}

uint32_t sinePhaseInit(int32_t mdeg) {
  int32_t m = mdeg % 360000;
  if (m < 0) m += 360000;
  return (uint32_t)(((uint64_t)m << 32) / 360000u);
}

void sineDerive(SineConst& out, const SineDef& s, uint8_t chMask,
                uint32_t cycPerUs, uint32_t samplesPerCyc,
                uint32_t fsMinHz, uint32_t fsMaxHz) {
  const uint64_t f0 = (chMask & 1) ? s.freq0_mHz : 0;
  const uint64_t f1 = (chMask & 2) ? s.freq1_mHz : 0;
  uint64_t fs_mHz = (f0 > f1 ? f0 : f1) * samplesPerCyc;
  if (fs_mHz < (uint64_t)fsMinHz * 1000) fs_mHz = (uint64_t)fsMinHz * 1000;
  if (fs_mHz > (uint64_t)fsMaxHz * 1000) fs_mHz = (uint64_t)fsMaxHz * 1000;
  out.sampleCyc    = (uint32_t)(((uint64_t)cycPerUs * 1000000000ull + fs_mHz / 2) / fs_mHz);
  out.phaseInc[0]  = sinePhaseInc(s.freq0_mHz, out.sampleCyc, cycPerUs);
  out.phaseInc[1]  = sinePhaseInc(s.freq1_mHz, out.sampleCyc, cycPerUs);
  out.phaseInit[0] = sinePhaseInit(s.phase0_mdeg);
  out.phaseInit[1] = sinePhaseInit(s.phase1_mdeg);
}

} // namespace SampleGen
