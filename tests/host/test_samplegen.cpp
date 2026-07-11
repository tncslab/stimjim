//    stimjimAWG — host-side tests for SampleGen: ramp Bresenham iterator and
//    envelope (Phase 4), sine table/phase accumulator (Phase 5). No Arduino
//    dependencies — build & run on the development machine:
//
//      g++ -std=c++17 -Wall -Wextra -I stimjimAWG tests/host/test_samplegen.cpp
//          stimjimAWG/SampleGen.cpp -o test_samplegen     (one command line)
//      ./test_samplegen         (exit code = number of failed checks)
//
//    GPL-3.0-or-later; see stimjimAWG/Config.h header.

#include "SampleGen.h"
#include <cstdio>
#include <cstdint>
#include <cmath>

using namespace SampleGen;

static int failures = 0;

#define CHECK(cond) do { \
    if (!(cond)) { failures++; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

#define CHECK_EQ(a, b) do { \
    long long va = (long long)(a), vb = (long long)(b); \
    if (va != vb) { failures++; \
      printf("FAIL %s:%d  %s=%lld != %s=%lld\n", __FILE__, __LINE__, #a, va, #b, vb); } \
  } while (0)

static const uint32_t CYC = 120;   // SJ_CYC_PER_US on the Teensy 3.5
static const uint32_t DT  = 20;    // SJ_TARGET_DT_US

// Walk one stage with the cursor and check every sample against the closed
// forms: t_k = start + floor(k*durCyc/N), c_k = cStart + floor((k*dc+N/2)/N).
static void walkStage(uint32_t dur_us, int32_t s0, int32_t e0, int32_t s1, int32_t e1,
                      uint64_t stageStart) {
  RampStage st;
  rampStageInit(st, dur_us, CYC, DT, s0, e0, s1, e1);
  const uint64_t durCyc = (uint64_t)dur_us * CYC;
  const int64_t  dc0 = e0 - s0, dc1 = e1 - s1;
  RampCursor c;
  rampEnter(c, st, stageStart, s0, s1);
  for (uint32_t k = 1; k <= st.N; k++) {
    CHECK_EQ(c.k, k);
    CHECK_EQ(c.t, stageStart + (uint64_t)(( (unsigned long long)k * durCyc) / st.N));
    // floor((k*dc + N/2)/N) — round-half-up, exact in int64
    long long num0 = (long long)k * dc0 + st.N / 2;
    long long num1 = (long long)k * dc1 + st.N / 2;
    // C++ integer division truncates toward zero; emulate floor for negatives
    long long f0 = (num0 >= 0) ? num0 / st.N : -((-num0 + st.N - 1) / st.N);
    long long f1 = (num1 >= 0) ? num1 / st.N : -((-num1 + st.N - 1) / st.N);
    CHECK_EQ(c.c0, s0 + f0);
    CHECK_EQ(c.c1, s1 + f1);
    if (k < st.N) rampStep(c, st);
  }
  // drift-free stage boundary: the N-th sample is exact on both axes
  CHECK_EQ(c.t, stageStart + durCyc);
  CHECK_EQ(c.c0, e0);
  CHECK_EQ(c.c1, e1);
}

static void testRamp() {
  // divisible case: 200 us / 20 us = 10 samples, dc = 1000 -> steps of 100
  {
    RampStage st;
    rampStageInit(st, 200, CYC, DT, 0, 1000, 0, -1000);
    CHECK_EQ(st.N, 10);
    CHECK_EQ(st.qt, 2400);   // 24000 cycles / 10
    CHECK_EQ(st.rt, 0);
    RampCursor c;
    rampEnter(c, st, 1000000, 0, 0);
    CHECK_EQ(c.t, 1002400);
    CHECK_EQ(c.c0, 100);
    CHECK_EQ(c.c1, -100);
  }
  walkStage(200, 0, 1000, 0, -1000, 1000000);

  // non-divisible duration and negative slope: 250 us -> N = 13, 30000 % 13 != 0
  walkStage(250, 500, -777, -32768, 32767, 3);

  // sub-interval stage: dur < targetDt -> a single sample at the stage END
  {
    RampStage st;
    rampStageInit(st, 5, CYC, DT, 0, 300, 0, 0);
    CHECK_EQ(st.N, 1);
    RampCursor c;
    rampEnter(c, st, 100, 0, 0);
    CHECK_EQ(c.t, 100 + 5 * CYC);
    CHECK_EQ(c.c0, 300);
  }

  // 0-duration stage = instant jump: one sample at the stage start time
  {
    RampStage st;
    rampStageInit(st, 0, CYC, DT, 150, -20000, 0, 7);
    CHECK_EQ(st.N, 1);
    CHECK_EQ(st.qt, 0);
    CHECK_EQ(st.rt, 0);
    RampCursor c;
    rampEnter(c, st, 42, 150, 0);
    CHECK_EQ(c.t, 42);
    CHECK_EQ(c.c0, -20000);
    CHECK_EQ(c.c1, 7);
  }

  // rounding threshold of N: 9 us rounds to 0 -> clamped to 1; 10 us -> 1; 30 us -> 2
  {
    RampStage st;
    rampStageInit(st, 9, CYC, DT, 0, 1, 0, 1);
    CHECK_EQ(st.N, 1);
    rampStageInit(st, 10, CYC, DT, 0, 1, 0, 1);
    CHECK_EQ(st.N, 1);
    rampStageInit(st, 30, CYC, DT, 0, 1, 0, 1);
    CHECK_EQ(st.N, 2);
  }

  // long stage, full-scale swing: exactness must hold for big k*dc products
  walkStage(2000000, -32768, 32767, 32767, -32768, 0);   // 100k samples

  // two-stage continuity: stage 1 starts from stage 0's exact end value
  {
    RampStage a, b;
    rampStageInit(a, 130, CYC, DT, 0, 900, 0, -50);      // N = 7 (130/20 rounds)
    rampStageInit(b, 70, CYC, DT, 900, 100, -50, 60);    // N = 4 (70/20 rounds)
    RampCursor c;
    rampEnter(c, a, 0, 0, 0);
    while (c.k < a.N) rampStep(c, a);
    CHECK_EQ(c.t, 130 * CYC);
    CHECK_EQ(c.c0, 900);
    CHECK_EQ(c.c1, -50);
    rampEnter(c, b, c.t, a.end0, a.end1);
    while (c.k < b.N) rampStep(c, b);
    CHECK_EQ(c.t, 200 * CYC);
    CHECK_EQ(c.c0, 100);
    CHECK_EQ(c.c1, 60);
  }
}

static void testEnv() {
  // 100 us in, 200 us out, 1000 us duration (all in cycles below)
  const uint64_t t0 = 5000000;
  const uint64_t in = 100 * CYC, out = 200 * CYC, dur = 1000 * CYC;
  EnvCoef e;
  envInit(e, t0, dur, in, out);
  CHECK(e.on);
  CHECK_EQ(envQ15(e, t0), 0);                       // train start
  CHECK_EQ(envQ15(e, t0 + in / 2), 16384);          // mid ramp-in
  CHECK_EQ(envQ15(e, t0 + in), 32768);              // ramp-in complete
  CHECK_EQ(envQ15(e, t0 + dur / 2), 32768);         // plateau
  CHECK_EQ(envQ15(e, t0 + dur - out), 32768);       // last plateau instant
  CHECK_EQ(envQ15(e, t0 + dur - out / 2), 16384);   // mid ramp-out
  CHECK_EQ(envQ15(e, t0 + dur), 0);                 // train end
  CHECK_EQ(envQ15(e, t0 + dur + 12345), 0);         // overrunning events clamp

  // precision against a double reference across a long (10 s) ramp-in
  {
    EnvCoef el;
    const uint64_t inl = 10000000ull * CYC;         // 1.2e9 cycles
    envInit(el, 0, 2 * inl, inl, 0);
    for (uint64_t t = 0; t <= inl; t += inl / 97) {
      double ref = 32768.0 * (double)t / (double)inl;
      double got = (double)envQ15(el, t);
      CHECK(std::fabs(got - ref) <= 2.0);           // FP32 mantissa + rounding
    }
  }

  // one-sided ramps and the off switch
  EnvCoef ei;
  envInit(ei, 100, 1000, 0, 0);
  CHECK(!ei.on);
  envInit(ei, 100, 1000, 0, 400);                   // ramp-out only
  CHECK(ei.on);
  CHECK_EQ(envQ15(ei, 100), 32768);                 // no ramp-in: full from t0
  CHECK_EQ(envQ15(ei, 900), 16384);
  envInit(ei, 100, 1000, 400, 0);                   // ramp-in only
  CHECK_EQ(envQ15(ei, 300), 16384);
  CHECK_EQ(envQ15(ei, 1100), 32768);                // no ramp-out: flat through tEnd
  CHECK_EQ(envQ15(ei, 1101), 0);                    // overrunning events still clamp
}

static void testScaleQ15() {
  CHECK_EQ(scaleQ15(12345, 32768), 12345);          // identity is exact
  CHECK_EQ(scaleQ15(-12345, 32768), -12345);
  CHECK_EQ(scaleQ15(30000, 0), 0);
  CHECK_EQ(scaleQ15(1000, 16384), 500);
  CHECK_EQ(scaleQ15(-1000, 16384), -500);
  CHECK_EQ(scaleQ15(65535, 32768), 65535);          // full delta range, no overflow
  CHECK_EQ(scaleQ15(-65535, 32768), -65535);
  // round-half-up at the quantum boundary: 3 * 1/2 = 1.5 -> 2
  CHECK_EQ(scaleQ15(3, 16384), 2);
}

int main() {
  testRamp();
  testEnv();
  testScaleQ15();
  if (failures == 0) printf("test_samplegen: all checks passed\n");
  else               printf("test_samplegen: %d FAILURES\n", failures);
  return failures;
}
