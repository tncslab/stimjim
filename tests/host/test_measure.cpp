//    stimjimAWG — host-side tests for Measure: the sine-peak solver, the
//    measurement-window fit arithmetic, stage selection and the single-pass
//    mean/spread estimator. No Arduino dependencies — build & run on the
//    development machine:
//
//      g++ -std=c++17 -Wall -Wextra -I stimjimAWG tests/host/test_measure.cpp
//          stimjimAWG/Measure.cpp -o test_measure          (one command line)
//      ./test_measure           (exit code = number of failed checks)
//
//    GPL-3.0-or-later; see stimjimAWG/Config.h header.

#include "Measure.h"
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <cstring>

using namespace Measure;

static int failures = 0;

#define CHECK(cond) do { \
    if (!(cond)) { failures++; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

#define CHECK_EQ(a, b) do { \
    long long va = (long long)(a), vb = (long long)(b); \
    if (va != vb) { failures++; \
      printf("FAIL %s:%d  %s=%lld != %s=%lld\n", __FILE__, __LINE__, #a, va, #b, vb); } \
  } while (0)

#define CHECK_NEAR(a, b, tol) do { \
    double va = (double)(a), vb = (double)(b); \
    if (fabs(va - vb) > (tol)) { failures++; \
      printf("FAIL %s:%d  %s=%.6f != %s=%.6f\n", __FILE__, __LINE__, #a, va, #b, vb); } \
  } while (0)

// The Teensy 3.5 register-path numbers the constants were sized for.
static const uint32_t CYC     = 120;                  // SJ_CYC_PER_US
static const uint32_t PRELOAD = 9 * CYC;              // SJ_PRELOAD_US + SJ_DAC_PROG2_US
static const uint32_t AREAD   = 3 * CYC;              // SJ_ADC_READ_US
static const uint32_t ASWITCH = 4 * CYC;              // SJ_ADC_SWITCH_US
static const uint32_t GUARD   = 1 * CYC;              // SJ_MEAS_GUARD_US
static const uint32_t SETTLE  = 4 * CYC;              // SJ_DAC_SETTLE_US

static const uint32_t PEAK_90  = 0x40000000u;
static const uint32_t PEAK_270 = 0xC0000000u;

// Budget and room for nReads values — the arithmetic the plan document states.
static uint32_t budgetOf(unsigned nReads) { return nReads * (AREAD + ASWITCH) + GUARD; }
static uint32_t roomOf(unsigned nReads)   { return PRELOAD + budgetOf(nReads) + SETTLE; }

static void fillGeometry(Geometry& g, uint8_t type, uint8_t chMask) {
  memset(&g, 0, sizeof g);
  g.type         = type;
  g.chMask       = chMask;
  g.preloadCyc   = PRELOAD;
  g.adcReadCyc   = AREAD;
  g.adcSwitchCyc = ASWITCH;
  g.guardCyc     = GUARD;
  g.settleCyc    = SETTLE;
}

static void defaultDef(TrainDef& t, uint8_t mode0, uint8_t mode1) {
  memset(&t, 0, sizeof t);
  t.mode0 = mode0;
  t.mode1 = mode1;
  t.meas.what0 = 3;
  t.meas.what1 = 3;
  t.meas.when  = 0;
  t.meas.stage = -1;
}

// ---------------------------------------------------------------- peak solver

static void testPeakSolver() {
  // 64 samples per cycle: one sample advances the Q32 phase by 2^26.
  const uint32_t inc = 0x04000000u;
  CHECK_EQ(peakSampleIndex(0, inc, PEAK_90),  16);   // 2^30 / 2^26
  CHECK_EQ(peakSampleIndex(0, inc, PEAK_270), 48);
  // Starting at 90 degrees the positive peak is sample 0 and the negative one
  // half a turn (32 samples) later.
  CHECK_EQ(peakSampleIndex(PEAK_90, inc, PEAK_90),  0);
  CHECK_EQ(peakSampleIndex(PEAK_90, inc, PEAK_270), 32);
  // Just past the peak: the crossing is a whole turn away, i.e. the last
  // sample of the period, never sample 0 again.
  CHECK_EQ(peakSampleIndex(PEAK_90 + 1, inc, PEAK_90), 64);

  // A phase increment that does not divide the target rounds up: the first
  // sample at or after the crossing.
  const uint32_t inc2 = 100000000u;                  // ~43 samples per turn
  const uint32_t k    = peakSampleIndex(0, inc2, PEAK_90);
  CHECK((uint64_t)k * inc2 >= PEAK_90);
  CHECK((uint64_t)(k - 1) * inc2 < PEAK_90);

  // Two samples per turn is the Nyquist edge the engine already refuses above;
  // the solver must still answer without dividing by zero or overflowing.
  CHECK_EQ(peakSampleIndex(0, 0x80000000u, PEAK_90), 1);
}

// ------------------------------------------------------------- HOLD / S plans

static void testHoldPlan() {
  TrainDef t;
  defaultDef(t, 0, 1);                               // both channels driven
  t.nStages = 3;
  const uint32_t dur[3] = {100, 200, 50};            // us
  uint64_t cum[4] = {0, 0, 0, 0};
  for (int i = 0; i < 3; i++) {
    t.stages[i].dur_us = dur[i];
    cum[i + 1] = cum[i] + (uint64_t)dur[i] * CYC;
  }
  Geometry g;
  fillGeometry(g, PIECEWISE_HOLD, 0b11);
  g.nStages = 3;
  g.cum     = cum;

  Plan p;
  planBuild(p, 7, t, g);
  CHECK(p.on);
  CHECK_EQ(p.slot, 7);
  CHECK_EQ(p.nPoints, 3);
  CHECK_EQ(p.skipMask, 0);
  CHECK_EQ(p.lines[0], 3);
  CHECK_EQ(p.lines[1], 3);
  CHECK_EQ(p.budgetCyc, budgetOf(4));
  CHECK(!p.envGate);
  for (uint8_t i = 0; i < 3; i++) {
    CHECK_EQ(p.label[i], i);
    CHECK_EQ(p.chMask[i], 0b11);
    // the reads end exactly where the next latch starts programming
    CHECK_EQ(p.atCyc[i], cum[i + 1] - PRELOAD - budgetOf(4));
    CHECK(p.atCyc[i] >= cum[i] + SETTLE);
  }
  // ascending, so the ISR can walk the list without sorting
  CHECK(p.atCyc[0] < p.atCyc[1] && p.atCyc[1] < p.atCyc[2]);
}

static void testHoldStageTooShort() {
  // V+I on both channels needs roomOf(4) = 42 us; a 30 us stage cannot host it
  // and must be refused rather than pushing the next latch late.
  TrainDef t;
  defaultDef(t, 0, 1);
  t.nStages = 2;
  t.stages[0].dur_us = 30;
  t.stages[1].dur_us = 500;
  uint64_t cum[3] = {0, 30u * CYC, (30u + 500u) * CYC};
  Geometry g;
  fillGeometry(g, PIECEWISE_HOLD, 0b11);
  g.nStages = 2;
  g.cum     = cum;

  Plan p;
  planBuild(p, 0, t, g);
  CHECK(p.on);
  CHECK_EQ(p.nPoints, 2);
  CHECK_EQ(p.skipMask, 1);                 // point 0 only
  CHECK_EQ(p.needCyc, roomOf(4));
  CHECK_EQ(p.roomCyc, 30u * CYC);

  // Measuring one line on one channel needs roomOf(1) = 21 us and fits.
  t.meas.what0 = 1;
  t.meas.what1 = 0;
  planBuild(p, 0, t, g);
  CHECK_EQ(p.lines[0], 1);
  CHECK_EQ(p.lines[1], 0);
  CHECK_EQ(p.budgetCyc, budgetOf(1));
  CHECK_EQ(p.skipMask, 0);
  CHECK_EQ(p.chMask[0], 1);                // channel 1 is not read at all
}

static void testStageSelection() {
  TrainDef t;
  defaultDef(t, 0, 1);
  t.nStages = 3;
  uint64_t cum[4] = {0, 100u * CYC, 300u * CYC, 600u * CYC};
  for (int i = 0; i < 3; i++) t.stages[i].dur_us = 100;
  Geometry g;
  fillGeometry(g, PIECEWISE_HOLD, 0b11);
  g.nStages = 3;
  g.cum     = cum;

  t.meas.stage = 1;
  Plan p;
  planBuild(p, 0, t, g);
  CHECK_EQ(p.nPoints, 1);
  CHECK_EQ(p.label[0], 1);
  CHECK_EQ(p.atCyc[0], cum[2] - PRELOAD - budgetOf(4));
}

static void testUndrivenAndDisabled() {
  Geometry g;
  uint64_t cum[2] = {0, 1000u * CYC};
  TrainDef t;

  // Channel 1 not driven (mode 3): MEAS may ask for it, the plan must not.
  defaultDef(t, 0, 3);
  t.nStages = 1;
  t.stages[0].dur_us = 1000;
  fillGeometry(g, PIECEWISE_HOLD, 0b01);
  g.nStages = 1;
  g.cum     = cum;
  Plan p;
  planBuild(p, 0, t, g);
  CHECK(p.on);
  CHECK_EQ(p.lines[1], 0);
  CHECK_EQ(p.chMask[0], 1);
  CHECK_EQ(p.budgetCyc, budgetOf(2));

  // what = 0 on every channel (the 90/91 mode sugar): nothing to measure.
  t.meas.what0 = 0;
  t.meas.what1 = 0;
  planBuild(p, 0, t, g);
  CHECK(!p.on);
  CHECK_EQ(p.nPoints, 0);
}

static void testEnvelopeGate() {
  TrainDef t;
  defaultDef(t, 0, 1);
  t.nStages = 1;
  t.stages[0].dur_us = 1000;
  t.env.rampIn_us = 5000;
  uint64_t cum[2] = {0, 1000u * CYC};
  Geometry g;
  fillGeometry(g, PIECEWISE_HOLD, 0b11);
  g.nStages = 1;
  g.cum     = cum;
  Plan p;
  planBuild(p, 0, t, g);
  CHECK(p.envGate);
  t.env.rampIn_us = 0;
  planBuild(p, 0, t, g);
  CHECK(!p.envGate);
}

// -------------------------------------------------------------- RAMP / L plan

static void testRampGapIsTheSampleInterval() {
  // A 1 ms ramp stage sampled every 20 us has 50 samples: the free gap is
  // 20 us, not the 1000 us stage, so a four-read point cannot fit.
  TrainDef t;
  defaultDef(t, 0, 1);
  t.nStages = 1;
  t.stages[0].dur_us = 1000;
  uint64_t cum[2] = {0, 1000u * CYC};
  uint32_t stageN[1] = {50};
  Geometry g;
  fillGeometry(g, PIECEWISE_RAMP, 0b11);
  g.nStages = 1;
  g.cum     = cum;
  g.stageN  = stageN;

  Plan p;
  planBuild(p, 0, t, g);
  CHECK_EQ(p.skipMask, 1);
  CHECK_EQ(p.roomCyc, 20u * CYC);          // 1000 us / 50 samples

  // Coarser sampling (10 samples => 100 us apart) leaves room.
  stageN[0] = 10;
  planBuild(p, 0, t, g);
  CHECK_EQ(p.skipMask, 0);
  CHECK_EQ(p.roomCyc, 0);
  CHECK_EQ(p.atCyc[0], cum[1] - PRELOAD - budgetOf(4));
}

// -------------------------------------------------------------- SINE / W plan

static void testSinePlan() {
  TrainDef t;
  defaultDef(t, 0, 1);
  t.meas.when = 3;                          // both peaks
  const uint32_t inc = 0x04000000u;         // 64 samples per turn
  Geometry g;
  fillGeometry(g, SINE, 0b11);
  g.sampleCyc    = 100 * CYC;               // 100 us per sample: room to spare
  g.burstCyc     = 100000u * CYC;
  g.phaseInit[0] = 0;
  g.phaseInit[1] = 0;
  g.phaseInc[0]  = inc;
  g.phaseInc[1]  = inc;

  Plan p;
  planBuild(p, 3, t, g);
  CHECK(p.on);
  CHECK_EQ(p.nPoints, 2);
  CHECK_EQ(p.skipMask, 0);
  CHECK_EQ(p.label[0], 90);
  CHECK_EQ(p.label[1], 270);
  CHECK(!p.peakMismatch);
  CHECK_EQ(p.peakChannel, 0);
  // reads start SETTLE after the peak sample latches
  CHECK_EQ(p.atCyc[0], (uint64_t)16 * g.sampleCyc + SETTLE);
  CHECK_EQ(p.atCyc[1], (uint64_t)48 * g.sampleCyc + SETTLE);

  // Only the negative peak.
  t.meas.when = 2;
  planBuild(p, 3, t, g);
  CHECK_EQ(p.nPoints, 1);
  CHECK_EQ(p.label[0], 270);

  // A start phase past 90 degrees puts the negative peak first, and the list
  // must still come out ascending.
  t.meas.when    = 3;
  g.phaseInit[0] = PEAK_90 + inc;           // one sample past the positive peak
  g.phaseInit[1] = g.phaseInit[0];
  planBuild(p, 3, t, g);
  CHECK_EQ(p.nPoints, 2);
  CHECK(p.atCyc[0] < p.atCyc[1]);
  CHECK_EQ(p.label[0], 270);
  CHECK_EQ(p.label[1], 90);
}

static void testSineTooFast() {
  TrainDef t;
  defaultDef(t, 0, 1);
  t.meas.when = 1;
  Geometry g;
  fillGeometry(g, SINE, 0b11);
  g.sampleCyc    = 20 * CYC;                // 20 us between samples: too tight
  g.burstCyc     = 10000u * CYC;
  g.phaseInc[0]  = 0x04000000u;
  g.phaseInc[1]  = 0x04000000u;

  Plan p;
  planBuild(p, 0, t, g);
  CHECK_EQ(p.nPoints, 1);
  CHECK_EQ(p.skipMask, 1);
  CHECK_EQ(p.needCyc, roomOf(4));
  CHECK_EQ(p.roomCyc, 20u * CYC);
}

static void testSinePeakChannelMismatch() {
  TrainDef t;
  defaultDef(t, 0, 1);
  t.meas.when = 1;
  Geometry g;
  fillGeometry(g, SINE, 0b11);
  g.sampleCyc   = 100 * CYC;
  g.burstCyc    = 100000u * CYC;
  g.phaseInc[0] = 0x04000000u;
  g.phaseInc[1] = 0x02000000u;              // half the frequency
  Plan p;
  planBuild(p, 0, t, g);
  CHECK(p.peakMismatch);
  CHECK_EQ(p.peakChannel, 0);

  // Only channel 1 measured: the peaks follow it and nothing is inconsistent.
  t.meas.what0 = 0;
  planBuild(p, 0, t, g);
  CHECK_EQ(p.peakChannel, 1);
  CHECK(!p.peakMismatch);
  CHECK_EQ(p.atCyc[0], (uint64_t)32 * g.sampleCyc + SETTLE);   // inc = 2^25
}

static void testSineBurstTooShort() {
  // The peak lies inside the burst, but the reads would run past its end.
  TrainDef t;
  defaultDef(t, 0, 1);
  t.meas.when  = 1;
  t.meas.what0 = 1;
  t.meas.what1 = 0;
  Geometry g;
  fillGeometry(g, SINE, 0b11);
  g.sampleCyc   = 100 * CYC;
  g.phaseInc[0] = 0x04000000u;              // positive peak at sample 16
  g.phaseInc[1] = 0x04000000u;
  g.burstCyc    = (uint64_t)16 * g.sampleCyc + SETTLE + 1;
  Plan p;
  planBuild(p, 0, t, g);
  CHECK_EQ(p.nPoints, 1);
  CHECK_EQ(p.skipMask, 1);
}

// ------------------------------------------------------------------ estimator

static void testAccumStats() {
  Accum a = {0, 0, 0};
  double mean = -1, sd = -1;
  CHECK(!accumStats(a, mean, sd));

  // One sample: mean is the sample, spread undefined and reported as 0.
  a = {1, 100, 100 * 100};
  CHECK(accumStats(a, mean, sd));
  CHECK_NEAR(mean, 100.0, 1e-12);
  CHECK_NEAR(sd, 0.0, 1e-12);

  // 10, 12, 14: mean 12, sample sd 2.
  a = {3, 36, 100 + 144 + 196};
  CHECK(accumStats(a, mean, sd));
  CHECK_NEAR(mean, 12.0, 1e-12);
  CHECK_NEAR(sd, 2.0, 1e-12);

  // Identical samples: the subtraction must not produce a negative variance
  // and then a NaN square root.
  a = {5, 5 * 4096, 5LL * 4096 * 4096};
  CHECK(accumStats(a, mean, sd));
  CHECK_NEAR(mean, 4096.0, 1e-12);
  CHECK_NEAR(sd, 0.0, 1e-12);

  // Negative codes accumulate the same way.
  a = {2, -30, 500};                        // -10 and -20
  CHECK(accumStats(a, mean, sd));
  CHECK_NEAR(mean, -15.0, 1e-12);
  CHECK_NEAR(sd, sqrt(50.0), 1e-12);
}

int main() {
  testPeakSolver();
  testHoldPlan();
  testHoldStageTooShort();
  testStageSelection();
  testUndrivenAndDisabled();
  testEnvelopeGate();
  testRampGapIsTheSampleInterval();
  testSinePlan();
  testSineTooFast();
  testSinePeakChannelMismatch();
  testSineBurstTooShort();
  testAccumStats();
  printf(failures ? "%d check(s) failed\n" : "all checks passed\n", failures);
  return failures;
}
