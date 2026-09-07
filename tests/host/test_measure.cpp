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

#define CHECK_STR(got, want) do { \
    if (strcmp((got), (want))) { failures++; \
      printf("FAIL %s:%d  \"%s\" != \"%s\"\n", __FILE__, __LINE__, (got), (want)); } \
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

// 50 repetitions, and `fit` strict: a point that does not fit its gap is
// refused. The rotation tests below set fit = SJ_FIT_ROTATE explicitly, which
// keeps the two behaviours visibly separate in every other test.
static void defaultDef(TrainDef& t, uint8_t mode0, uint8_t mode1) {
  memset(&t, 0, sizeof t);
  t.mode0 = mode0;
  t.mode1 = mode1;
  t.period_us   = 10000;
  t.duration_us = 500000;
  t.meas.what0 = 3;
  t.meas.what1 = 3;
  t.meas.when  = 0;
  t.meas.stage = -1;
  t.meas.fit   = SJ_FIT_STRICT;
}

// Read mask bit of one (channel, line) pair, as Plan::grpMask packs it.
static uint8_t readBit(uint8_t ch, uint8_t line) { return (uint8_t)(1u << (ch * 2 + line)); }

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

// planBuild no longer zeroes the whole Plan: it clears the compiled region and
// the accumulators of the points it produced, because the full 1224-byte memset
// cost 5-7 us inside the start latency (docs/timing.md §7). Anything that lands
// in the struct after `acc` therefore has to be reset explicitly, and a field
// added there without that would silently inherit the previous train's value.
// Pre-poisoning the struct is what catches it.
static void testPlanResetsWhatItMustNotInherit() {
  TrainDef t;
  defaultDef(t, 0, 1);
  t.nStages = 2;
  uint64_t cum[3] = {0, 100 * CYC, 300 * CYC};
  t.stages[0].dur_us = 100;
  t.stages[1].dur_us = 200;
  Geometry g;
  fillGeometry(g, PIECEWISE_HOLD, 0b11);
  g.nStages = 2;
  g.cum     = cum;

  Plan p;
  memset(&p, 0xAA, sizeof p);       // every byte non-zero going in
  planBuild(p, 4, t, g);
  CHECK(p.on);
  CHECK_EQ(p.nPoints, 2);
  // result state of the points that exist
  for (uint8_t i = 0; i < p.nPoints; i++)
    for (uint8_t ch = 0; ch < 2; ch++)
      for (uint8_t ln = 0; ln < 2; ln++) {
        CHECK_EQ(p.acc[i][ch][ln].n, 0);
        CHECK_EQ(p.acc[i][ch][ln].sum, 0);
        CHECK_EQ(p.acc[i][ch][ln].sumsq, 0);
      }
  CHECK_EQ(p.next, 0);
  CHECK_EQ(p.envSkipped, 0);
  // the tag is the caller's to set, so planBuild must leave it invalid rather
  // than letting poison read as a valid cached compile
  CHECK(!p.tagValid);
  // the compiled region is zeroed, so masks and counters do not inherit either
  CHECK_EQ(p.skipMask, 0);
  CHECK_EQ(p.rotMask, 0);
  CHECK_EQ(p.needCyc, 0);

  // A plan that measures nothing returns early; the same must hold for it.
  t.meas.what0 = t.meas.what1 = 0;
  memset(&p, 0xAA, sizeof p);
  planBuild(p, 4, t, g);
  CHECK(!p.on);
  CHECK_EQ(p.nPoints, 0);
  CHECK_EQ(p.next, 0);
  CHECK_EQ(p.envSkipped, 0);
  CHECK(!p.tagValid);
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
  // Strict `fit` asked for all four reads in one gap, so that is what the
  // refusal reports — the rotation tests below check the other number.
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

// ------------------------------------------------------------------- rotation

static void testRotateHoldStage() {
  // Read windows on this board: roomOf(1..4) = 21, 28, 35, 42 us. A 25 us
  // stage therefore holds exactly one read per repetition, and the four reads
  // of a V+I-on-both-channels point become four groups.
  TrainDef t;
  defaultDef(t, 0, 1);
  t.meas.fit = SJ_FIT_ROTATE;
  t.nStages = 1;
  t.stages[0].dur_us = 25;
  uint64_t cum[2] = {0, 25u * CYC};
  Geometry g;
  fillGeometry(g, PIECEWISE_HOLD, 0b11);
  g.nStages = 1;
  g.cum     = cum;

  Plan p;
  planBuild(p, 0, t, g);
  CHECK_EQ(p.skipMask, 0);                 // measured after all
  CHECK_EQ(p.rotMask, 1);
  CHECK_EQ(p.nGrp[0], 4);
  CHECK_EQ(p.budCyc[0], budgetOf(1));
  CHECK(!p.rotShort);                      // 50 repetitions, 4 groups
  // one read per group, in (channel, line) order
  CHECK_EQ(p.grpMask[0][0], readBit(0, 0));
  CHECK_EQ(p.grpMask[0][1], readBit(0, 1));
  CHECK_EQ(p.grpMask[0][2], readBit(1, 0));
  CHECK_EQ(p.grpMask[0][3], readBit(1, 1));
  // placement uses the one-read window, so the point sits later than it would
  // with all four reads in one gap
  CHECK_EQ(p.atCyc[0], cum[1] - PRELOAD - budgetOf(1));

  // A 30 us stage takes two reads per gap: two groups of two, and the window
  // reserved is the two-read one.
  t.stages[0].dur_us = 30;
  cum[1] = 30u * CYC;
  planBuild(p, 0, t, g);
  CHECK_EQ(p.skipMask, 0);
  CHECK_EQ(p.rotMask, 1);
  CHECK_EQ(p.nGrp[0], 2);
  CHECK_EQ(p.budCyc[0], budgetOf(2));
  CHECK_EQ(p.grpMask[0][0], (uint8_t)(readBit(0, 0) | readBit(0, 1)));
  CHECK_EQ(p.grpMask[0][1], (uint8_t)(readBit(1, 0) | readBit(1, 1)));
  CHECK_EQ(p.atCyc[0], cum[1] - PRELOAD - budgetOf(2));

  // A stage with room for everything does not rotate: one group holds all four
  // reads, which is what keeps the unmeasured-path behaviour bit-identical.
  t.stages[0].dur_us = 1000;
  cum[1] = 1000u * CYC;
  planBuild(p, 0, t, g);
  CHECK_EQ(p.rotMask, 0);
  CHECK_EQ(p.nGrp[0], 1);
  CHECK_EQ(p.budCyc[0], budgetOf(4));
  CHECK_EQ(p.grpMask[0][0], 0x0F);
}

static void testRotateRamp() {
  // The headline coverage case: an L ramp with V+I on both channels. At the
  // default 20 us sample interval not even one read fits and rotation cannot
  // help; at 25 us one read fits per gap, so four repetitions cover the four
  // lines and the ramp itself is untouched.
  TrainDef t;
  defaultDef(t, 0, 1);
  t.meas.fit = SJ_FIT_ROTATE;
  t.nStages = 1;
  t.stages[0].dur_us = 1000;
  uint64_t cum[2] = {0, 1000u * CYC};
  uint32_t stageN[1] = {50};                // 1000 us / 50 = 20 us
  Geometry g;
  fillGeometry(g, PIECEWISE_RAMP, 0b11);
  g.nStages = 1;
  g.cum     = cum;
  g.stageN  = stageN;

  Plan p;
  planBuild(p, 0, t, g);
  CHECK_EQ(p.skipMask, 1);                  // rotation is not magic
  CHECK_EQ(p.rotMask, 0);
  CHECK_EQ(p.needCyc, roomOf(1));
  CHECK_EQ(p.roomCyc, 20u * CYC);

  stageN[0] = 40;                           // 25 us per sample
  planBuild(p, 0, t, g);
  CHECK_EQ(p.skipMask, 0);
  CHECK_EQ(p.rotMask, 1);
  CHECK_EQ(p.nGrp[0], 4);
  CHECK_EQ(p.atCyc[0], cum[1] - PRELOAD - budgetOf(1));
}

static void testRotationShortTrain() {
  // One repetition cannot cover four groups: the plan says so rather than
  // reporting three lines with n = 0 and no explanation.
  TrainDef t;
  defaultDef(t, 0, 1);
  t.meas.fit    = SJ_FIT_ROTATE;
  t.period_us   = 10000;
  t.duration_us = 10000;                    // exactly one pulse
  t.nStages = 1;
  t.stages[0].dur_us = 25;                   // one read per gap => four groups
  uint64_t cum[2] = {0, 25u * CYC};
  Geometry g;
  fillGeometry(g, PIECEWISE_HOLD, 0b11);
  g.nStages = 1;
  g.cum     = cum;

  Plan p;
  planBuild(p, 0, t, g);
  CHECK_EQ(p.reps, 1);
  CHECK_EQ(p.nGrp[0], 4);
  CHECK(p.rotShort);

  t.duration_us = 40000;                    // four pulses: enough
  planBuild(p, 0, t, g);
  CHECK_EQ(p.reps, 4);
  CHECK(!p.rotShort);
}

static void testRotateSine() {
  // 40 us between samples holds three reads but not four, so the four reads
  // split into two balanced groups of two.
  TrainDef t;
  defaultDef(t, 0, 1);
  t.meas.fit  = SJ_FIT_ROTATE;
  t.meas.when = 1;                          // positive peak only
  Geometry g;
  fillGeometry(g, SINE, 0b11);
  g.sampleCyc    = 40 * CYC;
  g.burstCyc     = 100000u * CYC;
  g.phaseInc[0]  = 0x04000000u;             // peak at sample 16
  g.phaseInc[1]  = 0x04000000u;

  Plan p;
  planBuild(p, 0, t, g);
  CHECK_EQ(p.nPoints, 1);
  CHECK_EQ(p.skipMask, 0);
  CHECK_EQ(p.rotMask, 1);
  CHECK_EQ(p.nGrp[0], 2);
  CHECK_EQ(p.budCyc[0], budgetOf(2));
  // rotation does not move the point: the reads still start SETTLE after the
  // peak sample latches, on every repetition
  CHECK_EQ(p.atCyc[0], (uint64_t)16 * g.sampleCyc + SETTLE);
}

// ------------------------------------------------------------------ estimator

static void testAccumStats() {
  // The extremes play no part in the mean/spread estimator, so they are left
  // at 0 here; testAccumRange covers them.
  Accum a = {0, 0, 0, 0, 0};
  double mean = -1, sd = -1;
  CHECK(!accumStats(a, mean, sd));

  // One sample: mean is the sample, spread undefined and reported as 0.
  a = {1, 100, 100 * 100, 0, 0};
  CHECK(accumStats(a, mean, sd));
  CHECK_NEAR(mean, 100.0, 1e-12);
  CHECK_NEAR(sd, 0.0, 1e-12);

  // 10, 12, 14: mean 12, sample sd 2.
  a = {3, 36, 100 + 144 + 196, 0, 0};
  CHECK(accumStats(a, mean, sd));
  CHECK_NEAR(mean, 12.0, 1e-12);
  CHECK_NEAR(sd, 2.0, 1e-12);

  // Identical samples: the subtraction must not produce a negative variance
  // and then a NaN square root.
  a = {5, 5 * 4096, 5LL * 4096 * 4096, 0, 0};
  CHECK(accumStats(a, mean, sd));
  CHECK_NEAR(mean, 4096.0, 1e-12);
  CHECK_NEAR(sd, 0.0, 1e-12);

  // Negative codes accumulate the same way.
  a = {2, -30, 500, 0, 0};                        // -10 and -20
  CHECK(accumStats(a, mean, sd));
  CHECK_NEAR(mean, -15.0, 1e-12);
  CHECK_NEAR(sd, sqrt(50.0), 1e-12);
}

// The decimal appenders that replaced the row path's five snprintf calls. They
// only earn their place if they are byte-identical to what they replaced, so
// this compares them against snprintf over the whole range each one can see.
static void testDecimalAppenders() {
  char got[32], want[32];

  // putU32 over every decade boundary and either side of it, plus the ceiling.
  uint32_t probes[] = {0u, 1u, 9u, 10u, 99u, 100u, 999u, 1000u, 65535u,
                       999999u, 1000000u, 2147483647u, 4294967295u};
  for (uint32_t v : probes) {
    *putU32(got, v) = '\0';
    snprintf(want, sizeof want, "%lu", (unsigned long)v);
    CHECK_STR(got, want);
  }
  // ... and exhaustively over the first few thousand, where the carry logic is.
  for (uint32_t v = 0; v < 5000u; v++) {
    *putU32(got, v) = '\0';
    snprintf(want, sizeof want, "%lu", (unsigned long)v);
    if (strcmp(got, want)) { CHECK_STR(got, want); break; }
  }

  // putU64 past the 32-bit edge, which is where the log timestamp lives after
  // 71 minutes of uptime.
  uint64_t big[] = {0ull, 4294967295ull, 4294967296ull, 1000000000000ull,
                    18446744073709551615ull};
  for (uint64_t v : big) {
    *putU64(got, v) = '\0';
    snprintf(want, sizeof want, "%llu", (unsigned long long)v);
    CHECK_STR(got, want);
  }

  // putCenti against the exact format it replaced, over the full range a
  // reading can reach (+-15 V in hundredths of a millivolt is +-1.5e6) and both
  // signs, including the -0.00 case the `neg` flag exists for.
  int32_t centi[] = {0, 1, 9, 10, 99, 100, 101, 999, 1000, 1500000,
                     -1, -99, -100, -1500000};
  for (int32_t c : centi) {
    for (int pass = 0; pass < 2; pass++) {
      const bool neg = (pass == 1);
      *putCenti(got, c, neg) = '\0';
      const uint32_t a = (c < 0) ? (uint32_t)(-(int64_t)c) : (uint32_t)c;
      snprintf(want, sizeof want, "%s%lu.%02lu", (neg || c < 0) ? "-" : "",
               (unsigned long)(a / 100u), (unsigned long)(a % 100u));
      CHECK_STR(got, want);
    }
  }
  // Exhaustive over a contiguous block, so no carry between the whole part and
  // the two decimals goes unchecked.
  for (int32_t c = -20000; c <= 20000; c++) {
    *putCenti(got, c, false) = '\0';
    const uint32_t a = (c < 0) ? (uint32_t)(-(int64_t)c) : (uint32_t)c;
    snprintf(want, sizeof want, "%s%lu.%02lu", (c < 0) ? "-" : "",
             (unsigned long)(a / 100u), (unsigned long)(a % 100u));
    if (strcmp(got, want)) { CHECK_STR(got, want); break; }
  }
}

// The extremes: seeded by the reset so they can never be mistaken for data,
// gated on n so an untouched accumulator reports nothing, and reported exactly
// as they were stored.
static void testAccumRange() {
  Accum a;
  memset(&a, 0, sizeof a);
  int16_t lo = 123, hi = 456;
  CHECK(!accumRange(a, lo, hi));              // n == 0: nothing to report
  CHECK_EQ(lo, 123);                          // ... and the outputs are untouched
  CHECK_EQ(hi, 456);

  a.n = 3; a.mn = -4096; a.mx = 8191;
  CHECK(accumRange(a, lo, hi));
  CHECK_EQ(lo, -4096);
  CHECK_EQ(hi, 8191);

  // planBuild must leave the sentinels in place, not the memset's zeros: 0 is an
  // ordinary reading, so a zeroed minimum would stick at 0 for a channel whose
  // codes are all positive.
  Plan pl;
  Geometry g;
  fillGeometry(g, PIECEWISE_HOLD, 0b11);
  uint64_t cum[3] = {0, 5000 * CYC, 10000 * CYC};
  g.nStages = 2;
  g.cum = cum;
  TrainDef def;
  defaultDef(def, 0, 1);
  planBuild(pl, 0, def, g);
  CHECK_EQ(pl.nPoints, 2);
  for (uint8_t i = 0; i < pl.nPoints; i++)
    for (uint8_t ch = 0; ch < 2; ch++)
      for (uint8_t ln = 0; ln < 2; ln++) {
        CHECK_EQ(pl.acc[i][ch][ln].mn, 32767);
        CHECK_EQ(pl.acc[i][ch][ln].mx, -32768);
        CHECK(!accumRange(pl.acc[i][ch][ln], lo, hi));
      }
}

int main() {
  testPeakSolver();
  testHoldPlan();
  testPlanResetsWhatItMustNotInherit();
  testHoldStageTooShort();
  testStageSelection();
  testUndrivenAndDisabled();
  testEnvelopeGate();
  testRampGapIsTheSampleInterval();
  testSinePlan();
  testSineTooFast();
  testSinePeakChannelMismatch();
  testSineBurstTooShort();
  testRotateHoldStage();
  testRotateRamp();
  testRotationShortTrain();
  testRotateSine();
  testAccumStats();
  testAccumRange();
  testDecimalAppenders();
  printf(failures ? "%d check(s) failed\n" : "all checks passed\n", failures);
  return failures;
}
