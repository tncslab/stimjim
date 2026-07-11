//    stimjimAWG — Engine implementation, Phase 1 scope. GPL-3.0-or-later.

#include "Engine.h"
#include "Config.h"
#include "FastIO.h"

namespace Engine {

static KINETISK_PIT_CHANNEL_t* const PIT = KINETISK_PIT_CHANNELS;

static IntervalTimer reserve[2];      // held forever so the core never re-allocates
static uint8_t  pitIdx[2] = {0xFF, 0xFF};
static uint32_t kReload   = 0;        // CPU cycles between the CYCCNT read inside
                                      // pitProgram() and the PIT actually firing

uint8_t  pitChannelOf(uint8_t player) { return pitIdx[player]; }
uint32_t kReloadCycles()              { return kReload; }

// ------------------------------------------------------------- scheduling core
//
// Program player `p`'s PIT so it fires when CYCCNT reaches deadlineCyc.
// Absolute deadlines: ISR latency affects each event by its own latency only
// and never accumulates. kReload compensates the software+peripheral overhead
// of this very function (self-calibrated at boot, see calibrateKReload).
static inline void pitProgram(uint8_t p, uint64_t deadlineCyc, bool enableIrq) {
  KINETISK_PIT_CHANNEL_t* ch = &PIT[pitIdx[p]];
  ch->TCTRL = 0;
  ch->TFLG  = 1;
  uint64_t now = FastIO::cycles64();
  int64_t  dc  = (int64_t)(deadlineCyc - now) - (int64_t)kReload;
  if (dc < 4) dc = 4;                       // already (nearly) due: fire ASAP
  uint32_t ticks = (uint32_t)((uint64_t)dc >> 1);   // CPU cycles -> PIT ticks (F_BUS = F_CPU/2)
  ch->LDVAL = ticks ? ticks - 1 : 0;                // PIT period = LDVAL+1 ticks
  ch->TCTRL = enableIrq ? (PIT_TCTRL_TEN | PIT_TCTRL_TIE) : PIT_TCTRL_TEN;
}

static inline void pitStop(uint8_t p) {
  KINETISK_PIT_CHANNEL_t* ch = &PIT[pitIdx[p]];
  ch->TCTRL = 0;
  ch->TFLG  = 1;
}

// ---------------------------------------------------------- PIT jitter bench

static struct {
  volatile uint32_t remaining;
  volatile bool     active;
  uint64_t deadline;        // ISR-private while active
  uint32_t periodCyc;
  uint32_t preloadCyc;
  PitBenchResult res;
} bench;

static void benchIsrBody() {
  pitStop(0);
  uint64_t dl  = bench.deadline;
  uint64_t now = FastIO::cycles64();
  if (bench.preloadCyc) {
    // Real-player behavior: woke PRELOAD early, spin on CYCCNT to the deadline.
    // The residue is < preload << 2^31, so 32-bit wrap-safe comparison suffices.
    while ((int32_t)(ARM_DWT_CYCCNT - (uint32_t)dl) < 0) ;
    now = FastIO::cycles64();
  }
  int32_t err = (int32_t)(now - dl);
  PitBenchResult* r = &bench.res;
  if (err < r->minErr) r->minErr = err;
  if (err > r->maxErr) r->maxErr = err;
  r->sumErr += err;
  // bin 0 = early; bins 1.. = 60-cycle (0.5 us) buckets of lateness
  uint32_t bin = (err < 0) ? 0 : (uint32_t)err / 60 + 1;
  if (bin >= SJ_BENCH_BINS) bin = SJ_BENCH_BINS - 1;
  r->hist[bin]++;
  r->n++;

  if (--bench.remaining) {
    bench.deadline = dl + bench.periodCyc;              // drift-free repeat chain
    pitProgram(0, bench.deadline - bench.preloadCyc, true);
  } else {
    bench.active = false;
  }
}

bool benchPitLatency(uint32_t period_us, uint32_t reps, uint32_t preload_us,
                     PitBenchResult* out) {
  if (bench.active) return false;
  bench.res = PitBenchResult();
  bench.res.minErr = INT32_MAX;
  bench.res.maxErr = INT32_MIN;
  bench.periodCyc  = period_us * SJ_CYC_PER_US;
  bench.preloadCyc = preload_us * SJ_CYC_PER_US;
  bench.remaining  = reps;
  bench.active     = true;
  bench.deadline   = FastIO::cycles64() + bench.periodCyc;
  pitProgram(0, bench.deadline - bench.preloadCyc, true);

  // Blocking wait in command context (players preempt us — that's fine).
  uint64_t timeout = FastIO::cycles64()
                   + (uint64_t)(reps + 4) * bench.periodCyc + SJ_US_TO_CYC(100000);
  while (bench.active) {
    if ((int64_t)(FastIO::cycles64() - timeout) > 0) {   // should never happen
      pitStop(0);
      bench.active = false;
      *out = bench.res;
      return false;
    }
  }
  *out = bench.res;
  return true;
}

// -------------------------------------------------------- K_RELOAD calibration
//
// Closed-loop: run the *actual* pitProgram() path with a known deadline and
// poll TFLG (interrupts off, TIE off), measuring when the timer really fired.
// The median error is folded into kReload. Residual detection latency of the
// poll loop (a few cycles) stays inside the constant, which is fine: players
// wake PRELOAD (~4 us) early and spin on CYCCNT, so K_RELOAD only needs to be
// accurate to well under PRELOAD, not to the ns.
static int32_t calOnce() {
  __disable_irq();
  uint64_t deadline = FastIO::cycles64() + SJ_US_TO_CYC(50);
  pitProgram(0, deadline, false);
  while (!(PIT[pitIdx[0]].TFLG & 1)) ;
  uint64_t fired = FastIO::cycles64();
  pitStop(0);
  __enable_irq();
  return (int32_t)(fired - deadline);
}

void calibrateKReload(uint16_t reps, int32_t* outMin, int32_t* outMed, int32_t* outMax) {
  static int32_t s[201];
  if (reps < 3)   reps = 3;
  if (reps > 201) reps = 201;
  for (uint16_t i = 0; i < reps; i++) {
    s[i] = calOnce();
    // insertion into sorted prefix so the median is a direct index
    for (uint16_t j = i; j > 0 && s[j - 1] > s[j]; j--) {
      int32_t t = s[j]; s[j] = s[j - 1]; s[j - 1] = t;
    }
  }
  int32_t med = s[reps / 2];
  kReload += med;               // pitProgram already subtracted the old value
  if (outMin) *outMin = s[0];
  if (outMed) *outMed = med;
  if (outMax) *outMax = s[reps - 1];
}

// ------------------------------------------------------------------ players
//
// Phase 3 replaces these with the ChannelPlayer event loop. In Phase 1 the
// vectors exist so BENCHPIT can run on player 0; player 1 just disarms itself.

static void player0Isr() {
  if (bench.active) { benchIsrBody(); return; }
  pitStop(0);
}

static void player1Isr() {
  pitStop(1);
}

// --------------------------------------------------------------------- begin

static void dummyIsr() {}

static uint32_t enabledMask() {
  uint32_t m = 0;
  for (int i = 0; i < 4; i++)
    if (PIT[i].TCTRL & PIT_TCTRL_TEN) m |= 1u << i;
  return m;
}

// Reserve one PIT channel through IntervalTimer (so the core marks it used),
// identify which hardware channel we got by diffing TCTRL enable bits, then
// take over its vector and priority. Works whatever the allocation order —
// no dependence on the core's low-to-high scan (verified but not relied on).
static uint8_t grabChannel(IntervalTimer& t, void (*isr)(void)) {
  uint32_t before = enabledMask();
  bool ok = t.begin(dummyIsr, 1000000);    // 1 s period: cannot fire before takeover
  uint32_t added = enabledMask() & ~before;
  if (!ok || !added) return 0xFF;
  uint8_t idx = __builtin_ctz(added);
  PIT[idx].TCTRL = 0;
  PIT[idx].TFLG  = 1;
  attachInterruptVector((IRQ_NUMBER_t)(IRQ_PIT_CH0 + idx), isr);
  NVIC_SET_PRIORITY(IRQ_PIT_CH0 + idx, SJ_PLAYER_PRIO);
  return idx;
}

void begin() {
  bench.active = false;
  pitIdx[0] = grabChannel(reserve[0], player0Isr);
  pitIdx[1] = grabChannel(reserve[1], player1Isr);
  // Failure here means another library consumed the PITs — impossible in this
  // sketch, and loud is better than subtly broken timing.
  if (pitIdx[0] == 0xFF || pitIdx[1] == 0xFF) {
    while (true) {
      Serial.println("ERR engine: could not reserve 2 PIT channels");
      delay(1000);
    }
  }
  calibrateKReload(65, nullptr, nullptr, nullptr);   // pass 1: bulk of the constant
  calibrateKReload(65, nullptr, nullptr, nullptr);   // pass 2: residual refinement
}

void poll() {
  (void)FastIO::cycles64();   // keep the 64-bit extension alive while idle
}

// Phase 2 stubs — ChannelPlayer state arrives in Phase 3.
int16_t activeSlot(uint8_t) { return -1; }
bool    anyActive()         { return false; }

} // namespace Engine
