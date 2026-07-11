//    stimjimAWG — Engine implementation: Phase 1 scheduling core + BENCH,
//    Phase 3 ChannelPlayer (HOLD trains). GPL-3.0-or-later.

#include "Engine.h"
#include "Config.h"
#include "FastIO.h"
#include <string.h>
#include <stdio.h>

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

// ------------------------------------------------------------------ players
//
// Copy-on-arm state (plan §3.3). Stage amplitudes are pre-converted to DAC
// codes and stage boundaries to cumulative cycle offsets, so the ISR does no
// unit math. Event e in [0..nStages] latches code[e] at pulseStart + cum[e];
// event nStages is the "off" event returning the DAC to its offset. A train
// whose modes drive no channel (or with 0 stages) degenerates to per-period
// bookkeeping (nStages = 0, chMask = 0) — the legacy "empty train".
struct Player {
  // arm-time constants (ISR-private while active)
  uint8_t  slot;
  uint8_t  chMask;                     // bit0/bit1 = channel driven
  uint8_t  outMode0, outMode1;         // OE code while a pulse is on (0 = V, 1 = I)
  uint8_t  nStages;
  int16_t  code0[SJ_MAX_STAGES + 1];   // [nStages] = offset (off) code
  int16_t  code1[SJ_MAX_STAGES + 1];
  uint64_t cum[SJ_MAX_STAGES + 1];     // latch-event offsets from pulse start, cycles
  uint64_t periodCyc, durationCyc;
  uint32_t preloadCyc;                 // DAC programming budget + CYCCNT spin margin
  uint64_t t0;
  // live state
  uint64_t pulseStart;                 // current pulse's absolute start
  volatile uint32_t nPulses;
  uint8_t  evIdx;
  volatile bool active;
  volatile uint32_t seq;               // seqlock: odd while the ISR updates
};
static Player player[2];

// Completion ring — SPSC: player ISRs produce, Commands::poll() consumes.
static Completion   compRing[8];
static volatile uint8_t compHead = 0, compTail = 0;

static void pushCompletion(uint8_t eng) {
  uint8_t h = compHead, next = (uint8_t)((h + 1) & 7);
  if (next == compTail) return;        // 8 unread completions — cannot happen in practice
  compRing[h].eng     = eng;
  compRing[h].slot    = player[eng].slot;
  compRing[h].chMask  = player[eng].chMask;
  compRing[h].nPulses = player[eng].nPulses;
  compHead = next;
}

bool popCompletion(Completion& out) {
  if (compTail == compHead) return false;
  out = compRing[compTail];
  compTail = (uint8_t)((compTail + 1) & 7);
  return true;
}

static inline void groundClaimed(uint8_t chMask) {
  if (chMask & 1) { Stimjim.setOutputMode(0, 3); digitalWriteFast(LED0, LOW); }
  if (chMask & 2) { Stimjim.setOutputMode(1, 3); digitalWriteFast(LED1, LOW); }
}

// The player event loop, entered from the PIT ISR. Future events are scheduled
// preload-early (program-early/latch-on-deadline); events within
// preload + MIN_SCHEDULE of now are processed inline in the same pass so
// 0-duration stage chains and overlong pulses never re-enter through the NVIC.
static void playerRun(uint8_t p) {
  Player& pl = player[p];
  pitStop(p);
  if (!pl.active) return;

  for (;;) {
    // Train end is checked at pulse granularity: pulse k runs iff its start
    // lies before t0 + duration (legacy elapsed-vs-duration semantics).
    if (pl.pulseStart - pl.t0 >= pl.durationCyc) {
      groundClaimed(pl.chMask);        // outputs already parked by the off event
      pushCompletion(p);
      pl.active = false;
      return;
    }

    uint64_t dl  = pl.pulseStart + pl.cum[pl.evIdx];
    uint64_t now = FastIO::cycles64();
    if ((int64_t)(dl - now) > (int64_t)(pl.preloadCyc + SJ_US_TO_CYC(SJ_MIN_SCHEDULE_US))) {
      // genuinely future: wake preload-early; chunk long gaps (keeps the
      // 64-bit CYCCNT extension alive, PIT max load is ~71 s)
      uint64_t wake = dl - pl.preloadCyc;
      if ((int64_t)(wake - now) > (int64_t)SJ_US_TO_CYC(SJ_MAX_SLICE_US))
        wake = now + SJ_US_TO_CYC(SJ_MAX_SLICE_US);
      pitProgram(p, wake, true);
      return;
    }

    if (pl.chMask) {
      // program during the preload window, spin, latch exactly on the deadline
      if (pl.chMask == 0b11)  FastIO::dacProgramBoth(pl.code0[pl.evIdx], pl.code1[pl.evIdx]);
      else if (pl.chMask & 1) FastIO::dacProgram(0, pl.code0[pl.evIdx]);
      else                    FastIO::dacProgram(1, pl.code1[pl.evIdx]);
      while ((int64_t)(FastIO::cycles64() - dl) < 0) ;
      FastIO::dacLatch(pl.chMask);
      // OE toggles per pulse like the legacy firmware: connect right after the
      // stage-0 latch, back to ground right after the off latch.
      if (pl.evIdx == 0) {
        if (pl.chMask & 1) Stimjim.setOutputMode(0, pl.outMode0);
        if (pl.chMask & 2) Stimjim.setOutputMode(1, pl.outMode1);
      } else if (pl.evIdx == pl.nStages) {
        if (pl.chMask & 1) Stimjim.setOutputMode(0, 3);
        if (pl.chMask & 2) Stimjim.setOutputMode(1, 3);
      }
    }

    if (pl.evIdx < pl.nStages) { pl.evIdx++; continue; }

    // off event done (or bookkeeping tick of an empty train): pulse complete
    pl.seq++;                          // odd: update in progress
    pl.nPulses = pl.nPulses + 1;
    pl.evIdx = 0;
    pl.pulseStart += pl.periodCyc;
    pl.seq++;                          // even again
  }
}

// ---------------------------------------------------------- start/stop (loop)

// Identical float expression to the legacy pulse() conversion — bit-exact DAC
// codes for in-range amplitudes; out-of-range (already WARNed at parse time)
// saturates instead of wrapping. mode is 0 (voltage) or 1 (current) here.
static int16_t ampToCode(int32_t amp, uint8_t mode, uint8_t ch) {
  int v = amp / ((mode == 0) ? MILLIVOLTS_PER_DAC : MICROAMPS_PER_DAC)
        + ((mode == 0) ? Stimjim.voltageOffsets[ch] : Stimjim.currentOffsets[ch]);
  if (v >  32767) v =  32767;
  if (v < -32768) v = -32768;
  return (int16_t)v;
}

bool startTrain(uint8_t eng, uint8_t slotIdx, const TrainDef& def,
                char* err, size_t errsz) {
  Player& pl = player[eng];
  if (pl.active) {
    snprintf(err, errsz, "engine busy (slot %u) — start dropped, stop with %c-1",
             pl.slot, eng ? 'U' : 'T');
    return false;
  }
  if (def.type != PIECEWISE_HOLD) {
    snprintf(err, errsz, "only S (rectangular) slots play in Phase 3 — L arrives in Phase 4, W in Phase 5");
    return false;
  }

  uint8_t mask = (uint8_t)(((def.mode0 <= 1) ? 1 : 0) | ((def.mode1 <= 1) ? 2 : 0));
  if (def.nStages == 0) mask = 0;      // empty train: bookkeeping only
  const Player& other = player[eng ^ 1];
  if (other.active && (mask & other.chMask)) {
    snprintf(err, errsz, "channel conflict with the running %c train — start dropped",
             eng ? 'T' : 'U');
    return false;
  }

  pl.slot     = slotIdx;
  pl.chMask   = mask;
  pl.outMode0 = def.mode0 & 1;
  pl.outMode1 = def.mode1 & 1;
  pl.nStages  = mask ? def.nStages : 0;
  uint64_t acc = 0;
  pl.cum[0] = 0;
  for (uint8_t i = 0; i < pl.nStages; i++) {
    pl.code0[i] = (mask & 1) ? ampToCode(def.stages[i].a0, pl.outMode0, 0) : 0;
    pl.code1[i] = (mask & 2) ? ampToCode(def.stages[i].a1, pl.outMode1, 1) : 0;
    acc += SJ_US_TO_CYC(def.stages[i].dur_us);
    pl.cum[i + 1] = acc;
  }
  // the off event parks the DAC on the mode's offset (legacy inter-pulse state)
  pl.code0[pl.nStages] = (int16_t)(pl.outMode0 ? Stimjim.currentOffsets[0] : Stimjim.voltageOffsets[0]);
  pl.code1[pl.nStages] = (int16_t)(pl.outMode1 ? Stimjim.currentOffsets[1] : Stimjim.voltageOffsets[1]);

  pl.periodCyc   = SJ_US_TO_CYC(def.period_us);
  pl.durationCyc = SJ_US_TO_CYC(def.duration_us);
  pl.preloadCyc  = (uint32_t)SJ_US_TO_CYC(SJ_PRELOAD_US +
                     (mask == 0b11 ? SJ_DAC_PROG2_US : SJ_DAC_PROG1_US));
  pl.nPulses     = 0;
  pl.evIdx       = 0;
  pl.t0          = FastIO::cycles64() + SJ_US_TO_CYC(SJ_START_LATENCY_US);
  pl.pulseStart  = pl.t0;

  if (mask & 1) digitalWriteFast(LED0, HIGH);
  if (mask & 2) digitalWriteFast(LED1, HIGH);
  pl.active = true;
  pitProgram(eng, pl.t0 - pl.preloadCyc, true);   // first latch in ISR context
  return true;
}

void stopTrain(uint8_t eng) {
  Player& pl = player[eng];
  FastIO::busLock();                   // masks the player ISRs (BASEPRI)
  bool wasActive = pl.active;
  pl.active = false;
  pitStop(eng);
  if (wasActive && pl.chMask) {
    // park like a finished pulse: offsets latched, OE grounded, LEDs off
    if (pl.chMask == 0b11)  FastIO::dacProgramBoth(pl.code0[pl.nStages], pl.code1[pl.nStages]);
    else if (pl.chMask & 1) FastIO::dacProgram(0, pl.code0[pl.nStages]);
    else                    FastIO::dacProgram(1, pl.code1[pl.nStages]);
    FastIO::dacLatch(pl.chMask);
    groundClaimed(pl.chMask);
  }
  FastIO::busUnlock();
}

uint8_t claimedMask(uint8_t eng) { return player[eng].active ? player[eng].chMask : 0; }
int16_t activeSlot(uint8_t p)    { return player[p].active ? (int16_t)player[p].slot : -1; }
bool    anyActive()              { return player[0].active || player[1].active; }

void status(uint8_t eng, EngineStatus& out) {
  Player& pl = player[eng];
  if (!pl.active) { out.slot = -1; out.nPulses = 0; out.elapsed_us = 0; out.duration_us = 0; return; }
  uint32_t n;
  do {                                 // seqlock: retry while the ISR is mid-update
    uint32_t s1 = pl.seq;
    if (s1 & 1) continue;
    n = pl.nPulses;
    if (pl.seq == s1) break;
  } while (true);
  uint64_t now = FastIO::cycles64();
  uint64_t el  = (now > pl.t0) ? (now - pl.t0) : 0;
  if (el > pl.durationCyc) el = pl.durationCyc;
  out.slot        = (int16_t)pl.slot;
  out.nPulses     = n;
  out.elapsed_us  = (uint32_t)SJ_CYC_TO_US(el);
  out.duration_us = (uint32_t)SJ_CYC_TO_US(pl.durationCyc);
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
  if (bench.active || player[0].active) return false;   // bench borrows player 0's PIT
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

// ----------------------------------------------------------------------- ISRs

static void player0Isr() {
  if (bench.active) { benchIsrBody(); return; }
  playerRun(0);
}

static void player1Isr() {
  playerRun(1);
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
  memset(player, 0, sizeof(player));
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

} // namespace Engine
