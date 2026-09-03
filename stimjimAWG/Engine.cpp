//    stimjimAWG — Engine implementation: the deadline scheduling core, the two
//    ChannelPlayers (HOLD / RAMP / SINE playback with the ENV envelope) and the
//    BENCH services. GPL-3.0-or-later.

#include "Engine.h"
#include "Config.h"
#include "FastIO.h"
#include "SampleGen.h"
#include "Triggers.h"
#include "Measure.h"
#include "Cal.h"
#include "TrainStore.h"   // slot definitions and the epoch the derived caches key on
#include <string.h>
#include <stdio.h>

namespace Engine {

static uint8_t  pitIdx[2] = {0xFF, 0xFF};
static uint32_t kReload   = 0;        // CPU cycles between the CYCCNT read inside
                                      // pitProgram() and the timer actually firing

// The ISR each player's timer must call. Indirection exists for two reasons:
// the portable backend has to hand the callback to IntervalTimer::begin() on
// every re-arm, and the K_RELOAD calibration temporarily borrows player 0's
// timer for its own measurement ISR.
static void (*isrFn[2])(void) = {nullptr, nullptr};

uint8_t  pitChannelOf(uint8_t player) { return pitIdx[player]; }
uint32_t kReloadCycles()              { return kReload; }

// ------------------------------------------------------------- scheduling core
//
// pitProgram(p, deadlineCyc, enableIrq) makes player `p`'s timer fire when
// CYCCNT reaches deadlineCyc; pitStop(p) disarms it. Absolute deadlines: ISR
// latency affects each event by its own latency only and never accumulates.
// kReload compensates the software+peripheral overhead of pitProgram itself
// and is self-calibrated at boot (calibrateKReload), so it absorbs whatever
// the selected backend costs — that is what makes the portable route usable
// without hand-tuned magic numbers.

#if SJ_TIMER_REGISTER
// ---- Backend A: raw Kinetis PIT channels with our own vectors --------------
static KINETISK_PIT_CHANNEL_t* const PIT = KINETISK_PIT_CHANNELS;
static IntervalTimer reserve[2];      // held forever so the core never re-allocates

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

#else
// ---- Backend B: IntervalTimer (portable; Teensy 4.x and others) ------------
//
// IntervalTimer is periodic, so each event re-arms it with the interval to the
// *next* deadline; playerRun() already calls pitStop() on entry, which both
// prevents a re-entry if an event body overruns its interval and releases the
// hardware channel. Re-acquiring it costs a short scan of the four channels
// inside begin() — part of what K_RELOAD absorbs. Nothing else in this sketch
// uses IntervalTimer, so the channel always comes straight back.
//
// The float overload keeps sub-microsecond resolution (the core multiplies by
// its timer clock in ticks/us before truncating), so the scheduling grid is
// not coarsened to 1 us.
static IntervalTimer tmr[2];

// Below this the core's begin() refuses the period (Teensy 4 needs >= 17 timer
// ticks) — fire as soon as the hardware allows instead.
static const uint32_t MIN_ARM_CYC = SJ_CYC_PER_US;   // 1 us

static inline void pitProgram(uint8_t p, uint64_t deadlineCyc, bool /*enableIrq*/) {
  uint64_t now = FastIO::cycles64();
  int64_t  dc  = (int64_t)(deadlineCyc - now) - (int64_t)kReload;
  if (dc < (int64_t)MIN_ARM_CYC) dc = MIN_ARM_CYC;   // already (nearly) due
  // The priority set once in begin() survives end()/begin() — IntervalTimer
  // keeps it in a member and re-applies it on every beginCycles().
  tmr[p].begin(isrFn[p], (float)dc / (float)SJ_CYC_PER_US);
}

static inline void pitStop(uint8_t p) {
  tmr[p].end();
}
#endif // SJ_TIMER_REGISTER

// ------------------------------------------------------------------ players
//
// Copy-on-arm state (plan §3.3/§3.5). Stage amplitudes are converted to
// DAC-code *deltas relative to the channel offset* (so the envelope scales
// them without moving the baseline) and stage boundaries to cumulative cycle
// offsets, so a latch event costs no unit math. The conversion of stage i > 0
// happens while stage i-1 plays rather than in the arm — one stage's worth of
// unit math per idle gap, which is what keeps the start latency independent of
// stage count. A train whose modes drive no channel
// (or an S/L train with 0 stages) degenerates to per-period bookkeeping
// (type HOLD, nStages = 0, chMask = 0) — the legacy "empty train".
//
// Per-pulse event sequences (all deadlines absolute, cycles):
//   HOLD  event e in [0..nStages] latches offset+d[e] at pulseStart + cum[e];
//         e = nStages is the "off" event parking the DAC on the offset.
//   RAMP  EV_INIT latches the offset at pulseStart (OE connect anchor), then
//         each stage emits its Bresenham samples (SampleGen::RampCursor, the
//         last one exactly on the stage boundary/end value), then EV_OFF
//         parks + grounds at pulseStart + cum[nStages]. A 0-duration stage is
//         one sample at its start time — the instant jump.
//   SINE  samples at pulseStart + k*sampleCyc while inside the burst (phase
//         accumulators restart at phaseInit each burst, so bursts are
//         identical and drift-free), then EV_OFF parks + grounds at
//         pulseStart + burstCyc.
struct Player {
  // arm-time constants (ISR-private while active)
  uint8_t  slot;
  uint8_t  chMask;                     // bit0/bit1 = channel driven
  uint8_t  outMode0, outMode1;         // OE code while a pulse is on (0 = V, 1 = I)
  uint8_t  type;                       // TrainType (HOLD also covers empty trains)
  uint8_t  nStages;
  int16_t  off0, off1;                 // park codes (mode's calibration offset)
  // Stage constants, derived lazily: entries [0, nDerived) are valid and the
  // player fills the rest while it plays (deriveStage/deriveAhead below). The
  // arm derives stage 0 alone, because that is the only one due at t0.
  int32_t  d0[SJ_MAX_STAGES];          // HOLD levels / RAMP stage-end values,
  int32_t  d1[SJ_MAX_STAGES];          //   DAC-code deltas rel. offset
  uint64_t cum[SJ_MAX_STAGES + 1];     // stage boundaries from pulse start, cycles
  SampleGen::RampStage rst[SJ_MAX_STAGES];
  StageDef stages[SJ_MAX_STAGES];      // copy-on-arm source the derivation reads
  uint32_t dtUs;                       // RAMP: sample interval in force
  uint8_t  nDerived;                   // stages [0, nDerived) are derived
  SampleGen::EnvCoef   env;
  uint32_t phaseInit0, phaseInit1;     // SINE: Q32 start phase (per-burst restart)
  uint32_t phaseInc0, phaseInc1;       // SINE: Q32 turns per sample
  int32_t  sAmp0, sAmp1;               // SINE: amplitude deltas rel. offset
  uint32_t sampleCyc;                  // SINE: exact cycles per sample
  uint64_t burstCyc;
  uint64_t periodCyc, durationCyc, delayCyc;
  uint32_t preloadCyc;                 // DAC programming budget + CYCCNT spin margin
  uint64_t t0;
  // live state
  uint64_t pulseStart;                 // current pulse's absolute start
  volatile uint32_t nPulses;
  uint8_t  evIdx;                      // HOLD: event index; RAMP: current stage
  uint8_t  evPhase;                    // RAMP/SINE: EV_INIT / EV_SAMP / EV_OFF
  SampleGen::RampCursor rc;
  uint32_t ph0, ph1;                   // SINE: live phase accumulators
  uint32_t sampK;                      // SINE: sample index within the burst
  uint64_t sampT;                      // SINE: absolute deadline of the next sample
  bool     meas;                       // this train has measurement points to fire
  bool     evOverdue;                  // the event being emitted was already due on arrival
  uint64_t prevEvDl;                   // previous emitted deadline (spots coincident events)
  uint32_t lateEvents;                 // latches whose programming overran (see progLatch)
  uint32_t maxLateCyc;                 //   and the worst overrun, CPU cycles
  uint32_t overdueEvents;              // events already due when the player reached them
  uint32_t maxOverdueCyc;              //   and the worst, CPU cycles
  uint16_t startNeedUs;                // 0, or the STARTLAT this arm needed (see Completion)
  volatile bool active;
  volatile uint32_t seq;               // seqlock: odd while the ISR updates
};
static Player player[2];

enum : uint8_t { EV_INIT = 0, EV_SAMP = 1, EV_OFF = 2 };

// Completion ring — SPSC: player ISRs produce, Commands::poll() consumes.
static Completion   compRing[8];
static volatile uint8_t compHead = 0, compTail = 0;

static void pushCompletion(uint8_t eng) {
  // Freeze this train's measurement plan before anything can re-arm the engine
  // onto the other buffer: printSummary reads the frozen one, and loop()'s
  // housekeeping leaves it alone until the summary is out.
  Measure::trainDone(eng);
  uint8_t h = compHead, next = (uint8_t)((h + 1) & 7);
  if (next == compTail) return;        // 8 unread completions — cannot happen in practice
  compRing[h].eng     = eng;
  compRing[h].slot    = player[eng].slot;
  compRing[h].chMask  = player[eng].chMask;
  compRing[h].nPulses = player[eng].nPulses;
  compRing[h].lateEvents = player[eng].lateEvents;
  compRing[h].maxLateCyc = player[eng].maxLateCyc;
  compRing[h].overdueEvents = player[eng].overdueEvents;
  compRing[h].maxOverdueCyc = player[eng].maxOverdueCyc;
  compRing[h].startNeedUs   = player[eng].startNeedUs;
  compHead = next;
}

void timingFaults(uint8_t eng, Completion& out) {
  out.lateEvents    = player[eng].lateEvents;
  out.maxLateCyc    = player[eng].maxLateCyc;
  out.overdueEvents = player[eng].overdueEvents;
  out.maxOverdueCyc = player[eng].maxOverdueCyc;
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
  Triggers::marker(false);   // a train that ends mid-pulse must not leave it high
}

static inline void spinTo(uint64_t dl) {
  while ((int64_t)(FastIO::cycles64() - dl) < 0) ;
}

// Program during the preload window, spin, latch exactly on the deadline.
//
// If programming has already overrun the deadline the spin is skipped and the
// latch happens as soon as it can, late. That is counted rather than hidden:
// the count is the timing design's acceptance measurement and it needs no
// oscilloscope. It counts only events that were still in the future when the
// player reached them, so what it measures is exactly "the programming budget
// was too small" -- most often a measurement window (SJ_ADC_*) or a DAC
// programming estimate (SJ_DAC_PROG*) that has not been recalibrated. Events
// that were *already* due on arrival are a different fault (coincident
// deadlines) and are counted separately in playerRun.
static inline void progLatch(Player& pl, uint64_t dl, int16_t c0, int16_t c1) {
  if (pl.chMask == 0b11)  FastIO::dacProgramBoth(c0, c1);
  else if (pl.chMask & 1) FastIO::dacProgram(0, c0);
  else                    FastIO::dacProgram(1, c1);
  uint64_t now = FastIO::cycles64();
  if ((int64_t)(now - dl) > 0) {
    if (!pl.evOverdue) {
      pl.lateEvents++;
      uint32_t by = (uint32_t)(now - dl);
      if (by > pl.maxLateCyc) pl.maxLateCyc = by;
    }
  } else {
    while ((int64_t)(FastIO::cycles64() - dl) < 0) ;
  }
  FastIO::dacLatch(pl.chMask);
}

// OE toggles per pulse like the legacy firmware: connect right after the
// pulse's first latch, back to ground right after the off latch. Marker-mode
// trigger pins follow the same edges, which is what "driven high during
// stimulus" means (protocol §3) — two GPIO writes at most.
static inline void oeConnect(const Player& pl) {
  if (pl.chMask & 1) Stimjim.setOutputMode(0, pl.outMode0);
  if (pl.chMask & 2) Stimjim.setOutputMode(1, pl.outMode1);
  Triggers::marker(true);
}
static inline void oeGround(const Player& pl) {
  if (pl.chMask & 1) Stimjim.setOutputMode(0, 3);
  if (pl.chMask & 2) Stimjim.setOutputMode(1, 3);
  Triggers::marker(false);
}

// offset + envelope-scaled delta, saturated to the DAC range. envq = 32768
// (identity) reproduces the un-enveloped codes bit-exactly (scaleQ15 is exact
// there and the saturation already happened in ampToDelta).
static inline int16_t mkCode(int16_t off, int32_t delta, int32_t envq) {
  int32_t v = off + ((envq == 32768) ? delta : SampleGen::scaleQ15(delta, envq));
  if (v >  32767) v =  32767;
  if (v < -32768) v = -32768;
  return (int16_t)v;
}

// ------------------------------------------------- lazy stage derivation
//
// Amplitude (mV or uA) -> DAC-code delta relative to `off`, the channel's
// calibration offset for the mode it is driven in. Identical float expression
// to the legacy pulse() conversion, so in-range amplitudes give bit-exact
// legacy codes; out-of-range ones (already WARNed at parse time) saturate
// instead of wrapping, and the saturation is why the offset cannot simply be
// cancelled out of the expression.
//
// Written against `off` rather than against Stimjim's offset tables because
// the player derives stages in ISR context and must not read live calibration
// state; the arm captured the same number into pl.off0/pl.off1.
static inline int32_t ampToDelta(int32_t amp, uint8_t mode, int16_t off) {
  int v = amp / ((mode == 0) ? MILLIVOLTS_PER_DAC : MICROAMPS_PER_DAC) + off;
  if (v >  32767) v =  32767;
  if (v < -32768) v = -32768;
  return v - off;
}

// Stage i's playback constants: the two DAC-code deltas and, for a ramp, the
// Bresenham constants carrying the stage from its predecessor's end value.
// Strictly in order — rst[i] starts where rst[i-1] finishes.
//
// This ran for every stage inside startTrain until phase 13, where it cost
// 0.89 us per `S` stage and 1.93 us per `L` stage of the start latency. Stage i
// is first read when stage i is entered, which for i > 0 is at least one stage
// duration after the first latch, so the arm now derives stage 0 — the only one
// due at t0 — and the player derives the rest as it plays
// (docs/PLAN_lazy-stages.md).
static SJ_HOT void deriveStage(Player& pl, uint8_t i) {
  const int32_t p0 = i ? pl.d0[i - 1] : 0;   // ramp entry value = previous stage's
  const int32_t p1 = i ? pl.d1[i - 1] : 0;   //   end; 0 at pulse start
  pl.d0[i] = (pl.chMask & 1) ? ampToDelta(pl.stages[i].a0, pl.outMode0, pl.off0) : 0;
  pl.d1[i] = (pl.chMask & 2) ? ampToDelta(pl.stages[i].a1, pl.outMode1, pl.off1) : 0;
  if (pl.type == PIECEWISE_RAMP)
    SampleGen::rampStageInit(pl.rst[i], pl.stages[i].dur_us, SJ_CYC_PER_US, pl.dtUs,
                             p0, pl.d0[i], p1, pl.d1[i]);
  pl.nDerived = (uint8_t)(i + 1);
}

// Guarantee stage `upto` is derived. Normally a no-op, because deriveAhead did
// it in idle time; correctness never depends on that, which is why every use
// site calls this and why the call sites sit *after* a latch rather than
// before one.
static inline void ensureDerived(Player& pl, uint8_t upto) {
  while (pl.nDerived <= upto && pl.nDerived < pl.nStages) deriveStage(pl, pl.nDerived);
}

// Derive stages while there is measurably room before `wake`, the instant the
// player is about to sleep until — idle time by construction, since the
// caller's next act is to program the timer for it and return.
//
// It derives as many as fit rather than exactly one, and that is what covers a
// 0-duration `L` jump: its single sample shares a deadline with the previous
// stage's last one, so entering it yields no gap of its own and the stage after
// it has to be ready already. Two 0-duration stages in a row are refused at
// parse time, so the chain is at most two stages long.
static SJ_HOT void deriveAhead(Player& pl, uint64_t wake) {
  const uint32_t cost = (uint32_t)SJ_US_TO_CYC(SJ_STAGE_DERIVE_US);
  while (pl.nDerived < pl.nStages) {
    if ((int64_t)(wake - FastIO::cycles64()) <= (int64_t)cost) return;
    deriveStage(pl, pl.nDerived);
  }
}

// The player event loop, entered from the PIT ISR. Future events are scheduled
// preload-early (program-early/latch-on-deadline); events within
// preload + MIN_SCHEDULE of now are processed inline in the same pass so
// 0-duration jump chains and overlong pulses never re-enter through the NVIC.
static SJ_HOT void playerRun(uint8_t p) {
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

    // ---- deadline of the next latch event (per type)
    uint64_t dl;
    if (pl.type == PIECEWISE_RAMP) {
      dl = (pl.evPhase == EV_INIT) ? pl.pulseStart
         : (pl.evPhase == EV_OFF)  ? pl.pulseStart + pl.cum[pl.nStages]
                                   : pl.rc.t;
    } else if (pl.type == SINE) {
      // burst end is a deadline comparison, not a sample count (plan §3.5);
      // burstCyc = 0 degenerates to the off event alone
      uint64_t burstEnd = pl.pulseStart + pl.burstCyc;
      if (pl.evPhase == EV_SAMP && pl.sampT >= burstEnd) pl.evPhase = EV_OFF;
      dl = (pl.evPhase == EV_OFF) ? burstEnd : pl.sampT;
    } else {                           // HOLD (also empty-train bookkeeping)
      dl = pl.pulseStart + pl.cum[pl.evIdx];
    }

    // A measurement point due before the next latch takes the slot. The plan
    // placed it so its ADC reads finish before this latch's preload window
    // opens, so handling it here delays nothing (plan §3.6). Absolute 64-bit
    // deadlines never wrap, so a plain compare is safe against the UINT64_MAX
    // "no more points" sentinel.
    bool isMeas = false;
    if (pl.meas) {
      uint64_t mdl = Measure::nextDeadline(p, pl.pulseStart);
      if (mdl < dl) { dl = mdl; isMeas = true; }
    }

    uint64_t now = FastIO::cycles64();
    if ((int64_t)(dl - now) > (int64_t)(pl.preloadCyc + SJ_US_TO_CYC(SJ_MIN_SCHEDULE_US))) {
      // genuinely future: wake preload-early; chunk long gaps (keeps the
      // 64-bit CYCCNT extension alive, PIT max load is ~71 s)
      uint64_t wake = dl - pl.preloadCyc;
      if ((int64_t)(wake - now) > (int64_t)SJ_US_TO_CYC(SJ_MAX_SLICE_US))
        wake = now + SJ_US_TO_CYC(SJ_MAX_SLICE_US);
      // The only genuinely idle point in the player: everything below returns
      // to the NVIC and waits for `wake`. Stages the arm no longer derives are
      // derived here, as many as the gap has room for.
      if (pl.nDerived < pl.nStages) deriveAhead(pl, wake);
      pitProgram(p, wake, true);
      return;
    }

    // ---- measurement point: spin to the instant, read, accumulate
    if (isMeas) {
      int32_t q = pl.env.on ? SampleGen::envQ15(pl.env, dl) : 32768;
      spinTo(dl);
      Measure::fire(p, pl.nPulses, dl, q);
      continue;
    }

    // A latch whose deadline has already passed cannot be put back on time by
    // anything downstream, so it is separated from a programming-budget miss
    // (counted in progLatch) and reported on its own. Measurement events are
    // above this point deliberately: they latch nothing, and a read that
    // starts a microsecond late only shifts a sample instant where the
    // waveform's derivative is zero (plan §3.6).
    //
    // Two events *sharing* a deadline are excluded: that is a property of the
    // waveform, not a timing failure. An `L` train latches its last ramp
    // sample and then parks at the same stage boundary, and a `W` burst with
    // burst_us == period_us puts the off event on top of a sample — both are
    // defined that way (protocol §2), and the second latch of such a pair is
    // necessarily a few microseconds behind the first.
    pl.evOverdue = ((int64_t)(dl - now) < 0);
    if (pl.evOverdue && dl != pl.prevEvDl) {
      pl.overdueEvents++;
      uint32_t by = (uint32_t)(now - dl);
      if (by > pl.maxOverdueCyc) pl.maxOverdueCyc = by;
    }
    pl.prevEvDl = dl;

    // ---- emit the event, advance per-type state
    if (pl.type == PIECEWISE_RAMP) {   // chMask != 0 guaranteed (see startTrain)
      if (pl.evPhase == EV_INIT) {
        // pulse starts from the parked offsets; this latch anchors OE connect
        progLatch(pl, dl, pl.off0, pl.off1);
        oeConnect(pl);
        pl.evIdx = 0;
        SampleGen::rampEnter(pl.rc, pl.rst[0], pl.pulseStart, 0, 0);
        pl.evPhase = EV_SAMP;
        continue;
      }
      if (pl.evPhase == EV_SAMP) {
        int32_t q = pl.env.on ? SampleGen::envQ15(pl.env, dl) : 32768;
        progLatch(pl, dl, mkCode(pl.off0, pl.rc.c0, q), mkCode(pl.off1, pl.rc.c1, q));
        if (pl.rc.k < pl.rst[pl.evIdx].N) {
          SampleGen::rampStep(pl.rc, pl.rst[pl.evIdx]);
          continue;
        }
        pl.evIdx++;                    // stage done — its last sample was exact
        if (pl.evIdx < pl.nStages) {
          ensureDerived(pl, pl.evIdx);   // no-op unless deriveAhead found no room
          const SampleGen::RampStage& prev = pl.rst[pl.evIdx - 1];
          SampleGen::rampEnter(pl.rc, pl.rst[pl.evIdx],
                               pl.pulseStart + pl.cum[pl.evIdx], prev.end0, prev.end1);
        } else {
          pl.evPhase = EV_OFF;         // park+ground at the same boundary deadline
        }
        continue;
      }
      progLatch(pl, dl, pl.off0, pl.off1);   // EV_OFF
      oeGround(pl);
    } else if (pl.type == SINE) {      // chMask != 0 guaranteed (see startTrain)
      if (pl.evPhase == EV_SAMP) {
        int32_t q = pl.env.on ? SampleGen::envQ15(pl.env, dl) : 32768;
        // sample = offset + env * (amp * sin(phase)); two Q15 multiplies
        progLatch(pl, dl,
            mkCode(pl.off0, SampleGen::scaleQ15(pl.sAmp0, SampleGen::sineQ15(pl.ph0)), q),
            mkCode(pl.off1, SampleGen::scaleQ15(pl.sAmp1, SampleGen::sineQ15(pl.ph1)), q));
        if (pl.sampK == 0) oeConnect(pl);
        pl.sampK++;
        pl.ph0 += pl.phaseInc0;        // Q32 wrap IS the 360-degree wrap
        pl.ph1 += pl.phaseInc1;
        pl.sampT += pl.sampleCyc;      // exact integer grid: drift-free
        continue;
      }
      progLatch(pl, dl, pl.off0, pl.off1);   // EV_OFF: park + ground
      oeGround(pl);
    } else if (pl.chMask) {            // HOLD
      int32_t q   = pl.env.on ? SampleGen::envQ15(pl.env, dl) : 32768;
      bool    off = (pl.evIdx == pl.nStages);
      progLatch(pl, dl, mkCode(pl.off0, off ? 0 : pl.d0[pl.evIdx], q),
                        mkCode(pl.off1, off ? 0 : pl.d1[pl.evIdx], q));
      if (pl.evIdx == 0)    oeConnect(pl);
      else if (off)         oeGround(pl);
      // Stage evIdx is derived before its latch by exactly this line one event
      // earlier (the arm derives stage 0), so the derivation always lands in
      // the interval between two latches and never inside a preload window.
      if (!off) { pl.evIdx++; ensureDerived(pl, pl.evIdx); continue; }
    }

    // off event done (or bookkeeping tick of an empty train): pulse complete
    pl.seq++;                          // odd: update in progress
    pl.nPulses = pl.nPulses + 1;
    pl.evIdx = 0;
    pl.pulseStart += pl.periodCyc;
    if (pl.meas) Measure::pulseDone(p);   // rewind to the plan's first point
    if (pl.type == SINE) {             // next burst: restart phase on the new grid
      pl.evPhase = EV_SAMP;
      pl.sampK = 0;
      pl.ph0 = pl.phaseInit0;
      pl.ph1 = pl.phaseInit1;
      pl.sampT = pl.pulseStart;
    } else {
      pl.evPhase = EV_INIT;
    }
    pl.seq++;                          // even again
  }
}

// ---------------------------------------------------------- start/stop (loop)

// ------------------------------------------------- definition-derived cache
//
// A sine train's Fs, phase increments and start phases come from its
// definition alone, so they are derived once per definition rather than on
// every arm. One entry per slot (20 B each) with a single shared epoch: any
// slot write bumps TrainStore::epoch(), which drops the whole table, and the
// next arm of a sine slot re-derives that one entry. `deriveSine` warms an
// entry from command context right after a slot is written, so in practice the
// arm only ever reads.
//
// Both loop() and a trigger ISR can reach the derivation (through
// buildGeometry), so a warm can be preempted halfway through writing an entry.
// That is harmless rather than lucky: the derivation is a pure function of the
// definition, the definition cannot change under it (only loop() writes slots,
// and it cannot preempt itself), and the valid bit is set after the write --
// so the preempting arm finds the entry invalid, derives the identical 20
// bytes itself, and reads back its own complete write. Two writers laying down
// the same bytes cannot tear into a third value.
static SampleGen::SineConst sineCache[SJ_NUM_SLOTS];
static uint32_t sineCacheEpoch = 0;                        // 0 = nothing derived yet
static uint32_t sineCacheValid[(SJ_NUM_SLOTS + 31) / 32];  // bit per slot

static void sineCacheCheckEpoch() {
  const uint32_t e = TrainStore::epoch();
  if (e == sineCacheEpoch) return;
  sineCacheEpoch = e;
  memset(sineCacheValid, 0, sizeof sineCacheValid);
}

// The channels a train's modes drive — needed before the Player exists,
// because Fs must not be raised by an undriven channel's frequency.
static inline uint8_t driveMask(const TrainDef& def) {
  uint8_t m = (uint8_t)(((def.mode0 <= 1) ? 1 : 0) | ((def.mode1 <= 1) ? 2 : 0));
  if (def.type != SINE && def.nStages == 0) m = 0;
  return m;
}

static const SampleGen::SineConst& sineConst(uint8_t slot, const TrainDef& def, uint8_t mask) {
  sineCacheCheckEpoch();
  if (!(sineCacheValid[slot >> 5] & (1u << (slot & 31)))) {
    SampleGen::sineDerive(sineCache[slot], def.sine, mask, SJ_CYC_PER_US,
                          SJ_SINE_SAMPLES_PER_CYC, SJ_FS_MIN_HZ, SJ_FS_MAX_HZ);
    sineCacheValid[slot >> 5] |= (uint32_t)(1u << (slot & 31));
  }
  return sineCache[slot];
}

void deriveSine(uint8_t slot) {
  const TrainDef& def = TrainStore::slotConst(slot);
  if (def.type != SINE) return;         // nothing to derive for S/L
  sineCacheCheckEpoch();
  SampleGen::sineDerive(sineCache[slot], def.sine, driveMask(def), SJ_CYC_PER_US,
                        SJ_SINE_SAMPLES_PER_CYC, SJ_FS_MIN_HZ, SJ_FS_MAX_HZ);
  sineCacheValid[slot >> 5] |= (uint32_t)(1u << (slot & 31));
}

// Everything the measurement plan is compiled against, derived from the slot's
// definition and the CAL set and from nothing else -- no Player, no start time,
// no calibration offsets. That last exclusion is why the plan tag does not
// carry the offsets: recalibrating them cannot change any of this.
//
// It exists so the plan can be compiled in loop() as well as in the arm
// (docs/PLAN_plan-out-of-arm.md), and there is exactly one copy of it because
// two would let a warm plan carry a tag claiming a geometry it does not
// describe. `cum` (SJ_MAX_STAGES+1 entries) and `stageN` (SJ_MAX_STAGES) are
// the caller's storage; `geo` points into them. Returns false, with a reason in
// `err` when that is non-NULL, for the trains startTrain refuses anyway.
SJ_HOT bool buildGeometry(uint8_t slotIdx, const TrainDef& def, const Cal::Def& cal,
                   uint64_t* cum, uint32_t* stageN, Measure::Geometry& geo,
                   uint32_t* dtUsOut, char* err, size_t errsz) {
  const uint8_t mask = driveMask(def);   // 0 = empty train: bookkeeping only
  // Programming budget of one latch on this train: the preload spin plus the
  // DAC write, which costs more when both channels are driven.
  const uint32_t progUs = (uint32_t)cal.us[Cal::PRELOAD] +
                          (mask == 0b11 ? cal.us[Cal::DACPROG2] : cal.us[Cal::DACPROG1]);
  const uint32_t dtUs   = def.dt_us ? def.dt_us : (uint32_t)SJ_TARGET_DT_US;
  *dtUsOut = dtUs;
  if (def.type == PIECEWISE_RAMP && mask && dtUs < progUs + SJ_MIN_SCHEDULE_US) {
    // A ramp interval below what one latch costs would schedule samples the
    // player cannot program in time — every one of them late. Refused here
    // rather than at parse time because the budget is a runtime quantity.
    if (err)
      snprintf(err, errsz, "ramp interval %lu us is below the %lu us this board needs "
               "per sample — start dropped", (unsigned long)dtUs,
               (unsigned long)(progUs + SJ_MIN_SCHEDULE_US));
    return false;
  }
  if (def.type == SINE && mask) {
    // Nyquist gate: above Fs/2 the phase increment exceeds half a turn per
    // sample — unrepresentable. Checked here (not at parse time) because
    // FS_MAX is an engine property (provisional pre-bench, plan §3.5).
    uint64_t f0 = (mask & 1) ? def.sine.freq0_mHz : 0;
    uint64_t f1 = (mask & 2) ? def.sine.freq1_mHz : 0;
    if ((f0 > f1 ? f0 : f1) > (uint64_t)SJ_FS_MAX_HZ * 1000 / 2) {
      if (err)
        snprintf(err, errsz, "sine frequency above Fs/2 = %d Hz — start dropped",
                 SJ_FS_MAX_HZ / 2);
      return false;
    }
  }

  const uint8_t type    = mask ? def.type : (uint8_t)PIECEWISE_HOLD;
  const uint8_t nStages = mask ? def.nStages : (uint8_t)0;
  uint64_t acc = 0;
  cum[0] = 0;
  for (uint8_t i = 0; i < nStages; i++) {
    acc += SJ_US_TO_CYC(def.stages[i].dur_us);
    cum[i + 1] = acc;
    // The sample count is all the plan needs of a ramp stage; the Bresenham
    // constants that go with it are the arm's business (SampleGen::rampStageN).
    stageN[i] = (type == PIECEWISE_RAMP) ? SampleGen::rampStageN(def.stages[i].dur_us, dtUs) : 1;
  }

  geo.type         = type;
  geo.chMask       = mask;
  geo.nStages      = nStages;
  geo.cum          = cum;
  geo.stageN       = (type == PIECEWISE_RAMP) ? stageN : nullptr;
  geo.preloadCyc   = (uint32_t)SJ_US_TO_CYC(progUs);
  geo.adcReadCyc   = (uint32_t)SJ_US_TO_CYC(cal.us[Cal::ADCREAD]);
  geo.adcSwitchCyc = (uint32_t)SJ_US_TO_CYC(cal.us[Cal::ADCSWITCH]);
  geo.guardCyc     = (uint32_t)SJ_US_TO_CYC(cal.us[Cal::GUARD]);
  geo.settleCyc    = (uint32_t)SJ_US_TO_CYC(cal.us[Cal::SETTLE]);
  if (type == SINE) {
    const SampleGen::SineConst& sc = sineConst(slotIdx, def, mask);
    geo.sampleCyc    = sc.sampleCyc;
    geo.burstCyc     = SJ_US_TO_CYC(def.sine.burst_us);
    geo.phaseInit[0] = sc.phaseInit[0];
    geo.phaseInit[1] = sc.phaseInit[1];
    geo.phaseInc[0]  = sc.phaseInc[0];
    geo.phaseInc[1]  = sc.phaseInc[1];
  } else {
    geo.sampleCyc    = 0;
    geo.burstCyc     = 0;
    geo.phaseInit[0] = geo.phaseInit[1] = 0;
    geo.phaseInc[0]  = geo.phaseInc[1]  = 0;
  }
  return true;
}

SJ_HOT bool startTrain(uint8_t eng, uint8_t slotIdx, const TrainDef& def,
                char* err, size_t errsz, uint64_t anchorCyc) {
  Player& pl = player[eng];
  if (pl.active) {
    snprintf(err, errsz, "engine busy (slot %u) — start dropped, stop with %c-1",
             pl.slot, eng ? 'U' : 'T');
    return false;
  }
  // The timing budgets are runtime state (Cal.h): one copy is taken here and
  // used for the whole arm, so a `CAL` line that lands mid-arm cannot make one
  // train use two different budgets.
  const Cal::Def cal = Cal::live();
  // stageN is read only while the plan is compiled, so a stack copy is enough
  // -- the Player does not carry the ramp sample counts.
  uint32_t stageN[SJ_MAX_STAGES];
  Measure::Geometry geo;
  uint32_t dtUs;
  // Writes pl.cum before the channel-conflict check below can refuse the start.
  // Safe: the busy check above passed, so this engine's player is idle and
  // nothing reads pl.cum until pl.active is set at the end of this function.
  if (!buildGeometry(slotIdx, def, cal, pl.cum, stageN, geo, &dtUs, err, errsz))
    return false;
  const uint8_t mask = geo.chMask;
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
  // undriven/empty trains degenerate to HOLD bookkeeping (0 stages, no events)
  pl.type     = geo.type;
  pl.nStages  = geo.nStages;
  // inter-pulse park level = the mode's calibration offset (legacy state);
  // stage amplitudes become deltas from it so the envelope can scale them
  pl.off0 = (int16_t)(pl.outMode0 ? Stimjim.currentOffsets[0] : Stimjim.voltageOffsets[0]);
  pl.off1 = (int16_t)(pl.outMode1 ? Stimjim.currentOffsets[1] : Stimjim.voltageOffsets[1]);

  if (pl.type == SINE) {
    // The Fs choice and the phase coefficients depend only on the definition,
    // and deriving them costs ~10 us -- most of it soft-float double inside
    // sinePhaseInc, which the M4F has no hardware for. buildGeometry read them
    // from the per-slot cache the `W` commit fills; only the amplitudes are
    // computed here, because they need the calibration offsets.
    pl.sampleCyc  = geo.sampleCyc;
    pl.phaseInc0  = geo.phaseInc[0];
    pl.phaseInc1  = geo.phaseInc[1];
    pl.phaseInit0 = geo.phaseInit[0];
    pl.phaseInit1 = geo.phaseInit[1];
    pl.sAmp0 = (mask & 1) ? ampToDelta(def.sine.amp0, pl.outMode0, pl.off0) : 0;
    pl.sAmp1 = (mask & 2) ? ampToDelta(def.sine.amp1, pl.outMode1, pl.off1) : 0;
    pl.burstCyc = geo.burstCyc;
  }
  // pl.cum was filled by buildGeometry; the stage amplitudes could not be,
  // because they read the calibration offsets and those are deliberately
  // outside the plan tag.
  //
  // Copy-on-arm of the stage triplets, so the derivation below and the ones the
  // player does later never read TrainStore: a slot write is refused only for
  // the slot a train is *attached to*, and that check is made in loop() while a
  // trigger edge can arm from an ISR. 12 bytes a stage, one memcpy.
  pl.dtUs = dtUs;
  if (pl.nStages) memcpy(pl.stages, def.stages, (size_t)pl.nStages * sizeof(StageDef));
  // Stage 0 is the only stage whose values are due at t0 itself, inside the
  // first latch's preload window; every later stage is derived by the player in
  // the gap before a latch it is already waiting for, which is what makes the
  // arm's cost independent of stage count (docs/PLAN_lazy-stages.md).
  pl.nDerived = 0;
  if (pl.nStages) deriveStage(pl, 0);

  pl.periodCyc   = SJ_US_TO_CYC(def.period_us);
  pl.durationCyc = SJ_US_TO_CYC(def.duration_us);
  pl.preloadCyc  = geo.preloadCyc;
  pl.nPulses     = 0;
  pl.lateEvents  = 0;
  pl.maxLateCyc  = 0;
  pl.overdueEvents = 0;
  pl.maxOverdueCyc = 0;
  pl.startNeedUs   = 0;
  pl.evOverdue     = false;
  pl.prevEvDl      = ~(uint64_t)0;     // cannot match a real deadline
  pl.evIdx       = 0;
  pl.evPhase     = EV_INIT;

  // Attach the measurement plan. This must stay on this side of the t0
  // assignment below: anything done after t0 is taken is subtracted from
  // START_LATENCY. Normally it is one index write -- loop() compiled the spare
  // buffer for this slot already (Engine::warmPlans) and cleared its
  // accumulators -- and it falls back to compiling here when it did not.
  Measure::armPlan(eng, slotIdx, def, geo, TrainStore::epoch(), Cal::epoch());
  pl.meas = Measure::hasPlan(eng);

  // t0 is taken *after* all precomputation so the arm->first-latch latency
  // stays the fixed STARTLAT regardless of train complexity. The slot's
  // delay_us is added on top: the train's whole timebase (pulse grid, envelope,
  // duration) starts at t0, so the delay shifts the waveform without changing
  // its length. It applies to every start path — trigger edge, T/U and menu —
  // so a delay can be verified over the serial port before it is wired to a
  // trigger.
  //
  // A caller that knows *when* the start was requested passes that cycle count
  // as `anchorCyc` (the trigger ISR timestamps the edge at its entry). t0 is
  // then measured from the request instead of from the end of this function,
  // so neither interrupt entry nor the precomputation above shows up in the
  // delivered latency, whatever the train's complexity. TRIGCOMP is the one
  // part that remains outside software's view — the hardware pin-to-ISR-entry
  // delay — and is subtracted so that edge + STARTLAT is what the output sees.
  // It defaults to 0: uncompensated until someone measures it on a scope.
  pl.delayCyc    = SJ_US_TO_CYC(def.delay_us);
  const uint64_t base = anchorCyc ? anchorCyc - SJ_US_TO_CYC(cal.us[Cal::TRIGCOMP])
                                  : FastIO::cycles64();
  pl.t0          = base + SJ_US_TO_CYC(cal.us[Cal::STARTLAT]) + pl.delayCyc;
  pl.pulseStart  = pl.t0;
  if (pl.type == SINE) {               // first burst starts at t0
    pl.evPhase = EV_SAMP;
    pl.sampK   = 0;
    pl.ph0     = pl.phaseInit0;
    pl.ph1     = pl.phaseInit1;
    pl.sampT   = pl.t0;
  }
  SampleGen::envInit(pl.env, pl.t0, pl.durationCyc,
                     SJ_US_TO_CYC(def.env.rampIn_us), SJ_US_TO_CYC(def.env.rampOut_us));
  if (!mask) pl.env.on = false;


  if (mask & 1) digitalWriteFast(LED0, HIGH);   // lit from arm, i.e. through the delay
  if (mask & 2) digitalWriteFast(LED1, HIGH);
  pl.active = true;
  // First latch in ISR context, never in the caller's. A delay longer than one
  // slice is chunked here the same way playerRun() chunks long inter-event
  // gaps — the timers cannot be loaded with an arbitrary number of cycles, and
  // the intermediate wakeups keep the 64-bit CYCCNT extension alive.
  uint64_t wake = pl.t0 - pl.preloadCyc;
  uint64_t now  = FastIO::cycles64();
  // Everything this function did since `base` was taken came out of STARTLAT.
  // If the first latch's preload window has already opened, the latch cannot
  // land on time and the delivered latency is no longer the fixed constant the
  // trigger path promises. Report the STARTLAT that would have covered this
  // arm instead of leaving the player's overdue counter as the only trace --
  // it is one `CAL STARTLAT` line away from correct, and BENCHARM measures the
  // arm directly. A slot delay counts as room, because it is: the check is
  // against t0, which the delay pushes out.
  if ((int64_t)(now - wake) > 0) {
    uint64_t shortCyc = now - wake;
    uint32_t shortUs  = (uint32_t)((shortCyc + SJ_CYC_PER_US - 1) / SJ_CYC_PER_US);
    uint32_t needUs   = (uint32_t)cal.us[Cal::STARTLAT] + shortUs;
    pl.startNeedUs = (needUs > 0xFFFFu) ? 0xFFFFu : (uint16_t)needUs;
  }
  if ((int64_t)(wake - now) > (int64_t)SJ_US_TO_CYC(SJ_MAX_SLICE_US))
    wake = now + SJ_US_TO_CYC(SJ_MAX_SLICE_US);
  pitProgram(eng, wake, true);
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
    if (pl.chMask == 0b11)  FastIO::dacProgramBoth(pl.off0, pl.off1);
    else if (pl.chMask & 1) FastIO::dacProgram(0, pl.off0);
    else                    FastIO::dacProgram(1, pl.off1);
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
  if (!pl.active) { out = EngineStatus{-1, 0, 0, 0, 0, 0, false}; return; }
  uint32_t n;
  do {                                 // seqlock: retry while the ISR is mid-update
    uint32_t s1 = pl.seq;
    if (s1 & 1) continue;
    n = pl.nPulses;
    if (pl.seq == s1) break;
  } while (true);
  uint64_t now = FastIO::cycles64();
  bool     pre = (int64_t)(pl.t0 - now) > 0;     // still inside the start delay
  uint64_t el  = pre ? 0 : (now - pl.t0);
  if (el > pl.durationCyc) el = pl.durationCyc;
  out.slot        = (int16_t)pl.slot;
  out.nPulses     = n;
  out.elapsed_us  = (uint32_t)SJ_CYC_TO_US(el);
  out.duration_us = (uint32_t)SJ_CYC_TO_US(pl.durationCyc);
  out.delay_us    = (uint32_t)SJ_CYC_TO_US(pl.delayCyc);
  out.remaining_delay_us = pre ? (uint32_t)SJ_CYC_TO_US(pl.t0 - now) : 0;
  out.waiting     = pre;
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
// measure when the timer really fired. The median error is folded into
// kReload. K_RELOAD only needs to be accurate to well under PRELOAD (players
// wake PRELOAD early and spin on CYCCNT), not to the ns — which is why the two
// backends may measure it slightly differently.
static void player0Isr();

#if SJ_TIMER_REGISTER
// Register backend: poll TFLG with interrupts off and TIE off. The few cycles
// of poll-detection latency stay inside the constant.
static bool calFailed = false;

static int32_t calOnce() {
  __disable_irq();
  uint64_t deadline = FastIO::cycles64() + SJ_US_TO_CYC(50);
  pitProgram(0, deadline, false);
  // Bounded spin. An unbounded one with interrupts off would turn any timer
  // misconfiguration into a dead board that not even USB answers. 1 ms is 20x
  // the scheduled interval, so the bound can only be hit by a real fault.
  uint64_t giveUp = deadline + SJ_US_TO_CYC(1000);
  while (!(PIT[pitIdx[0]].TFLG & 1)) {
    if ((int64_t)(FastIO::cycles64() - giveUp) > 0) { calFailed = true; break; }
  }
  uint64_t fired = FastIO::cycles64();
  pitStop(0);
  __enable_irq();
  return (int32_t)(fired - deadline);
}
#else
// Portable backend: IntervalTimer offers no way to run a channel without its
// interrupt, so the fire time is stamped by a borrowed ISR. That folds the
// NVIC entry latency into K_RELOAD as well — which is the right thing here,
// because on this route every real event pays it too.
static volatile uint64_t calFired;
static volatile bool     calDone;
static bool              calFailed = false;

static void calIsr() {
  calFired = FastIO::cycles64();
  pitStop(0);
  calDone = true;
}

static int32_t calOnce() {
  calDone  = false;
  isrFn[0] = calIsr;
  uint64_t deadline = FastIO::cycles64() + SJ_US_TO_CYC(50);
  pitProgram(0, deadline, true);
  uint64_t giveUp = deadline + SJ_US_TO_CYC(1000);       // bounded, see backend A
  while (!calDone) {
    if ((int64_t)(FastIO::cycles64() - giveUp) > 0) { calFailed = true; calFired = giveUp; break; }
  }
  isrFn[0] = player0Isr;
  return (int32_t)(calFired - deadline);
}
#endif

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
  if (calFailed) {
    // The timer never fired within the bound: folding this median in would
    // poison every future deadline. Leave K_RELOAD alone and say so — the
    // engine still runs, just with an uncompensated scheduling offset.
    Serial.println("WARN engine: K_RELOAD calibration timed out — timer not firing");
    calFailed = false;
  } else {
    kReload += med;             // pitProgram already subtracted the old value
  }
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

#if SJ_TIMER_REGISTER
static void dummyIsr() {}

// Ungate the PIT before any of its registers is read. On the K64 a load from a
// clock-gated peripheral is a bus fault, not a zero, and enabledMask() below
// samples TCTRL before the first IntervalTimer::begin() would have ungated it.
// The nop mirrors the core's own workaround comment ("solves timing problem on
// Teensy 3.5"); MCR = 1 is the core's setting (MDIS = 0 enable, FRZ = 1 freeze
// while halted in a debugger).
static void pitClockEnable() {
  SIM_SCGC6 |= SIM_SCGC6_PIT;
  __asm__ volatile("nop");
  PIT_MCR = 1;
}

static uint32_t enabledMask() {
  uint32_t m = 0;
  for (int i = 0; i < 4; i++)
    if (PIT[i].TCTRL & PIT_TCTRL_TEN) m |= 1u << i;
  return m;
}

// Reserve both PIT channels through IntervalTimer (so the core marks them
// used), identifying each by diffing the TCTRL enable bits around its begin().
// Works whatever the allocation order — no dependence on the core's
// low-to-high scan (verified but not relied on).
//
// Both channels must be claimed *before* either is disarmed: IntervalTimer
// picks a channel by scanning for TCTRL == 0, so clearing the first channel's
// TCTRL before claiming the second would hand the same channel out twice and
// leave both players sharing it. The 1 s period cannot elapse before the
// takeover below.
static bool grabChannels(void (*isr0)(void), void (*isr1)(void)) {
  uint32_t m0 = enabledMask();
  bool ok0 = reserve[0].begin(dummyIsr, 1000000);
  uint32_t m1 = enabledMask();
  bool ok1 = reserve[1].begin(dummyIsr, 1000000);
  uint32_t m2 = enabledMask();

  uint32_t a0 = m1 & ~m0, a1 = m2 & ~m1;
  if (!ok0 || !ok1 || !a0 || !a1) return false;
  pitIdx[0] = (uint8_t)__builtin_ctz(a0);
  pitIdx[1] = (uint8_t)__builtin_ctz(a1);

  void (*isr[2])(void) = {isr0, isr1};
  for (uint8_t p = 0; p < 2; p++) {
    uint8_t idx = pitIdx[p];
    PIT[idx].TCTRL = 0;
    PIT[idx].TFLG  = 1;
    attachInterruptVector((IRQ_NUMBER_t)(IRQ_PIT_CH0 + idx), isr[p]);
    NVIC_SET_PRIORITY(IRQ_PIT_CH0 + idx, SJ_PLAYER_PRIO);
  }
  return true;
}
#endif

void begin() {
  bench.active = false;
  memset(player, 0, sizeof(player));
  isrFn[0] = player0Isr;
  isrFn[1] = player1Isr;
#if SJ_TIMER_REGISTER
  pitClockEnable();
  // Failure here means another library consumed the PITs — impossible in this
  // sketch, and loud is better than subtly broken timing.
  if (!grabChannels(player0Isr, player1Isr)) {
    while (true) {
      Serial.println("ERR engine: could not reserve 2 PIT channels");
      delay(1000);
    }
  }
#else
  // The portable backend acquires and releases its channel per event, so there
  // is no fixed channel index to report; the priority set here persists across
  // every later end()/begin() pair (IntervalTimer keeps it in a member).
  for (uint8_t p = 0; p < 2; p++) {
    tmr[p].priority(SJ_PLAYER_PRIO);
    pitIdx[p] = 0xFE;                      // "IntervalTimer-managed"
  }
#endif
  calibrateKReload(65, nullptr, nullptr, nullptr);   // pass 1: bulk of the constant
  calibrateKReload(65, nullptr, nullptr, nullptr);   // pass 2: residual refinement
}

void poll() {
  (void)FastIO::cycles64();   // keep the 64-bit extension alive while idle
}

// The slot an enabled TRIG route would start on engine `eng`, or -1. Joint mode
// runs slot0 on engine 0 and nothing on engine 1; independent mode gives engine
// e its own slot. The two inputs are scanned in order, so a board whose inputs
// point different slots at one engine warms input 0's — the other one pays the
// compile in its arm, exactly as it did before.
static int16_t routedSlot(uint8_t eng) {
  for (uint8_t in = 0; in < 2; in++) {
    const TriggerRoute& r = Triggers::route(in);
    if (r.mode == 1) { if (eng == 0 && r.slot0 >= 0) return r.slot0; }
    else if (r.mode == 2) {
      const int8_t s = eng ? r.slot1 : r.slot0;
      if (s >= 0) return s;
    }
  }
  // No route names this engine, so the best guess at its next train is the slot
  // the operator just edited -- which is what a `T`/`U` start almost always
  // follows. Wrong guesses cost nothing: a compile in loop() that the arm then
  // does not use, and the arm compiles what it needs as it always did.
  return TrainStore::lastWritten();
}

void warmPlans() {
  const uint32_t defEpoch = TrainStore::epoch(), calEpoch = Cal::epoch();
  for (uint8_t eng = 0; eng < 2; eng++) {
    Measure::housekeep(eng);         // the deferred accumulator clear
    const int16_t slot = routedSlot(eng);
    if (slot < 0) continue;
    // Four loads: cheap enough to ask every loop() pass, and it keeps the
    // geometry build below out of the steady state entirely.
    if (Measure::planReady(eng, (uint8_t)slot, defEpoch, calEpoch)) continue;
    const TrainDef& def = TrainStore::slotConst((uint8_t)slot);
    uint64_t cum[SJ_MAX_STAGES + 1];
    uint32_t stageN[SJ_MAX_STAGES], dtUs;
    Measure::Geometry geo;
    // A train the arm would refuse has no plan to warm; the refusal message is
    // the arm's to print, so no error buffer is passed.
    if (!buildGeometry((uint8_t)slot, def, Cal::live(), cum, stageN, geo, &dtUs, nullptr, 0))
      continue;
    Measure::warmPlan(eng, (uint8_t)slot, def, geo, defEpoch, calEpoch);
  }
}

} // namespace Engine
