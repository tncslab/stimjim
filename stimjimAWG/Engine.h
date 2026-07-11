//    stimjimAWG — Engine: PIT deadline scheduler + channel players (plan §3.1/§3.3).
//    GPL-3.0-or-later; see Config.h header.
//
//    Phase 1: reserve two PIT channels from the Teensyduino core, take over
//    their vectors at SJ_PLAYER_PRIO, self-calibrate K_RELOAD, BENCH services.
//    Phase 3: ChannelPlayer — copy-on-arm start/stop, absolute-deadline HOLD
//    (`S`) playback with program-early/latch-on-deadline, seqlock status,
//    completion ring.
//    Phase 4: RAMP (`L`) playback (SampleGen Bresenham samples, 0-duration
//    jump chains) and the ENV envelope on S/L. SINE arrives in Phase 5.

#ifndef STIMJIMAWG_ENGINE_H
#define STIMJIMAWG_ENGINE_H

#include <stdint.h>
#include <stddef.h>
#include "WaveformDef.h"

namespace Engine {

// Reserve PIT channels (via IntervalTimer so the core can't reuse them),
// attach our vectors, set priorities, then run the K_RELOAD calibration.
void begin();

// Housekeeping from loop(): keeps cycles64 alive. Completion records are
// drained by Commands::poll() (all printing happens in loop context).
void poll();

uint8_t  pitChannelOf(uint8_t player);   // hardware PIT channel index owned by player 0/1
uint32_t kReloadCycles();                // calibrated scheduling overhead (CPU cycles)

// ------------------------------------------------------------ Phase 3 players
//
// Start `def` (copied — copy-on-arm, live serial editing stays safe) on engine
// 0 (`T`) or 1 (`U`). The first latch happens at now + SJ_START_LATENCY_US in
// PIT-ISR context (never in the caller's). S and L slots play as of Phase 4
// (SINE arrives in Phase 5). Returns false with a short reason in err (busy
// engine, channel conflict with the other engine, unsupported type) — caller
// prints the WARN/ERR (ignore-and-warn policy, plan §2.5).
bool startTrain(uint8_t eng, uint8_t slotIdx, const TrainDef& def,
                char* err, size_t errsz);

// `T-1`/`U-1` (loop context): disarm under busLock, park the DACs on their
// offsets and ground the claimed channels. Safe no-op when idle.
void stopTrain(uint8_t eng);

// Channels the engine's running train drives (bit0/bit1); 0 when idle.
uint8_t claimedMask(uint8_t eng);

// Slot currently attached to a running player (-1 = idle). Drives the
// edit-refusal ("ERR ... stop first") and the V/A/E-during-train WARN.
int16_t activeSlot(uint8_t player);
bool    anyActive();

// Lock-free snapshot for STAT (seqlock against the player ISR).
struct EngineStatus {
  int16_t  slot;         // -1 = idle (remaining fields 0)
  uint32_t nPulses;      // pulses completed so far
  uint32_t elapsed_us;   // since t0, clamped to [0, duration]
  uint32_t duration_us;
};
void status(uint8_t eng, EngineStatus& out);

// End-of-train records pushed by the player ISR, drained in loop context
// (Commands::poll — result summary printing). Manual stops don't push one
// (legacy `T-1` printed only its own line).
struct Completion {
  uint8_t  eng;
  uint8_t  slot;
  uint8_t  chMask;       // channels that were driven (now grounded)
  uint32_t nPulses;
};
bool popCompletion(Completion& out);

// Re-run the K_RELOAD calibration (BENCHK): schedules the *real* programming
// path with a known deadline and measures fire-time error by polling TFLG.
// Folds the median into the constant; fills min/median/max of the residuals.
void calibrateKReload(uint16_t reps, int32_t* outMin, int32_t* outMed, int32_t* outMax);

// BENCHPIT: run `reps` PIT wakeups every period_us on player 0's channel and
// histogram the error vs the absolute deadline. preload_us = 0 measures raw
// ISR wake latency (sizes SJ_PRELOAD_US); preload_us > 0 wakes early and spins
// on CYCCNT like the real player, measuring residual latch jitter.
// Blocking (called from command context); returns false if a run is active.
#define SJ_BENCH_BINS 24   // bin 0 = early, then 0.5 us (60-cycle) bins
struct PitBenchResult {
  uint32_t n;
  int32_t  minErr, maxErr;   // CPU cycles vs deadline (positive = late)
  int64_t  sumErr;
  uint32_t hist[SJ_BENCH_BINS];
};
bool benchPitLatency(uint32_t period_us, uint32_t reps, uint32_t preload_us,
                     PitBenchResult* out);

} // namespace Engine

#endif // STIMJIMAWG_ENGINE_H
