//    stimjimAWG — Engine: PIT deadline scheduler + channel players (plan §3.1/§3.3).
//    GPL-3.0-or-later; see Config.h header.
//
//    Phase 1 scope: reserve two PIT channels from the Teensyduino core, take
//    over their vectors at SJ_PLAYER_PRIO, self-calibrate K_RELOAD, and provide
//    the BENCH services (PIT wake/latch jitter). ChannelPlayer arrives Phase 3.

#ifndef STIMJIMAWG_ENGINE_H
#define STIMJIMAWG_ENGINE_H

#include <stdint.h>

namespace Engine {

// Reserve PIT channels (via IntervalTimer so the core can't reuse them),
// attach our vectors, set priorities, then run the K_RELOAD calibration.
void begin();

// Housekeeping from loop(): keeps cycles64 alive; Phase 3 adds completion drain.
void poll();

uint8_t  pitChannelOf(uint8_t player);   // hardware PIT channel index owned by player 0/1
uint32_t kReloadCycles();                // calibrated scheduling overhead (CPU cycles)

// Slot currently attached to a running player (-1 = idle). Drives the Phase-2
// edit-refusal ("ERR ... stop first") and the V/A-during-train WARN; players
// arrive in Phase 3, so the Phase-2 implementation always reports idle.
int16_t activeSlot(uint8_t player);
bool    anyActive();

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
