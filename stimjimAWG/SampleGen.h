//    stimjimAWG — SampleGen: pure sample math, no Arduino dependencies so it is
//    host-testable (plan §3.5). Implemented in Phases 4 (ramps/envelope) and
//    5 (sine). GPL-3.0-or-later; see Config.h header.

#ifndef STIMJIMAWG_SAMPLEGEN_H
#define STIMJIMAWG_SAMPLEGEN_H

#include <stdint.h>

namespace SampleGen {

// --- Phase 4: RAMP stages ---------------------------------------------------
// Bresenham-style incremental division on both the time axis and the DAC-code
// axis: N = max(1, round(dur/TARGET_DT_US)) samples; sample k=N lands exactly
// on the stage boundary and end value.
struct RampIter {
  // state to be defined in Phase 4 (start/end codes, N, accumulated remainders)
};

// --- Phase 5: sine -----------------------------------------------------------
// Q32 phase accumulator; 1025-entry int16 Q15 quarter-wave-free full table with
// linear interpolation (worst-case error ~2.4e-6 FS, well under 1 DAC LSB).
struct SineState {
  uint32_t phaseAcc;    // Q32 turns
  uint32_t phaseInc;    // = f_mHz * 2^32 / (1000 * Fs)
  int32_t  ampQ;        // arm-time fixed-point amplitude coefficient (DAC codes)
};

// --- Phase 4: envelope --------------------------------------------------------
// env(t): 0->1 over rampIn from train start, 1->0 ending exactly at duration;
// evaluated per event as a Q15 multiply.

} // namespace SampleGen

#endif // STIMJIMAWG_SAMPLEGEN_H
