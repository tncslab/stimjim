//    stimjimAWG — waveform definition types (plan §4, protocol §2/§4/§5).
//    GPL-3.0-or-later; see Config.h header.

#ifndef STIMJIMAWG_WAVEFORMDEF_H
#define STIMJIMAWG_WAVEFORMDEF_H

#include <stdint.h>
#include "Cal.h"       // EepromImage carries the CAL timing budget

#define SJ_NUM_SLOTS    100
#define SJ_MAX_STAGES   10
// Slots persisted by `P`. Overridable from the build: the image must fit the
// board EEPROM (4096 B on Teensy 3.5 and 4.1, only 1080 B on a Teensy 4.0 --
// TrainStore.cpp static_asserts this and names the fix).
#ifndef SJ_EEPROM_SLOTS
#define SJ_EEPROM_SLOTS 10
#endif
// Ceiling of the per-slot post-trigger delay (2000 s). Defined here rather than in
// Config.h because the host-testable parser validates against it.
#define SJ_MAX_DELAY_US 2000000000u
// Bounds of the per-train ramp sample interval (`L` only; 0 = use the build
// default SJ_TARGET_DT_US). The floor here is a parser sanity bound only — the
// interval the player can actually keep up with depends on the CAL budgets and
// is checked at start, which is where the arithmetic is known.
#define SJ_MIN_DT_US 2u
#define SJ_MAX_DT_US 1000000u

enum TrainType : uint8_t {
  PIECEWISE_HOLD = 0,   // `S` — rectangular steps, legacy bit-exact semantics
  PIECEWISE_RAMP = 1,   // `L` — linear ramp between points; 0-duration stage = jump
  SINE           = 2,   // `W`
};

// One `S`/`L` triplet. Amplitudes in mV (voltage modes) or uA (current modes).
struct StageDef {
  int32_t  a0, a1;
  uint32_t dur_us;
};

struct SineDef {
  int32_t  amp0, amp1;            // mV or uA
  uint32_t burst_us;              // burst length per period
  uint32_t freq0_mHz, freq1_mHz;  // stored in mHz (protocol accepts decimal Hz)
  int32_t  phase0_mdeg, phase1_mdeg; // start phase in millidegrees — applied (fixes legacy bug)
};

// `ENV` — amplitude envelope: 0->1 over rampIn from train start, 1->0 ending
// exactly at duration_us. shape 0 = linear (1 = raised-cosine reserved).
struct EnvDef {
  uint32_t rampIn_us, rampOut_us;
  uint8_t  shape;
};

// `MEAS` — what: 0 none, 1 V, 2 I, 3 both (mode 90/91 sugar in train lines maps
// onto what=0); when: type-specific — S/L: 0 = near stage end; W: 1 = +peak,
// 2 = -peak, 3 = both peaks (one period per burst); stage: -1 = all stages,
// 0..nStages-1 = that stage only (S/L; W requires -1);
// report bitmask: +1 stream MDATA, +2 log to SD (summary always kept);
// fit: what to do when the reads of one point do not fit the free gap they
// live in — 0 = refuse the point (loud, nothing measured), 1 = rotate, i.e.
// split the reads into groups that do fit and fire one group per repetition.
// Rotation never moves a read away from the instant its label names; it only
// spreads the lines over consecutive repetitions, so each line accumulates
// about nPulses/nGroups samples.
struct MeasDef {
  uint8_t what0, what1, when;
  int8_t  stage;
  uint8_t report;
  uint8_t fit;
};
#define SJ_FIT_STRICT 0
#define SJ_FIT_ROTATE 1

struct TrainDef {
  uint8_t  type;          // TrainType
  uint8_t  mode0, mode1;  // original numbering 0-3 (see `M`); 2/3 = channel not
                          // driven. 90/91 are normalized to 0/1 + meas.what=0 at
                          // parse time and re-rendered on serialization.
  uint32_t period_us;     // interval between pulse/burst starts
  uint32_t duration_us;   // total train length
  uint32_t delay_us;      // wait between the start request (trigger edge, T/U or
                          // menu) and the train's first sample; 0 = legacy
                          // behaviour. Optional 6th header field of S/L/W.
  uint32_t dt_us;         // `L` only: ramp sample interval, 0 = SJ_TARGET_DT_US.
                          // Optional 7th header field of an L line, or `DT`.
                          // A coarser interval is how a ramp makes room for a
                          // measurement point (docs/serial-protocol.md §4).
  uint8_t  nStages;       // used by PIECEWISE_* only
  union {
    StageDef stages[SJ_MAX_STAGES];
    SineDef  sine;
  };
  EnvDef  env;
  MeasDef meas;
};

// `TRIG` routing entry (per input 0/1). mode: 0 disabled, 1 joint, 2 independent,
// 3 output marker. slots -1 = none. edge: 0 rising, 1 falling.
struct TriggerRoute {
  uint8_t mode;
  int8_t  slot0, slot1;
  uint8_t edge;
};

// EEPROM image (written by `P`): versioned + checksummed so a
// layout change can never mis-restore silently. v3: MeasDef gained the stage
// field and modes returned to the original 0-3 numbering. v4: TrainDef gained
// delay_us. v5: TrainDef gained dt_us, MeasDef gained fit, and the image
// carries the CAL timing budget. v6: the layout is unchanged, but two CAL
// defaults were corrected after being measured on hardware (SETTLE 4 -> 9 us,
// STARTLAT 20 -> 60 us) and a v5 image would restore the wrong ones in
// silence -- a stored budget that is merely wrong still validates, so nothing
// else would have caught it. An image of an older version is rejected
// outright and boot defaults apply — never re-interpreted.
struct EepromImage {
  uint32_t     magic;    // 'S''J''A''W' = 0x534A4157
  uint16_t     version;  // SJ_EEPROM_VERSION
  uint16_t     crc;      // CRC-16/CCITT over everything after this field
  TrainDef     slots[SJ_EEPROM_SLOTS];
  TriggerRoute trig[2];
  Cal::Def     cal;      // restored only if it still validates (Cal::validate)
};
#define SJ_EEPROM_MAGIC   0x534A4157u
#define SJ_EEPROM_VERSION 6

#endif // STIMJIMAWG_WAVEFORMDEF_H
