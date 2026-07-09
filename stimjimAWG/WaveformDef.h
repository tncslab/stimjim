//    stimjimAWG — waveform definition types (plan §4, protocol §2/§4/§5).
//    GPL-3.0-or-later; see Config.h header.

#ifndef STIMJIMAWG_WAVEFORMDEF_H
#define STIMJIMAWG_WAVEFORMDEF_H

#include <stdint.h>

#define SJ_NUM_SLOTS    100
#define SJ_MAX_STAGES   10
#define SJ_EEPROM_SLOTS 10   // slots 0-9 persisted by `P`

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

// `MEAS` — what: 0 none, 1 V, 2 I, 3 both; when: 0 first stage, 1 all stages,
// 2 sine peak; report bitmask: +1 stream MDATA, +2 log to SD (summary always kept).
struct MeasDef {
  uint8_t what0, what1, when, report;
};

struct TrainDef {
  uint8_t  type;          // TrainType
  uint8_t  mode0, mode1;  // command modes 0-5 (see `M`)
  uint32_t period_us;     // interval between pulse/burst starts
  uint32_t duration_us;   // total train length
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

// EEPROM image v2 (written by `P`, Phase 2/8): versioned + checksummed so a
// layout change can never mis-restore silently.
struct EepromImageV2 {
  uint32_t     magic;    // 'S''J''A''2' = 0x534A4132
  uint16_t     version;  // 2
  uint16_t     crc;      // CRC-16/CCITT over everything after this field
  TrainDef     slots[SJ_EEPROM_SLOTS];
  TriggerRoute trig[2];
};
#define SJ_EEPROM_MAGIC   0x534A4132u
#define SJ_EEPROM_VERSION 2

#endif // STIMJIMAWG_WAVEFORMDEF_H
