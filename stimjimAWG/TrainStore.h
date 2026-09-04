//    stimjimAWG — TrainStore: 100 waveform slots with staging/validate/commit
//    (atomic — a malformed line never half-updates a slot), round-trip
//    serializers, defaults per protocol §5, versioned EEPROM persistence.
//
//    Everything except the EEPROM block is host-testable (no Arduino deps) —
//    see tests/host/. Parsing/validation lives here so the protocol grammar
//    can be exercised off-target; Commands.cpp only does I/O and dispatch.
//    GPL-3.0-or-later; see Config.h header.

#ifndef STIMJIMAWG_TRAINSTORE_H
#define STIMJIMAWG_TRAINSTORE_H

#include <stddef.h>
#include "WaveformDef.h"

// Longest ERR/WARN message text produced by the parser/validators.
#define SJ_MSG_MAX 120
// Longest canonical round-trip line (S line with 10 stages is the worst case).
#define SJ_SERIALIZE_MAX 512

namespace TrainStore {

void begin();   // initialize all slots to defaults (EEPROM restore is separate)

// Boot default (protocol §5): grounded (not-driven) modes, period 10 ms,
// duration 500 ms, 0 stages, type S, build-default ramp interval; ENV 0,0,0;
// MEAS 3,3,auto-when,-1,0,1 (rotate reads rather than refuse a point).
void slotDefault(TrainDef& t);

TrainDef&       slot(uint8_t idx);         // idx asserted < SJ_NUM_SLOTS by caller
const TrainDef& slotConst(uint8_t idx);

// ---------------------------------------------------------- parse + validate
//
// Parse the body of an S/L/W line (everything after the slot index) into
// `staged`, starting from defaults but preserving `current`'s ENV/MEAS per
// protocol §2 (a W line always carries/resets its own envelope; MEAS `when`
// is auto-coerced on a type change). On failure returns false with err set
// and staged undefined; on success warn may hold accepted-with-caveat notes
// ("; "-separated, empty string if none). Nothing is committed here.
bool parseTrainBody(char letter, const char* body, const TrainDef& current,
                    TrainDef& staged, char* err, size_t errsz,
                    char* warn, size_t warnsz);

// Validate an ENV/MEAS update against a slot. Return NULL if acceptable,
// else a static error string. validateMeas may also fill warn (size >= SJ_MSG_MAX).
const char* validateEnv(const TrainDef& t, const EnvDef& e);
const char* validateMeas(const TrainDef& t, const MeasDef& m, char* warn, size_t warnsz);

// The shortest nonzero interval this definition puts between two consecutive
// DAC latches, in microseconds; UINT32_MAX when it asks for fewer than two.
// Compared against Cal::minLatchUs to warn about stage boundaries the board
// cannot deliver -- which the parser cannot do on its own, because the figure
// it would have to compare against is runtime state.
//
// Gaps of 0 are excluded: an instant jump *means* two latches at one instant,
// and the second of a coincident pair is late by definition. `dtUs` is the ramp
// sample interval in force (never 0 -- the caller resolves the default). A sine
// is exempt: its sample grid comes from the engine's Fs policy rather than from
// the definition, and the Nyquist gate at start covers it.
uint32_t minLatchGapUs(const TrainDef& t, uint32_t dtUs);

void commit(uint8_t idx, const TrainDef& staged);   // atomic slot replacement

// Bumped by every write to any slot — commit(), begin() and the EEPROM
// restore, which are the only three. Anything caching a value derived from a
// definition stores this alongside it and recomputes when it no longer matches
// (the measurement plan and the sine constants both do). It is deliberately
// one counter for all slots rather than one per slot: editing slot 7 then
// costs the next arm of slot 3 one recompile, which is cheaper than the
// bookkeeping that would avoid it.
uint32_t epoch();

// The slot the last commit() wrote. A guess at what the operator is about to
// start, and the only guess available for a `T`/`U` start, which names no slot
// in advance the way a TRIG route does. Engine::prepareArms uses it to prepare a
// plan for engines no route points at, so the first serial start of a freshly
// edited slot does not compile inside its own start latency.
uint8_t lastWritten();

// ------------------------------------------------------ round-trip serializers
// Canonical one-line set-commands (protocol §1 query contract), no spaces.
void serializeTrain(uint8_t idx, const TrainDef& t, char* buf, size_t n);
void serializeDelay(uint8_t idx, uint32_t delay_us, char* buf, size_t n);
void serializeDt(uint8_t idx, uint32_t dt_us, char* buf, size_t n);
void serializeEnv(uint8_t idx, const EnvDef& e, char* buf, size_t n);
void serializeMeas(uint8_t idx, const MeasDef& m, char* buf, size_t n);

// Format a milli-unit value (mHz / mdeg) as decimal text with trailing zeros
// trimmed ("500", "0.25", "-12.5"). buf must hold >= 16 bytes.
void milliToStr(long long milli, char* buf);

// ------------------------------------------------------------------ defaults
uint8_t defaultWhen(uint8_t type);            // 0 for S/L, 3 for W (protocol §5)
bool isDefaultTrain(const TrainDef& t);       // ignores env/meas
bool isDefaultEnv(const EnvDef& e);
bool isDefaultMeas(const TrainDef& t);        // auto-when aware

// ------------------------------------------------------------------- EEPROM
#ifdef ARDUINO
// `P`: persist slots 0-9, the trigger table and the CAL timing budget as
// EepromImage (versioned, CRC-16/CCITT). eepromRestore returns false (and
// touches nothing) unless magic, version and CRC all match. A stored budget
// that no longer passes Cal::validate is dropped — the build defaults stay in
// force and *calRejected is set, for the caller to report.
void eepromSave(const TriggerRoute trig[2]);
bool eepromRestore(TriggerRoute trigOut[2], bool* calRejected = nullptr);
#endif

} // namespace TrainStore

#endif // STIMJIMAWG_TRAINSTORE_H
