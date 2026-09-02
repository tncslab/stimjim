# Phase 2: TrainStore + protocol core

Date: 2026-07-11

**Done:** full waveform-definition protocol (compiles clean, 83 KB flash / 25 KB RAM). `S`/`L`/`W`
parse into a staging `TrainDef` → validate → commit (atomic; malformed lines leave the slot
untouched, fixing the legacy half-update at `stimjimPulser.ino:925`); `?` queries return canonical
round-trip lines (letter follows the slot's actual type), bare `S<idx>` keeps the legacy-style
human dump (+ env/meas rows, ends `OK`); `ENV`/`MEAS` setters+queries with type-aware validation
and auto-`when` coercion on type change; byte-exact `M`/`V`/`A`/`E` replies (identical float-math
expressions and `print` sequences as legacy — BIST contract); `B`/`C`/`D`/`D?`/`P`; `DUMP`
(TRIG lines as `#` comments until Phase 8); EEPROM v2 save/boot-restore (CRC-16/CCITT, ~1.6 KB
image, magic+version gated); `TRIG<t>?`/`R<t>?` queries (setters remain Phase 8); updated `HELP`.
Parsing/validation/serialization live in `TrainStore.cpp` with **no Arduino deps** — host-tested by
`tests/host/test_trainstore.cpp` (g++, ~90 checks, all pass): README/header example lines,
canonical-form idempotence, decimal mHz/mdeg round-trips, rejection and warning paths, ENV/MEAS
preservation semantics.

**Decisions taken in-phase (all documented in serial-protocol.md §6):** (a) legacy `M` passed the
command mode 0-5 *raw* into the 2-bit OE decoder — so old `M2`/`M3` really produced hi-Z/ground and
`M4`/`M5` connected the voltage/current source(!); new firmware maps documented semantics (BIST
unaffected, it only uses 0/1). (b) Negative `W` frequencies (legacy: time-reversed sine via table
wraparound; the old header example `-500` Hz) → `ERR`. (c) `B` replies `OK` (legacy printed
nothing); `B`/`C` refused while a train runs. (d) 0-stage `S`/`L` stays accepted (legacy empty
train, boot default). (e) Sub-mHz/sub-mdeg decimals → `ERR` (round-trip exactness over silent
rounding). (f) S/L preserve stored `ENV` but ERR if it no longer fits a shortened duration.
Supporting plumbing: `Engine::activeSlot()/anyActive()` stubs (edit-refusal + V/A/E during-train
WARN paths are wired and activate in Phase 3); `Triggers::setRoute()`; `FastIO::acquireBus()` now
also re-asserts the MISO PORT mux (legacy `readAdc` calls `SPI.setMISO` behind FastIO's cache —
`E`/`B`/`C` would have left it stale).

**Hardware-bench assessment (user asked: is the deferred bench blocking?):** Nothing in Phase 2
needed hardware. Phases 3–6 are *implementable* without the bench — `SJ_PRELOAD_US`, the
2.75/4.5 µs figures etc. are plain constants to re-measure later. The bench is blocking only for
(1) Phase 3 *acceptance* (latch jitter < 200 ns via `BENCHPIT`, scope A/B `BENCHSQ`/`BENCHSQL`),
and (2) the genuinely unforeseeable behaviors: AD7321 first-conversion validity after a line
switch, AD5752 settling (both gate the Phase 7 measurement GUARD), and the MISO PORT-mux glitch
check (gates FastIO-path ADC reads — note `E` uses the legacy read path, so it is not exposed).
Recommended single bench session after Phase 3 code lands: `BENCHDAC/ADC/SW/MISO/K`,
`BENCHPIT,1000,5000[,4]`, `BENCHSQ` vs `BENCHSQL` scope A/B, then `T0` vs old firmware.
StimJimBIST + example-line replay on the device also await that session (host tests cover the
parser side).

**Open questions:** plan §7 items 1–3 + 6 unchanged (hardware). `R<t>?` legacy view for TRIG mode 2
(independent) has no legacy equivalent — currently renders as `-1,0`; revisit in Phase 8.

**Next entry point:** Phase 3 — `ChannelPlayer` + PIT deadline scheduler ISRs, `T`/`U` start/stop,
copy-on-arm, `START_LATENCY`, completion ring drained in `loop()` (result summary printing),
single-channel HOLD trains; make `Engine::activeSlot()/anyActive()` real (edit refusal + STAT go
live). Bench session recommended at phase end (acceptance criteria above).
