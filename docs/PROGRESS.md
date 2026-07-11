# stimjimAWG progress log

Phase handoff entries per project convention (CLAUDE.md): decisions, rationale, open questions,
next entry point. Newest entry last.

## 2026-07-09 — Phase 0: spec analysis and implementation plan

**Done:** analyzed `firmware-spec.md`, `stimjimPulser/stimjimPulser.ino` (szinuszgenerator
lineage), `lib/stimjim`, StimJimBIST and the PCB/BOM; wrote the planning documents:

- [awg-implementation-plan.md](awg-implementation-plan.md) — architecture + 8 implementation phases
- [serial-protocol.md](serial-protocol.md) — protocol v1 draft, defaults, compatibility appendix
- [hardware-notes.md](hardware-notes.md) — distilled hardware reference (replaces deleted `datasheets/`)
- appended the SD-card logging requirement to `firmware-spec.md`

**Key decisions (user-confirmed):** target Teensy 3.5 (Rev C PCB in repo is RP2354B — not our
hardware); backward-compatible protocol superset (BIST `M`/`V`/`E` single-line replies frozen);
linear ramps as new `L` command (legacy `S` stays rectangular, bit-exact); sines stay on `W` with
explicit fields + envelope triplet; trigger conflicts = ignore + WARN; measurement summary-first
with MDATA format frozen and SD logging via onboard SDIO; buttons repurposed to menu navigation.

**Architecture decisions (spec's "find out and decide"):** clock-based absolute-deadline
scheduling on 2 raw PIT channels + 64-bit DWT CYCCNT timebase (1 µs = exactly 120 CPU cycles),
program-early/latch-on-deadline for <200 ns NLDAC jitter; **no DMA** (GPIO chip selects + dual
MISO make eDMA chains fragile; register-level ISR engine reaches ~100 kS/s dual — sufficient);
generation from ISRs at priority 64 with USB serial at 112 so serial can never delay waveforms;
no start path emits samples in caller context (fixes the first-pulse preemption bug).

**Open questions / bench-verify (plan §7):** AD7321 line-switch first-conversion validity; AD5752
settling vs NLDAC (sets measurement GUARD); MISO PORT-mux glitch behavior; real GPIO header pins
(`Stimjim.h` defines all `GPIO_x` as 36); Teensyduino PIT allocation order + CYCCNT-at-reset;
`K_RELOAD` variance.

**Next entry point:** Phase 1 — create `stimjimAWG/` skeleton (all modules compiling), implement
`FastIO` (split-phase register-level DAC/ADC, `cycles64()`, `busLock()`), `BENCH` harness,
`K_RELOAD` self-calibration; scope-verify `dacProgram`+`dacLatch` against `Stimjim.writeToDac`.

## 2026-07-09 — Phase 1: scaffold + FastIO + bench harness

**Done:** `stimjimAWG/` created on branch `arbitrary_waveform`; all 14 planned modules compile
clean (`arduino-cli compile --fqbn teensy:avr:teensy35 --warnings more`: 0 warnings, 67 KB flash /
23.7 KB RAM). Functional in this phase: `FastIO` (register-level SPI0 split-phase DAC/ADC, MISO
PORT-mux swap, `cycles64()`, nestable BASEPRI `busLock()`), `Engine` PIT reservation +
deadline-programming core + boot `K_RELOAD` self-calibration (two passes, median of 65),
`Protocol` bounded line assembler, and the `BENCH` command group (now documented in
serial-protocol.md §4): DAC/ADC/latch/mux/cycles64 timing, `BENCHK`, `BENCHPIT` jitter histogram
(raw ISR wake or preload+spin latch mode), `BENCHSQ`/`BENCHSQL` square generators for the scope
A/B against `Stimjim.writeToDac`. `UiInput` (debounced ISR event ring) and a display splash work;
SampleGen/Measure/SdLog/Triggers are documented stubs; TrainStore holds the 100 slots with
protocol-§5 defaults. Legacy single-letter commands answer a one-line `ERR … Phase 2+`.

**Findings vs plan §7 (core source, Teensyduino 1.59):** (a) DWT CYCCNT is **not** enabled by the
Teensy 3 startup code (only AudioStream enables it on demand) — `FastIO::begin()` enables
TRCENA+CYCCNTENA itself; (b) `IntervalTimer` does allocate PIT channels low-to-high, but
`Engine::grabChannel()` doesn't rely on it: it diffs the PIT `TCTRL` enable bits around
`begin()` to identify the granted channel, then takes over its vector at priority 64 while
keeping the `IntervalTimer` allocated so the core can never hand the channel out again.

**Implementation deviations from plan (documented in code):** the 24-bit DAC word is shifted as
**two 12-bit frames** (not 8+16) because both frames must share CTAR0 (CTAR1 is the ADC's) and
K64 FIFO frames are ≤16 bits — the AD5752 only counts SCLK edges while SYNC is low, so this is
equivalent. `dacProgramBoth` is sequential per channel (shared MOSI ⇒ inherent), the FIFO overlap
is within each 24-bit word. `K_RELOAD` is calibrated closed-loop by running the real
`pitProgram()` path and polling `TFLG`; the few-cycle poll-detection latency stays inside the
constant, which is fine because players wake `PRELOAD` (≈4 µs) early and spin on CYCCNT.

**Open questions / next bench session (hardware needed):** run `BENCHDAC/ADC/SW/MISO` to replace
the calibrated 2.75/4.5 µs figures with measured ones; `BENCHPIT,1000,5000` (raw) to size
`SJ_PRELOAD_US`, then `BENCHPIT,1000,5000,4` for the latch-jitter acceptance (<200 ns);
`BENCHK` variance; scope A/B `BENCHSQ,0,10000,50,100` vs `BENCHSQL,…` (Phase-1 exit criterion);
AD7321 first-conversion validity after line switch and AD5752 settling (plan §7 items 1–2) remain
open for the measurement GUARD. Real GPIO header pins still unverified (`Stimjim.h` bug).

**Next entry point:** Phase 2 — TrainStore staging/validate/commit, legacy `S`/`W` + new `L`
parsers with atomic slot updates, `?` queries and round-trip serializers, `M/V/A/E` byte-exact
replies; verify against StimJimBIST and the README/header example lines.

## 2026-07-11 — Phase 2: TrainStore + protocol core

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

## 2026-07-11 — Phase 3: mode/measurement redesign + scheduler + HOLD trains

**Mode-numbering investigation (user question):** the hi-Z/ground confusion was **introduced by
the lab modification, not upstream**. Upstream open-ephys (`upstream/main`) documents modes
0 V / 1 I / 2 hi-Z / 3 ground and `Stimjim::setOutputMode` decodes the two low bits directly onto
the OE pins — `M2`/`M3` were correct there. The lab commit ff51733 ("Added measure/stim modes")
renumbered only the documentation/mode strings (2/3 → unmeasured V/I, 4/5 → hi-Z/ground) while
`M` kept passing the mode raw into the decoder, so lab `M2`/`M3` really produced hi-Z/ground and
`M4`/`M5` connected the V/I source. (Lab *train* paths were correct: `mode & 1` under `< 4`.)

**Protocol redesign (user-requested; serial-protocol.md + plan §3.6 updated):**
(a) modes revert to the **original 0–3 numbering** everywhere; in train definitions 2/3 =
channel not driven (upstream semantics); `M` maps 0–3 raw (correct by construction now).
(b) measurement participation moved out of the mode space into `MEAS`, with **90/91** accepted
in train definitions as sugar for V/I-with-measurement-off (stored as mode 0/1 + `what=0`,
re-rendered as 90/91 by queries so the flag round-trips; plain 0/1 promotes a stored `what=0`
back to 3, explicit 1/2 refinements preserved). Lab scripts using 2/3 must migrate to 90/91 —
documented loudly (protocol §6.3), fails safe (2/3 now = no output). (c) `MEAS` schema is now
`what0,what1,when,stage[,report]`: `when` is type-specific (S/L: 0 = near stage end; W: 1/2/3 =
+peak/−peak/both, default both, one period per burst measured after envelope ramp-in),
`stage` = −1 all / n single (subsumes old first-stage-only; kept because it is one comparison at
plan-compile time and lets short-stage trains pick the one stage that fits the ADC budget).
(d) summaries gain a **std-dev estimate** from single-pass Σv/Σv² (int64) accumulation; frozen
`MSUM` record defined (point = stage index, or 90/270 for sine peaks). (e) new **`READ<ch>[,n]`**
manual averaged measurement (mean+sd via the calibrated legacy path, default n=16, refused while
a train runs); `E` stays the raw BIST-frozen single read. EEPROM image bumped to **v3**
(MeasDef layout + mode renumbering; old images rejected → boot defaults).

**Phase 3 engine (implemented; compiles clean, 89 KB flash / 25.5 KB RAM; host tests pass):**
`ChannelPlayer` ×2 in `Engine.cpp` — copy-on-arm (`startTrain` precomputes DAC codes with the
legacy float expression, saturating out-of-range; cumulative stage offsets in cycles), absolute
deadlines `pulseStart + cum[e]`, program-early/spin/latch with per-train preload
(`SJ_PRELOAD_US` + new `SJ_DAC_PROG1/2_US` budgets), events within preload+`MIN_SCHEDULE`
processed inline (0-duration chains never re-enter the NVIC), long gaps chunked by `MAX_SLICE`.
OE toggles per pulse like legacy (connect after the stage-0 latch, ground after the off-event
latch); the off event parks DACs on offsets. Empty/undriven trains degenerate to per-period
bookkeeping. `T`/`U` keep legacy byte-exact reply lines (`Started/Forcing/Invalid`); busy engine
or cross-engine channel conflict → drop + WARN; strict index parse (legacy `atoi` started train 0
on garbage). `T-1` stops under `busLock` and parks outputs. Completion ring (SPSC) drained by new
`Commands::poll()` in `loop()` — prints `Train #<n> complete. Delivered <p> pulses.` (+ Phase-3
"no measurement" note; MSUM replaces it in Phase 7). `STAT` live via seqlock; `activeSlot`/
`anyActive` real (edit refusal, V/A/E-during-train WARN and BENCH-while-running refusal active).
`M?` shadow tracked across trains/stops/`B`/`C` (boot state 3).

**Open questions / deferred:** bench session still owed (Phase-1/3 acceptance: `BENCHPIT`
preload sizing + <200 ns latch jitter, `BENCHSQ` vs `BENCHSQL` scope A/B, `BENCHDAC/ADC/SW/MISO/K`
to replace the 2.75/4.5 µs and `SJ_DAC_PROG*` budgets, plan §7 items 1–4); trigger-latency
acceptance (`START_LATENCY` ±1 µs) needs the Rigol+scope; overlong stage chains (Σdur ≫ period)
run back-to-back like legacy but keep the ISR busy — acceptable, parse-time WARN exists. `W`
single-peak `when` (1/2) with `burst_us` shorter than the relevant half-period: the measurement
point falls outside the burst → must be skipped+flagged (Phase 7 implements the flag).

**Next entry point:** Phase 4 — RAMP (`L`) playback via SampleGen Bresenham iterator, 0-duration
jump chains, drift-free repeats, envelope (`ENV`) application; then the deferred bench session
(can piggyback Phase 3 acceptance + Phase 4 ramp-rate checks in one sitting).

## 2026-07-11 — Phase 4: RAMP playback + envelope

**Done (compiles clean, 91.4 KB flash / 27.4 KB RAM; host tests pass):** `SampleGen` implemented
as a pure, host-testable module (new `tests/host/test_samplegen.cpp`, g++, per-sample closed-form
checks incl. a 100k-sample full-swing stage): ramp Bresenham iterator (`RampStage` arm-time
constants + `RampCursor`), envelope (`EnvCoef`/`envQ15`), Q15 scaling. `Engine` plays
`PIECEWISE_RAMP`: per-pulse event chain = offset latch at pulseStart (OE-connect anchor) →
per-stage Bresenham samples (`N = max(1, round(dur/20 µs))`, last sample exact on the stage
boundary/end value, stage i starting from stage i−1's exact end) → off event at the last boundary
(park + ground). 0-duration stages = one sample at their start time (instant jump), chained
inline through the existing `MIN_SCHEDULE` path. `ENV` is now applied to S and L trains:
per-sample on L, per-stage-latch on S (stair-step, documented); amplitudes are stored as DAC-code
**deltas from the channel's calibration offset**, so the envelope scales the stimulus without
moving the parked baseline (Phase 3 HOLD codes stay bit-exact — identity path when env is off).

**Design choices documented in plan §3.5 (user questions answered):** (a) *drift-free repeat* =
absolute deadlines `t0 + k·periodCyc` in exact integer cycles — each event is off by its own
bounded latency only, errors never accumulate (unlike the legacy `delayMicroseconds` chain);
likewise within stages (Bresenham makes sample N exact on the boundary) and across bursts.
(b) *timebase* = 64-bit `cycles64()` (8.33 ns, exact ×120 µs↔cycles, wraps in ~4900 years; the
35.8 s hardware CYCCNT wrap is bridged by the loop()+`MAX_SLICE` keep-alive). (c) *integer vs
float*: event times and sine phase are integer-only (FP32's 24-bit mantissa can't represent
>0.14 s of cycles; FP64 is software-emulated on the M4F — banned from ISRs); amplitudes are
integer Q15 multiplies on offset-relative deltas; the envelope's single division becomes an
arm-time FP32 reciprocal → one hardware-FPU multiply per event (~20 cycles incl. lazy stacking,
error ≪ 1 Q15 LSB). Arm-time (loop-context) coefficient math may use double. (d) *sine* (Phase 5
design, fixed now in §3.5): on-the-fly synthesis from the shared 2 KB Q15 table — per-train
amplitude-baked tables rejected (RAM + arm-time latency, cannot absorb the time-varying envelope,
would constrain Fs to integer samples/period; the multiply it would save costs ~1 cycle).

**Semantics settled in-phase:** L pulses ramp from the parked offset; the final stage's end value
is latched exactly at the last boundary and immediately parked (append a same-value stage to hold
it) — protocol §2 updated. Envelope with `rampOut=0` stays flat through `tEnd`; events overrunning
`duration_us` (legacy: stages run to completion) clamp to 0 when a ramp-out exists.

**Open questions:** hardware items unchanged (plan §7 1–4, 6; bench session still owed — Phase 3
acceptance + an `L` ramp scope check share one sitting). Envelope-vs-measurement interplay
(mid-ramp-in peaks under-read) already handled by the Phase 7 plan (measure after ramp-in).

**Next entry point:** Phase 5 — sine playback: `SINE_TAB` init + `sineQ15` lerp, Q32 phase
accumulator with per-burst restart, applied start phase, per-train
`Fs = clamp(64·f_max, 1 kHz, FS_MAX)` as an exact integer sample period, `f_max > Fs/2` refusal;
SINE arm/emit paths in `Engine`.
