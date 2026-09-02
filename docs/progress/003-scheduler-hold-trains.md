# Phase 3: mode/measurement redesign + scheduler + HOLD trains

Date: 2026-07-11

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
