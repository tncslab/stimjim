# Phase 4: RAMP playback + envelope

Date: 2026-07-11

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
*>0.14 s* of cycles; FP64 is software-emulated on the M4F — banned from ISRs); amplitudes are
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
