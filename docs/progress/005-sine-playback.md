# Phase 5: sine (`W`) playback

Date: 2026-07-11

**Done (compiles clean, 97.4 KB flash / 29.5 KB RAM — +2 KB is the sine table; host tests
pass):** `SampleGen` gains the sine core: `SINE_TAB` (1025-entry int16 Q15 full wave, filled once
at boot from double `sin`, [1024] duplicating [0] so interpolation never wraps), `sineQ15` (top
10 phase bits index, next 16 bits Q16 lerp fraction, round-half-up — measured max error ≤1.5 LSB
vs double sin in the host sweep), `sinePhaseInc` (double at arm time; the final ≤0.5/2³² rounding
is the only frequency error — deterministic, <10⁻⁷ relative, non-accumulating) and
`sinePhaseInit` (mdeg → Q32, exact integer, any sign — the phase field finally *does* something).
`Engine` plays `SINE`: per period one burst of samples at `pulseStart + k·sampleCyc` (exact
integer grid), burst end enforced by deadline comparison (`EV_OFF` parks + grounds at
`pulseStart + burstCyc`), **phase accumulators restart at `phaseInit` each burst** so all bursts
are identical (legacy-consistent; required by the Phase 7 peaks-per-burst plan). Per-train
`Fs = clamp(64·f_max, SJ_FS_MIN_HZ=1 kHz, SJ_FS_MAX_HZ=50 kHz)` realized as
`sampleCyc = round(120 MHz/Fs)`; `phaseInc` is derived from the actual `sampleCyc` so frequency
exactness never depends on Fs rounding. `f_max > Fs/2` (25 kHz) → start refused with WARN
(Nyquist; checked at start, not parse, because FS_MAX is an engine property). Envelope applies
per sample like ramps: `code = offset + env·(amp·sin(phase))`, two Q15 multiplies (~25 cycles ≪
the 2.75 µs SPI cost — the on-the-fly decision from plan §3.5 confirmed: no per-train table).

**Notes:** `burst_us = period_us` gives continuous sine except a few-µs park at each period
boundary (legacy-consistent; documented in protocol §2 with a possible future gapless mode).
Undriven-channel sine coefficients arm to 0 (the accumulator still runs — harmless). SINE slots
keep `nStages = 0`, so the empty-train degeneration path is untouched. `SJ_FS_MAX_HZ = 50 kHz` is
deliberately conservative pre-bench (min sample period 20 µs ≫ preload+program budget ≈ 9 µs);
Phase 6 publishes the measured ceiling with ~30 % margin.

**Open questions:** unchanged hardware items (plan §7 1–4, 6); the bench session now also covers
sine sample-rate headroom (raises `SJ_FS_MAX_HZ`) and a scope check of applied phase/per-burst
restart (dual-channel phase offset is the natural test — but starts on *different* engines are
not synchronized; same-slot dual-channel `W` via one engine is).

**Next entry point:** Phase 6 — dual channel: dualSync via `TRIG` joint mode groundwork,
collision-jitter benchmark between two independent players, publish the measured FsMax table into
`Config.h` with ~30 % margin. The long-owed hardware bench session (Phases 1/3/4/5 acceptance)
should precede or accompany it.

**User request:** Some parts of the code are optimized for Teensy 3.5, e.g., KINETISK_SPI0 in
`FastIO.cpp`, however OpenEPhys plans to update hardware and use Teensy 4 (due to older version
being out of stock). Implement an alternative route (#ifdef) with standard Arduino for those
hardware verions. Check for similar cases in other source files, and indicate if these need
recalibratin of measured timing constants.
