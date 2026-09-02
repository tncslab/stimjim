# Phase 6: first hardware run, hardware variants, post-trigger delay, triggers, UI

Date: 2026-09-02

**Scope note:** this phase replaces the originally planned Phase 6 ("dual channel: dualSync,
collision-jitter benchmark, published FsMax table"). Three things displaced it: the firmware had
never actually run on hardware and did not boot; the user asked for a Teensy 4 route and a
per-waveform post-trigger delay; and the delay is untestable without the trigger start path, so
the trigger half of Phase 8 came forward. The dual-channel benchmark and the measured `FsMax`
table move to Phase 7.

**Done** (Teensy 3.5 on COM4 + PicoScope 2204A; all host tests pass; `--warnings more` clean on
Teensy 3.5 / 3.5-portable / 4.1 / 4.0):

1. **The firmware booted for the first time — and did not.** Phases 1–5 were compile-only. Two
   fatal defects in the existing Kinetis path, both found by a `-DSJ_BOOT_TRACE` build that makes
   `setup()` wait for a serial host and announce each init step:
   - `Engine::begin()` read PIT `TCTRL` **before the PIT clock was ungated**. On the K64 a load
     from a clock-gated peripheral is a bus fault, so the board locked up in `setup()` with
     interrupts disabled: USB stayed enumerated, nothing ever answered, and it looked exactly
     like a broken upload. Fixed with an explicit `pitClockEnable()` mirroring the core's own
     sequence (`SIM_SCGC6 |= SIM_SCGC6_PIT`, `nop`, `PIT_MCR = 1`).
   - **Both players were given PIT channel 0.** `IntervalTimer` claims a channel by scanning for
     `TCTRL == 0`; the old `grabChannel()` claimed one, immediately cleared its `TCTRL` during
     takeover, and the second claim then found the same channel free. The banner said
     `PIT channels 0/0`. Fixed by claiming both before disarming either (`grabChannels()`).
   The unbounded `while (!(TFLG & 1))` spin in the `K_RELOAD` calibration is what turned the
   first defect into an unrecoverable hang; it is now bounded, and a timeout leaves `K_RELOAD`
   untouched with a warning rather than poisoning every future deadline.

2. **Hardware variants (`#if` backends).** `SJ_FASTIO_REGISTER` and `SJ_TIMER_REGISTER` select
   register-level Kinetis code or a portable route on the standard Arduino `SPI` library and
   `IntervalTimer`; either can be forced from the build to A/B the portable path on a known-good
   Teensy 3.5. `SJ_CYC_PER_US` now derives from `F_CPU` (600 on a Teensy 4.x), and two constants
   that silently assumed 120 MHz — the NLDAC hold pulse and the `BENCH` cycles→ns conversion —
   are derived rather than literal. Full rationale, the recalibration procedure and the
   Teensy 4.0 EEPROM limit are in [hardware-variants.md](../hardware-variants.md).

3. **Per-waveform post-trigger delay** (the user's request). Optional 6th field of the `S`/`L`/`W`
   header, plus a `DELAY<idx>,<us>` setter and query. It applies to every start path, so it is
   testable over serial before any trigger is wired. **Measured on the scope: 2000 µs set →
   1999.3 µs, 5000 → 5000.2, 20000 → 20002.7** (skew-corrected; the residual is the scope's own
   sample interval).

4. **Triggers** (Phase 8 brought forward): `TRIG` and `R` setters, edge ISRs at priority 80 that
   arm and start in place, the stimulus-marker output, and deferred reject warnings printed from
   `loop()`. `DUMP` now emits `TRIG` lines as real set-commands, so a dump pastes back complete.

5. **Display redesign + `SCREEN`.** Three views (browse / waiting / running) with a title bar,
   engine tags and a progress bar, redrawn only when the composed text changes. `SCREEN` prints
   the framebuffer as ASCII art *and* the composed row text — the panel cannot be photographed
   over a serial link, so this is how layout changes get reviewed. Captures:
   [006-screens.txt](006-screens.txt).

6. **Bench harness** under `tests/device/`: `sjcon.py` (serial console), `pico2000.py` (ctypes
   binding for the legacy `ps2000` driver — the 2204A needs it, and it uses the DLL that ships
   with PicoScope 7, so nothing had to be installed), `smoke.py` (28 serial checks + screen
   captures) and `capture.py` (scope acceptance, figures to `figs/`, raw samples to `tmp/`).

**Decisions taken in-phase:**

- *Delay semantics*: the delay shifts the whole train timebase, so it never changes train length;
  outputs stay grounded through it. Omitting the 6th field **resets** the delay to 0, matching
  the rule the `W` envelope triplet already follows — a waveform line fully defines its header.
  That makes replayed legacy scripts reproduce legacy behaviour exactly, at the cost of an
  asymmetry with the `DELAY` setter, which is documented in protocol §2 and in the command's own
  help text. Cap 2 000 000 000 µs, chosen because `strtol` bounds the setter at `LONG_MAX`, so a
  higher cap would be reachable through one entry path and not the other.
- *STAT is unchanged* (still 8 fields). "Waiting out the delay" is exposed through `EngineStatus`
  for the display only, so no GUI parser has to change; a waiting engine reports `elapsed_us = 0`.
- *Trigger mode 1 ("joint")* starts one slot on engine 0. Which physical channels move is a
  property of the slot's own mode fields, so channel synchronisation is expressed in the waveform
  definition, not the routing. Mode 2 arms the two engines sequentially inside one ISR, which
  puts engine 1 **7.5 µs** (measured) behind engine 0 — documented, and the reason sample-
  synchronous work belongs in one slot rather than two.
- *Reentrancy*: loop-context `T`/`U` takes the bus lock around `startTrain`, since a trigger edge
  (priority 80) can preempt `loop()` and the busy check plus the arm writes are not atomic.
  Trigger ISRs need no lock: they cannot preempt each other, and the players — the only thing
  above them — never start trains.
- *Teensy 4.0 EEPROM*: the ~1.6 KB image does not fit its 1080 B, and `EEPROM.put()` past `E2END`
  is a silent no-op in the core, so the build **fails** with a `static_assert` naming the fix
  (`-DSJ_EEPROM_SLOTS=6`) rather than silently persisting less than asked.

**Scope-verified.** Traces below (PNGs committed under `figs/`, raw samples written to `tmp/` by
`tests/device/capture.py`). All three waveform types had been compile-only until now, so this
closes the Phase 3/4/5 shape acceptance. The traces also carry the bench circuit's own signature
— the 1 k/1 k divider cross-talk onto the idle channel and the ~2.2 V LED clamp — which confirms
the measurement is of the real outputs and not of an artefact.

| Figure | What it shows |
|---|---|
| [stimjim-trigger-skew.png](../../figs/stimjim-trigger-skew.png) | Both engines started by one trigger edge with delay 0: the −7.5 µs arming skew that the delay measurements are corrected for |
| [stimjim-trigger-delay-2000us.png](../../figs/stimjim-trigger-delay-2000us.png) | 2000 µs delay set → 1999.3 µs measured |
| [stimjim-trigger-delay-5000us.png](../../figs/stimjim-trigger-delay-5000us.png) | 5000 µs delay set → 5000.2 µs measured |
| [stimjim-trigger-delay-20000us.png](../../figs/stimjim-trigger-delay-20000us.png) | 20000 µs delay set → 20002.7 µs measured |
| [stimjim-shape-S-biphasic.png](../../figs/stimjim-shape-S-biphasic.png) | `S`: biphasic rectangular, ±5 V, 2 ms per phase |
| [stimjim-shape-L-ramp.png](../../figs/stimjim-shape-L-ramp.png) | `L`: a clean triangle, 4 ms up and 4 ms down, with the 20 µs Bresenham steps visible |
| [stimjim-shape-W-sine.png](../../figs/stimjim-shape-W-sine.png) | `W`: 500 Hz sine burst with the 32 kHz sample staircase (Fs = 64·f) |

**Bench notes worth keeping:** the 2204A is 8-bit, so on the ±10 V range one code is 78 mV and a
trigger threshold within a few codes of ground sits inside the trigger hysteresis and never
fires — scope trigger levels here are kept ≥1 V clear of the baseline. Its capture buffer is
3968 samples with both channels enabled. A one-shot train cannot be started *after* arming the
scope from the host: the serial round trip alone is tens of milliseconds, so shape captures run
against a multi-second train.

**Open questions / still owed:**

- The register-path timing constants (`SJ_DAC_PROG*_US`, `SJ_PRELOAD_US`, `SJ_FS_MAX_HZ`) are
  still Phase-1 desk estimates: the `BENCH` group has never been run on hardware, even though the
  board is now known to work. That is the first thing to do in Phase 7 and it is now cheap.
- Plan §7 items 1–4 (AD7321 first-conversion validity after a line switch, AD5752 settling vs
  NLDAC, MISO mux glitch, the real GPIO header pins) remain open; 1 and 2 gate the Phase 7
  measurement GUARD.
- Neither Teensy 4 build nor the portable backends have run on silicon — they compile and are
  structurally correct, nothing more.
- `K_RELOAD` measured 96–112 cycles across boots; `BENCHK` should quantify that spread.

**Next entry point:** Phase 7 — run the `BENCH` group on hardware and replace the estimated
constants; then the measurement engine (`MeasurePlan` execution, `MSUM` summaries), `SdLog` and
the MDATA ring. The displaced dual-channel collision benchmark and the measured `FsMax` table
belong with that bench session.
