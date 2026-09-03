# stimjimAWG progress index

One line per phase, newest first. Full handoff entries — decisions, rationale, open
questions, next entry point — live in `docs/progress/<NNN>-<topic>.md` (project convention,
CLAUDE.md). To pick up work read this index, then at most the newest one or two entries.

- 010 2026-09-03 arm-cost — the start latency accounted for term by term and most of the arm moved out of it: plan compiled once per definition, only the used plan zeroed, sine constants derived at parse time, exact 32-bit ramp divisions, hot functions in RAM (`SJ_CODE_IN_RAM`), consecutive 0-duration `L` stages refused; `docs/timing.md` is new, predictions await `bench_arm.py` on silicon (progress/010-arm-cost.md)
- 009 2026-09-03 measured-budgets — `STARTLAT` and `SETTLE` measured on silicon and both corrected (20 → 60 µs, 4 → 9 µs), `BENCHARM` and `BENCHSETTLE` added, latch contention quantified, timing and parametrization figures, README as a quick start, EEPROM v6 (progress/009-measured-budgets.md)
- 008 2026-09-03 cal-coverage — timing budgets as runtime state (`CAL`, EEPROM v5), per-slot ramp interval (`DT`), reads rotated over repetitions (`MEAS` fit), trigger edge as the latency anchor; host-tested and four builds clean, not yet run on silicon (progress/008-cal-coverage.md)
- 007 2026-09-03 measurement-sd-bench — MEAS execution with its window-fit rule, MSUM/MDATA, SD logging, the SD file group serving the card over serial, BENCH on silicon (42 ns latch jitter), per-latch deadline counters (progress/007-measurement-sd-bench.md)
- 006 2026-09-02 hardware-bench-delay-ui — first hardware run (two boot-fatal PIT bugs fixed), Teensy 4 / portable backends, post-trigger delay, TRIG setters, OLED redesign (progress/006-hardware-bench-delay-ui.md)
- 005 2026-07-11 sine-playback — W sine playback, Q32 phase accumulator, per-train Fs (progress/005-sine-playback.md)
- 004 2026-07-11 ramp-envelope — L ramp playback via Bresenham, 0-duration jumps, ENV envelope (progress/004-ramp-envelope.md)
- 003 2026-07-11 scheduler-hold-trains — original 0-3 modes, MEAS redesign, PIT scheduler, HOLD playback (progress/003-scheduler-hold-trains.md)
- 002 2026-07-11 trainstore-protocol — S/L/W parsing with atomic staging, queries, EEPROM v2, host tests (progress/002-trainstore-protocol.md)
- 001 2026-07-09 scaffold-fastio-bench — stimjimAWG skeleton, FastIO register-level SPI, BENCH harness (progress/001-scaffold-fastio-bench.md)
- 000 2026-07-09 planning — spec analysis, architecture decisions, 8-phase plan (progress/000-planning.md)
