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
