# stimjimAWG — implementation plan

Status: approved plan (Phase 0, 2026-07-09). Target folder `stimjimAWG/` (branch `arbitrary_waveform`).
Companion documents: [serial-protocol.md](serial-protocol.md), [hardware-notes.md](hardware-notes.md), [PROGRESS.md](PROGRESS.md).
License: GPL-3.0-or-later (same as the rest of the project).

## 1. Context and goals

`firmware-spec.md` requests a new arbitrary-waveform-generator firmware replacing
`stimjimPulser/stimjimPulser.ino` (szinuszgenerator lineage), whose known limitations are:

- the first pulse of a train runs synchronously in the caller context (`loop()` or a pin/button
  ISR at priority 128) while repeats run from an `IntervalTimer` ISR at priority 64 — the repeat
  can preempt a still-running first pulse;
- stage timing uses hand-calibrated `delayMicroseconds` compensation (DAC write 2.75 µs, ADC read
  4.5 µs subtracted per stage) and the sine generator busy-waits on `micros()` at an uncontrolled
  sample rate;
- both channels are slaved to one `PulseTrain`; only synchronized dual-channel use is possible;
- sine phase is parsed but never applied; "piecewise linear" stages are actually rectangular steps;
- no systematic query support; undocumented defaults; malformed `S`/`W` half-updates a train;
  the serial input buffer is unbounded past 1000 bytes.

## 2. Binding decisions (user-confirmed)

1. **Target MCU: Teensy 3.5** (MK64FX512, 120 MHz Cortex-M4F). The Rev C PCB in this repo uses an
   RP2354B, but the physical device in use is Teensy-based. Code is structured so a later RP2350
   port only replaces `FastIO`/`Engine` internals.
2. **Sines stay on a separate `W` command** with explicit documented fields (no unified
   typed-stage syntax).
3. **Backward-compatible superset protocol** — `S/W/T/U/R/M/V/A/E/B/C/D/P` keep syntax and
   byte-compatible replies (StimJimBIST reads exactly one reply line for `M`/`V`/`E`).
4. **Linear ramps via a new `L` command** — triplet syntax identical to `S` but with
   ramp-between-points semantics (0-duration stage = instant jump, so piecewise-constant remains
   expressible). Legacy `S` stays bit-exact rectangular steps: silently reinterpreting `S` as
   ramps would change the stimulation delivered by existing scripts.
5. **Trigger/start conflict policy: ignore + WARN** (drop the start, increment a queryable
   counter, print the warning from `loop()`).
6. **Measurement: end-of-train summary in v1**; the MDATA streaming record format is fixed now,
   streaming implemented later; **timestamped measurement logging to the optional onboard SD card**
   (Teensy 3.5 native SDIO — does not touch the DAC/ADC SPI bus).
7. **Buttons fully repurposed to menu navigation** (Btn0 = OK, Btn1/Btn2 = prev/next), mapping 1:1
   onto the future rotary+back/ok module. Direct-start via buttons is dropped.

## 3. Decided architecture

These answer the spec's "find out and decide" items.

### 3.1 Timing: clock-based absolute-deadline scheduling (drop calibrated delays)

Each channel player owns one PIT channel. Two of the four PITs are reserved by acquiring
`IntervalTimer` objects first in `setup()` (so the Teensyduino core cannot hand them out), then
their `PIT_LDVALn/TCTRLn/TFLGn` registers are driven directly with our own ISRs. The core's
low-to-high PIT allocation must be confirmed once in the Teensyduino source (Phase 1 checklist).

Timebase: DWT `CYCCNT` extended to 64 bits in software (`cycles64()`, 8.33 ns resolution). At
120 MHz CPU / 60 MHz bus, **1 µs = exactly 120 CPU cycles = exactly 60 PIT cycles** — all
µs↔cycle conversions are exact integer multiplications, no floats, no rounding drift.

- Deadlines are absolute: `deadline[k] = t0 + f(k)`. ISR-latency jitter affects each event by its
  own latency only and never accumulates; pulse repetitions `t0 + k·periodCyc` are drift-free.
- Sub-µs latch alignment: the ISR is scheduled `PRELOAD` (≈3–6 µs) *before* the nominal deadline,
  programs the DAC(s) over SPI, then spins on CYCCNT for the short residue and pulses NLDAC exactly
  at the deadline (target jitter < 200 ns). This exploits the AD5752's separation of *program*
  (SPI shift) from *execute* (NLDAC pulse) — the spec's "programming and command execution may be
  separated" item.
- Events closer than `MIN_SCHEDULE` (≈3 µs, e.g. 0-duration jump chains) are processed inline in
  the same ISR invocation; gaps longer than `MAX_SLICE` (10 s) are chunked into intermediate
  wakeups (also keeps the 64-bit CYCCNT extension alive; PIT max load at 60 MHz is ~71 s).
- The PIT reload constant `K_RELOAD` (cycles between CYCCNT read and timer re-enable) is
  self-calibrated at boot, not hard-coded — no compiler-profile-dependent magic numbers.

### 3.2 No DMA — optimized register-level ISR engine

DMA-driven SPI on the K64 is technically possible but the wrong trade for this board:

- Chip selects and NLDAC are plain GPIOs (not hardware PCS on these pins), so a DMA DAC update is
  a ~10-step eDMA channel-linking chain writing `GPIOx_PSOR/PCOR`, paced off SPI FIFO flags that
  signal "FIFO has room", not "last SCK edge done" — fragile to prove correct on a stimulator.
- The dual-MISO arrangement (PORT-mux swap between pins 12 and 8 per ADC read, because the digital
  isolators do not tristate) keeps ADC reads CPU-driven in any design.
- Throughput does not demand it: a register-level, FIFO-overlapped path costs ≈4–5 µs per
  dual-channel sample → ≈100 kS/s dual-sync, ≈40 kS/s per channel with two independent trains.
  Application need is kHz-range sines at ≥64 samples/cycle and ≥10 µs pulse stages — covered with
  margin. Stages carrying V+I measurement are ADC-bound (~9–11 µs/pair) in *every* design, DMA
  included.

The `dacProgram`/`dacLatch` split in `FastIO` is exactly the seam a future DMA backend would
replace if Phase 6 benchmarks disappoint — the decision is revisitable without redesign.

### 3.3 Generation from ISR, not loop(); priority map

| NVIC priority (lower = higher) | Owner |
|---|---|
| 32 | SysTick (core default; ≤0.5 µs occasional jitter on players — accepted) |
| **64** | `PLAYER_PRIO`: PIT player ISRs, both channels (equal priority ⇒ NVIC serializes ⇒ SPI mutual exclusion for free) |
| **80** | `TRIG_PRIO`: pin ISRs on IN0=22 / IN1=23 (body ≈1 µs: timestamp + arm + return) |
| 112 | USB serial (core default) — **preempted by players: serial can never delay waveforms** |
| 128 | everything else |

No start path ever emits samples in its own context. Serial `T`/`U`, triggers, and the menu all
call

```c
void engineArmAndStart(const TriggerRoute& r, uint64_t tNow);   // any context
```

which copies the waveform definition into player-private state (copy-on-arm ⇒ live serial editing
is safe), precomputes all fixed-point coefficients, sets `t0 = tNow + START_LATENCY` (a fixed
small constant, so trigger-to-output latency is *deterministic*), and pends the player ISR. The
first DAC latch always happens in PIT-ISR context — this removes the first-pulse-in-caller-context
bug by construction.

End of train: the player ISR grounds the outputs, drops LEDs/markers, and pushes a
`CompletionRecord` into a small SPSC ring. **All printing and display I/O happens in `loop()`**
(the old firmware prints the result summary from timer-ISR context — another latent bug removed).

Start requests for a busy channel are rejected per decision 5 (`trigRejectCount`, WARN from
loop). `TARGET_BOTH_SYNC` claims both players; player 0 drives both DACs and latches with a common
`dacLatch(0b11)` — hardware-synchronous channel updates, tighter than the old firmware.

### 3.4 FastIO: split-phase register-level hardware access (`lib/stimjim` untouched)

```c
void    dacProgram(uint8_t ch, int16_t code);   // CS low, 8+16-bit frames via SPI0 FIFO, CS high
void    dacProgramBoth(int16_t c0, int16_t c1); // overlaps compute with FIFO shifting
void    dacLatch(uint8_t chMask);               // NLDAC pulse(s), ~100 ns, GPIO only
void    adcSelectLine(uint8_t ch, uint8_t line);// AD7321 control write (pre-issued during slack)
int16_t adcRead(uint8_t ch, uint8_t line);      // line pre-selected: ≈2 µs instead of 4.5 µs
void    adcSetMiso(uint8_t ch);                 // PORT-mux swap pin 12 ↔ 8
uint64_t cycles64(void);                        // 64-bit DWT CYCCNT timebase
void    busLock(void); void busUnlock(void);    // BASEPRI raise to PLAYER_PRIO
```

SPI0 CTAR0 is preconfigured for the DAC (30 MHz, mode 1) and CTAR1 for the ADC (10 MHz, mode 2);
`PUSHR.CTAS` selects per frame — no SPI-library transactions in the hot path. `lib/stimjim` is
reused unmodified for everything non-realtime: `begin()`, calibration (`B`/`C`), `setOutputMode()`
(two GPIO writes, ISR-safe), and the immediate serial commands `V`/`A`/`E` wrapped in
`busLock()` (delays a running player by at most one bounded transaction, ≤ ~5 µs, with a WARN when
issued during a train).

### 3.5 Sample synthesis

- **RAMP stages (`L`)**: at stage entry `N = max(1, round(dur / TARGET_DT_US))` (default 20 µs,
  per-train overridable); Bresenham-style incremental division on both the time axis
  (`t0s + (k·D_cyc)/N`) and the DAC-code axis, so sample `k = N` lands exactly on the stage
  boundary and end value. HOLD stages (`S`) emit one sample; 0-duration stages chain inline.
- **Sine (`W`)**: Q32 phase accumulator per channel, `phaseInc = f_mHz·2³²/(1000·Fs)`; start phase
  is applied as `phaseAcc0 = phase·2³²/360°` (fixes the ignored-phase bug). The 8192-entry float
  table (32 KB RAM) is replaced by a **1025-entry int16 Q15 quarter-wave-free full table + linear
  interpolation** (2 KB; worst-case error ≈2.4·10⁻⁶ FS ≪ 1 DAC LSB). All unit conversions
  (`MILLIVOLTS_PER_DAC` etc.) are folded into arm-time fixed-point coefficients — the ISR does
  integer math only (resolves the old float-speed TODO). Per-train sample rate
  `Fs = clamp(64·f_max, 1 kHz, FsMax)`; train end is enforced by deadline comparison, not sample
  count.
- **Envelope (`ENV`)**: scalar env(t) ramps 0→1 over `rampIn_us` from train start and 1→0 ending
  exactly at `duration_us`; evaluated per event as a Q15 multiply; applies to all waveform types
  (linear shape in v1; `shape` field reserved for raised-cosine).

### 3.6 Measurement

Measurement participation is decoupled from the output mode: train modes carry the original
0–3 numbering (2/3 = channel not driven; the lab's 2/3-as-unmeasured renumbering is retired,
see protocol §6.8), and *whether/what/when* to measure lives in the per-slot `MEAS` config
(`what` per channel; 90/91 mode sugar in train definitions maps onto `what=0`). A per-train
`MeasurePlan` is compiled at arm time from that config.

For `S`/`L` slots, each stage selected by `stage` (−1 = all, n = that stage only) gets a
MEASURE event near the stage end (`when=0`, transient settled), scheduled at

```
t_meas = stageEnd − Σ(adcRead ≈ 2 µs per selected line) − lineSwitch(≈2 µs if needed) − GUARD(≈1 µs)
```

so reads finish just before the next latch, with ADC programming time explicitly budgeted
instead of silently eaten from the stage duration. `adcSelectLine` is pre-issued during earlier
slack when the plan knows the next line. Stages too short for their plan get the measurement
skipped and flagged in the summary.

For `W` slots, `when` selects the positive peak (1), negative peak (2) or both (3, default);
the plan solves the phase accumulator for the 90°/270° crossings within **one period per
burst** — the first full period after envelope ramp-in completes (mid-ramp peaks would
under-read, and the V+I ADC budget of ~9–11 µs rules out per-sample measurement at generation
rates). The derivative is zero at a peak, so ADC sample-instant uncertainty is second order.

Results accumulate per point/line/channel as `n`, Σv (int32) and Σv² (int64) — the legacy
mean is `Σv/n`, and the summary additionally reports the sample standard deviation
`sqrt((Σv² − (Σv)²/n)/(n−1))`. The single-pass estimator is deliberately simple; with 13-bit
ADC codes and realistic repetition counts the precision loss is irrelevant, and it makes a
spread estimate available even for waveforms defined from very many repetitions. Summary
lines use the frozen `MSUM` record (protocol §4); per-repetition values use `MDATA`.

Manual (out-of-train) measurement is the `READ` command (protocol §4): n averaged calibrated
V+I reads with mean and sd, loop-context only, refused while a train runs. `E` stays the
single raw BIST-frozen read.

### 3.7 SD logging (new requirement)

`SdLog` uses the Teensy 3.5 onboard micro-SD socket via native SDIO (SdFat, bundled with
Teensyduino, MIT license — GPL-compatible). Only `loop()` writes to the card, draining the same
MDATA ring buffer the future streaming path uses. Record format (CSV):
`timestamp_us,slot,pulse,stage,V0_mV,I0_uA,V1_mV,I1_uA` (empty fields where not measured);
timestamps from `cycles64()/120` relative to boot; file header line carries `IDN` output + train
definitions. Flush at train end or every N records. SD write-latency spikes are harmless because
waveform generation is ISR-driven.

### 3.8 Concurrency summary

- SPI single-owner by equal ISR priority (players + engine measurement at 64; loop users under
  `busLock`). Worst case: one player delayed ≤ ~11 µs by the other player's V+I measurement event
  — the documented jitter bound for two *independent* trains (dual-sync and single trains are
  unaffected).
- Waveform definitions are loop-owned; players read only their arm-time copy.
- Status queries via per-player seqlock (ISR increments, loop retries on odd/changed) — lock-free.
- Completion reporting and MDATA via SPSC rings; ISRs never print.

## 4. Module layout (`stimjimAWG/`)

Arduino builds every `.cpp` in the sketch folder; `lib/stimjim` stays installed unmodified so
`stimjimPulser` keeps working.

| File | Content |
|---|---|
| `stimjimAWG.ino` | `setup()`/`loop()` wiring: serial→protocol, MDATA/SD drain, completion reporting, UI tick/render |
| `Config.h` | pins, priorities, FW/protocol versions, feature switches, US↔cycle macros, engine limits |
| `WaveformDef.h` | `TrainDef` (type `PIECEWISE_HOLD`/`PIECEWISE_RAMP`/`SINE`; modes; period/duration; env; meas; union stages[10]/`SineDef`), `TriggerRoute`, versioned EEPROM image |
| `FastIO.h/.cpp` | split-phase DAC/ADC ops, MISO mux, `cycles64()`, `busLock()` |
| `SampleGen.h/.cpp` | pure sample math (host-testable, no Arduino deps): ramp Bresenham, sine accumulator + int16 table, envelope |
| `Engine.h/.cpp` | `ChannelPlayer` ×2, PIT deadline scheduler ISRs, arm/start/stop, seqlock status, completion ring |
| `Measure.h/.cpp` | `MeasurePlan`, accumulation, MDATA ring, summary formatting |
| `SdLog.h/.cpp` | SD file management + record writer (`LOG` command backend) |
| `TrainStore.h/.cpp` | 100 slots, staging/validate/commit (atomic), defaults, EEPROM v2 (versioned, checksummed: slots 0–9 + ENV/MEAS + TRIG) |
| `Protocol.h/.cpp` | bounded line assembler, tokenizer (`strtol`+endptr, positional errors), dispatch table, ERR/WARN/OK helpers |
| `Commands.cpp` | legacy byte-compatible handlers + new commands + round-trip serializers |
| `Triggers.h/.cpp` | routing table, edge ISRs, `R`↔`TRIG` mapping |
| `UiInput.h/.cpp` | debounced ISR event ring (`BTN_OK/BACK/ENC_CW/CCW/CLICK`) |
| `UiMenu.h/.cpp` | non-blocking menu FSM + SSD1306 render (loop-only, dirty-flag, ≥50 ms throttle) |

Serial protocol details: see [serial-protocol.md](serial-protocol.md).

## 5. UI design

ISRs only debounce (25 ms per-input last-edge) and push 1-byte events into a lock-free SPSC ring.
The menu FSM runs in `loop()`: `HOME → SELECT` (prev/next scroll slots, compact serialized view)
`→ ARMED` (OK) `→ RUNNING` (progress; OK = stop) `→ RESULT → HOME`; an `EDIT` state is reserved
for the spec's "button … may allow setting the waveform". Screen geometry lives behind `Config.h`
defines for the taller-display upgrade; current buttons map Btn0→OK (BACK synthesized from OK
long-press), Btn1/Btn2→prev/next — 1:1 replaceable by the rotary encoder events.

## 6. Implementation phases

Per project convention (CLAUDE.md): commit at each phase end and append a handoff entry to
[PROGRESS.md](PROGRESS.md).

- **Phase 0 — Planning (this document).** docs/ + firmware-spec.md SD amendment + commit.
- **Phase 1 — Scaffold + FastIO + bench harness.** All files compiling; register-level split
  DAC/ADC ops; `cycles64`; `busLock`; `BENCH` commands; `K_RELOAD` boot self-calibration.
  Scope-verify `dacProgram`+`dacLatch` ≡ `Stimjim.writeToDac`.
- **Phase 2 — TrainStore + protocol core.** Legacy commands with atomic staging, `?` queries,
  round-trip serializers. Verify against StimJimBIST and the README/header example command lines.
- **Phase 3 — Scheduler + HOLD trains.** `T`/`U` live for `S` slots (copy-on-arm, completion
  ring, `STAT`); `READ` manual measurement; jitter histograms vs acceptance (latch < 200 ns;
  ≤ 1 µs cumulative over a 10 s train; trigger latency = `START_LATENCY` ± 1 µs); A/B against
  old firmware on the scope.
- **Phase 4 — RAMP (`L`), 0-duration chains, drift-free repeats, envelope (`ENV`).**
- **Phase 5 — Sine (`W`).** int16 table, applied phase, per-train Fs; confirm ~30 KB RAM saved.
- **Phase 6 — Dual channel.** dualSync + two independent players; collision-jitter benchmark;
  publish the measured FsMax table into `Config.h` with ~30 % margin.
- **Phase 7 — Measurement engine + SD.** `MeasurePlan` execution (stage-end / sine-peak events,
  per-stage selection, Σv/Σv² accumulation), `MSUM` summaries; `SdLog` + `LOG`; MDATA ring
  (record format final; live streaming optional).
- **Phase 8 — Triggers, UI, persistence, final protocol.** `TRIG`/`R`, button/menu UI, EEPROM v2,
  `STAT`/`IDN`/`HELP`/`DUMP` final; full verification battery (Rigol DG800 Pro → IN0
  trigger-latency measured on the TDS 2004B; long-run drift).

## 7. Bench-verify, don't guess

1. AD7321 first-conversion validity right after a control-register line switch (sets the true
   `lineSwitch` budget; the legacy 4.5 µs calibration bundles it).
2. AD5752 output settling vs NLDAC (defines `GUARD` and the minimum useful stage duration).
3. MISO PORT-mux swap at register level: no glitch/wrong first bit after swap (must match the SPI
   library's `setMISO` teardown order).
4. Real GPIO header pins — `lib/stimjim/src/Stimjim.h:52-62` defines all `GPIO_x` as pin 36
   (`GPIO_7` twice, `GPIO_8` missing); check the PCB netlist before wiring activity outputs.
5. Teensyduino core source inspection: CYCCNT enabled at reset on 3.5; `IntervalTimer` allocates
   PIT channels low-to-high.
6. `K_RELOAD` boot-calibration variance.

Fallbacks: latch jitter too high → increase `PRELOAD`; ISR cost too high → lower FsMax and/or
split measurement into a follow-up event; raw-register SPI troublesome → temporary SPI-library
fallback inside the same FastIO API (≈30–50 kS/s dual, still meets pulse-train specs).
