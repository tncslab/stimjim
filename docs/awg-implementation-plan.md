# stimjimAWG — implementation plan

Status: in progress. Target folder `stimjimAWG/` (branch `arbitrary_waveform`).
Companion documents: [serial-protocol.md](serial-protocol.md), [timing.md](timing.md), [hardware-notes.md](hardware-notes.md), [hardware-variants.md](hardware-variants.md), [PROGRESS.md](PROGRESS.md).
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
low-to-high PIT allocation is not relied on: the granted channel is identified by diffing the
TCTRL enable bits around each begin().

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
- **This is not a busy loop.** The CPU is occupied only for `PRELOAD + program ≈ 7 µs` per event
  (the spin covers the residue between the early wakeup and the deadline); between events the PIT
  is armed and `loop()` runs serial, SD and display work. A 20 µs event grid therefore costs about
  a third of the CPU. The two exceptions are inline-processed event clusters (above) and a
  measurement point, which spins to its instant before reading. The legacy firmware, by contrast,
  busy-waits every stage with `delayMicroseconds` in ISR context for the whole pulse.

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
replace if the benchmarks disappoint — the decision is revisitable without redesign.

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

Decided numerics (the pure math lives in `SampleGen`, host-tested by
`tests/host/test_samplegen.cpp`):

**Timebase and integer/float policy.** All event *times* are 64-bit CPU-cycle counts on the
`cycles64()` timebase (§3.1): 8.33 ns resolution, exact µs↔cycle conversion (×120), wraps after
~4 900 years — no overflow in any realistic run (the 32-bit hardware counter wraps every 35.8 s;
the software extension is kept alive by `loop()` and the ≤10 s `MAX_SLICE` chunking). Floats are
unusable here: FP32's 24-bit mantissa loses cycle exactness beyond 2²⁴ cycles (0.14 s), and FP64
is software-emulated on the M4F (hundreds of cycles, banned from ISRs). *Amplitudes* are integer
Q15 multiplies of DAC-code **deltas relative to the channel's calibration offset** (so scaling
never moves the parked baseline). The single place a division appears — the envelope's
`t/rampLength` — is replaced by an arm-time FP32 reciprocal, leaving one hardware-FPU multiply
per event (~20 cycles incl. lazy stacking; FP32 error ~2⁻²⁴ ≪ the 1/32768 Q15 quantum). Arm-time
coefficient computation (loop context, not time-critical) may use `double`.

**Drift-free repeats** means errors never *accumulate*: pulse k latches at the absolute deadline
`t0 + k·periodCyc` (exact integers), so each event is off by only its own ISR latency (and the
spin-latch bounds that to <200 ns), never by the sum of its predecessors' — unlike the legacy
`delayMicroseconds` chain, which accumulated per-stage calibration error over the whole train.
The same holds inside a ramp stage (exact Bresenham, below) and across bursts (per-burst phase
restart, below).

- **RAMP stages (`L`)**: at arm time `N = max(1, round(dur / TARGET_DT_US))` (default 20 µs);
  sample `k = 1..N` lands at `stageStart + ⌊k·D_cyc/N⌋` with value
  `start + ⌊(k·Δcode + N/2)/N⌋` — both realized as Bresenham-style incremental divisions
  (quotient step + remainder accumulator with carry, all constants precomputed per stage at arm
  time), so the ISR does ~3 adds per axis and sample `k = N` lands *exactly* on the stage
  boundary and end value; direct evaluation of `k·D_cyc` would overflow uint64 for long stages.
  Stage `i` ramps from stage `i−1`'s exact end (the offset at pulse start); a 0-duration stage
  degenerates to one sample at its start time — the instant jump — and chains inline through the
  `MIN_SCHEDULE` path. Each pulse begins with an offset latch at `pulseStart` (anchors OE
  connect) and ends with the off event at the last stage boundary: the final ramp value is
  latched exactly there and immediately parked (hold it by appending a same-value stage).
- **Sine (`W`)**: Q32 integer phase accumulator per channel — 2³² = one turn, so the natural
  wrap *is* the 360° wrap: exact modular arithmetic, no drift. The only rounding is the one-time
  quantization of `phaseInc` (≤0.5/2³² turn/sample — a deterministic relative frequency offset
  <10⁻⁷, not an accumulating error). Start phase `phaseAcc0 = mdeg·2³²/360000` (exact integer)
  fixes the ignored-phase bug; **phase restarts at each burst** so every burst is identical
  (legacy-consistent, and required by the peaks-per-burst measurement plan §3.6).
  Synthesis is **on the fly**: a shared 1025-entry int16 Q15 full-wave table (2 KB, filled once
  at boot) + linear interpolation (max error ≈1 LSB), then two Q15 multiplies (amplitude,
  envelope) — ~25 cycles/sample, negligible against the ~2.75 µs SPI programming cost, and
  *constant*, so it adds no jitter. Precomputing amplitude-baked per-train tables was rejected:
  it saves only the 1-cycle amplitude multiply, costs RAM and arm-time latency, cannot absorb
  the time-varying envelope anyway, and would force integer samples-per-period. Per-train sample
  rate `Fs = clamp(64·f_max, 1 kHz, FsMax)` realized as an exact integer cycle count per sample
  (`phaseInc` is computed from the actual sample period, so frequency exactness never depends on
  Fs rounding); `f_max > Fs/2` is refused at start (unrepresentable). Train end is enforced by
  deadline comparison, not sample count.
- **Envelope (`ENV`)**: scalar env(t) ramps 0→1 over `rampIn_us` from train start and 1→0 ending
  exactly at `duration_us`; evaluated per latch event against the event's absolute deadline and
  applied as a Q15 multiply on the code delta. Applies to all waveform types: L/W follow it per
  sample; S trains sample it at each stage latch (stair-step — use L for smooth ramps). Events
  overrunning `duration_us` (legacy stages run to completion) clamp to 0 when a ramp-out exists;
  with `rampOut = 0` the envelope is flat through the end. Linear shape in v1; `shape` reserved
  for raised-cosine.

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

## 6. What is built and what remains

Per project convention (CLAUDE.md), each phase ends with a commit and a handoff entry under
`docs/progress/`, indexed by [PROGRESS.md](PROGRESS.md); that log is where the history lives.

**Built and on hardware.** The scaffold and `FastIO` (register-level split DAC/ADC ops,
`cycles64`, `busLock`, the `BENCH` group, `K_RELOAD` boot self-calibration); `TrainStore` with
atomic staging, `?` queries and round-trip serializers; the deadline scheduler and both channel
players, playing `S`, `L` and `W` slots with the `ENV` envelope, copy-on-arm, the completion ring
and `STAT`; the per-slot post-trigger delay; `TRIG`/`R` routing with edge ISRs and the stimulus
marker; the measurement engine (`MeasurePlan` compilation with its window-fit rule, stage-end and
sine-peak events, Σv/Σv² accumulation, `MSUM` summaries and the `MDATA` ring); `SdLog` with `LOG`
and the `SD` file group that serves the card over the serial port; EEPROM persistence; the OLED
status display with `SCREEN` capture; and the register / portable backend split that lets the same
source run on Teensy 3.x and 4.x ([hardware-variants.md](hardware-variants.md)).

The `BENCH` group has been run on silicon and the measured figures are tabulated in
[serial-protocol.md](serial-protocol.md) §4. The two headline acceptance numbers hold: residual
latch jitter is **42 ns** against a 200 ns target, and every latch of every waveform type meets
its deadline with measurement enabled — checked by the engine's own per-latch deadline
comparison rather than by a scope, so any board can be re-qualified over the serial port alone.

**Also built.** The timing budgets are runtime state (`Cal.h`, the `CAL` command group, persisted
by `P` and reported by `IDN`), the ramp sample interval is per slot (`L`'s 7th header field or
`DT`), and a measurement point whose reads do not fit its free gap rotates them over consecutive
repetitions instead of being refused (`MEAS` `fit`). The trigger ISRs timestamp the edge and the
engine measures the start latency from that timestamp, so interrupt entry and the arm-time
precomputation are spent *inside* `CAL STARTLAT` rather than added to it — which is why
`STARTLAT` has to be wide enough to hold the arm, and is reported by name when a train's arm does
not fit it. Since phase 14 prepared the arm in `loop()` it is **20 µs** on the board in hand (a
6.0 µs warmed arm plus `PRELOAD + DACPROG2 + 3 + TRIGCOMP`), against a compiled default of 35 that
still covers the cold path and the independent route. `BENCHARM` measures the arm and
`BENCHSETTLE` the settling time; `TRIGCOMP`, the last of the four budgets that needed an
oscilloscope, was measured in phase 16 and is 2 µs.

**Remaining work.**

- **Sine ceiling and dual-channel collisions.** `SJ_FS_MAX_HZ` is still the desk estimate. A
  single unmeasured sine train keeps every deadline up to Fs = 49.9 kHz, so the ceiling is not
  obviously wrong, but the *dual-channel* collision case (two independent trains contending for
  the SPI bus) has not been measured, and that is what sets the published FsMax table with its
  ~30 % margin. The remaining scope work is long-run drift (≤ 1 µs cumulative over a 10 s train)
  and the full trigger-latency battery.
- **One number still guessed.** `CAL TRIGCOMP`, the pin-edge-to-ISR-entry delay of the trigger
  path, is 0 because software cannot see the physical edge. It is the only budget left that
  needs an oscilloscope; [bench-wiring.md](bench-wiring.md) describes the wiring and the
  procedure. `CAL SETTLE`, which used to sit beside it, was measured in phase 9 with
  `BENCHSETTLE` and needed no scope at all.
- **Menu UI.** The button editing FSM (`HOME → SELECT → ARMED → RESULT`). This is the last
  unbuilt item of the requested-feature list in `firmware-spec.md`.
- **The arm costs 9.0 µs before it does anything train-specific**, and since that cost sits
  inside `STARTLAT` it is the trigger latency. It has been profiled and cut from 16.2–37.5 µs to
  9.0–15.0 µs ([timing.md](timing.md) §7); what remains is a floor no rearrangement inside
  `startTrain` gets below, so the next step is not a smaller arm but no arm at all on the edge —
  pre-arming the train and firing on a bare `NLDAC` pulse (§3 of that document).
- **A pre-armed trigger path could reach single-digit microseconds.** The AD5752 already separates
  program from execute, and `dacLatch` costs 0.44 µs, so if the *whole* arm moved before the edge —
  armed when the `TRIG` route is set, or at the end of the previous train — the edge ISR would only
  pulse `NLDAC`, switch the output enable out of ground and program the PIT for the second event:
  an estimated 1–3 µs delivered latency instead of 60. The costs are that copy-on-arm would then
  happen at route-set time (slot edits would need a re-arm to take effect) and that the preloaded
  input register would have to be rewritten after every `V`/`A`/`B`/`C`/`READ` and after each park.
  Worth it only for waveforms whose first sample is a jump away from the parked level (`S`, a
  0-duration first `L` stage, a `W` with nonzero start phase); the 8–9 µs analog settling is the
  floor either way. See [timing.md](timing.md) §3.
- **The first `adcSelectLine` could be issued inside the `SETTLE` window.** A control-register
  write samples nothing, so it can overlap the settling; that would return `ADCSWITCH` = 4 µs of
  the 9 µs every measurement point now pays.

## 7. Bench-verify, don't guess

1. AD7321 first-conversion validity right after a control-register line switch — **measured**:
   `BENCHSW` puts select+read at 4.58 µs average, 6.10 µs worst, against 2.22 µs for a read with
   the line already selected. The switch therefore costs ~2.4 µs and the conversion following it
   is valid; the budget carries 7 µs so the worst case is covered outright.
2. AD5752 output settling vs NLDAC — **measured in phase 9, and it needed no scope**:
   `BENCHSETTLE` latches the same step repeatedly and reads it back at increasing delays, so the
   delay at which the readings stop moving is the answer. It is 8–9 µs on either channel, at
   either polarity and at 2000 or 8000 codes of step; the 4 µs the firmware had assumed reads
   8–9 % short. `CAL SETTLE` is now 9 µs, which is the number that bounds the shortest useful
   measured stage: 24 µs of free gap for one reading, 47 µs for V+I on both channels.
3. MISO PORT-mux swap at register level — **measured**: `BENCHMISO` alternates channels at
   2.29 µs per read, statistically the same as a same-channel read, and the readings are
   plausible on both channels, so the swap costs nothing and glitches nothing.
4. Real GPIO header pins — `lib/stimjim/src/Stimjim.h:52-62` defines all `GPIO_x` as pin 36
   (`GPIO_7` twice, `GPIO_8` missing); check the PCB netlist before wiring activity outputs.
   **Still open**; nothing built so far needs those pins.
5. Teensyduino core source inspection: CYCCNT enabled at reset on 3.5; `IntervalTimer` allocates
   PIT channels low-to-high. **Done in phase 6** (and both assumptions were wrong in ways that
   stopped the board from booting).
6. `K_RELOAD` boot-calibration variance — **measured**: `BENCHK` residuals span 11 cycles
   (92 ns) over 200 repetitions, and boot-to-boot values sit between 84 and 98 cycles. The
   closed-loop calibration is what makes that spread irrelevant.
7. Trigger pin edge to ISR entry — **still open on the scope**, and the only part of the
   trigger-to-output latency software cannot see: the ISR timestamps the edge at its own first
   instruction, so everything after that is already compensated. `CAL TRIGCOMP` carries the
   hardware part and is 0 until measured.

Fallbacks: latch jitter too high → increase `CAL PRELOAD`; ISR cost too high → lower FsMax and/or
split measurement into a follow-up event; raw-register SPI troublesome → temporary SPI-library
fallback inside the same FastIO API (≈30–50 kS/s dual, still meets pulse-train specs).
