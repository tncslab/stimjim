# stimjimAWG latency and timing

This document answers, in one place, how long `stimjimAWG` takes to put signal on the output
after a trigger edge, why that number is what it is, how it compares with the original
`stimjimPulser` firmware, whether the DAC can be preloaded to fire faster, what the trigger
interrupt actually starts, whether waveform generation occupies the CPU, and what SD logging
costs and stores. The short answers: a trigger edge delivers the first latch a fixed **35 µs**
later on a Teensy 3.5 with the register backends (`CAL STARTLAT`, adjustable without a rebuild),
most of which is the arm — the copy and precomputation the edge ISR does before it hands the
train to the timer, 9.0 µs for the simplest train and 15.0 µs for the heaviest — and
none of which is the DAC write; the original firmware had no fixed
figure at all, because it played the first pulse inside the trigger ISR after two USB
`Serial.print` calls; the AD5752 *can* hold a preloaded code and fire on a bare `NLDAC` pulse in
0.44 µs, so a pre-armed trigger path could reach a few microseconds, but that needs the arm moved
before the edge and is not built; the edge ISR only computes `t0` and programs a timer, and every
sample is emitted from the timer ISR; generation is interrupt-driven with a bounded spin of a few
microseconds per event, not a busy loop; and an SD write can never delay a waveform, because only
`loop()` touches the card and the player ISRs preempt it.

## 0. Terminology: what "latch" means here

A **latch** is one DAC output update. The AD5752 keeps a written value in its input register and
does nothing with it until the `NLDAC` pin is pulsed low, which transfers the value to the DAC
register and moves the output. Analog Devices names that pin "load DAC", so **update** and **load**
are the neutral synonyms; the word carries no sense of a software lock or mutex. This document uses
"latch" for the event itself — the instant the output moves — and "update" wherever "latch" would
read as locking. The engine's whole timing design rests on the two halves being separable: it
*programs* (SPI write) during a preload window and *latches* (one `NLDAC` pulse, 0.44 µs) exactly
on the deadline.

## 1. Trigger-to-output latency

A trigger start delivers its first DAC latch at

```
t0 = <edge timestamp> − CAL TRIGCOMP + CAL STARTLAT + <slot delay_us>
```

`STARTLAT` is **35 µs** by default on a Teensy 3.5 at 120 MHz with the register backends, and
120 µs on the portable (Arduino-SPI + `IntervalTimer`) build. It is runtime state:
`CAL,STARTLAT,<us>` changes it, `P` persists it, `CAL?` reports what a running board uses. A board
that already has an EEPROM image keeps the budget it stored — the image is not rejected over a
default that only got smaller — so adopting the new one takes `CAL,STARTLAT,35` and `P`.

The latency is deterministic because the edge ISR timestamps the edge in its own first
instruction, and everything it then does is spent *inside* `STARTLAT` rather than added after it.
What has to fit in the window:

| Term | Cost (T3.5, register backends) |
|---|---|
| `Engine::startTrain` — copy-on-arm and fixed-point precomputation | 9.0 µs for a train that drives nothing, 11–12 µs for a one-stage `S`/`L`/`W`, 15.0 µs for a ten-stage `L`. Neither in-train measurement nor stage count adds much: the plan is compiled in `loop()` and stage i+1 is derived while stage i plays — §7 measures every case |
| `CAL PRELOAD` — how early the player ISR wakes before the first latch | 4 µs |
| `CAL DACPROG2` — the dual-channel SPI write inside that window | 5 µs budgeted (2.75 µs measured) |
| `SJ_MIN_SCHEDULE_US` | 3 µs |

So `STARTLAT ≥ arm + 12 µs`, and the arm is the term that sizes it — **the DAC write is not the
bottleneck, the arm is**. When an arm does not fit, the train still runs with a late first latch,
and its completion names the `STARTLAT` that would have covered it.

One case exceeds the 35 µs default and reports itself that way: a `TRIG` route in independent
mode arms two engines inside one ISR and pays the arm twice, so two ten-stage `L` trains need
about 39 µs. An engine re-triggered so fast that `loop()` never ran in between prepares its
measurement plan inside the arm instead, which costs 0.55 µs per point plus 3.1 µs per point if
the slot changed too — a ten-point plan then needs 29 µs, which the default still covers. A train
has to *finish* before its engine can be re-armed, so that takes `loop()` starved for a whole
train, not merely a fast trigger.

Jitter, not latency, is what the design buys: residual latch jitter is **42 ns** (`BENCHPIT` with
a preload), and the per-slot delay is exact to the scope's own sample interval (2000 µs set →
1999.3 µs measured, 20000 → 19996.0).

Two qualifications on the 35 µs:

- **`CAL TRIGCOMP` is 0 and unmeasured.** It is the delay from the physical edge at the input pin
  to the ISR's first instruction, which software cannot see; expect a few hundred nanoseconds.
  The bench measures trigger delay differentially (the same edge starts a reference pulse on the
  other engine), so the *absolute* edge-to-output figure has never been captured — see
  [bench-wiring.md](bench-wiring.md) configuration B.
- **The analog path adds its own 8–9 µs.** After a latch the output needs that long to reach its
  final value (`BENCHSETTLE`), so the load sees the onset at `STARTLAT` and full amplitude 8–9 µs
  later. No scheduling change can shorten that.

An independent two-engine route is additionally limited by latch contention: both player ISRs run
at priority 64, so the second waits for the first and lands **7.3 µs** later by the firmware's own
counter, 9–10 µs at the output. Two channels that must be sample-aligned belong in one train that
drives both, where a single `dacProgramBoth` and a single latch pulse serve both.

## 2. Comparison with the original `stimjimPulser`

The original firmware has no fixed trigger latency, because its trigger ISR plays the first pulse
itself. `startIT0ViaInputTrigger` → `startIT0` runs, in this order: two 80-byte `memset`s of the
measurement history, `micros()`, `IntervalTimer::begin(pulse0, period)`, **`Serial.print` plus
`Serial.println` of "Started T train with parameters of PulseTrain \<n\>"**, the LED and marker
GPIO writes, and only then `pulse0()`, which converts amplitudes with float divisions and writes
the DACs (`stimjimPulser/stimjimPulser.ino:663-726`). The comment on that call says why it is
there — `//intervalTimer starts with delay - we want to start with pulse!` — an `IntervalTimer`
fires its first callback one whole period after `begin()`, which would have delayed the first
pulse by the inter-pulse interval.

The consequences are structural rather than tunable:

- **The USB print is in the latency path.** A buffered CDC write costs microseconds; a write that
  finds the transmit buffer full blocks until the host drains it, which is milliseconds. The
  delivered latency therefore depends on whether a host is listening and how promptly.
- **Nothing is compensated.** The arm's cost is *added* to the onset, not subtracted from a
  budget, so it varies with train complexity and there is no constant to correct for.
- **The first pulse can be preempted by its own repeat.** The pin ISR runs at the core default
  priority while the repeat `IntervalTimer` runs at 64.
- **Desk arithmetic, not a measurement:** the non-print terms sum to roughly 10 µs, so the best
  case is under the 35 µs of `stimjimAWG` and the worst case is unbounded. Neither
  firmware's absolute edge-to-output latency has been measured on this bench; what is measured is
  the arm cost (`BENCHARM`) and the delay accuracy.

`stimjimAWG` differs by construction: no ISR ever prints (completions go through a ring and
`loop()` prints them), `t0` is anchored to the edge timestamp so the arm is subtracted rather than
added, the first latch always happens in the player ISR, and the trigger ISR sits at priority 80
*below* the players, so a trigger can never delay a waveform already playing. The trade is that
the constant is currently 35 µs — larger than the original firmware's best case, smaller than its
worst, and unlike either, repeatable to 42 ns.

## 3. Can values be preloaded into the DAC to fire faster?

Yes in hardware, and the firmware already relies on the mechanism for every latch. The AD5752
separates *program* (a 24-bit SPI write into the input register) from *execute* (an `NLDAC`
pulse), so `FastIO::dacProgram` and `FastIO::dacLatch` are two independent operations, and
`dacLatch` costs **0.44 µs** measured. A code written into the input register stays there
indefinitely until something overwrites it.

What prevents the trigger path from exploiting that today is not the DAC — it is that the arm runs
*inside* the trigger ISR. Preloading the code buys nothing while the 9–29 µs of
copy-and-precompute still has to happen after the edge. Reaching a few microseconds needs the
whole arm moved *before* the edge — armed when the `TRIG` route is set, or at the end of the
previous train — leaving the edge ISR three cheap steps: pulse `NLDAC`, switch the output enable
from ground to the channel's mode (two GPIO writes), and program the PIT for the second event.
That is an estimated **1–3 µs** delivered latency, dominated by interrupt entry and the port-ISR
dispatch. It is not built and not measured.

The costs of that design, which is why it is an open item and not the default:

- Copy-on-arm would happen when the route is set, so slot edits after that point would not take
  effect until the route was re-armed — today an edge always picks up the current definition.
- The preloaded input register would have to be rewritten after anything else that touches the
  DAC: `V`, `A`, `B`, `C`, `READ`, and the park write at train end.
- A trigger that never arrives leaves the board holding a charged input register. That is harmless
  — the output stays grounded and unlatched — but it is state that has to be tracked.

**Which waveforms it would actually help.** The first sample of *every* type is known before the
edge, because it comes from stored parameters and not from the trigger time, so preloading is
technically possible for `S`, `L` and `W` alike. It buys a visibly earlier onset only where that
first sample is a discontinuous jump away from the parked level: a rectangular `S` stage, an `L`
train whose first stage has 0 duration (an instant jump), or a `W` sine whose start phase has a
nonzero sine. A plain `L` ramp starts *at* the park level and a `W` at phase 0 starts at zero
amplitude, so their first latch changes nothing at the output; moving it earlier shifts the
timebase, not the onset. In that practical sense, the expectation that this matters for square
waves is right, with those two extensions.

The ceiling stands regardless: the output still needs 8–9 µs after the latch to reach full
amplitude, so a pre-armed path would move the onset from ~35 µs to a few µs without sharpening the
rise.

## 4. What the trigger interrupt starts

The edge ISR emits no signal. It timestamps the edge, arms the player (copy-on-arm plus all
fixed-point precomputation), computes `t0`, and programs that player's PIT channel to wake
`CAL PRELOAD` before `t0`. The first DAC latch — like every later one — happens in the PIT player
ISR. **The interrupt supplies the time reference; the clock emits the samples.**

This is deliberate, and it removes the original firmware's first-pulse-in-caller-context behaviour
by construction:

- the delivered latency is a constant that does not depend on the train's complexity, on how busy
  `loop()` is, or on how long interrupt entry took;
- the trigger ISR (priority 80) is below the players (64), so an edge can never delay a waveform
  already playing, and a first pulse can never be preempted by its own repeat;
- both engines started by one edge share that edge's timestamp as their `t0`, so they sit on one
  grid (engine-to-engine offset −0.25 µs, against −7.5 µs when each engine anchored on its own
  arm).

## 5. Busy loop or interrupts?

Interrupt-driven, with a bounded spin of a few microseconds per event. One latch costs:

```
PIT wakes CAL PRELOAD (4 µs) early
  → dacProgram / dacProgramBoth (1.4 / 2.75 µs measured)
  → spin on cycles64() to the exact deadline
  → NLDAC pulse (0.44 µs)
```

Between events the PIT is armed and the CPU is in `loop()` running serial, SD and display work —
it is not spinning. The CPU cost per event is about `PRELOAD + program ≈ 7 µs` with both channels
driven, so a 20 µs event grid (the default `DT`, and a 50 kHz sine) runs at roughly a third duty
cycle.

Two places where the ISR stays in and spins longer:

- events within `PRELOAD + MIN_SCHEDULE` (7 µs) of each other are handled inline in the same ISR
  pass rather than re-entering the NVIC — 0-duration jump chains and dense sample grids become one
  continuous busy stretch;
- a measurement point spins to its instant before reading the ADC, because the reading has to be
  taken where the plan placed it.

Gaps longer than `SJ_MAX_SLICE_US` (10 s) are chunked into intermediate wakeups: the PIT cannot be
loaded with more, and the wakeups keep the 64-bit `CYCCNT` extension alive.

The original firmware is the opposite: `pulse()` busy-waits each stage with `delayMicroseconds`,
subtracting hand-calibrated DAC and ADC costs (2.75 µs and 4.5 µs) from the requested duration,
and the sine generator polls `micros()`. The CPU is occupied for the whole pulse, in ISR context,
and each stage boundary carries the accumulated error of its predecessors.

## 6. SD logging: latency and content

**Latency: none that a waveform can see.** No write ever happens in ISR context. The player ISR
pushes a fixed-size record into a 128-entry single-producer/single-consumer ring; `loop()` formats
it and hands it to SdFat, which buffers into a 512-byte block. A physical SDIO write happens at a
block boundary, at train end, every 64 rows, or once a second, whichever comes first. Because the
player ISRs (priority 64) preempt `loop()` unconditionally, and the card is on the Teensy's native
SDIO rather than the DAC/ADC SPI bus, a card write cannot delay a latch, lengthen a pulse, or take
the bus lock.

What card latency *can* cost is measurement records. SD write latency is dominated by the card's
own controller: typically well under a millisecond for a buffered block, with occasional stalls of
tens of milliseconds — hundreds on cheap cards — during internal housekeeping. **Those figures are
the general behaviour of SD cards, not a measurement on this board: there is no `BENCHSD`, and the
write path has never been timed here.** The exposure is the ring's headroom — a stall costs records
once it exceeds `128 / (measurement points per second)`, which is 64 ms at 2000 rows/s. An
overflow is never silent; it prints

```
WARN MEAS: MDATA ring overflowed — <n> records dropped (the host is not reading fast enough)
```

Two card operations do block `loop()` for a long time: `SDINFO,1` walks the whole free-cluster
chain (seconds, and it is refused while a train runs), and `SDGET` streams a file over the serial
port. Neither disturbs a waveform, for the reason above.

**Content.** Log files are `LOG0000.CSV` upwards — the lowest free index, never reused, because
the board has no clock — or a name given to `LOG1,<name>`. A file contains, in order:

1. written when the file is opened: a header line (`# stimjimAWG log — fw=…, proto=…`), the `IDN`
   identity block, and the whole session configuration as the paste-back-able lines `DUMP` prints
   (every slot's canonical `S`/`L`/`W` line, `ENV`, `MEAS`, the `TRIG` table and the `CAL` timing
   budget, so the budget the readings were taken with is recorded next to them);
2. a `# columns: timestamp_us,slot,pulse,point,V0_mV,I0_uA,V1_mV,I1_uA` line;
3. a `# train:` block for every train that arms while the file is open — that slot's canonical
   waveform line, its `ENV` line if non-default, and its `MEAS` line. This is what records
   configuration changes made after the file was opened, and what keeps a log self-describing when
   trains were fired by trigger edges with no host attached;
4. one CSV row per measurement repetition, for slots whose `MEAS` `report` has bit 1 set (`+2`):
   `<timestamp_us>,<slot>,<pulse>,<point>,<V0_mV>,<I0_uA>,<V1_mV>,<I1_uA>`, timestamp in µs since
   boot. Lines that were not read are empty fields.

Nothing else goes to the card. Output samples are not dumped, and the persistent configuration
lives in the EEPROM, not on the card. The socket sits under the instrument cover, so the whole
card is readable back over the serial port with the `SD` group — see
[serial-protocol.md](serial-protocol.md) §4.


## 7. Where the arm's microseconds go

`STARTLAT` is the trigger latency and `Engine::startTrain` is what sizes it, so this section
accounts for the arm. The **goal is an arm inside one latch interval**: the shortest interval the
engine schedules is 20 µs (the default `DT`, and the sine sample interval at the 50 kHz `FS_MAX`
ceiling), and `STARTLAT` must also cover `PRELOAD + DACPROG2 + MIN_SCHEDULE` = 12 µs, so the arm
has to fit in **8 µs**.

What the arm produces, in the order it runs: gatekeeping (busy and channel-conflict checks, the
Nyquist and ramp-interval refusals, a copy of the `CAL` set); copy-on-arm of the definition into
the 912-byte player state, which is what keeps live serial editing safe mid-train; unit conversion
of every stage amplitude to a DAC-code delta relative to the park code and every duration to a
cumulative cycle offset, so the ISR does no unit math; the shape-specific constants (`L` Bresenham
steps per stage, `W` sample rate and phase increments, the envelope's reciprocals); the
measurement plan; and finally `t0` plus one timer program.

Of all that, only the train-level scalars and **one stage's worth** of codes and times are needed
at `t0`. Stage i's constants are first read at the end of stage i−1 — 20 µs to seconds later — and
the measurement plan's first point cannot fire until at least `SETTLE` after the first latch. Both
observations have since been acted on, which is why the table below barely varies with either
stage count or plan size.

### Measured

`BENCHARM` on a Teensy 3.5 at 120 MHz (120 cycles/µs), register backends, `hot=RAM`, via
`tests/device/bench_arm.py`. Three columns, because the arm has three regimes:

- **warmed** — one arm per `loop()` pass, which is what a running board delivers. `loop()`
  compiles the next measurement plan and clears the last train's accumulators before the edge
  arrives (`Engine::warmPlans`), so neither shows up here.
- **typical** — the minimum over `--reps` arms run back to back inside one bus lock.
- **worst** — the maximum over the same run: an engine re-triggered before `loop()` could prepare
  anything. This is the fallback path, and the only one that still builds a plan inside the arm.

| Case | warmed, µs | typical, µs | worst, µs |
|---|---|---|---|
| undriven, 0 stages | 9.04 | 9.04 | 9.50 |
| `S`, 1 ch, 1 stage | 10.79 | 10.79 | 11.29 |
| `S`, 2 ch, 1 stage | 11.12 | 11.33 | 11.58 |
| `S`, 2 ch, 1 stage, `MEAS` V+I both | 11.12 | 11.12 | 12.83 |
| `S`, 2 ch, 10 stages | 13.29 | 13.29 | 13.79 |
| `S`, 2 ch, 10 stages, `MEAS` V+I on every stage | 13.29 | 13.29 | 19.96 |
| `L`, 2 ch, 1 stage | 12.17 | 12.38 | 12.62 |
| `L`, 2 ch, 10 stages | 15.00 | 15.00 | 15.54 |
| `W`, 2 ch | 10.96 | 10.96 | 11.50 |

Read the third and fourth rows together, and the fifth and sixth: **in-train measurement costs the
arm nothing at all**, whether the plan has one point or ten. Read the third against the fifth and
the seventh against the eighth: **nine extra stages cost 2.2 µs on an `S` train and 2.8 µs on an
`L` one**, where they used to cost 8.0 and 17.4. What a train costs the arm is now:

| Term | Cost | What it is |
|---|---|---|
| floor | 9.04 µs | gatekeeping, the `CAL` copy, the geometry, the train-level scalars, `envInit`, `t0`, one `pitProgram` |
| stage 0 | 0.6 µs (`S`) / 1.6 µs (`L`) | the one stage whose values are due at `t0` — two amplitude conversions and, for `L`, `rampStageInit` |
| per further stage | 0.25 µs (`S`) / 0.31 µs (`L`) | one 64-bit `dur_us × 120` into `cum[]` and, for `L`, the sample-count division `rampStageN`. Both feed the measurement plan, so they cannot be deferred the way the rest of a stage can |
| per measurement point | 0 | compiled and cleared in `loop()`. 0.55 µs to clear, and 3.1 µs more to compile, only when an engine is re-triggered before `loop()` ran |

So `CAL STARTLAT` is **35 µs**. The worst warmed arm is the ten-stage `L` train's 15.0 µs, which
needs 15.0 + `PRELOAD` + `DACPROG2` = 24 µs — and that is not arithmetic alone: set `CAL,STARTLAT,23`,
put a real trigger edge on that train, and the engine reports `set CAL STARTLAT >= 24 us`, while 25
and everything above it runs clean. The worst *cold* arm — an engine re-triggered with `loop()`
starved, which compiles its own ten-point plan — is 19.96 µs and needs 29. 35 covers both with
margin. The one case that still exceeds it says so at train end rather than failing quietly: a
`TRIG` independent route of two ten-stage `L` slots, which pays the arm twice and needs ~39 µs.

**The 8 µs target is not reachable by removing work from the arm.** The floor alone is 9.0 µs, for
a train that drives nothing. Getting under one latch interval needs the arm moved *before* the
edge (§3), not made smaller.

### Where the arm no longer spends anything

Eight changes took the arm from the 16.2–37.5 µs it once cost to the table above. All of them move
work out of the arm rather than making it faster.

1. **The measurement plan is compiled in `loop()`, not in the arm.** The compiled region depends
   only on `(slot, definition, CAL)` — never on the calibration offsets — so it carries that tag,
   and `loop()` compiles it for whatever slot a `TRIG` route would start next, falling back to the
   last slot written for an engine no route names. That is worth 3.1 µs per measurement point,
   which was the largest single term in the arm.
2. **The accumulators are cleared in `loop()` too**, after the summary has been printed — 0.55 µs
   per point. No memory operation makes this free where it stood: 96 bytes is 24 SRAM words, the
   generic `memset` spends ~66 cycles on them, and a hand-rolled `STMIA` clear would only halve
   that. Moving it was the only way to reach zero.

   Both of those rest on one structural change: **each engine holds two plans**, and the arm swaps
   between them instead of rewriting one in place. The player ISR only ever reads the live buffer,
   so `loop()` owns the other and can prepare it with no lock — a `volatile` flag handshake covers
   the one window where a trigger edge lands inside a `loop()` compile, and the arm then rewrites
   the live buffer in place exactly as the single-buffer firmware did. It costs 2448 B of RAM, and
   it also means a train re-armed before `loop()` drained its completion keeps its summary, which a
   single plan could not.
3. **Only the plan a train uses is zeroed.** The compiled region is 242 bytes of the struct's 1224,
   and the accumulators are cleared per existing point (96 bytes each) instead of all ten. An
   unmeasured train zeroes nothing at all.
4. **The sine constants are derived when `W` is parsed.** `sampleCyc`, both phase increments and
   both start phases depend on the definition alone, so they are computed in command context and
   cached per slot (2 kB), keyed on the definition epoch. That is why a `W` arm now costs about
   what an `S` arm does — it was 18.3 µs more, nine soft-float `double` calls per channel in
   `sinePhaseInc`, which the M4F has no hardware for. The arm still derives on demand if the cache
   was invalidated, so correctness never depends on the warm-up — only the latency does.
5. **The ramp's time-axis division is 32-bit.** `qt = durCyc/N` and `rt = durCyc%N` are computed as
   `qt = (dur_us/N)·cycPerUs + ((dur_us%N)·cycPerUs)/N`, `rt = ((dur_us%N)·cycPerUs)%N`, which is
   an algebraic identity, not an approximation: four hardware `UDIV`s replace two
   `__aeabi_uldivmod` calls with bit-identical results. Stages longer than ~35 s fall back to the
   64-bit form, and `tests/host/test_samplegen.cpp` checks both paths against the 64-bit reference
   over the whole range including both overflow guards.
6. **`SJ_CODE_IN_RAM` (default on for Kinetis) puts `startTrain`, `buildGeometry`, `playerRun`,
   `rampStageInit` and `rampStep` in the `.fastrun` section**, which the core copies from flash
   into SRAM at boot, so they execute without flash wait states. It costs about 6.4 kB of RAM and
   nothing else — the copy is remade from the flash image on every reset, and the MCU's POR/LVD
   resets the chip long before SRAM contents could decay, so there is no failure mode here that the
   stack and globals do not already have. `IDN` reports `hot=RAM`, `hot=flash` or `hot=ITCM`
   (Teensy 4.x runs all code from RAM anyway), because a `BENCHARM` figure is only comparable
   against a binary with the same answer.
7. **`SampleGen::rampStageN` is a header inline, not a call.** It is the one piece of a ramp stage
   the measurement plan needs — the sample count — so both `buildGeometry` and `rampStageInit`
   compute it; as an out-of-line function it cost 0.35 µs per ramp stage, because `SJ_HOT` carries
   `noinline`, which is more than the division it wraps.
8. **Stage i+1 is derived while stage i plays.** The arm derives stage 0 — the one stage whose DAC
   codes are due at `t0` — and the player derives each later stage in the gap before a latch it is
   already waiting for. That takes 0.6 µs per `S` stage and 1.6 µs per `L` stage out of the arm and
   is what makes a ten-stage `L` train cost 15.0 µs instead of 28.7.

   The mechanism is one counter, `nDerived`: stages below it have their DAC-code deltas and, for a
   ramp, their Bresenham constants. `Engine::deriveAhead` tops it up on the one path in `playerRun`
   that programs the timer and returns — idle time by construction — and only while the gap is
   wider than `SJ_STAGE_DERIVE_US`. `ensureDerived` at each use site is the guarantee, so
   correctness never depends on the look-ahead having run; the call sites sit *after* a latch, so
   a derivation that does fall back spends the interval to the next latch and never a preload
   window. It derives as many stages as fit rather than exactly one, which is what covers a
   0-duration `L` jump: its single sample shares a deadline with the previous stage's last one, so
   entering it yields no gap of its own and the stage after it has to be ready already. Two
   0-duration stages in a row are refused at parse time, so that chain is at most two long.

   The player carries a 120-byte copy of the stage triplets for this (+336 B of RAM for the two
   engines), because it must not read `TrainStore` while a train runs: a slot write is refused only
   for the slot a train is *attached to*, and that check is made in `loop()` while a trigger edge
   can arm from an ISR. The conversion was rewritten to take the channel's park code as an argument
   instead of reading `Stimjim`'s offset tables, so nothing in the ISR path depends on live
   calibration state; the expression and therefore every DAC code is unchanged.

   **Measured on the board, not argued:** `tests/device/stage_timing.py` runs the six trains that
   stress the stage machinery — ten `S` stages a millisecond apart and twelve microseconds apart,
   ten `L` stages at the default 20 µs interval and at the 12 µs floor, an alternating 0-duration
   jump chain, and a measured ramp — and every one completes with `lateEvents` and `overdueEvents`
   both zero. The same script prints each case's `MSUM` rows, and they match the pre-change
   firmware to within 7.3 mV, against per-point standard deviations of 2–10 mV.

**What RAM residency is actually worth, measured.** Building the phase-11 sources with
`-DSJ_CODE_IN_RAM=0` and running the same sweep. The absolute numbers predate the last two changes
above, so read the third column, not the first two:

| Case | `hot=RAM` | `hot=flash` | saved |
|---|---|---|---|
| undriven, 0 stages | 8.45 | 9.88 | 1.43 µs |
| `S`, 2 ch, 1 stage | 9.88 | 11.92 | 2.04 µs |
| `S`, 2 ch, 10 stages | 17.88 | 21.04 | 3.16 µs |
| `L`, 2 ch, 1 stage | 11.38 | 13.75 | 2.37 µs |
| `L`, 2 ch, 10 stages | 28.79 | 33.04 | 4.25 µs |
| `W`, 2 ch | 10.12 | 12.29 | 2.17 µs |

13–17 %, not the 30–50 % the flash-wait-state hypothesis implied. The distinction the numbers draw
is between a **large straight-line function** and a **small hot loop**: `startTrain` is 2.8 kB and
`playerRun` 3.2 kB, both far past the K64's 512-byte flash cache, and those are where the saving
comes from. Moving the 180-byte `rampStageInit` into RAM as well saved only 0.037 µs per ramp stage
(0.33 µs over ten) — the cache holds a loop that size perfectly well.

To build the flash-resident half of the A/B, pass the define through the property the Teensy
recipe actually reads — `compiler.cpp.extra_flags` is silently ignored here
([hardware-variants.md](hardware-variants.md) §1):

```
arduino-cli --config-file tmp/arduino-cli.yaml compile --fqbn teensy:avr:teensy35 \
  --build-property "build.flags.defs=-D__MK64FX512__ -DTEENSYDUINO=160 -DSJ_CODE_IN_RAM=0" \
  --warnings more stimjimAWG
```

### Planned, not implemented

Ordered by what the measurements say each is worth, largest first.

- **Move `cum[]` and the ramp's per-stage `N` out of the arm as well.** They are what is left of
  the per-stage cost — 0.25 µs per `S` stage and 0.31 µs per `L` one, so 2.8 µs on the worst train.
  They stay because `Engine::buildGeometry` is shared with the `loop()`-side plan warm, which needs
  both arrays whole and pays nothing for them there; making them lazy means splitting that function
  in two, and the plan tag's guarantee that a warmed plan describes the geometry the arm computes
  rests on there being exactly one copy of it.
- **Pre-arm the whole train and fire on a bare `NLDAC` pulse** (§3). This makes the arm's cost
  irrelevant rather than smaller, and is the only path to a single-digit *total* trigger latency —
  and, given the 9.0 µs floor, the only path to an arm inside one latch interval.
- **`CAL TRIGCOMP`** is still 0 and unmeasured — the one term of the delivered latency that
  software cannot see. [bench-wiring.md](bench-wiring.md) configuration B measures it.
