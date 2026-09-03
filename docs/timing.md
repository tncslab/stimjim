# stimjimAWG latency and timing

This document answers, in one place, how long `stimjimAWG` takes to put signal on the output
after a trigger edge, why that number is what it is, how it compares with the original
`stimjimPulser` firmware, whether the DAC can be preloaded to fire faster, what the trigger
interrupt actually starts, whether waveform generation occupies the CPU, and what SD logging
costs and stores. The short answers: a trigger edge delivers the first latch a fixed **60 µs**
later on a Teensy 3.5 with the register backends (`CAL STARTLAT`, adjustable without a rebuild),
of which 16–42 µs is the arm itself and none is the DAC write; the original firmware had no fixed
figure at all, because it played the first pulse inside the trigger ISR after two USB
`Serial.print` calls; the AD5752 *can* hold a preloaded code and fire on a bare `NLDAC` pulse in
0.44 µs, so a pre-armed trigger path could reach a few microseconds, but that needs the arm moved
before the edge and is not built; the edge ISR only computes `t0` and programs a timer, and every
sample is emitted from the timer ISR; generation is interrupt-driven with a bounded spin of a few
microseconds per event, not a busy loop; and an SD write can never delay a waveform, because only
`loop()` touches the card and the player ISRs preempt it.

## 1. Trigger-to-output latency

A trigger start delivers its first DAC latch at

```
t0 = <edge timestamp> − CAL TRIGCOMP + CAL STARTLAT + <slot delay_us>
```

`STARTLAT` is **60 µs** by default on a Teensy 3.5 at 120 MHz with the register backends, and
120 µs on the portable (Arduino-SPI + `IntervalTimer`) build. It is runtime state:
`CAL,STARTLAT,<us>` changes it, `P` persists it, `CAL?` reports what a running board uses.

The latency is deterministic because the edge ISR timestamps the edge in its own first
instruction, and everything it then does is spent *inside* `STARTLAT` rather than added after it.
What has to fit in the window:

| Term | Cost (T3.5, register backends) |
|---|---|
| `Engine::startTrain` — copy-on-arm, fixed-point precomputation, measurement plan | 16.2 µs undriven, 19.2 µs two-channel `S`, 23.5 µs measured `S`, 28.3 µs ten-stage `S`, 22.9 µs `L`, 37.5 µs `W` (`BENCHARM`) |
| `CAL PRELOAD` — how early the player ISR wakes before the first latch | 4 µs |
| `CAL DACPROG2` — the dual-channel SPI write inside that window | 5 µs budgeted (2.75 µs measured) |
| `SJ_MIN_SCHEDULE_US` | 3 µs |

So `STARTLAT ≥ arm + 12 µs`, and the arm is the term that sizes it — **the DAC write is not the
bottleneck, the arm is**. A `TRIG` route in independent mode arms two engines inside one ISR and
pays the arm twice: two `S` or `L` trains fit in 60 µs, two sine trains need about 100 µs. When an
arm does not fit, the train still runs with a late first latch, and its completion names the
`STARTLAT` that would have covered it.

Jitter, not latency, is what the design buys: residual latch jitter is **42 ns** (`BENCHPIT` with
a preload), and the per-slot delay is exact to the scope's own sample interval (2000 µs set →
1999.3 µs measured, 20000 → 19996.0).

Two qualifications on the 60 µs:

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
  case is well under the 60 µs of `stimjimAWG` and the worst case is unbounded. Neither
  firmware's absolute edge-to-output latency has been measured on this bench; what is measured is
  the arm cost (`BENCHARM`) and the delay accuracy.

`stimjimAWG` differs by construction: no ISR ever prints (completions go through a ring and
`loop()` prints them), `t0` is anchored to the edge timestamp so the arm is subtracted rather than
added, the first latch always happens in the player ISR, and the trigger ISR sits at priority 80
*below* the players, so a trigger can never delay a waveform already playing. The trade is that
the constant is currently 60 µs — larger than the original firmware's best case, smaller than its
worst, and unlike either, repeatable to 42 ns.

## 3. Can values be preloaded into the DAC to fire faster?

Yes in hardware, and the firmware already relies on the mechanism for every latch. The AD5752
separates *program* (a 24-bit SPI write into the input register) from *execute* (an `NLDAC`
pulse), so `FastIO::dacProgram` and `FastIO::dacLatch` are two independent operations, and
`dacLatch` costs **0.44 µs** measured. A code written into the input register stays there
indefinitely until something overwrites it.

What prevents the trigger path from exploiting that today is not the DAC — it is that the arm runs
*inside* the trigger ISR. Preloading the code buys nothing while the 16–42 µs of
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
amplitude, so a pre-armed path would move the onset from ~60 µs to a few µs without sharpening the
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
