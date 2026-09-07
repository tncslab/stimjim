# 015 — the result display, the buttons, the clock, and a headless box

This phase gave `stimjimAWG` four things an operator standing at the bench needs and a fifth the
row path needed. The firmware now boots and runs with no OLED panel and no SD card attached, and
says which of them it found. The panel carries a flat list of pages walked by one button and by a
new `PAGE` command: a status page, up to four result pages showing the last completed train's
per-point voltage, current and load resistance per channel, and a system page. The other two
buttons fire the two trigger inputs' routes, as `stimjimPulser` wired them, with bounce suppressed
by arming rather than by a lockout, so one press is one train whatever the contact does. A new
`Clock` module reads and sets the RTC on both MCU families and reports where its epoch came from,
because the epoch is a compile time until a host sets it; `CLK?` pairs the wall clock with the
microsecond timebase, and the log file carries the same pair as `# clock:` anchor lines rather
than changing its CSV columns. Finally the per-row number formatting moved from
`snprintf("%.2f")` to fixed point, which takes newlib's `_dtoa_r` out of the path that bounds the
sustainable row rate, and `BENCHFMT` was added to measure what is left. Firmware version is now
0.8.0; the EEPROM image stays at v6 because nothing new needs persisting.

Everything below was verified on a Teensy 3.5 at 120 MHz with a panel, a card and a VBAT battery
fitted, as well as by compiling all four target configurations and running five host suites. The
one thing that could not be tested from here is the buttons themselves, because pressing them
needs a hand on the bench.


## 1. What was built, and why each choice

### 1.1 The display is what made a missing panel expensive

`Adafruit_SSD1306::begin()` (2.5.17, `Adafruit_SSD1306.cpp:496-638`) returns `false` only when its
512-byte framebuffer `malloc` fails. It never probes the bus, and every I2C write it makes is
unchecked. So the old `displayOk` was true whether or not a panel was connected, and `UiMenu::tick()`
then pushed a frame at up to 10 Hz for ever.

With the panel unplugged there are no pull-ups on SDA/SCL — the Teensy 3.5 has none on board and
the core deliberately does not enable the internal ones in I2C mode (`WireKinetis.cpp:70-76`). The
Kinetis `Wire` driver is bounded rather than hanging: `wait_idle()` gives up after 16 ms and
`endTransmission()` after a further 4 ms per phase (`WireKinetis.cpp:513-600`). One
`display.display()` is 1 + 16 chunked transactions, so a frame could cost about 340 ms of blocked
`loop()` every 100 ms. Waveform timing was never at risk — the player ISRs at priority 64 preempt
`loop()` unconditionally — but `Measure::poll()` (the 128-entry MDATA ring), `SdLog::poll()` and
`Engine::prepareArms()` were all starved.

The fix probes the panel and gates only the bus traffic:

- `UiMenu::begin()` calls `Wire.begin()`, then one raw `beginTransmission`/`endTransmission` to
  `SJ_OLED_ADDR`. A non-zero return means no panel. Worst case one 20 ms timeout, once, at boot.
- `display.begin()` runs either way, because it is what allocates the framebuffer and there is no
  other way to get one. With no panel its ~25 unchecked transactions cost up to half a second,
  once, at boot.
- `paint()` composes into the framebuffer exactly as before and skips the final
  `display.display()` when `panelPresent` is false. Everything else in the render path is RAM work.
- `SCREEN` therefore works headless, which is what remote testing needs. It re-probes the bus
  first, so a panel plugged in after boot is picked up by sending `SCREEN` once. There is no
  automatic retry: a periodic probe would cost a 20 ms `loop()` stall for ever on a board meant to
  run without a panel.
- Boot prints `# display: SSD1306 at 0x3C` or `# display: no panel — rendering to the framebuffer
  only (SCREEN still works; it re-probes)`.

The card path needed no mechanism change: `SD.begin(BUILTIN_SDCARD)` is bounded at 1 s by the
Teensy SDIO driver (`SdioTeensy.cpp:36`), `writeRow()`/`poll()` are no-ops with no open file, and
`mount()` is retried only from explicit commands. What changed there is a comment that had become
wrong (see 1.4).

### 1.2 The buttons do what the Pulser's buttons did

`stimjimPulser` wired Btn0 (pin 17) to `sayHello()`, Btn1 (pin 39) to the IN0 edge handler and
Btn2 (pin 16) to the IN1 one, with `// TODO: protection against rolling buttons` above them
(`stimjimPulser.ino:888-896`). `stimjimAWG` had made all three menu keys. The new mapping restores
the intent: Btn0 is the page key, Btn1 and Btn2 fire the routes. `Config.h` names changed with the
roles (`SJ_BTN_PAGE`, `SJ_BTN_TRIG0`, `SJ_BTN_TRIG1`), as did `UiInput::Event`
(`EV_PAGE`, `EV_TRIG0`, `EV_TRIG1`). `UiMenu::browseSlot()` had no callers and went with the
browse cursor.

**Bounce.** The old scheme was a 25 ms lockout from the last accepted edge. It suppressed bounce
on contact make but not on break: a switch held for 300 ms and released with a bouncy break
generated a second accepted rising edge, which for a trigger button means a second train. It is
now an explicit arm/re-arm:

- the ISR fires only while `armed[i]` is set; it pushes one event, clears the flag and returns —
  a load, a store and a ring push;
- `UiInput::poll()`, called from `loop()`, samples the three pins. A pin reading high records the
  time; a button is re-armed once its pin has read low continuously for `SJ_BTN_DEBOUNCE_MS`.

One press yields exactly one event whatever the contact does, and a `loop()` stall only delays
re-arming, which is the safe direction.

**Dispatch context.** `Triggers::edge()` was factored so the route dispatch is callable from
`loop()`: `Triggers::fireRoute(uint8_t input, uint64_t at)`, where `at` is the cycle count the
start was requested at and 0 means "now". The edge ISRs still pass their own entry timestamp; the
button path calls it from `UiMenu::tick()` under `FastIO::busLock()`, exactly as `handleStart()`
does for `T`/`U`, with `at = 0`.

This is a deliberate difference from the Pulser, which armed from the button ISR. A press is a
human action, so a `loop()`-scale dispatch latency — microseconds normally, tens of milliseconds
while the card flushes — does not matter, and it buys two things: no second ISR-context arming
path to reason about, and no need to raise `IRQ_PORTA`/`IRQ_PORTB` to `SJ_TRIG_PRIO` so that a
trigger edge cannot preempt a button's arm. Delivered waveform timing is unchanged either way,
because `t0` is anchored at the start request. If ISR-context arming is ever wanted it is a
three-line change plus the two NVIC priorities.

A button whose input has no train routed (`TRIG` mode 0 or 3) prints
`WARN button: input <t> has no train routed — set TRIG<t>` from `loop()`. A press that hits a busy
engine is refused by `Engine::startTrain` and counted by `Triggers::poll()`'s existing reject
WARN, the same as an electrical edge.

### 1.3 The pages

One flat list, advanced by the page button and by `PAGE`, wrapping at the end:

| Page | Content |
|---|---|
| 0 `STATUS` | a running train's progress (the old RUN view), or when both engines are idle: what each trigger button would fire, and the log state |
| 1..N `RESULT` | the last completed train, one page per measurement point, N capped at `SJ_UI_RESULT_PAGES` = 4 |
| N+1 `SYSTEM` | board and firmware, card and panel presence, uptime, the clock and its source |

The BROWSE view and its cursor are gone. Btn1/Btn2 no longer scroll, and the useful thing to show
when idle is what the buttons will actually fire — the routed slots — so `STATUS` subsumes what
BROWSE was for. A completion that measured something jumps the page to 1; one that measured
nothing leaves the page alone.

`PAGE` (advance), `PAGE,<n>` (select) and `PAGE?` (query) all reply `PAGE,<index>,<count>,<name>`.
Paired with `SCREEN` this makes every page reviewable from a host with no panel attached.

**The RESULT layout.** Four rows of 21 characters, channel 0 left, channel 1 right:

```
#7 T n=500 s0 1/2
V    100.2   -99.8 mV
I     99.5  -100.0 uA
R    1.01k   1.00k
```

The title bar (inverted, as row 0 always is) carries the run number `#n` — the one the
`Train #n complete` line prints — then the engine *and the slot* as `T04`, which reads the way
the STATUS header's engine tags do. Without the slot the page said what was measured but not what
was played. Then the sample count, the point's label — stage index for `S`/`L`, degrees for `W` — and the page's position
within the result pages. Each data row is a three-character tag, then per channel a
seven-character value and a one-character marker, then the unit: 3 + 8 + 8 + 2 = 21. The marker
column comes out of the gap between the channels, not out of the numbers.

The numbers come from the same accumulators `MSUM` prints. `Commands::poll()` already drains the
completion in `loop()` context; it now calls `UiMenu::noteResult()` immediately *before*
`Measure::printSummary`, which is what clears `summaryPending` and releases the finished train's
plan buffer. `Measure::resultSnapshot()` reads that buffer without consuming it and converts once
into integer microvolts and nanoamps with the standard error of each mean, so the panel itself is
float-free. The `T-1`/`U-1` stop path fills the snapshot the same way. The set is 10 points x 2
channels x 24 bytes, about 500 bytes.

### 1.3.1 Resistance: unit and rounding

`R = V/I`, computed in integers: with V in microvolts and I in nanoamps,
`R[milliohm] = V[uV] * 1e6 / I[nA]`, which needs `int64_t` (it peaks near 2e10 at the smallest
current the module will divide by).

Unit and decimals are not fixed. They follow the precision the measurement supports:

1. Standard error of each mean, `sd / sqrt(n)`, from the accumulator `MSUM` already carries,
   floored at the converter's own quantisation deviation (2.44 mV / sqrt(12) = 704 uV,
   0.85 uA / sqrt(12) = 245 nA) — a run of identical codes gives sd = 0, which is not a claim the
   hardware supports.
2. Relative uncertainty, floored: `rV = max(seV/|V|, 1 %)`, `rI = max(seI/|I|, 1 %)`. The 1 %
   floor stands in for the ADC path's gain accuracy, which this project has not characterised;
   without it a 500-repetition train would claim five significant digits it does not have. It is
   `SJ_UI_REL_FLOOR_PPM` in `UiFmt.h` so a measured figure can replace it.
3. `rR = sqrt(rV^2 + rI^2)`, so at the floor `rR = 1.4 %`.
4. The unit puts the mantissa in [1, 1000): ohms below 999.5, kilohms below 999.5 k, megohms above.
5. Decimals: round `rR * |R|` to one significant figure, give `R` the same decimal place, then cap
   the whole thing at three significant figures.

At the 1.4 % floor that yields `471`, `1.01k`, `47.1k`, `1.02M` — three significant figures, the
honest ceiling for a 2.44 mV / 0.85 uA converter. A short or noisy train loses digits by itself:
10 % on the current gives `1.0k` where the floor gave `1.01k`. All four worked examples and the
coarsening are host-tested.

Two cases print a word instead of a number:

- `open` when the current is not distinguishable from zero (`|I| < 3*seI`, `seI` floored as above);
- `--` when the line was not measured, or when neither V nor I differs from zero (an undriven or
  grounded channel).

A voltage that does not differ from zero while a real current flows is a *short*, not a missing
reading, and renders as the small number it is.

The old firmware's rule — kilohms above 100 k, ohms below, integers throughout
(`stimjimPulser.ino:494-501`) — is superseded: it printed six-digit ohm values in a six-character
field and claimed integer-ohm precision on a +-2.44 mV reading.

### 1.3.2 Limit markers

Requested after the plan was approved, and the one addition to it. A `*` after a value means the
number may be the hardware talking rather than the load:

- **9 V or more** on the voltage row. The output stage runs off a +-15 V rail but its driver
  saturates below that, so a reading this large is more likely the driver IC's ceiling than the
  requested amplitude. **This threshold is a working figure, not a measured one** — the saturation
  point has not been characterised on this board. It is `SJ_UI_VLIMIT_UV` in `UiFmt.h`.
- **3 mA or more** on the current row. The current pump is designed to 3.33 mA and amplitudes
  above 3000 uA are known to convert incorrectly on the DAC; `TrainStore`'s parse-time warning
  already uses that number (`WARN_LIMIT_UA`), so the display and the parser agree.

The marker carries through to the resistance row: a resistance derived from a suspect reading is
suspect for the same reason. It keys off the extremes of the train rather than the mean (§1.3.3). It is the same kind of feedback `open` gives, in the column the
layout reserved for it.

The Pulser's *compliance* markers — inverted video on a voltage- or current-limited field — are
still not built. They compared the reading against the requested amplitude, which the AWG's `MSUM`
path does not carry, and the flag deserves its own decision about what "limited" means for a ramp
or a sine.

### 1.3.3 The extremes, and what the marker keys off

A mean and a spread describe the bulk of a train and hide a single excursion almost completely:
one repetition in five hundred that reached the output driver's ceiling moves the mean by a
five-hundredth of the distance it travelled itself. So `Accum` gained `int16_t mn, mx`, updated by
two comparisons per reading in the player ISR @EM@ against a ~3 µs ADC read, free @EM@ and eight
bytes per accumulator, 1.6 KB across the four plans.

They cannot start at 0 the way the sums do, because 0 is an ordinary reading and a channel whose
codes are all positive would keep a minimum of 0 for ever. `planResetResults` seeds them inverted
(`INT16_MAX` / `INT16_MIN`), which is what lets the ISR be two unconditional comparisons with no
first-sample branch; `accumRange` gates on `n`, so the sentinels are never read as data. That
seeding runs in loop() context, and in the arm only on the fallback path that compiles in place.

They surface two ways:

- **`MRANGE`**, one record per `MSUM` line and immediately after it, carrying the extremes in the
  same units and the same field positions the means occupy. A separate record rather than four
  more `MSUM` fields, so a host parsing `MSUM` by field count is unaffected.
- **the panel's limit markers**, which now key off the extremes rather than the mean. A train
  whose average sits below 9 V but which touched it once is marked, which is the case the marker
  exists for and the one a mean is worst at reporting.

On silicon, a five-repetition biphasic train:

```
MSUM,4,5,0,4489.53,2.04,1102.93,0.38,8053.22,10.15,989.14,0.76
MRANGE,4,5,0,4486.60,4491.48,1102.25,1103.10,8042.48,8069.32,988.64,990.34
```

@EM@ and the second channel is the point: mean 8053 mV, but one of the five repetitions reached
8069.

### 1.4 The clock

**Three clocks, three jobs.** The cycle counter stays the only fine clock: `FastIO::cycles64()`
extends the DWT counter to 64 bits (8.33 ns per tick at 120 MHz, an exact integer count per
microsecond) and is the source of every waveform instant, every measurement deadline and the log's
`timestamp_us`. Nothing in this phase touched it.

**On overflow the log column is safe, but it has a keep-alive requirement.** 2^64 cycles is about
4900 years. What can go wrong is the software extension: it detects a 32-bit wrap by comparing
against the previous reading, so **something must call `cycles64()` at least once per 2^32 cycles
= 35.8 s** or a wrap is missed and the timestamp jumps back by that much. `loop()` guarantees it
through `Engine::poll()`, and the long-running card and serial loops (`SdLog::list`, `SdLog::get`,
`Measure::poll`) call it explicitly for exactly this reason. That guarantee used to be a code
comment only; it is now in `docs/timing.md` §6 and in the protocol reference next to the column
list, because it is the one way the column can go wrong.

**The RTC is a coarse label, never an authority.** What the firmware had to be careful about is
the *epoch*, and the two MCU families differ:

*Teensy 3.5/3.6 (Kinetis K64/K66).* Seconds in `RTC_TSR`, a 32768 Hz prescaler in `RTC_TPR` (about
30.5 us of readable resolution), backed by VBAT. The core (`mk20dx128.c:1128-1155`) sets the RTC
from `__rtc_localtime` when `RTC_SR_TIF` says the time is invalid — a power-up with no battery —
and writes a "known stale" flag `0x5A94C3A5` into the VBAT register file at `0x4003E01C`; on the
reset that follows an upload it sets it again from a *fresh* compile time and clears the flag.
`__rtc_localtime` is a linker `--defsym` the IDE fills from `{extra.time.local}`
(`boards.txt:1010`), that is, **the build host's local time at compile, not UTC**. So with no
battery every power-up starts at the binary's compile time in that zone; with a battery the clock
is set once, at the first upload after the battery goes in. The crystal is uncompensated, so tens
of ppm — seconds per day.

*Teensy 4.x (i.MX RT1062).* The core does **not** use the compile time: if the SRTC is not running
it starts it at 1546300800 = 2019-01-01T00:00:00Z (`startup.c:178-182`). Same 32768 Hz, in
`SNVS_HPRTCMR`/`SNVS_HPRTCLR` with 15 fractional bits.

The new `Clock` module therefore:

- **reads consistently.** `rtc_get()` returns seconds only, and reading the two registers
  separately can straddle a second. The Kinetis path reads `TPR`, `TSR`, `TPR` and retries when
  the prescaler rolled; the i.MX path is the core's own paired loop, extended to keep the 15
  fractional bits it discards.
- **sets with sub-second alignment.** `rtc_set()` zeroes `TPR`, throwing the fraction away.
  `Clock::set()` writes both, through the same enable/disable sequence.
- **classifies the source.** `SRC_BUILD` when the Kinetis stale flag is set, or when the reading
  is within a minute of `__rtc_localtime` (a fresh upload); `SRC_BATT` otherwise. On the i.MX a
  reading inside the first day after 2019-01-01Z is `SRC_BUILD`, anything later `SRC_BATT`.
  `set()` always moves to `SRC_HOST`. The second half of the Kinetis test is a heuristic and is
  documented as one.
- **states the source everywhere.** A `SRC_BUILD` reading is the build host's *local* time; a host
  that sends `CLK` naturally sends UTC. The firmware holds UTC once a host has set it, does not
  pretend to convert a build-sourced value, and never presents any of it as accurate on its own.
  Nothing in the firmware needs a zone.

**`CLK`.** `CLK?` replies `CLK,<unix>,<ms>,<us_since_boot>,<src>` plus a `#` line with the ISO 8601
rendering and a sentence saying what that source means. `CLK,<unix>[,<ms>]` sets and replies the
same. The reply reads the RTC and `cycles64()` back to back, so a host computes the offset between
its own clock and the board's monotonic timebase and can bound its uncertainty by timing the round
trip. That pair — not the RTC — is what anchors a log to a computer log. USB CDC round trips on a
Teensy are typically 0.1–1 ms, so wall-clock anchoring is good to about a millisecond and
everything finer comes from the microsecond column. A host that never sets the clock still gets a
usable anchor from `CLK?` alone.

**In the log.** The CSV columns did **not** change — host tools depend on them, and rendering a
wall clock per row would add cost to the very path §1.6 is about making cheaper. The file carries
anchor lines instead:

```
# clock: 2026-09-07T14:32:05.123Z src=host us=41234567
```

written at file open (in `writeHeader()`), before each train's `# train:` block (in
`noteTrain()`), and from `SdLog::poll()` at most once per 60 s while rows are actually being
written. Any row's `us` maps to a wall clock through the nearest anchor, and the drift between two
anchors is visible in the file rather than hidden inside it.

`FsDateTime::setCallback()` is now fed from `Clock::read()`, so log files carry real modification
dates on the card — but only when the source is not `SRC_BUILD`, because a wrong date is worse
than none: a host sorting by date would believe it. `LOG1` keeps `LOGnnnn.CSV` numbering; the
comment in `openLog()` that said the index "is the only thing that identifies a run on a board
without a clock" was rewritten to say why the index still is what identifies a run.

### 1.5 Why the accumulators stayed accumulators

Storing every reading and computing the spread by definition was considered and rejected, on two
grounds that were measured rather than assumed.

**Accuracy is not a reason to change.** The usual objection to `sumsq - sum*sum/n` assumes the sums
were accumulated in floating point; here they are exact `int64` and the only rounding is the final
subtraction. Compared against a two-pass long-double reference over n = 5..1e6, means 0..8000 codes
and spreads 0.29..50 codes, the **worst relative error is 1.6e-8** @EM@ four orders below what a
two-decimal field can show.

**Memory rules it out anyway.** The accumulators cost 4 896 B for all four plans *regardless of
train length*. Storing raw readings would cost n x 40 lines x 2 B per plan:

| n | per plan | all four | |
|---|---|---|---|
| 50 | 4 kB | 16 kB | fits |
| 500 | 40 kB | 160 kB | only just, against 55 kB already used |
| 5 000 | 400 kB | 1.6 MB | does not fit 256 kB |

500 repetitions is five seconds at a 10 ms period, so this would put an arbitrary cap on train
length to buy an accuracy improvement of 1e-8.

**And the option already exists in the right place.** `MEAS ...,<report>` with +1 streams every
individual reading as `MDATA` and +2 writes every one to the card, so a host that wants medians,
distributions or per-definition statistics already has the raw data @EM@ on a machine that has the
memory for it. What the accumulators owed was the one thing a summary genuinely could not give,
which is the extremes, and that is what §1.3.3 adds.

### 1.6 The row formatting is now fixed point

Per row the old path ran four `snprintf("%.2f")` conversions plus the row `snprintf`. The Teensy
3.5 links full newlib (`teensy35.build.flags.libs` carries no `nano.specs`), so `%f` goes through
`_dtoa_r`, which does double software arithmetic and allocates `_Bigint` scratch.

None of that can affect a waveform: it all runs in `Measure::poll()`, in `loop()` context, the
player ISRs preempt it unconditionally, the card is on native SDIO and no write path takes the bus
lock. What it bounds is the row rate, and through that the headroom of the 128-entry MDATA ring.

Both unit scales are exact hundredths of their unit — 2.44 mV and 0.85 uA per code — so the whole
conversion is exact in integers. `Stimjim.adcOffset10[]` is folded into a Q16 integer offset once
per calibration (`Measure::noteOffsets()`, called from `begin()` and from `B` and `C`), and the
fields print as `%ld.%02lu`. The `MSUM` mean takes the same path straight from the integer sums,
which is exact where the double quotient was not. Only the spread still multiplies in floating
point, because it comes out of a `sqrt` — but it no longer prints through `_dtoa_r` either.

**How byte-identical the output is, measured on the host.** Over the whole converter range and
every calibration offset the library can produce (a multiple of 0.01, from a 100-reading average):

- the **voltage column never differs**;
- the **current column** differs when the offset's hundredths value is congruent to 10 modulo 20 —
  one offset in twenty. On such a board *every* reading of that line sits exactly on a `.005`
  boundary, and 37–70 % of rows differ by one in the last digit. Where they differ the new digit
  is the correctly rounded one: `%.2f` rounds the nearest *double* of a product involving the
  inexact 0.85, the integer path rounds the exact value half away from zero. 0.01 uA is 1.2 % of
  one converter code.

That is documented in the protocol reference under `LOG`, because it is a change a host comparing
old and new log files could otherwise be puzzled by.

`BENCHFMT[,n]` times the formatting and `BENCHSD[,rows]` times the card write that follows it, so
between them they account for the whole write path. Measured, n = 2000, a 46-byte row:

| Stage | min / avg / max | Rows per second |
|---|---|---|
| `BENCHFMT`, 0.7.0: `snprintf("%.2f")` per field | 285.8 / 336.7 / 350.1 us | 2 970 |
| `BENCHFMT`, fixed point but still `snprintf` | 160.9 / 171.1 / 180.2 us | 5 850 |
| `BENCHFMT`, fixed point + hand-rolled decimals | **10.0 / 11.7 / 13.9 us** | 85 000 |
| `BENCHSD`, the card after it | 9.9 / 56.4 / 4 140 us | 17 254 |

**The measurement corrected the plan twice.** The plan's estimate for the *old* path was right
@EM@ "tens of microseconds per conversion, 100-300 us per row, a ceiling of a few thousand rows per
second". Its estimate for the fixed-point path was wrong by an order of magnitude: it expected "a
few microseconds per field" and got 171 us per row, because what remained after `_dtoa_r` was
newlib's `vfprintf` machinery itself, entered five times per row at roughly 34 us a call. Removing
the doubles saved 166 us (about 41 us per field); plain integer `snprintf` cost the rest.

So the row path went through a second step the plan did not contain: three integer appenders
(`putU32`, `putU64`, `putCenti`) replace all five `snprintf` calls, and an `MDATA` line and a log
row are now built from one shared tail rather than formatted twice. That took the row from 171 us
to 11.7 us. The output bytes are unchanged, and `tests/host/test_measure.cpp` proves it: each
appender is compared against the `printf` conversion it replaced, exhaustively over the ranges it
can see (every value below 5 000 for `putU32`, a contiguous +-20 000 block for `putCenti`, plus the
decade boundaries and the 32- and 64-bit edges).

**And it disposed of the plan's step 3.** A binary log format was offered as the fallback "if step
2 is not enough". It is not needed: a row now costs 68 us end to end, of which the card is 56, and
the card is where the cost belongs. The plan's own recommendation against a binary format stands,
now with a measurement behind it.

**Floating point: audited, and there is nothing to gain.** The Cortex-M4F has a single-precision
FPU, so `float` is hardware and `double` is a software library; the obvious question is whether the
log path would go faster in `float`. A disassembly audit of the linked image @EM@ which functions
call `__aeabi_d*`, `_dtoa_r` or `sqrt` @EM@ answers no, for a better reason than "not much":

- **the row path, the player ISRs, the arm and `Measure::fire` contain no floating point at all.**
  The waveform and logging paths are integer end to end, so there is nothing to convert.
- What is left @EM@ `accumStats`, `printSummary`, `resultSnapshot`, `noteOffsets`,
  `SampleGen::sineDerive`, the human `printf`s @EM@ runs once per train, per command or per slot
  definition. Never per row, per sample or per latch.
- `accumStats` **must** stay double, and it is exactly the "risk of losing important digits" case:
  its variance is `sumsq - sum*sum/n`, where at 500 repetitions `sum*sum` is around 1.7e13 and a
  24-bit float mantissa carries an absolute error near 1e6 on a difference that can itself be 1e4.

The lesson worth keeping is the other one: **on this libc `snprintf` costs tens of microseconds a
call whatever the conversion**, so any per-event formatting is worth doing by hand.

## 2. Files

New: `stimjimAWG/Clock.h`, `stimjimAWG/Clock.cpp`, `stimjimAWG/UiFmt.h`, `stimjimAWG/UiFmt.cpp`,
`tests/host/test_uifmt.cpp`, this file.

Changed: `Config.h` (version 0.8.0, button names and roles, `SJ_UI_RESULT_PAGES`), `Commands.cpp`
(`CLK`, `PAGE`, `BENCHFMT`, `noteResult` in both completion paths, `noteOffsets` after `B`/`C`,
help), `Measure.h`/`Measure.cpp` (`ResultSet`, `resultSnapshot`, the `mn`/`mx` extremes with
`accumRange` and the `MRANGE` record, fixed-point fields, the decimal appenders,
`benchFormatRow`, `noteOffsets`), `Protocol.cpp` (the boot `# clock:` line), `SdLog.h`/`SdLog.cpp`
(anchor lines, the FAT date callback, `cardPresent`/`name`/`bytes`), `Triggers.h`/`Triggers.cpp`
(`fireRoute`), `UiInput.h`/`UiInput.cpp` (roles and the arm/re-arm debounce), `UiMenu.h`/
`UiMenu.cpp` (rewritten around the page list), `stimjimAWG.ino` (`Clock::begin`, `UiInput::poll`,
header comment), `README.md`, `docs/serial-protocol.md`, `docs/timing.md`,
`docs/hardware-variants.md`, `docs/hardware-notes.md`, `tests/device/smoke.py`,
`tests/device/README.md`.

No EEPROM change: nothing new needs persisting, so the image version stays at v6.


## 3. Verification

**On a Teensy 3.5 at 120 MHz, register backends, panel + card + VBAT battery fitted.**

- `tests/device/smoke.py COM4 --screens tmp/screens`: **all checks pass**, including the new page
  walk, `CLK` before and after a host set, and a RESULT page checked against the `MSUM` line it was
  built from (panel `4494` mV against `MSUM` `4493.58`).
- Two bugs the run found, both in the test harness rather than the firmware, both fixed:
  - `sjcon.cmd1()` could not cope with a setter answering "accepted, with a caveat" — one `WARN`
    line and then the canonical echo. Phase 14 added exactly such a warning for a short latch gap
    and was never run on silicon, so this had been broken since then. `cmd1` now sets the warning
    aside like a comment, unless the `WARN` is the only line, which several checks assert on.
  - the page checks assumed a fresh boot. The suite has to be re-runnable, so it now selects page 0
    and the last page rather than asserting what the previous run left behind.
- One cosmetic firmware fix the render showed: `SYSTEM Teensy3.5 0.8.0` clipped its last digit at
  21 columns. The board name moved to row 1, next to the code placement and the uptime.
- **The limit markers, on hardware.** A slot driving 12 000 mV into the bench load measured
  `MSUM,9,5,0,9386.39,...` and the panel rendered

  ```
  #1 T n=5 s0 1/1
  V     9386*     -- mV
  I     2645      -- uA
  R    3.55k*     --
  ```

  — the voltage past its 9 V threshold marked, the current at 2 645 uA below its 3 mA threshold
  not marked, the marker carried through to the resistance derived from the flagged reading, and
  the undriven channel `--` on all three rows.
- **The log anchors, on the card.** A file with two trains gave one `# clock:` at open and one
  before each `# train:` block. The arithmetic ties out: the anchor `us=78020018` at
  `10:52:34.236Z` and the first data row at `78022020` us, 2.0 ms later.
- **`BENCHFMT`**: 160.9 / 171.1 / 180.2 us per row, against 285.8 / 336.7 / 350.1 us for the
  `%.2f` path measured beside it. See §1.6 and `docs/timing.md` §6.
- **The clock source heuristic behaves as designed on a battery-backed board.** After an upload it
  reports `batt`, not `build` — correct, and it is what the documentation predicts: the core only
  re-sets the RTC from `__rtc_localtime` when its VBAT "known-stale" flag is still set, and a
  battery-backed board cleared that flag at the first upload after the cell went in. So later
  uploads leave the clock alone and the reading stays far from the compile time. **The `build`
  branch has therefore not been exercised on silicon** — that needs a board with the cell removed.
- **`arduino-cli` was found**, bundled with Arduino IDE 2.3.10 at
  `resources/app/lib/backend/resources/arduino-cli.exe` (CLI 1.5.1), so all four configurations
  were really built and linked this time:

  | Configuration | Flash | RAM |
  |---|---|---|
  | Teensy 3.5, register | 180 588 B | 53 532 B |
  | Teensy 3.5, portable | 181 696 B | 53 788 B |
  | Teensy 4.1 | code 153 612 B | RAM1 variables 71 648 B |
  | Teensy 4.0, `SJ_EEPROM_SLOTS=6` | code 109 284 B | RAM1 variables 66 432 B |

- **A build-system bug in earlier phases, found while doing this.** The Teensy platform's
  `recipe.cpp.o.pattern` never references `{compiler.cpp.extra_flags}` (`platform.txt:47`), so the
  `--build-property compiler.cpp.extra_flags=-DSJ_FASTIO_REGISTER=0 -DSJ_TIMER_REGISTER=0` recorded
  in `tmp/a3-portable/build.options.json` was accepted and silently dropped. **Every "Teensy 3.5
  portable" build this project has reported was in fact a register build.** The property that works
  is `build.flags.defs`, which replaces the board's own value and so must repeat it; the Teensy 4.0
  build always used it and was therefore genuine. The four working command lines are now in
  `tests/device/README.md`, with a one-line way to check that a define arrived.
- **Host suites, all five clean at `-Wall -Wextra` and passing:** `test_cal`, `test_samplegen`,
  `test_trainstore`, `test_measure`, and the new `test_uifmt` — the ohm/kilohm/megohm selection and
  the decimal rule across the range, the 1 % floor from both sides, `open`, `--`, the short case,
  `int64` headroom at +-15 V and +-3.33 mA, the limit markers, the field widths, and the
  ISO 8601 / civil-date conversion including leap days and the `uint32` ceiling.
- **The fixed-point conversion was compared against `%.2f`** over 13.1 million conversions on the
  host — the whole code range crossed with 801 offsets — which is where the numbers in §1.6 come
  from.
**A third silicon round, after the extremes.** `MRANGE` brackets `MSUM` on a real train
(`MRANGE,4,5,0,4486.60,4491.48,...` against `MSUM,4,5,0,4489.53,2.04,...`), `smoke.py` checks that
relation for every point it measures, and the panel marks a 9385 mV train and leaves a 2558 mV one
clean. `BENCHFMT` is unchanged at 11.8 us, so the two ISR comparisons cost the row path nothing.
Host tests cover `accumRange`'s gate on `n`, the seeding `planBuild` must leave behind, and the
flag deciding from a range whose mean would not have tripped it.

**A second silicon round, after `BENCHSD` and the appenders.**

- `SDINFO,1` **never worked**, and the audit that found the `SCREEN,1` argument bug found it too.
  `parseFields` deliberately refuses the separating comma and every call site behind one skips it
  with a `p + 1`; two sites passed `args` unskipped, so `SDINFO,1` silently behaved exactly like a
  bare `SDINFO` and never scanned the used space. Both now go through a `skipComma()` helper, and
  the board answers `SD,1,30554112,128,...` where it used to answer `-1`.
- `smoke.py`'s `screen()` wrote its capture file only *after* its check passed, so a failing run
  left the previous run's file in place — which is what made a stale capture look like a
  firmware regression for several minutes. It now writes the capture first.
- `SCREEN` prints text by default and `SCREEN,1` adds the pixel block, so the suite asks for pixels
  only when it has somewhere to put them. That takes a capture from about 4 KB to about 150 bytes.
- The full suite passes again, and the RESULT capture reads `#2 T04 n=5 s0 1/2`.

- **Not tested: the buttons.** Nothing here can press one. The firmware path they share with the
  serial commands (`Triggers::fireRoute`, the page advance) is exercised, but the ISR, the
  arm/re-arm debounce and the no-route warning are not.

## 4. Next entry point

**The buttons.** They are the one thing this phase built that has never been exercised. On the
bench: Btn0 should walk the pages and wrap; Btn1 and Btn2 should start whatever `TRIG0`/`TRIG1`
route, exactly once per press, and print `WARN button: input <t> has no train routed` when nothing
is. Hold one for a second and release it — one train, not two. That last check is the whole point
of the arm/re-arm debounce and the only way to confirm it.

**The `build` clock branch.** On this board the source classifies as `batt`, correctly, and the
`build` path is therefore untested. Pulling the VBAT cell and power-cycling should give `build`; so
should a Teensy 4.x, which has no cell at all.

Still outstanding from earlier phases and untouched here:

- **Measure the prepared arm and lower `CAL STARTLAT`** — phase 14's next entry point.
  `SJ_START_LATENCY_US` is still 35, the board's stored budget is 60, and `bench_arm.py` has not
  been re-run on silicon.
- **Re-check anything that was ever A/B'd against a "portable" build**, because §3 shows those
  builds were register builds. The portable backend's `CAL` defaults in `Config.h` are marked
  RECALIBRATE and were never measured anyway, so nothing published is wrong — but the portable path
  has now compiled and linked for the first time, and has still never run.

Two thresholds in this phase are working figures rather than measurements, named so they can be
replaced without touching logic: `SJ_UI_REL_FLOOR_PPM` (1 %, standing in for the ADC path's gain
accuracy) and `SJ_UI_VLIMIT_UV` (9 V, standing in for the output driver's saturation point). Either
one measured on the bench is a one-line change.

**If the log row rate ever matters**, §1.6 says what to do and what not to: replace the five
`snprintf` calls with hand-rolled integer-to-decimal appends, keeping the bytes identical, rather
than adopting a binary format.
