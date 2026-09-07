# Plan: headless boot, result display, buttons, and the clock

This plan covers four changes to `stimjimAWG` and one question of fact. The firmware must boot
and run with no OLED panel and no SD card attached; the OLED must show what a finished train
measured, including the load resistance, over several pages that a pushbutton and a new `PAGE`
serial command switch between; the other two pushbuttons must fire the two trigger routes with
contact bounce properly suppressed; and the firmware needs a wall clock so that SD log rows can
be anchored to a host computer's log. The findings that drive the design are: the OLED library's
`begin()` never checks whether a panel answers, so the current firmware would spend up to a third
of a second per frame in I2C timeouts with the panel unplugged; the SD path already fails safely;
the existing `us` log timestamp is a 64-bit count that cannot overflow in any practical service
life, but it depends on a keep-alive call the code already makes; the Teensy 3.5 RTC is set from
the *compile* time of the binary and holds the build host's *local* time, which makes it a coarse
label and never an authority; and the `%.2f` conversions in the log row path run in `loop()` where
no waveform can see them, so they bound the sustainable row rate and nothing else. Recommendation
on the row format: keep ASCII, replace the `%f` conversions with integer fixed-point, and measure
the result with a new `BENCHFMT` bench rather than estimating it.

Status: done, 2026-09-07 — implemented in phase 015 (docs/progress/015-ui-results-clock.md),
with one addition the plan did not carry: the limit markers of section 3.3's follow-up note, in the
narrower form of a `*` on a reading at the driver's 9 V ceiling or the current pump's 3 mA design
limit. The Pulser's compliance markers, which compare against the *requested* amplitude, are still
not built.


## 1. Booting with no panel and no card

### 1.1 The display is the real defect

`Adafruit_SSD1306::begin()` (version 2.5.17, `Adafruit_SSD1306.cpp:496-638`) returns `false` only
when its 512-byte framebuffer `malloc` fails. It never probes the bus. Every I2C write it makes is
unchecked. So `UiMenu::begin()`'s `displayOk` is true whether or not a panel is connected, and
`UiMenu::tick()` then calls `paint()` at up to 10 Hz forever.

With the panel unplugged there are no pull-ups on SDA/SCL — the Teensy 3.5 has none on board, and
the core deliberately does not enable the internal ones in I2C mode (`WireKinetis.cpp:70-76`). The
Kinetis `Wire` driver is bounded, not hanging: `wait_idle()` gives up after 16 ms and
`endTransmission()` after a further 4 ms per phase (`WireKinetis.cpp:513-600`). One
`display.display()` is 1 + 16 chunked transactions, so a frame costs up to ~340 ms of blocked
`loop()`, every 100 ms. That starves `Measure::poll()` (the 128-entry MDATA ring overflows),
`SdLog::poll()` and `Engine::prepareArms()`. Waveform timing itself is safe — the player ISRs at
priority 64 preempt `loop()` unconditionally — but measurement records and prepared arms are lost.

**Fix.** Probe the panel ourselves before trusting it, and gate only the bus traffic on the result:

- `UiMenu::begin()` does one raw `Wire.beginTransmission(SJ_OLED_ADDR); Wire.endTransmission()`
  and treats a non-zero return as "no panel". Worst case cost: one 20 ms timeout, once, at boot.
- If a panel answered, `display.begin()` runs as now.
- If none answered, `display.begin()` is **still** called, because it is what allocates the
  framebuffer and there is no other way to get one. Its ~25 unchecked transactions cost up to
  0.5 s once at boot. `panelPresent` is then false and no later frame is pushed.
- `paint()` composes into the framebuffer exactly as now and skips the final
  `display.display()` when `panelPresent` is false. Everything else in the render path is RAM
  work.
- `SCREEN` therefore keeps working headless, which is what the remote testing in
  `tests/device/smoke.py` needs. It re-probes the bus first (an explicit, human-initiated command,
  so one timeout is irrelevant), so a panel plugged in after boot is picked up by sending `SCREEN`
  once. There is no automatic retry: a periodic probe would cost a 20 ms `loop()` stall forever on
  a board that is meant to run without a panel.
- Boot prints `# display: SSD1306 at 0x3C` or `# display: no panel — rendering to the framebuffer
  only (SCREEN still works)`.

### 1.2 The card path is already safe, and says so badly

`SdLog::begin()` calls `SD.begin(BUILTIN_SDCARD)`; the Teensy SDIO driver bounds every wait at
1 s (`SdioTeensy.cpp:36`), returns false, and `begin()` prints `# SD: no card mounted`.
`writeRow()` and `poll()` are no-ops with no open file, and `mount()` is retried only from
explicit commands. Nothing to fix in the mechanism. Two things to change:

- Boot re-checks the mount result before claiming a size, which it already does.
- The comment in `SdLog::openLog()` that says the index "is the only thing that identifies a run
  on a board without a clock" becomes wrong once section 4 lands, and is rewritten.

A build with `SJ_USE_DISPLAY 0` or on a Teensy 4.0 (`SJ_USE_SD 0`) is unaffected; those paths
already compile out. Verification: build all four configurations, and run `smoke.py` on silicon
with the panel unplugged and the card removed.


## 2. The three pushbuttons

`stimjimPulser` wired Btn0 (pin 17) to `sayHello()`, Btn1 (pin 39) to the IN0 edge handler and
Btn2 (pin 16) to the IN1 one, with `// TODO: protection against rolling buttons` above them
(`stimjimPulser.ino:888-896`). `stimjimAWG` currently makes all three menu keys. The new mapping
restores the Pulser's intent:

| Pin | Pulser | New role |
|---|---|---|
| 17 | `sayHello()` | switch display page |
| 39 | start IN0's route | fire input 0's route |
| 16 | start IN1's route | fire input 1's route |

The `Config.h` names change with the roles: `SJ_BTN_OK` becomes `SJ_BTN_PAGE`, `SJ_BTN_PREV`
becomes `SJ_BTN_TRIG0`, `SJ_BTN_NEXT` becomes `SJ_BTN_TRIG1`, and `UiInput::Event` becomes
`EV_PAGE`, `EV_TRIG0`, `EV_TRIG1`. Nothing outside `UiInput` and `UiMenu` reads these;
`UiMenu::browseSlot()` has no callers at all and is removed with the browse cursor (section 3.1).

### 2.1 Bounce

The present debounce is a 25 ms lockout from the last accepted edge (`UiInput.cpp:35-40`). It
suppresses bounce on contact make but not on break: a switch held for 300 ms and released with a
bouncy break generates a second accepted rising edge, which for a trigger button means a second
train. Replace it with an explicit arm/re-arm:

- The ISR fires only when `armed[i]` is set. It pushes one event, clears `armed[i]`, and returns.
  Body stays a handful of instructions.
- `UiInput::poll()`, called from `loop()`, samples each pin. Whenever a pin reads high it records
  the time. A button is re-armed once its pin has read low continuously for
  `SJ_BTN_DEBOUNCE_MS` (25 ms).

One press yields exactly one event, whatever the contact does, and a `loop()` stall only delays
re-arming, which is the safe direction. `poll()` costs three `digitalReadFast` calls per pass.

### 2.2 What a trigger button does

`Triggers::edge()` is factored so the route dispatch is callable from `loop()`:
`Triggers::fireRoute(uint8_t input, uint64_t at)`. The button path calls it from `UiMenu::tick()`
under `FastIO::busLock()`, exactly as `handleStart()` does for `T`/`U`, with `at = 0` ("now").

This is a deliberate difference from the Pulser, which armed from the button ISR. A button press
is a human action, so a `loop()`-scale dispatch latency (normally microseconds, up to tens of
milliseconds while the card flushes) does not matter, and it buys two things: no second
ISR-context arming path to reason about, and no need to raise `IRQ_PORTA`/`IRQ_PORTB` to
`SJ_TRIG_PRIO` so that a trigger edge cannot preempt a button's arm. Delivered waveform timing is
unchanged either way, because `t0` is anchored at the start request. If ISR-context arming is
wanted later it is a three-line change plus the two NVIC priorities.

A button whose input has no train routed (`TRIG` mode 0 or 3) prints
`WARN button: input <t> has no train routed — set TRIG<t>` once per press, from `loop()`. A press
that hits a busy engine is refused by `Engine::startTrain` and counted by `Triggers::poll()`'s
existing reject WARN, the same as an electrical edge.


## 3. What the panel shows

### 3.1 Pages

One flat page list, advanced by the page button and by `PAGE`, wrapping at the end:

| Page | Content |
|---|---|
| 0 `STATUS` | a running train's progress (the present RUN view), or when both engines are idle: what each trigger button would fire, and the log state |
| 1..N `RESULT` | the last completed train, one page per measurement point, N capped at `SJ_UI_RESULT_PAGES` = 4 |
| N+1 `SYSTEM` | board and firmware, card and panel presence, the clock and its source, uptime |

The BROWSE view and its cursor go away. Btn1/Btn2 no longer scroll, and the useful thing to show
when idle is what the buttons will actually fire, which is the routed slots — so `STATUS`
subsumes what BROWSE was for.

A completion that measured something jumps the page to 1, which is what the Pulser did by
redrawing the result summary at train end. A completion with no measurement leaves the page alone.

### 3.2 The RESULT page

Four rows of 21 characters. Left column is channel 0, right is channel 1:

```
#7 T n=500 pt0 1/2      <- title bar, inverted
V    100.2   -99.8mV
I     99.5  -100.0uA
R    1.01k   1.00k
```

Title bar: train counter, engine, sample count, the point's label (stage index for `S`/`L`,
degrees for `W`) and the page position within the result pages. A field the train did not measure
renders as `--`.

The numbers come from the same accumulators `MSUM` prints. `Commands::poll()` already drains the
completion in `loop()` context and calls `Measure::printSummary`; a new
`Measure::resultSnapshot(eng, ResultSet&)` reads the same buffer without consuming it and is
called immediately before `printSummary` (which clears `summaryPending`). `T-1`/`U-1` fills the
snapshot the same way. The snapshot is 10 points x 2 channels x (mean, sd, n) — about 400 bytes.

### 3.3 Resistance: unit and rounding

`R = V/I`. Computed in integers, so the UI stays float-free as it is today: with V in microvolts
and I in nanoamps, `R[milliohm] = V[uV] * 1e6 / I[nA]`, which peaks near 1.5e13 and so needs
`int64_t`.

Unit and decimals are not fixed. They follow the precision the measurement actually supports:

1. Standard error of each mean: `se = sd / sqrt(n)`, from the accumulator `MSUM` already carries.
2. Relative uncertainty, floored: `rV = max(seV/|V|, 0.01)`, `rI = max(seI/|I|, 0.01)`. The 1 %
   floor stands in for the ADC path's gain accuracy, which this project has not characterised —
   without it a 500-repetition train claims five significant digits it does not have. It is a
   named constant so it can be replaced by a measured figure later.
3. `rR = sqrt(rV^2 + rI^2)`, so at the floor `rR = 1.4 %`.
4. Unit puts the mantissa in [1, 1000): ohms below 999.5, kilohms below 999.5 k, megohms above.
5. Decimals: round `rR * |R|` to one significant figure and give `R` the same decimal place, then
   cap the whole thing at three significant figures.

At the 1.4 % floor that yields `471` (ohms), `1.01k`, `47.1k`, `1.02M` — three significant
figures, which is the honest ceiling for a 2.44 mV / 0.85 uA converter. A short or noisy train
automatically loses digits, which is the point.

Two special cases, both printed instead of a number:

- `open` when the current is not distinguishable from zero: `|I| < 3*seI`, with `seI` floored at
  the quantisation deviation `0.85/sqrt(12) = 0.245 uA`.
- `--` when the channel was not measured, or when neither V nor I differs from zero (an
  undriven or grounded channel).

The old firmware's rule — kilohms above 100 k, ohms below, integers throughout
(`stimjimPulser.ino:494-501`) — is the ancestor of this and is superseded: it printed six-digit
ohm values in a six-character field and claimed integer-ohm precision on a +-2.44 mV reading.

The Pulser's compliance markers (inverted video on a voltage- or current-limited field) are **not**
part of this change. They compared the reading against the requested amplitude, which the AWG's
`MSUM` path does not carry, and the flag deserves its own decision about what "limited" means for
a ramp or a sine. Noted as a follow-up.

`fmtOhm()` and the uncertainty rule are pure integer functions with no Arduino dependency, placed
so `tests/host/test_uifmt.cpp` can exercise them (section 6).

### 3.4 `PAGE`

| Form | Effect |
|---|---|
| `PAGE` | advance one page — the same action as the page button |
| `PAGE,<n>` | select page `n` |
| `PAGE?` | `PAGE,<index>,<count>,<name>` |

All three reply with the `PAGE,...` status line. Paired with the existing `SCREEN` this makes every
page reviewable from a host with no panel attached, which is what the remote testing asks for.


## 4. The clock

### 4.1 Three clocks, three jobs

**The cycle counter is and stays the only fine clock.** `FastIO::cycles64()` extends the Cortex-M
DWT cycle counter to 64 bits (`FastIO.cpp:229-239`): 8.33 ns per tick at 120 MHz, an exact integer
number per microsecond, and it is what every waveform instant, every measurement deadline and the
log's `timestamp_us` column are derived from. Nothing in this plan changes that.

**On overflow, the log column is safe.** The 64-bit cycle count wraps after 2^64 / 120e6 s, about
4.9e3 years; expressed as microseconds, 2^64 us is about 5.8e5 years. Neither is a service-life
concern. The real constraint is different and already met: the software extension detects a 32-bit
wrap by comparing against the previous reading, so **something must call `cycles64()` at least once
per 2^32 cycles = 35.8 s**, or a wrap is missed and the timestamp jumps back by 35.8 s. `loop()`
guarantees it through `Engine::poll()`, and the long-running card and serial loops
(`SdLog::list`, `SdLog::get`, `Measure::poll`) call it explicitly for exactly this reason. This
guarantee is currently a code comment; the plan documents it in `docs/timing.md` section 6 and in
the protocol reference next to the column list, because it is the one way the column can go wrong.

**The RTC is a coarse label, never an authority.** See section 4.2.

**The anchor is what ties them together.** See section 4.3.

### 4.2 What the Teensy RTC actually does

Your recollection is right for the Teensy 3.5, and wrong for the Teensy 4.x. Both are worth
writing down because this repository builds for both.

*Teensy 3.5/3.6 (Kinetis K64/K66).* The RTC counts seconds in `RTC_TSR` with a 32768 Hz prescaler
in `RTC_TPR`, so about 30.5 us of readable resolution, backed by the VBAT pin. The core
(`mk20dx128.c:1109-1154`) does two things at reset:

- If `RTC_SR_TIF` is set — an invalid time, which is what a power-up with no battery gives — it
  sets the RTC from `__rtc_localtime` and writes a "known stale" flag `0x5A94C3A5` into the VBAT
  register file at `0x4003E01C`.
- If the reset came from the reset pin (which the Teensy Loader asserts right after an upload) and
  that stale flag is set, it sets the RTC from `__rtc_localtime` again and clears the flag.

`__rtc_localtime` is a linker `--defsym` filled in by the IDE or `arduino-cli` from
`{extra.time.local}` (`boards.txt:1010`), that is, **the build host's local time at compile, not
UTC**. So:

- With no battery, every power-up starts the clock at the binary's compile time, in the build
  host's local zone. It is right just after an upload and wrong by however long that binary has
  been in service.
- With a battery fitted, the clock is set once, at the first upload after the battery goes in, and
  then keeps running; later uploads leave it alone because the flag is clear.
- The 32.768 kHz crystal is uncompensated. Tens of ppm is normal, so seconds per day of drift.
  `rtc_compensate()` exists but calibrating it is out of scope here.

*Teensy 4.x (i.MX RT1062).* The core does **not** use the compile time. If the SRTC is not running
it starts it at 1546300800 = 2019-01-01T00:00:00Z (`startup.c:178-182`). Resolution is the same
32768 Hz, in `SNVS_HPRTCMR`/`SNVS_HPRTCLR`, 15 fractional bits.

Conclusion for the design: the RTC's *rate* is fine for labelling events to the second over hours;
its *epoch* cannot be trusted unless a host has set it. So the firmware reports where the time
came from and never presents it as accurate on its own.

### 4.3 The design

A new `Clock` module (`Clock.h`/`Clock.cpp`) with a small surface:

```c++
enum Source : uint8_t { SRC_BUILD, SRC_BATT, SRC_HOST };
void   begin();                                   // classify the source at boot
Source source();
bool   read(uint32_t& unixSec, uint16_t& subMs);  // consistent read, both MCU families
void   set(uint32_t unixSec, uint16_t subMs);     // -> SRC_HOST
void   anchor(uint32_t& unixSec, uint16_t& subMs, uint64_t& us);  // both clocks, back to back
void   isoString(char* out, size_t n, uint32_t unixSec, uint16_t subMs);   // ISO 8601, UTC
```

- **Consistent read.** `rtc_get()` returns seconds only, so `read()` does its own paired read:
  Kinetis reads `TPR`, `TSR`, `TPR` and retries when the prescaler rolled over between them;
  the i.MX loop in `rtc.c:35-48` is copied and extended to keep the 15 fractional bits.
- **Setting with sub-second alignment.** `rtc_set()` zeroes `TPR`, which throws away the
  fraction. `set()` writes both: `RTC_SR = 0; RTC_TPR = subMs * 32768 / 1000; RTC_TSR = unixSec;
  RTC_SR = RTC_SR_TCE` on Kinetis, and `(sec << 15) | frac` into `SNVS_LPSRTCLR`/`MR` on i.MX,
  through the enable/disable sequence `rtc_set()` already uses.
- **Source classification.** Read the VBAT flag at `0x4003E01C` on Kinetis: `0x5A94C3A5` means the
  core set the clock from a stale compile time, so `SRC_BUILD`. Cleared, with the RTC running,
  means a fresh upload or a battery-backed clock, and comparing `read()` against
  `__rtc_localtime` separates them: within a few seconds is a fresh upload (`SRC_BUILD`, honest
  about being local time), far away is `SRC_BATT`. It is a heuristic and is documented as one. On
  the i.MX there is no equivalent flag: a reading below 2019-01-02 is `SRC_BUILD` (the core's
  default), anything else `SRC_BATT`. `set()` always moves to `SRC_HOST`.
- **The zone trap.** A `SRC_BUILD` time is the build host's *local* time; a host that sends `CLK`
  naturally sends UTC. The firmware therefore states the source on every clock line and in the log
  header, holds UTC once a host has set it, and does not pretend to convert a build-sourced value.
  Nothing in the firmware needs a zone.

**`CLK` command.**

| Form | Effect |
|---|---|
| `CLK?` | `CLK,<unix>,<ms>,<us_since_boot>,<src>` plus a `#` line with the ISO 8601 rendering |
| `CLK,<unix>[,<ms>]` | set the RTC, then reply with the same status line |

`CLK?` reads the RTC and `cycles64()` back to back and reports both, so a host computes the offset
between its own clock and the board's monotonic timebase, and can bound its own uncertainty by
timing the round trip. That pair — not the RTC — is what anchors a log to a computer log. USB CDC
round trips on a Teensy are typically 0.1-1 ms, so wall-clock anchoring is good to about a
millisecond and everything finer comes from the microsecond column. A host that never sets the
clock still gets a usable anchor from `CLK?` alone.

**In the log.** The CSV columns do **not** change — host tools depend on them, and rendering a wall
clock per row would add cost to the path section 5 is about making cheaper. Instead the file
carries anchor lines:

```
# clock: 2026-09-07T14:32:05.123Z src=host us=41234567
```

written at file open (in `writeHeader()`), before each train's `# train:` block (in `noteTrain()`),
and from `SdLog::poll()` at most once per 60 s while rows are being written. Any row's `us` maps to
wall clock through the nearest anchor, and the drift between two anchors is visible in the file
rather than hidden.

Two smaller consequences:

- `FsDateTime::setCallback()` (`SdFat/src/common/FsDateTime.h:105`) gets a callback fed from
  `Clock::read()`, so log files carry real modification dates on the card — but only when the
  source is not `SRC_BUILD`, because a wrong date is worse than none.
- `LOG1` keeps `LOGnnnn.CSV` numbering by default. Date-based naming is left as an option to
  decide (section 8).

### 4.4 On the panel

The `SYSTEM` page shows the clock and its source, for example `26-09-07 14:32 host` — so "is this
box's clock set" is answerable without a serial port. The boot banner prints one `# clock:` line
with the same content.


## 5. The SD log: is ASCII float conversion too slow?

**It can never affect a waveform.** All formatting happens in `Measure::poll()`, in `loop()`
context. The player ISRs at priority 64 preempt `loop()` unconditionally, the card is on native
SDIO and not the DAC/ADC bus, and no write path takes the bus lock. `docs/timing.md` section 6
already establishes this and it stays true.

**What it does bound is the row rate**, and through that the MDATA ring. Per row the path runs four
`snprintf("%.2f")` conversions plus one row `snprintf` plus `File::println`. The Teensy 3.5 links
full newlib (`teensy35.build.flags.libs` has no `nano.specs`), so `%f` goes through `_dtoa_r`,
which does double software arithmetic and allocates `_Bigint` scratch. **Order-of-magnitude
expectation, not a measurement on this board: tens of microseconds per conversion, so 100-300 us
per row, a ceiling of a few thousand rows per second.** That is the same order as the ring's drain
requirement (128 records; 64 ms of headroom at 2000 rows/s), so it is worth knowing rather than
assuming.

**Step 1: measure it.** New bench `BENCHFMT,<reps>` times exactly the `poll()` formatting of a
synthetic record, with no card and no serial write in the loop, and reports microseconds per row
like the other `BENCH` verbs. The result goes into `docs/timing.md` section 6, which currently
admits it has never timed the write path.

**Step 2: drop the float conversions, keep ASCII.** The values are `(raw - offset) * scale` with a
constant scale, so `field()` can format from integers: fold `Stimjim.adcOffset10[]` into a Q16
integer offset once at calibration, keep the scale as a rational, and print `%ld.%02lu`. That
removes `_dtoa_r` and its allocation from the row path and typically costs a few microseconds per
field. **The output bytes stay identical**, so no host, parser or existing log file is affected —
only exact `.005` rounding boundaries can differ by one in the last digit, and that is noted in the
protocol reference. The same helper serves the `MDATA` stream and `MSUM`, so all three get faster.

**Step 3: binary, only if step 2 is not enough.** A binary row would be about 20 bytes against
about 55 ASCII, so it also cuts card bandwidth threefold. I recommend against it unless
measurement forces it: this project deliberately built a log that is self-describing and whose
configuration block pastes back into the firmware, and a binary format costs that plus a host-side
decoder. If it becomes necessary it belongs behind a `LOG` mode flag, with the header staying
ASCII.


## 6. Tests

*Host, no board.* New `tests/host/test_uifmt.cpp` over the pure functions: the ohm/kilohm/megohm
selection and decimal rule across the range, the 1 % floor, `open` and `--`, `int64` headroom at
+-15 V and +-3.33 mA, and the ISO 8601 rendering (leap years, epoch edges). Built the same way as
the other four (one `g++ -std=c++17` line, exit code = failure count).

*Device.* `tests/device/smoke.py` gains: `PAGE?`/`PAGE,<n>`/`PAGE` with a `SCREEN` capture per
page into the capture directory; a train run followed by a check that the RESULT page's numbers
match the `MSUM` line; `CLK?` before and after `CLK,<host unix>` with a monotonicity check on the
`us` field; and a headless pass — panel unplugged, card removed — asserting that boot completes,
`SCREEN` still returns 32 rows, and `LOG1` reports the missing card rather than hanging.

*Builds.* The existing four configurations (Teensy 3.5 register, 3.5 portable, 4.0, 4.1) must stay
clean with `--warnings more`.


## 7. Documents to update

- `README.md`: headless operation; the buttons and what each does; the pages and `PAGE`; the clock
  section (what the RTC is worth, `CLK`, how to anchor a log to a computer log); the log timestamp
  and why it cannot overflow.
- `docs/serial-protocol.md`: `PAGE` and `CLK` in section 4 and in the utility table; the anchor
  lines in the `LOG` file description; the microsecond column's keep-alive guarantee; the
  fixed-point rounding note.
- `docs/timing.md` section 6: the `BENCHFMT` figure, replacing the admission that the write path
  has never been timed; the clock hierarchy and the 35.8 s keep-alive rule.
- `docs/hardware-variants.md`: the Teensy 3.x versus 4.x RTC difference, and the VBAT battery.
- `docs/hardware-build.md` or `hardware-notes.md`: fitting the RTC battery.
- `Config.h` header comment, `stimjimAWG.ino` header comment (the "not implemented: button menu
  editor" line changes meaning), `SJ_FW_VERSION` to `0.8.0`.
- `docs/progress/015-ui-results-clock.md` plus the `docs/PROGRESS.md` index line at the end.

No EEPROM change: nothing new needs persisting, so the image version stays at v6.


## 8. Decisions I would like confirmed

1. **Button dispatch context.** `loop()` rather than the button ISR (section 2.2). Waveform timing
   is unaffected; only the press-to-output delay grows to `loop()` scale. Say so if you want the
   Pulser's ISR arming instead.
2. **Losing the slot browse.** Btn1/Btn2 become triggers, so the BROWSE cursor goes and `STATUS`
   shows the routed slots instead (section 3.1). The alternative is keeping browse behind
   long-press gestures, which I would rather not build.
3. **The 1 % uncertainty floor** (section 3.3) stands in for an uncharacterised ADC gain accuracy.
   If you have a figure for the analog path, it belongs here instead.
4. **Result pages capped at 4** of up to 10 measurement points, so cycling with one button stays
   usable. Full detail remains in `MSUM`.
5. **Log file naming.** Keep `LOGnnnn.CSV`, or switch to `YYMMDDHH.CSV` when the clock source is
   `host`/`batt`? I lean towards keeping the numbering and letting the FAT timestamp and the
   in-file anchor carry the date.
6. **Compliance markers** (the Pulser's inverted-video limited fields) are deliberately out of
   scope (section 3.3). Worth a follow-up phase if you want them.


## 9. Work order

Each step ends with the four builds and the host tests clean.

1. `Clock` module, `CLK` command, boot banner line, log anchor lines, SdFat date callback.
   Documents: protocol section 4, `timing.md` section 6, `hardware-variants.md`, README.
2. Headless boot: panel probe, gated `display()`, headless `SCREEN`, boot lines, stale comment in
   `openLog()`.
3. Buttons: rename, arm/re-arm debounce, `UiInput::poll()`, `Triggers::fireRoute()`, the
   no-route WARN.
4. Pages: page list, `PAGE` command, RESULT view, `Measure::resultSnapshot()`, the
   integer resistance formatter, `test_uifmt.cpp`.
5. `BENCHFMT`, then the fixed-point `field()` conversion; `timing.md` section 6 gets the numbers.
6. README pass, `smoke.py` additions, progress entry, commit.
