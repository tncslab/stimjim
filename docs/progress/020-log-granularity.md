# 020 — log granularity: summaries on the card, config changes, auto-open

Date: 2026-09-09. Branch `arbitrary_waveform`.

## What started it

Three questions about the SD log, all with the same root: **the log recorded what was configured
when the file opened, and every individual reading, and nothing else.**

1. *Are run stats written to the log?* No. Everything the completion path produced went to the
   serial port only — the legacy `Train #<n> complete. Delivered <p> pulses.` line, the two
   `WARN engine:` timing-fault lines, the `# engine:` overdue note, and `MSUM`/`MRANGE` with their
   `# MSUM:` explanations. A file read back later could not say how many pulses came out, whether
   any latch missed its deadline, or what the per-point mean was.
2. *`SDGET` returned the header but no data — is it buffering?* No, and buffering was designed out
   twice over: `SdLog::get` calls `flushNow()` and serves the open log through the same handle
   rather than a second one, and the completion path flushes at every train end. The cause is the
   `MEAS` `report` field, whose default was 0 — summary only, no rows anywhere. `smoke.py` resets
   slot 4 to `report=0` as its own cleanup, so a smoke run leaves exactly that state behind.
3. *Can a log open itself when a card is present?*

## What the phase changed

### 1. `report` is three bits, and bit 1 changed meaning

| bit | | destination |
| --- | --- | --- |
| 0 | `+1` | stream `MDATA` lines over serial |
| 1 | `+2` | `MSUM`/`MRANGE` rows to the card |
| 2 | `+4` | `MDATA` rows to the card |

`+2` used to mean "per-repetition rows to SD"; those moved to `+4`. This is a **breaking change to
a documented, host-visible bitmask** and it fails quietly rather than loudly, because `report=2` is
still a valid request — it now asks for something else. It is called out in the protocol
compatibility appendix (entry 13) and beside the field itself. `smoke.py` moved to `report=6`.

A stored EEPROM image is not invalidated — no struct change, no version bump, so this board keeps
its persisted `CAL STARTLAT 20` / `TRIGCOMP 2` from phase 017 — but a persisted `report=2` shifts
meaning with the firmware. `MEAS<idx>?` is the check.

One consequence inside the player: the ISR pushes into the `MDATA` ring only for
`SJ_REPORT_PER_REP` = bits 0 and 2, so **a train asking only for summaries costs the ring, and
`loop()`, nothing per repetition** — the summary comes out of the accumulators that were already
being updated.

### 2. A driven, measured channel asks for SD summaries by itself

The mode field already had a promotion rule: a plain `0`/`1` promotes a stored `MEAS` `what` of 0
back to 3, while `90`/`91` force it to 0. The same rule now runs one field over — a plain `0`/`1`
promotes a stored `report` of 0 to `+2`. So defining an ordinary stimulus train is enough to get
its summary onto the card, which with §5 below is what makes a headless box log at all.

The test is `mode <= 1 && what != 0`, which after `normalizeMode` is exactly "the line carried a
plain 0/1 mode field": `90`/`91` leave `what` at 0 and promote nothing, correctly, because with no
measurement points there is no summary to write either.

**`isDefaultMeas` had to learn the same rule or `DUMP` round-trip would have broken.** A slot whose
owner deliberately set `report=0` while driving a channel must still serialize a `MEAS` line;
otherwise `DUMP` would omit it as "default" and replaying the `S` line would promote the card back
on. So the default `report` became a function of the train — `TrainStore::defaultReport()` — the
way `when` is already a function of the type. An omitted `report` field on a `MEAS` line takes that
same default, so "the default" means one thing in both places. The host suite checks the round trip
from both sides.

### 3. `MSUM`/`MRANGE` on the card reuse the row format

The CSV column shape is a compatibility promise, so a summary is not a new record type: it is a
normal row whose **repetition column carries a negative record code** instead of a pulse index.

| `<pulse>` | the four value columns hold |
| --- | --- |
| `-1` | `MSUM` means |
| `-2` | `MSUM` sample standard deviations |
| `-3` | `MRANGE` minima |
| `-4` | `MRANGE` maxima |
| `-5` | each line's own repetition count `n` |

The `point` column is untouched, so per-point (per-stage) reporting survives. Five rows per
measured point, sharing one timestamp read once per point — consecutive points get stamps a few
microseconds apart, which shows the write order. The `-5` row carries four counts rather than
`MSUM`'s single "largest of the lines' counts": under read rotation the lines differ by one, and
which line got how many reads is exactly what the single field cannot say. Its columns are counts,
not mV/µA, and the record code is what says so.

The values go through the same `fieldMean`/`fieldSpread`/`fieldCode` helpers `MSUM` and `MRANGE`
print through, so a logged number and a streamed one are the same bytes. `f`/`g` interleave
mean,sd and min,max per line, so the even indices are one row and the odd ones the other.

### 4. Configuration changes and run outcomes are session events in the file

`SdLog::noteEvent(tag, text)` writes `# <tag>: us=<n> <text>`, stamped with the same microsecond
timebase the rows carry, so it maps to a wall clock through the nearest anchor exactly as a row
does. `us=` is the key an anchor line already used for it. Three tags:

- **`set`** — one line per configuration change, `text` being the canonical round-trip line the
  setter already echoed over serial. `S`/`L`/`W`, `ENV`, `MEAS`, `DT`, `DELAY`, `TRIG`, `R`, `CAL`,
  `CALDEF`, `M`, `P`, `CLK`. Out of scope by decision: `V`/`A`, which are immediate output writes
  rather than configuration and which a host sweeping amplitudes would fill a file with, and
  `B`/`C` offset recalibration.
- **`done`** — one line per completed train: `train=`, `slot=`, `pulses=`, `late=`,
  `maxlate_ns=`, `overdue=`, `maxoverdue_ns=`, `startneed_us=`. Fixed shape, every counter present
  even at zero, so a parser needs no optional keys.
- **`stop`** — a train ended by hand. It pushes no completion record and `Engine::timingFaults`
  fills only the four fault fields, so it carries the slot and the counters and nothing else. A
  separate tag rather than a `done` line with invented numbers.

None of the three is gated on `report`: `report` selects measurement destinations, while a
configuration change or a train running is a session event.

Most setters needed one word changed — `Serial.println(line)` became `echo(line)` — because each
already composed the canonical line. Two did not: `M`'s reply is byte-frozen legacy prose, so the
log gets a composed `M<ch>,<mode>`; and `printCalLine` keeps its `Serial.printf` with the bare
newline it always had, composing a second string only for the log, because switching it to
`println` would have silently turned its line terminator into CRLF.

### 5. A log opens itself

`SdLog::autoOpen()` opens the next free `LOGnnnn.CSV`; `Commands::logIfConfigured()` decides
whether to call it — any slot whose `report` has an SD bit set — because `SdLog` does not know what
a slot is. Called at boot after the EEPROM restore, and after every slot commit.

Three details that took a second pass:

- **The latch belongs in `Commands`, not `SdLog`.** The first version put "an explicit `LOG0` keeps
  it closed" inside `closeLog()`/`openLog()`. But `BENCHSD` opens and closes a scratch file of its
  own, so running a benchmark would have silently disarmed the automatic path — and its `openLog`
  would have re-armed one the operator had deliberately closed. The flag now lives beside the
  `LOG` command handler, where the policy is, and BENCHSD cannot reach it.
- **One attempt, not one per commit.** `openLog` reports a card with no free `LOGnnnn.CSV` name;
  repeating that `ERR` on every slot commit would be worse than not logging. The same flag serves
  both purposes — the automatic path gets one turn, and `LOG0` overrules it — because both want the
  same thing: do not open a log behind the operator's back again until `LOG1` asks.
- **An empty socket spends no attempt.** `autoOpen()` returns whether there was a card to try, and
  only a real attempt sets the flag. So inserting a card, remounting with `SDINFO` or `LOG1`, and
  committing a slot opens a log as it would have at boot. `autoOpen` tests `cardPresent()` rather
  than calling `mount()`, so an empty socket never pays an SDIO probe timeout per commit — the cost
  there is a hundred byte tests and nothing else.

Nothing on the automatic path prints when there is no card. `Measure::printSummary` also tests
`SdLog::isOpen()` alongside the bit, because with §2 setting that bit on nearly every train a
card-less board would otherwise format five rows per point per train and throw them all away.

## Verification

### Host and build

- **All four build configurations compile and link clean at `--warnings more`**, from scratch —
  the first three through `SJ_USE_SD == 1` and the Teensy 4.0 through the no-socket stubs, which
  is the no-card path checked by the compiler. Sizes from `arm-none-eabi-size` on the ELF, so they
  are comparable with each other; the Arduino IDE's own "Sketch uses" figure counts differently
  and is not comparable with the tables in earlier entries.

  | Configuration | text | data | bss |
  |---|---|---|---|
  | Teensy 3.5, register | 185 356 | 0 | 44 808 |
  | Teensy 3.5, portable | 186 448 | 0 | 44 840 |
  | Teensy 4.1 | 163 136 | 35 520 | 56 288 |
  | Teensy 4.0, `SJ_EEPROM_SLOTS=6` | 118 080 | 32 448 | 54 144 |

- **Five host suites pass** (`test_trainstore`, `test_measure`, `test_cal`, `test_samplegen`,
  `test_uifmt`). New checks: `isMeasured` and `defaultReport` over the mode/`what` combinations,
  the parse-time promotion including "an explicit refinement survives a redefinition", the
  undriven-slot-keeps-its-bit case, `isDefaultMeas` in both directions for the `DUMP` round trip,
  and `validateMeas` accepting every value up to `SJ_REPORT_MAX`. One pre-existing assumption was
  corrected while writing them: a `90`/`91` slot has never had a default `MEAS`, because its
  `what` is 0 rather than 3.
- **A build-environment note**: `tmp/arduino-cli.yaml` has to point `directories.user` at
  `C:\Users\stipp\OneDrive\Documents\Arduino`, not `…\Documents\Arduino` — the sketchbook is
  inside OneDrive on this machine, and that is where `Adafruit_GFX`, `Adafruit_SSD1306`,
  `Adafruit_BusIO` and a copy of `stimjim` live. With the wrong path the build fails on
  `Adafruit_GFX.h` after getting past `Stimjim.h`.

### Silicon (Teensy 3.5, COM3, 30 GB card)

**The diagnosis of question 2 was confirmed on the board before anything was flashed**: `MEAS4?`
returned `MEAS4,3,3,0,-1,0,1`. `report` was 0, left there by `smoke.py`'s own cleanup, so no rows
had ever been written and `SDGET` could only ever have returned the header. Not buffering.

The flash kept `cal=custom` with `STARTLAT 20` / `TRIGCOMP 2` — phase 017's EEPROM values survive
another reflash. `S4?` came back as modes 3/3, so nothing was configured to log and no file
opened, which is the correct idle behaviour.

**The promotion and the automatic open, in one command.** Defining
`S4,0,1,10000,50000;5000,1000,2000;-5000,-1000,2000` printed `LOG,1,LOG0003.CSV,674` *before* its
own echo — `logIfConfigured()` runs ahead of `echo()` — and `MEAS4?` then read
`MEAS4,3,3,0,-1,2,1`. The file grew 674 → 745 bytes over that one command: 71 bytes, which is the
`# set:` line for the definition that opened it.

**What one train wrote**, read back with `SDGET`:

```
# columns: timestamp_us,slot,pulse,point,V0_mV,I0_uA,V1_mV,I1_uA
# clock: 2026-09-09T17:01:13.851Z src=build us=686517581
# set: us=686521683 S4,0,1,10000,50000;5000,1000,2000;-5000,-1000,2000
# clock: 2026-09-09T17:04:06.238Z src=build us=858901429
# train: S4,0,1,10000,50000;5000,1000,2000;-5000,-1000,2000
# train: MEAS4,3,3,0,-1,2,1
# done: us=858945823 train=3 slot=4 pulses=5 late=0 maxlate_ns=0 overdue=0 maxoverdue_ns=0 startneed_us=0
858946536,4,-1,0,4996.24,,9699.49,992.29
858946536,4,-2,0,5.00,,22.30,1.94
858946536,4,-3,0,4991.85,,9660.45,990.42
858946536,4,-4,0,5001.61,,9716.57,995.52
858946536,4,-5,0,5,,5,5
858946942,4,-1,1,-4984.33,,-9705.83,-987.02
858946942,4,-2,1,1.34,,26.34,1.96
858946942,4,-3,1,-4985.31,,-9730.23,-988.38
858946942,4,-4,1,-4982.87,,-9664.35,-984.13
858946942,4,-5,1,5,,5,5
```

Every value matches the `MSUM`/`MRANGE` the same train printed over serial, digit for digit. The
`I0` column is empty throughout because channel 0 is in voltage mode (phase 018), and the `-5` row
says `5,,5,5` — n for the three lines that were read and nothing for the one that was not, which
is the case a single `MSUM` `n` field cannot express.

**The two points are stamped separately**: 858946536 and 858946942, 406 µs apart. One timestamp is
read per point and shared by that point's five rows, so a host groups by `(timestamp, point)`.

**Benchmarks.** Neither path this phase touched, and neither moved:

| | this phase | phase 015 |
|---|---|---|
| `BENCHFMT,2000` (min/avg/max) | 11.29 / 13.93 / 16.08 µs | 11.8 µs |
| `BENCHSD,500` (min/avg/max) | 9.88 / 50.01 / 2165.6 µs, 18 456 rows/s | 56 µs avg, 4.1 ms worst |

`BENCHSD` also ran with the automatic path armed and left it exactly as it found it, which is the
point of having moved the latch out of `SdLog`.

**The one performance claim, measured.** A summary-only train never enters the `MDATA` ring, so it
should survive a measurement rate that drowns a row-writing one.
`S5,0,1,100,2000000;50,50,50;-50,-50,50` is 20 000 pulses in 2 s with two measured points each —
20 000 records per second, against the 18 456 rows/s `BENCHSD` says this card sustains.

| `report` | result |
|---|---|
| `4` (rows to SD) | `WARN MEAS: MDATA ring overflowed — 35128 records dropped` — 88 % of 40 000 lost |
| `2` (summaries only) | no warning, nothing dropped |

Both runs delivered 20 000 pulses and both summaries carry `n=20000`: accumulation happens in the
ISR before the ring, so the *measurement* was never the casualty — only the per-repetition record
of it. Asking for a summary on the card used to be impossible without paying that; now it is free.

**A wart the hardware run found.** After slot 4 was restored to undriven modes 3/3, `MEAS4?` still
read `report=2`: the promotion raises an unset bit and never clears one, by design, so a slot
switched back to undriven keeps what it was given. Harmless for output — no measurement points
means no summary rows — but `logIfConfigured()` was testing the bit alone, so such a slot would
have gone on opening logs it could write no rows to. Fixed by requiring both halves: the SD bit
*and* `TrainStore::isMeasured(t)`, a new predicate `defaultReport` is now defined in terms of.
Host-tested and built; **awaiting a reflash to confirm on silicon**, because the Arduino IDE's
`teensy-monitor` process had taken COM3 back.

## Open questions

- `BENCHSD` is refused while a log is open, and a log is now usually open by itself, so it wants a
  `LOG0` first. Documented rather than changed — the refusal is the right behaviour, it is simply
  reached more often.
- `logIfConfigured()` scans all `SJ_NUM_SLOTS` = 100 slots per commit until a log opens. Trivial in
  command context, but a cached "some slot wants SD" flag maintained by `commit()` would be O(1).
  Not worth it until something measures it.
- `smoke.py` could not be run: `pyserial` is not installed in the `compute` environment, the only
  conda environment on this machine. Everything above was driven through .NET's
  `System.IO.Ports.SerialPort` from PowerShell instead, which covers individual commands but not
  the suite. `pyserial` (BSD-3-Clause) is already listed as a requirement in
  `tests/device/README.md`.

## Next entry point

Reflash with the `isMeasured` fix and confirm that an undriven slot holding a stale `report=2` no
longer opens a log. Then run `smoke.py`, once `pyserial` is available, for the eleven new checks
it now carries.
