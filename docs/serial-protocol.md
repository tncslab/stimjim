# stimjimAWG serial protocol reference (draft)

Status: design draft, protocol version 1 (implemented from Phase 2 onward — see
[awg-implementation-plan.md](awg-implementation-plan.md)). The legacy sections below double as
documentation of the current `stimjimPulser` behavior; the "hardened" notes describe what
stimjimAWG changes.

Backward-compatibility contract: every command of `stimjimPulser` keeps its syntax and semantics;
`M`/`V`/`A`/`E` keep **byte-compatible single-line replies** because StimJimBIST performs exactly
one `ReadLine()` per command and parses `E`'s `(<value><unit>)` group.

## 1. Conventions

- **Framing**: lines terminated by `\n` (`\r` stripped); max 999 bytes + NUL (1000-byte buffer,
  bounded — overflow discards to next `\n` and replies `ERR line too long (max 999)`). Blank lines
  and lines starting with `#` are ignored, so `DUMP` output can be replayed verbatim.
- **Dispatch**: leading alphabetic run = command word. One letter → legacy command; two or more
  letters → long-form command. Collision-free: legacy letters are always followed by a digit, `-`,
  `,`, `?` or end of line. The single-letter namespace stays reserved for waveform-definition and
  terse commands (`L` is the one new single-letter command, sibling of `S`/`W`).
- **Query suffix `?`**: for every settable object, `<word><idx>?` returns exactly one line — the
  canonical set-command that reproduces the current state (round-trippable). `S<idx>?` on a slot
  answers with the letter matching its actual type (`S`, `L` or `W` line).
- **Replies**: new setters echo the canonical round-trip line of what was stored. Errors:
  `ERR <cmd>: <message>` — the command is rejected and prior state is fully intact (atomic
  staging-buffer commit; edits to a currently-running slot are refused). Warnings:
  `WARN <cmd>: <message>` — accepted with caveat. Multi-line human output (`HELP`, `DUMP`, legacy
  parameter dumps) ends with a lone `OK` line.
- **No silent defaults in new code paths**: omitted required fields produce a one-line `ERR`
  instead of assuming 0 (fail-loudly; see compatibility appendix §6).
- Machine parse rule: classify a reply line by its first token — `ERR`, `WARN`, `OK`, `#`, a
  command word (round-trip data), else legacy human text.

## 2. Waveform definition commands

Common header for `S`/`L`/`W`:
`<idx>` 0–99 (slot), `<mode0>,<mode1>` per physical channel (see `M`), `<period_us>` interval
between pulse/burst starts, `<duration_us>` total train length.

### `S` — piecewise-constant (rectangular step) train — legacy semantics, bit-exact

```
S<idx>,<mode0>,<mode1>,<period_us>,<duration_us>; <a0>,<a1>,<dur_us>; ...   (1–10 stages)
S<idx>          → query (safe, never writes)      S<idx>?  → canonical one-line form
```

Amplitudes in mV (voltage modes) or µA (current modes). Each stage *jumps* to its amplitudes and
holds for `dur_us`. After the last stage the DAC returns to offset (0) and outputs are grounded.

Hardened vs stimjimPulser: parse into staging buffer, validate, commit — a malformed line leaves
the slot untouched (old firmware half-updates, see fixme at `stimjimPulser.ino:925`); modes
outside 0–5 → `ERR` (old: silently forced to 5).

### `L` — piecewise-linear (ramp) train — NEW

```
L<idx>,<mode0>,<mode1>,<period_us>,<duration_us>; <a0>,<a1>,<dur_us>; ...
```

Identical syntax to `S`; each stage **ramps linearly from the previous end value** to
`(a0,a1)` over `dur_us`. `dur_us = 0` = instant jump, so piecewise-constant is expressible as
jump+hold pairs:

```
L0,...;1000,-1000,200      # ramp 0 → (1000,−1000) over 200 µs
L0,...;1000,0,0;1000,0,500 # jump to 1000, hold 500 µs (ramp from 1000 to 1000)
```

Ramp sample interval defaults to 20 µs (engine `TARGET_DT_US`); last sample lands exactly on the
stage boundary and end value.

### `W` — sine train — explicit fields, phase fixed

```
W<idx>,<mode0>,<mode1>,<period_us>,<duration_us>;
   <amp0>,<amp1>,<burst_us>;      # amplitudes (mV/µA), burst length per period
   <freq0>,<freq1>,0;             # frequency in Hz, decimals accepted (stored as mHz); 3rd field reserved, must be present
   <phase0>,<phase1>,0[;          # start phase in degrees — NOW APPLIED (old firmware ignored it)
   <rampIn_us>,<rampOut_us>,<shape>]   # optional 5th triplet = envelope (see ENV)
```

Omitting the 5th triplet resets the envelope to `0,0,0` — a full `W` line fully defines the slot,
keeping query output round-trip exact. Legacy `W` lines (4 triplets, integer Hz) parse unchanged.

Known hardware limit (see [hardware-notes.md](hardware-notes.md)): amplitudes above 3000 µA
convert incorrectly on the DAC → `WARN` on set.

## 3. Execution, trigger and immediate commands

| Cmd | Syntax | Reply / notes |
|---|---|---|
| `T` | `T<idx>` start slot on engine 0; `T-1` stop engine 0 | legacy start/stop lines; busy channel → drop + `WARN` (decision: ignore-and-warn) |
| `U` | `U<idx>` / `U-1` | same, engine 1 |
| `R` | `R<trig>,<idx>[,<output>]` | **legacy alias** writing the `TRIG` table: `output≠0` → marker mode (`TRIG<t>,3,-1,-1,0`); else joint start of `idx` on rising edge (`TRIG<t>,1,<idx>,-1,0`). `R<t>?` renders the legacy view. NOTE: the old README documented the 3rd arg as an edge selector — the code's actual meaning is the output-marker flag; edge selection lives in `TRIG`. |
| `M` | `M<ch>,<mode>` | exactly 1 line: `Set channel <ch> to mode <mode>`. Modes: 0 voltage, 1 current, 2 voltage (no measurement), 3 current (no measurement), 4 hi-Z, 5 grounded. `M<ch>?` returns shadow state (new). |
| `V` | `V<ch>,<mV>` immediate voltage | exactly 1 line: `Set channel <ch> to amplitude <mV> mV (dac value <dac>).` or `<dac> is out of range.` During a running train: executed under bus lock + `WARN`. |
| `A` | `A<ch>,<dac>` immediate raw DAC (−32768…32767) | exactly 1 line: `Set channel <ch> to amplitude <dac>` |
| `E` | `E<ch>,<line>` read ADC; line 0 = output voltage, 1 = current sense | exactly 1 line: `Read value: <raw> (<value>mV)` / `(<value>uA)` — format frozen for BIST |
| `B` | recalibrate ADC offsets (grounds outputs) | legacy lines |
| `C` | recalibrate current+voltage offsets | prints `WARN C: output will ramp` first — `getVoltageOffsets()` sweeps a voltage ramp on the outputs |
| `D` | print offsets (human); `D?` machine CSV: `D,<adc25_0>,<adc25_1>,<adc10_0>,<adc10_1>,<ioff0>,<ioff1>,<voff0>,<voff1>` | |
| `P` | save slots 0–9 + ENV/MEAS + TRIG table to EEPROM (versioned, checksummed) | legacy confirmation |

## 4. New long-form commands

### `ENV` — per-train amplitude envelope

```
ENV<idx>,<rampIn_us>,<rampOut_us>[,<shape>]      shape: 0 = linear (default; 1 = raised-cosine reserved)
ENV<idx>?  →  ENV<idx>,<in>,<out>,<shape>
```

Envelope 0→1 over `rampIn` from train start, 1→0 ending exactly at `duration_us`. Applies to all
waveform types. Validation: `rampIn + rampOut ≤ duration` else `ERR`. Default `0,0,0`.

### `MEAS` — per-train measurement configuration

```
MEAS<idx>,<what0>,<what1>,<when>[,<report>]
MEAS<idx>? →  MEAS<idx>,<what0>,<what1>,<when>,<report>
```

- `what<ch>`: 0 none, 1 voltage, 2 current, 3 both. Channel modes 2/3 force none (documented);
  modes 4/5 → `WARN` (meaningless).
- `when`: 0 first stage only, 1 all stages (pulse/ramp trains), 2 sine peak (first 90° crossing
  after envelope ramp-in). Type mismatch → `ERR`.
- `report` bitmask: 0 end-of-train summary (always kept), +1 stream `MDATA` lines, +2 log to SD.
  v1 implements summary + SD; streaming format is fixed now, implementation deferred.
- Defaults (reproduce legacy behavior): `what0=what1=3`, `when=1` for `S`/`L` slots, `2` for `W`
  slots, `report=0`.
- Sampling instant: near stage end, transient settled, ADC programming time budgeted (see plan
  §3.6). A reserved 5th field `<offset_us>` is documented for future manual placement.
- Stages too short to fit their measurement get it skipped and flagged in the summary.

**MDATA record** (stream and SD CSV, format frozen in v1):
`MDATA,<slot>,<pulse>,<stage>,<V0_mV>,<I0_uA>,<V1_mV>,<I1_uA>` — empty field where not measured.
SD rows are prefixed with `<timestamp_us>` (µs since boot) instead of the `MDATA` word.

### `TRIG` — trigger routing (per input 0/1)

```
TRIG<t>,<mode>,<slot0>,<slot1>,<edge>
TRIG<t>?  → canonical line
```

- `mode`: 0 disabled; 1 joint — edge starts `slot0` on both channels synchronized (`slot1` must be
  −1); 2 independent — edge starts `slot0` on channel 0 and `slot1` on channel 1 (−1 = none);
  3 output marker (pin driven high during stimulus — legacy `R…,…,1`; slots must be −1).
- `edge`: 0 rising, 1 falling (required for modes 1–2).
- A trigger for a busy channel is dropped; `trigRejectCount` increments and a `WARN` is printed
  from `loop()`.
- Boot default (no valid EEPROM): both triggers mode 3 — matches legacy boot state.

### Status and utility

| Cmd | Reply |
|---|---|
| `STAT` | one line: `STAT,<slot0>,<n0>,<elapsed0_us>,<dur0_us>,<slot1>,<n1>,<elapsed1_us>,<dur1_us>` (idle engine: slot −1, zeros). Cheap for GUI polling. |
| `IDN` | `IDN,stimjimAWG,Teensy3.5,fw=<x.y.z>,proto=1` (also in boot banner; lets the Python GUI feature-detect) |
| `HELP` | multi-line human command table with units and defaults, ends with `OK`. Bare `?` = alias. |
| `DUMP` | session export: `#` header, one round-trippable line per non-default slot, non-default `ENV`/`MEAS`, both `TRIG` lines, `OK`. Paste-back restores the configuration. |
| `LOG` | SD logging: `LOG?` status (card present, open file, bytes); `LOG1[,name]` open new file; `LOG0` close/flush. |
| `BENCH` | benchmark group (Phase 1+): jitter histograms, throughput sweep — documented separately once implemented. |

## 5. Defaults (authoritative table)

| Item | Default |
|---|---|
| Slot (boot, all 100) | mode0=mode1=5 (grounded), period=10000 µs, duration=500000 µs, 0 stages, type=S |
| `ENV` | 0,0,0 (no ramp) |
| `MEAS` | 3,3,auto-when (1 for S/L, 2 for W),0 (summary only) |
| Triggers | both `TRIG<t>,3,-1,-1,0` (output marker) |
| `R` third argument | 0 (trigger-input mode) |
| Ramp sample interval | 20 µs (`TARGET_DT_US`) |
| EEPROM restore | overrides boot defaults for slots 0–9 + ENV/MEAS + TRIG when version+checksum valid |
| Serial | USB CDC — baud irrelevant (fixes the `Serial.begin(112500)` typo) |

## 6. Compatibility appendix

Preserved byte-exact (BIST contract): the `M`/`V`/`A`/`E` single-line replies quoted in §3
(`stimjimPulser.ino:1011-1050`); bare `S<idx>` remains a query printing the parameter dump.

Deliberate behavior changes (documented compat risk, all fail-loudly):

1. Omitted required fields in `M`/`V`/`A`/`E`/`R` → one-line `ERR` instead of silently assuming 0.
   (BIST always sends full fields — unaffected.)
2. Malformed `S`/`W` lines no longer half-update a slot (atomic staging).
3. Out-of-range modes → `ERR` instead of silent coercion to 5.
4. `W` phase is now applied; old firmware ignored it (and printed it via a `>0` boolean bug at
   `stimjimPulser.ino:806`).
5. `C` prints a `WARN` line about the output ramp before calibrating.
6. Editing a slot attached to a running engine is refused (`ERR … stop first (T-1)`).
7. Buttons no longer directly start trigger-mapped trains — they navigate the menu (Btn0 = OK,
   Btn1/Btn2 = prev/next; see plan §5).

Corrections to stale legacy documentation:

- `R`'s third argument is the *output-marker flag* (code), not an edge selector (old README).
- `W` triplets 2–3 require all 3 fields (`stimjimPulser.ino:955`), despite the old header claiming
  the third can be omitted.
- The documented `V0`/`V1` verbose-reporting toggle never existed (dead `verbose` flag); `V` is
  only the immediate-voltage setter.
