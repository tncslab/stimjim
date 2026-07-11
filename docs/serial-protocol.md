# stimjimAWG serial protocol reference (draft)

Status: protocol version 1. Waveform definition, queries, immediate commands and persistence
(`S`/`L`/`W`, `ENV`/`MEAS`, `M`/`V`/`A`/`E`/`READ`, `B`/`C`/`D`/`P`, `DUMP`) are implemented as
of Phase 2/3; `T`/`U` execution runs all slot types with the `ENV` envelope as of Phase 5;
the measurement engine (`MEAS` execution, `MSUM`/`MDATA` output) arrives in Phase 7, `LOG` in
Phase 7, `TRIG`/`R` setters in Phase 8 (queries already answer) — see
[awg-implementation-plan.md](awg-implementation-plan.md). The
legacy sections below double as documentation of the old `stimjimPulser` behavior; the
"hardened" notes describe what stimjimAWG changes.

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
`<idx>` 0–99 (slot), `<mode0>,<mode1>` per physical channel (below), `<period_us>` interval
between pulse/burst starts, `<duration_us>` total train length.

**Mode field** — the original stimjim numbering (same as `M`): 0 voltage, 1 current,
2 disconnected (hi-Z), 3 grounded. In a train definition 2 and 3 both mean *this channel is not
driven by this train* (its output pins are left untouched, exactly like the original open-ephys
firmware; the channel is never measured). Two additional values are accepted **in train
definitions only**: `90` = voltage and `91` = current **with measurement disabled** on that
channel. They are parse-time sugar: the slot stores mode 0/1 and the `MEAS` `what` for that
channel is forced to 0; conversely a plain 0/1 re-enables measurement (a stored `what` of 1/2 —
a deliberate `MEAS` refinement — is preserved, a stored 0 is promoted back to the default 3).
Queries render 90/91 whenever the stored `what` is 0, so the flag round-trips through
`S<idx>?`/`DUMP`. Any other mode value → `ERR`.

> Lab-firmware migration note: the previous lab firmware re-documented 2/3 as
> voltage/current-without-measurement and 4/5 as hi-Z/ground. That numbering is retired
> (see §6.8): scripts using modes 2/3 for unmeasured stimulation must switch to 90/91 —
> under this firmware 2/3 mean an *inactive* channel again.

### `S` — piecewise-constant (rectangular step) train — legacy semantics, bit-exact

```
S<idx>,<mode0>,<mode1>,<period_us>,<duration_us>; <a0>,<a1>,<dur_us>; ...   (0–10 stages)
S<idx>          → query (safe, never writes)      S<idx>?  → canonical one-line form
```

A stage count of 0 stays accepted (legacy "empty train" — the boot default; the train runs its
period/duration bookkeeping but emits nothing).

Amplitudes in mV (voltage modes) or µA (current modes). Each stage *jumps* to its amplitudes and
holds for `dur_us`. After the last stage the DAC returns to offset (0) and outputs are grounded.

Hardened vs stimjimPulser: parse into staging buffer, validate, commit — a malformed line leaves
the slot untouched (old firmware half-updates, see fixme at `stimjimPulser.ino:925`); modes
outside {0–3, 90, 91} → `ERR` (old: silently coerced out-of-range values).

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
stage boundary and end value (integer Bresenham, plan §3.5 — no rounding accumulates across
samples or stages). Each pulse ramps from the parked offset (0) and, after the last stage
boundary, parks and grounds like `S`: the final stage's end value is latched exactly at the
boundary and immediately parked — append a same-value stage to hold it.

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

Playback (Phase 5, plan §3.5): each period runs one burst of `burst_us`; the **phase restarts at
`phase` every burst** (bursts are identical and drift-free; legacy-consistent), and the output
parks at offset + grounds between bursts. `burst_us = period_us` is continuous sine except for a
few-µs park at each period boundary (same as legacy; a gapless mode may suppress the off event
later). Sample rate per train: `Fs = clamp(64·f_max, 1 kHz, 50 kHz)` on an exact CPU-cycle grid
(the 50 kHz ceiling is provisional until the Phase 6 bench). Frequencies above `Fs/2 = 25 kHz`
are accepted at parse time but **refused at start** with a `WARN` (Nyquist).

Known hardware limit (see [hardware-notes.md](hardware-notes.md)): amplitudes above 3000 µA
convert incorrectly on the DAC → `WARN` on set.

## 3. Execution, trigger and immediate commands

| Cmd | Syntax | Reply / notes |
|---|---|---|
| `T` | `T<idx>` start slot on engine 0; `T-1` stop engine 0 | legacy reply lines kept: `\r\nStarted T train with parameters of PulseTrain <idx>` / `Forcing T train to stop` / `Invalid PulseTrain index.`. Busy engine or channel conflict with the other engine → drop + `WARN` (decision: ignore-and-warn). Strict index parse: `Tfoo` → `ERR` (legacy `atoi` silently started train 0). Train completion prints `Train #<n> complete. Delivered <p> pulses.` from `loop()` (Phase 7 appends `MSUM` lines). |
| `U` | `U<idx>` / `U-1` | same, engine 1 (players are per-engine; a train drives the channels its modes activate) |
| `R` | `R<trig>,<idx>[,<output>]` | **legacy alias** writing the `TRIG` table: `output≠0` → marker mode (`TRIG<t>,3,-1,-1,0`); else joint start of `idx` on rising edge (`TRIG<t>,1,<idx>,-1,0`). `R<t>?` renders the legacy view. NOTE: the old README documented the 3rd arg as an edge selector — the code's actual meaning is the output-marker flag; edge selection lives in `TRIG`. |
| `M` | `M<ch>,<mode>` | exactly 1 line: `Set channel <ch> to mode <mode>`. Modes: 0 voltage, 1 current, 2 disconnected (hi-Z), 3 grounded — original numbering, mapping 1:1 onto the OE decoder (§6.8). 90/91 are train-definition sugar and are rejected here. `M<ch>?` returns shadow state (new); running trains switch the OE pins autonomously (driven channels end grounded, shadow tracks that). |
| `V` | `V<ch>,<mV>` immediate voltage | exactly 1 line: `Set channel <ch> to amplitude <mV> mV (dac value <dac>).` or `<dac> is out of range.` During a running train: executed under bus lock + `WARN`. |
| `A` | `A<ch>,<dac>` immediate raw DAC (−32768…32767) | exactly 1 line: `Set channel <ch> to amplitude <dac>` |
| `E` | `E<ch>,<line>` read ADC; line 0 = output voltage, 1 = current sense | exactly 1 line: `Read value: <raw> (<value>mV)` / `(<value>uA)` — format frozen for BIST |
| `B` | recalibrate ADC offsets (grounds outputs) | `OK` when done (legacy printed nothing, §6); refused while a train runs |
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
waveform types, evaluated at each latch event (plan §3.5): `L`/`W` trains follow it per sample,
`S` trains sample it at each stage latch (stair-step over a long stage — use `L` for smooth
ramps). It scales amplitudes relative to the channel's calibration offset (the parked baseline
does not move). Validation: `rampIn + rampOut ≤ duration` else `ERR`. Default `0,0,0`.

### `MEAS` — per-train measurement configuration

```
MEAS<idx>,<what0>,<what1>,<when>,<stage>[,<report>]
MEAS<idx>? →  MEAS<idx>,<what0>,<what1>,<when>,<stage>,<report>
```

- `what<ch>`: 0 none, 1 voltage, 2 current, 3 both. Coupled to the train-definition mode field
  (§2): mode 90/91 forces 0; a plain 0/1 promotes a stored 0 back to 3 (an explicit 1/2 is
  preserved). Channels the train does not drive (mode 2/3) are never measured; setting a
  non-zero `what` on one is accepted with `WARN` (meaningless until the mode changes).
- `when` — the measurement instant inside the selected stage/period; codes are type-specific
  and a mismatch → `ERR` (changing a slot's type auto-coerces `when` and `stage` to the new
  type's defaults):
  - `S`/`L` slots: **0** = near stage end — transient settled, ADC programming time budgeted
    (plan §3.6). The only v1 code; a reserved `<offset_us>` extension field is documented for
    future manual placement.
  - `W` slots: **1** = positive peak, **2** = negative peak, **3** = both peaks. One period per
    burst is measured (the first full period after envelope ramp-in completes): at generation
    rates the V+I ADC budget (~9–11 µs) rules out per-sample measurement, and mid-ramp peaks
    would under-read.
- `stage`: −1 = every stage (default), 0…nStages−1 = only that stage (`S`/`L`; subsumes the
  earlier first-stage-only mode). `W` slots require −1. An `S`/`L` redefinition that shrinks
  the stage count below a stored selection → `ERR` (reset `MEAS` first) — same policy as a
  preserved `ENV` that no longer fits.
- `report` bitmask: 0 end-of-train summary (always kept), +1 stream `MDATA` lines, +2 log to SD.
  v1 implements summary + SD; streaming format is fixed now, implementation deferred.
- Repetitions (one measurement point per pulse/burst) accumulate `n`, Σv and Σv² per point,
  line and channel; the summary reports the mean **and the sample standard deviation** derived
  from those sums. This is the single-pass estimator — numerically simplified by design
  (documented trade-off; adequate for 13-bit ADC data at the repetition counts a train can
  reach), chosen so a waveform averaged over many repetitions also yields a spread estimate.
- Defaults: `what0=what1=3`, `when=0` for `S`/`L` slots / `3` for `W` slots, `stage=-1`,
  `report=0` — measure everything, summary only (reproduces legacy averaging behavior).
- Stages too short to fit their measurement get it skipped and flagged in the summary.

**MDATA record** (per-repetition stream and SD CSV, format frozen in v1):
`MDATA,<slot>,<pulse>,<point>,<V0_mV>,<I0_uA>,<V1_mV>,<I1_uA>` — empty field where not measured.
SD rows are prefixed with `<timestamp_us>` (µs since boot) instead of the `MDATA` word.

**MSUM record** (end-of-train summary, format frozen in v1, emitted from `loop()` after the
legacy `Train #<n> complete…` line):
`MSUM,<slot>,<n>,<point>,<V0_mV>,<V0_sd>,<I0_uA>,<I0_sd>,<V1_mV>,<V1_sd>,<I1_uA>,<I1_sd>`
— one line per measured point; means and sd in mV/µA. `<point>` is the stage index for `S`/`L`
and the peak phase in degrees (`90` / `270`) for `W`. `<n>` = repetitions accumulated; sd is
empty when `n < 2`; fields are empty where not measured.

### `READ` — manual averaged measurement (immediate)

```
READ<ch>[,<n>]        n = samples per line, default 16, max 10000
→  READ,<ch>,<n>,<V_mV>,<V_sd>,<I_uA>,<I_sd>
```

Reads the channel's output-voltage and current-sense lines `n` times each through the same
calibrated path as `E` (offset-corrected, `*_PER_ADC` conversion) and reports mean and sample
standard deviation in mV/µA with two decimals (sub-LSB resolution is meaningful once averaged).
Complements `E`, which stays the single raw read with the BIST-frozen reply. Refused with `ERR`
while any train runs — a long averaging burst under the bus lock would stall the players;
in-train measurement is `MEAS`'s job.

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
| `DUMP` | session export: `#` header, one round-trippable line per non-default slot, non-default `ENV`/`MEAS` (S/L slots only — a `W` line carries its envelope), both `TRIG` lines, `OK`. Paste-back restores the configuration. Until the `TRIG` setter exists (Phase 8) the `TRIG` lines are emitted as `#` comments so paste-back stays clean. |
| `LOG` | SD logging: `LOG?` status (card present, open file, bytes); `LOG1[,name]` open new file; `LOG0` close/flush. |
| `BENCH` | benchmark group (implemented in Phase 1, see below). |

**`BENCH` group** (Phase 1). Timing results are printed in CPU cycles (120/µs) and ns; multi-line
output ends with `OK`. `BENCH?` lists the group. DAC benches program without latching (or re-latch
the calibration offsets), so outputs never move; `BENCHSQ`/`BENCHSQL` do drive the DAC and print a
`WARN` first — keep outputs grounded (boot state).

| Cmd | Function |
|---|---|
| `BENCHDAC[,n]` / `BENCHDAC2[,n]` | `dacProgram` single / `dacProgramBoth` dual (no latch) |
| `BENCHLATCH[,n]` | `dacLatch(0b11)` pulse |
| `BENCHADC[,ch,line,n]` | `adcRead` with the line pre-selected |
| `BENCHSW[,ch,n]` | alternating `adcSelectLine`+`adcRead` (line-switch cost, bench-verify item 1) |
| `BENCHMISO[,n]` | alternating ch0/ch1 reads (MISO PORT-mux swap, bench-verify item 3) |
| `BENCHCYC[,n]` | `cycles64()` overhead |
| `BENCHK[,n]` | re-run the `K_RELOAD` self-calibration, print residual min/med/max (item 6) |
| `BENCHPIT,period_us,n[,preload_us]` | PIT wake (preload 0) or post-spin latch jitter vs absolute deadline, histogram in 0.5 µs bins |
| `BENCHSQ,ch,code,half_us,n` | square wave via FastIO program+latch — scope A/B vs |
| `BENCHSQL,ch,code,half_us,n` | the same square wave via legacy `Stimjim.writeToDac` |

## 5. Defaults (authoritative table)

| Item | Default |
|---|---|
| Slot (boot, all 100) | mode0=mode1=3 (grounded = not driven), period=10000 µs, duration=500000 µs, 0 stages, type=S |
| `ENV` | 0,0,0 (no ramp) |
| `MEAS` | 3,3,auto-when (0 for S/L, 3 for W),-1 (all stages),0 (summary only) |
| `READ` sample count | 16 |
| Triggers | both `TRIG<t>,3,-1,-1,0` (output marker) |
| `R` third argument | 0 (trigger-input mode) |
| Ramp sample interval | 20 µs (`TARGET_DT_US`) |
| EEPROM restore | overrides boot defaults for slots 0–9 + ENV/MEAS + TRIG when version+checksum valid |
| Serial | USB CDC — baud irrelevant (fixes the `Serial.begin(112500)` typo) |

## 6. Compatibility appendix

Preserved byte-exact (BIST contract): the `M`/`V`/`A`/`E` single-line replies quoted in §3
(`stimjimPulser.ino:1011-1050`); bare `S<idx>` remains a query printing the parameter dump.

Deliberate behavior changes (documented compat risk, all fail-loudly):

1. Omitted required fields in `M`/`V`/`A`/`E`/`R` → one-line `ERR` instead of silently assuming 0,
   and `T`/`U` parse their index strictly (legacy `atoi` turned `Tfoo` into "start train 0").
   (BIST always sends full fields — unaffected.)
2. Malformed `S`/`W` lines no longer half-update a slot (atomic staging).
3. Train modes return to the **original open-ephys numbering 0–3** (0 V, 1 I, 2 hi-Z,
   3 ground; 2/3 in a train = channel not driven), extended by 90/91 = V/I without
   measurement (§2). The lab firmware's renumbering (2/3 = unmeasured V/I, 4/5 = hi-Z/ground)
   is retired: **lab scripts that used modes 2/3 for unmeasured stimulation must switch to
   90/91** — replayed unchanged they now define an inactive channel (fails safe: no output
   rather than unexpected output). Out-of-range modes → `ERR` instead of silent coercion.
4. `W` phase is now applied; old firmware ignored it (and printed it via a `>0` boolean bug at
   `stimjimPulser.ino:806`).
5. `C` prints a `WARN` line about the output ramp before calibrating.
6. Editing a slot attached to a running engine is refused (`ERR … stop first (T-1)`).
7. Buttons no longer directly start trigger-mapped trains — they navigate the menu (Btn0 = OK,
   Btn1/Btn2 = prev/next; see plan §5).
8. `M` accepts modes 0–3 with the original semantics, mapping 1:1 onto the 2-bit OE decoder
   (`Stimjim::setOutputMode`) — **identical to the upstream open-ephys firmware, which was
   never confused about this**. The confusion was introduced by the lab modification
   ("Added measure/stim modes"): it renumbered only the *documentation and mode strings*
   (2/3 → unmeasured V/I, 4/5 → hi-Z/ground) while the `M` handler kept passing the mode raw
   into the decoder, so under the lab's documented numbering `M2`/`M3` really produced
   hi-Z/ground and `M4`/`M5` **connected the voltage/current source**. (The lab's pulse-train
   paths were correct — they masked with `mode & 1` under a `mode < 4` guard; only direct `M`
   diverged.) Reverting to the original numbering makes documentation and decoder agree again.
   BIST is unaffected (it only uses modes 0/1, identical in every numbering).
9. Negative `W` frequencies → `ERR`. The legacy firmware accepted them (producing a time-reversed
   sine through table-index wraparound); scripts relying on that should use the equivalent
   positive-frequency + phase form.
10. `B` replies with a lone `OK` (the legacy handler printed nothing at all). `B`/`C` are refused
    while a train is running (calibration would fight the players for the SPI bus for hundreds
    of ms).

Corrections to stale legacy documentation:

- `R`'s third argument is the *output-marker flag* (code), not an edge selector (old README).
- `W` triplets 2–3 require all 3 fields (`stimjimPulser.ino:955`), despite the old header claiming
  the third can be omitted.
- The documented `V0`/`V1` verbose-reporting toggle never existed (dead `verbose` flag); `V` is
  only the immediate-voltage setter.
