# Plan — log granularity: summaries on the card, config changes, auto-open

Phase 020. Four changes to what reaches the SD log, and one to when a log exists at all.

## 1. `report` becomes three bits

| bit | value | meaning |
| --- | ----- | ------- |
| 0 | +1 | stream `MDATA` lines over serial |
| 1 | +2 | `MSUM`/`MRANGE` rows to SD |
| 2 | +4 | `MDATA` rows to SD |

**This reassigns bit 1.** Until now `+2` meant "per-repetition rows to SD"; that moves to `+4`.
The change is deliberate and breaking: a host script asking for `report=2` gets summary rows
where it used to get per-repetition rows. `smoke.py` and the docs move with it. A stored EEPROM
image is *not* invalidated (no struct change, no version bump) but a persisted `report=2` shifts
meaning the same way — checked with `MEAS<idx>?` after the reflash.

Consequences inside `Measure`:

- the ISR only pushes into the `MDATA` ring for `report & (1|4)`; `report=2` alone costs the
  player nothing per repetition, since a summary comes out of the accumulators;
- `poll()` streams on bit 0 and writes rows on bit 2;
- the `# train:` block is written for either SD bit.

## 2. A driven, measured channel enables SD summaries by itself

Same shape as the existing mode↔`what` coupling (protocol §2): a plain `0`/`1` mode field
promotes a stored `report` of 0 to `+2`; any non-zero stored `report` is an explicit refinement
and is preserved. `90`/`91` (V/I with measurement disabled) promote nothing — with `what` forced
to 0 on both channels there are no measurement points and so no summary rows either.

The test is "this channel is driven *and* measured" — `mode <= 1 && what != 0` — which after
`normalizeMode` is exactly "a plain 0/1 mode field".

`isDefaultMeas` has to learn the same rule, or `DUMP` round-trip breaks: a slot whose owner
deliberately set `report=0` must still serialize a `MEAS` line, otherwise replaying the `S` line
promotes it back to 2. So the default `report` becomes a function of the train rather than the
constant 0, exactly as `when` already is a function of the type. An omitted `report` field on a
`MEAS` line takes that same default, so "the default" means one thing everywhere.

## 3. `MSUM`/`MRANGE` on the card reuse the row format

The CSV shape does not change — it is a compatibility promise (protocol §4). A summary row is a
normal row whose **repetition column carries a negative record code** instead of a pulse index:

| code | the four value columns hold |
| ---- | --------------------------- |
| −1 | `MSUM` means |
| −2 | `MSUM` sample standard deviations |
| −3 | `MRANGE` minima |
| −4 | `MRANGE` maxima |
| −5 | each line's own repetition count `n` |

The `point` column keeps the point label, so per-point (per-stage) reporting survives. `−5`
carries four counts rather than `MSUM`'s single "largest of the lines' counts", which is strictly
more informative under read rotation, where the lines differ by one. Units in a `−5` row are
counts, not mV/µA — the code says which. The timestamp is when the summary was written, a few
microseconds after the completion record was popped.

## 4. Configuration changes are echoed into an open log

Every setter that changes stored configuration already prints its canonical round-trip line. Each
of those also goes to the log as `# set: us=<n> <line>`, so a file records changes made *after* it
was opened, not just the `DUMP` block from when it was. Scope: `S`/`L`/`W`, `ENV`, `MEAS`, `DT`,
`DELAY`, `TRIG`, `R`, `CAL`, `CALDEF`, `M`, `P`, `CLK`. Out of scope by decision: `V`/`A`
(immediate output writes — a host sweeping voltages would fill the file) and `B`/`C` (offset
recalibration).

Gated only on a log being open, not on `report`: `report` is a per-slot measurement setting, a
configuration change is a session event.

## 5. A log opens itself when something is configured to write one

`SdLog::autoOpen()` opens the next free `LOGnnnn.CSV` when a card is mounted, no file is open,
and no `LOG0` has latched it off for this boot. `Commands` decides *whether* to call it: any slot
whose `report` has an SD bit set. Checked at boot after the EEPROM restore, and after every slot
commit.

No card is not an error on this path — it prints nothing and does nothing, and it never re-probes
the bus (`begin()` mounted once at boot; `LOG1` and `SDINFO` still remount, as documented). So a
board with an empty socket behaves exactly as before.

## 6. A train records what it did

The pulse count and both timing-fault counters existed only on the serial port, which is the gap
the phase opened with: a log said what was *configured* and what was *read*, never what happened.
One fixed-shape line per train closes it — every counter present even at zero, so a parser needs
no optional keys.

```
# done: us=<n> train=<n> slot=<n> pulses=<n> late=<n> maxlate_ns=<n> overdue=<n> maxoverdue_ns=<n> startneed_us=<n>
# stop: us=<n> slot=<n> late=<n> maxlate_ns=<n> overdue=<n> maxoverdue_ns=<n>
```

A hand-stopped train pushes no completion record and `Engine::timingFaults` fills only the four
fault fields, so `stop` is a separate tag rather than a `done` line with invented numbers. Neither
is gated on `report`: a train running is a session event, like a configuration change.

## Verification

- five host suites build clean and pass, with new checks for the promotion rule, the default
  `report`, `DUMP` round-trip in both directions, and the widened `report` validation;
- all four build configurations compile and link;
- on silicon: `smoke.py`, then a card-less run to prove nothing regressed, then a card run
  showing an auto-opened log carrying `# set:` lines and the five summary codes.
