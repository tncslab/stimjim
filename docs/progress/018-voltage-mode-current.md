# 018 — the voltage-mode current reading was never the load's, and is now gone

The firmware no longer reports a current for a channel driving in voltage mode, because on this
hardware there is nothing there to measure. Phase 016 read the symptom correctly — a 1 kΩ resistor
came back as 3988 Ω — but diagnosed it wrongly as a readback low by a factor of four. It was not a
calibration error of any kind. In voltage mode the current-sense shunt is not in the load path at
all, and what the sense amplifier was reading is the current pump's own branch current into an
on-board dummy resistor. The factor of four was an accident of the 1 kΩ load.

The question that unlocked it was the right one to ask: *why would the output mode affect an ADC
reading, when the DAC and the ADC are separate chips?* It does not. The output mode moves an analog
switch, and that switch decides what the shunt is in series with.

## The topology, from `PCB/stimjim.kicad_pcb`

The DG409 (U23 for channel 0, U12 for channel 1) is a **dual** 4:1 mux, both banks addressed by the
same two pins — and those pins are `OE1:OE0`, which `StimJim::setOutputMode` drives straight from
the mode number. Bank B picks what the output connector sees. **Bank A picks what `I_OUT` is tied
to**, and that is the half nobody had written down:

| mode | `OE1:OE0` | bank B → `CHANNEL_OUT` | bank A → `I_OUT` |
|---|---|---|---|
| 0 voltage | 00 | `V_OUT` | R14, a 1 kΩ dummy to `GRND` |
| 1 current | 01 | tied to `I_OUT` | tied to `CHANNEL_OUT` |
| 2 hi-Z | 10 | unconnected | R14 |
| 3 ground | 11 | `GRND` | R14 |

The sense chain is `HOWLAND_OUTPUT —[R2 3k]— (U10 buffer) —[R12 100]— I_OUT`, with the AD8421
(U24) measuring across R12 at a gain of 1 + 9.9k/360 = 28.5, giving the ~2872 V/A that
`MICROAMPS_PER_ADC` assumes. R12 is in the `I_OUT` branch. **There is no shunt anywhere in the
`V_OUT` path.**

So in voltage mode the load current leaves through `V_OUT` and is never sensed, while the Howland
pump — still driven by the same DAC code, because the mux does not disconnect it — pushes its
commanded current through R12 into R14. That is what the ADC read.

## The arithmetic that confirms it

If the reading is the pump's branch, it should equal the DAC code reinterpreted as a current:
`commanded_mV / MILLIVOLTS_PER_DAC × MICROAMPS_PER_DAC`. Against the phase-016 measurements:

| commanded | DAC code | pump current predicted | reported | error |
|---|---|---|---|---|
| 1000 mV | 2186 | 222.3 µA | 221.7 µA | −0.28 % |
| 2000 mV | 4373 | 444.7 µA | 441.9 µA | −0.64 % |
| 4000 mV | 8745 | 889.4 µA | 882.3 µA | −0.79 % |

Under 1 %, at every amplitude. The load-independent ratio between the two modes' readings was
4.035, and `MILLIVOLTS_PER_DAC / MICROAMPS_PER_DAC` is 4.497 mV/µA — which lands near four for a
1 kΩ load once the output droop is folded in. A different load would have given a different
"factor", which is exactly why it was never a calibration.

An early hypothesis that the AD7321's range register was wrong (±10 V versus ±2.5 V is a factor of
four) is ruled out by observation, not by datasheet: `setAdcRange` writes both ADC input lines in
one register write, and the *voltage* line agrees with a scope in both modes. A range error could
not be selective about the line.

## The fix

`planCompile` drops line 1 for a channel whose output mode is voltage, and flags it:

```c
const uint8_t keep0 = (def.mode0 & 1) ? 3u : 1u;   // voltage mode: line 0 only
pl.lines[0] = (g.chMask & 1) ? (uint8_t)(def.meas.what0 & 3 & keep0) : 0;
```

One place, and everything downstream follows: `nReads` drops (so a voltage-mode point needs a
narrower gap than before), `MSUM`/`MRANGE`/`MDATA` leave the current fields empty, the SD rows do
the same, and the OLED result pages already print `--` for both I and R when there are no current
samples — `fmtOhm` takes a `haveI` argument and phase 015 handled it. The stored `MEAS what` is
left alone: switch the slot to current mode and the current comes back.

One `#` line per train says why, from `printNote` in loop context:

```
# MEAS: slot 40: channel 0 and 1 are in voltage mode, where the current sense sits in the
disconnected I_OUT branch and reads the current pump's own current into the on-board 1 kOhm
dummy, not the load's — the current line is not measured. Use current mode to measure current.
```

**Not changed:** `READ` and the legacy `E` still read both ADC lines unconditionally. `E`'s reply
is byte-frozen by the compatibility contract, and the output mode is not tracked in software —
`setOutputMode` writes two GPIO pins and keeps no state — so suppressing it there means adding
mode tracking in two places (the `M` command and the engine's own mode switches). Documented in
`serial-protocol.md` instead.

Verified on the board (Teensy 3.5, rebuilt and reflashed):

```
VOLTAGE mode:  MSUM,40,10,0,1765.27,3.29,,,,,,          <- no current, as it should be
CURRENT mode:  MSUM,40,10,0,962.02,4.48,990.05,0.63,,,, <- 990 uA for 1000 set
```

## A second phase-016 claim corrected

"The voltage-mode output is 10–15 % below the commanded amplitude" is a **load droop**, not a gain
error. An unloaded channel reads 1996 mV for 2000 commanded (−0.2 %); the same command into 1 kΩ
gives 1766 mV (−12 %). That is a source impedance of about (2000 − 1766) mV / 1.77 mA ≈ **135 Ω**.
The phase-016 measurements were all taken into the same 1 kΩ, so nothing there could separate the
two. The unloaded reading came free: channel 1 is open on the configuration B bench, so a
two-channel voltage train measures loaded and unloaded at once.

## The smoke.py "intermittent failure" was not intermittent

Phase 017 recorded an unidentified check that failed twice in nine runs. It is
`trigger compensation is 0 until someone measures it`, and the pattern is deterministic:
`smoke.py` runs `CALDEF` a few checks later, which resets the whole runtime budget, so the first
run after anyone sets `CAL` fails that assertion and every run afterwards passes — because the
suite clobbered the thing it was asserting about. It reappeared the moment the reflash restored
`STARTLAT` 20 / `TRIGCOMP` 2 from EEPROM.

**A regression run must not be what silently un-calibrates an instrument.** `smoke.py` now takes a
copy of `CAL?` before the section, restores it after the `CALDEF` test in the order `CAL?` prints
them (`STARTLAT` before `TRIGCOMP`, the one ordering that cannot trip the validate invariant on the
way through), and checks that it left the budget as it found it. The `TRIGCOMP == 0` assertion is
replaced by a round-trip check that asserts no particular value.

Three other smoke checks assumed four measured lines on a `(voltage, current)` train and were
retargeted: the MRANGE bracket check now uses V0 and I1 and asserts I0 is empty, the rotation test
drives both channels in current mode so it still exercises the four-group path, and the
"readings actually varied" check applies only to the driven line — channel 1 is open on this bench,
so its current sits on one ADC code and min == max honestly.

## Tests

- `tests/host/test_measure.cpp`: new `testVoltageModeDropsCurrent` covers the rule (voltage +
  current, both voltage, voltage asking only for V, both current, and an undriven channel). The
  existing fit and rotation tests wanted four-read plans, which now means current mode on both
  channels, so `defaultDef(t, 0, 1)` became `defaultDef(t, 1, 1)` throughout.
- All five host suites pass; `smoke.py` passes; `trigcomp.py check` passes with the load reading
  991 Ω through current mode and no current in voltage mode.
- The firmware was rebuilt with `--warnings more` (clean) and uploaded. `IDN` reports `cal=custom`
  after the reflash, which incidentally proves the phase-017 EEPROM persistence survives a power
  cycle.

## Open

1. **`READ` and `E` still report the unusable current** outside current mode. Fixing it needs the
   output mode tracked in software.
2. **Nothing measures the load current in voltage mode**, and nothing on this board can — it is a
   hardware limitation, worth knowing before designing an experiment around voltage-mode
   stimulation with current monitoring.
3. Configuration C — dual-channel collision jitter and the real `SJ_FS_MAX_HZ` — is still unrun.

## Entry point

`python tests/device/trigcomp.py check` reads the load through the mode that can measure it and
fails if a current appears in voltage mode. `docs/hardware-notes.md` has the topology.
