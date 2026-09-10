# Plan — the end-of-pulse transient, and whether the park needs a dwell

Phase 021. A transient at the end of every pulse of a current-mode train into a large load.
This document is the investigation, not a fix: it records what the reported observations do and
do not establish, rules out five mechanisms arithmetically, names the two that survive, and
specifies the bench procedure that separates them. The firmware change is held until a number
decides its size.

## 1. The observation

A biphasic current train on CH0: 800 µA or 3000 µA, 500 µs each phase, 10000 µs period. Load a
10 kΩ and a 10 Ω in series. A brief voltage peak appears at the end of **every** pulse, in the
pause — not only at the end of the train. Two probe placements, both with the scope's ground at
CH0(−):

| Setup | Scope+ | Scope− | Reads | Peak reported |
| ----- | ------ | ------ | ----- | ------------- |
| 1 | node between the resistors | CH0(−) | the 10 Ω, i.e. load current | 150–500 mV, variable |
| 2 | CH0(+) | CH0(−) | the whole 10.010 kΩ, i.e. the output | ~2 V, on the settling arm |

Instrument: Voltcraft DSO-6104F (100 MHz). Two further facts from the same session:

- **A 0 mA lead-out stage removes the peak**, and the settling that remains is a clean
  exponential with no overshoot.
- At 3000 µA the output reaches only ~10 V and the current is below command. 3000 µA into
  10.010 kΩ needs 30 V; the compliance is ±13.7 V and this board's driver saturates near
  9–10 V ([hardware-notes.md](hardware-notes.md), known analog limitations), so it delivers
  ~1 mA and the pump sits **in saturation for the whole pulse**. Not a fault — the load's
  ceiling. The README's own figures agree: ±3.33 mA only into ≤4 kΩ.

The observer's recollection, explicitly offered as weak evidence: on the fine range needed to see
the 10 mV signal in setup 1, the peak was seen **capping** — flat against the screen limit. No
recollection of the same connection on a coarse (≈5 V) range. Section 3 argues this is probably
the whole of setup 1.

## 2. What the firmware does at the end of a pulse

Not high-Z. Two things happen back to back at the off event
([Engine.cpp:519-524](../stimjimAWG/Engine.cpp#L519-L524)):

1. `progLatch(..., off0, off1)` latches the calibrated zero code — the pump is commanded to
   ~0 µA;
2. `oeGround(pl)` ([Engine.cpp:294-298](../stimjimAWG/Engine.cpp#L294-L298)) writes mode 3,
   which on the DG409 shorts `CHANNEL_OUT` to the channel's ground and parks `I_OUT` on R14,
   the on-board 1 kΩ dummy.

The gap between them is one `dacLatch` (~0.44 µs) plus two `digitalWriteFast` calls. The mux
write comes first; `Triggers::marker(false)` follows it by a handful of instructions, which is
what makes the marker usable as a time reference (§6).

`oeGround` runs at every pulse's off event, which is why the transient repeats every period.
`groundClaimed` ([Engine.cpp:247-251](../stimjimAWG/Engine.cpp#L247-L251)) does the same at
train end and is reused by `stopTrain`.

Mode 1 → mode 3 changes only OE1, so there is no intermediate address. Mode 0 → 3 does pass
through 01 = current for one instruction, because `setOutputMode` writes OE0 first
([Stimjim.cpp:57-61](../lib/stimjim/src/Stimjim.cpp#L57-L61)) — irrelevant to a current-mode
train, but it would matter if a voltage-mode park were ever added.

## 3. Mechanisms ruled out, with the arithmetic

Each of these was proposed and each fails, most of them by orders of magnitude. They are recorded
because the failures are what leave §4 as the only place left to look.

**Nothing on the board holds charge at the output.** Checked in `PCB/channel.kicad_sch`, not
assumed: `CHANNEL_OUT` runs from the DG409 (U23) straight to the output connector J9 with no
component on it. The only 0.1 µF near J9 is **C50, and it sits between +15 V and GRND** — a
supply bypass. So the node's capacitance is board parasitics plus the user's wiring and probe,
and the question "what holds the charge" has the answer "nothing does". The pump actively drives
that node; reading the decay as a passive discharge was wrong.

**The exponential is the pump settling, not a discharge.** Consistent with
[hardware-notes.md:132](hardware-notes.md#L132): *"Into a 10 kΩ load the output converges more
slowly and measurements interfere with the load: use stages ≥ 100 µs and expect ~10 % error
(1 kΩ loads are fine down to ~10 µs)."* Phase 016's 1.91 µs settling does **not** bound this
case — that was voltage mode into 1 kΩ. Current-mode settling into 10 kΩ has never been measured
on this board.

**The pump is not the source of the overshoot.** The lead-out experiment separates the two events
in time: DAC to zero at *t*, mux to ground at *t* + lead-out. With them 100 µs apart the pump's
own step response is visible alone, and it is a clean exponential. So the loop is overdamped into
this load and cannot produce a superimposed spike. This is a deduction from the reported data,
not a model, and it is the strongest single result so far.

**Conduction through the chain cannot reach setup 1's amplitude.** Current through the 10 kΩ is at
most rail/10 kΩ ≈ 1.4 mA whatever happens upstream, so the 10 Ω can never show more than ~14 mV.
150 mV is 10× that ceiling and 500 mV is 35×. Independently: if the 2 V of setup 2 were conducted,
setup 1 should read 2 V × 10/10010 = **2 mV**.

**Capacitive coupling around the 10 kΩ does not rescue it.** The parasitic across a resistor and
its leads is ~1–3 pF, and node M is held at 10 Ω, so 2 pF × 20 V/µs × 10 Ω ≈ 0.4 mV. To reach
150 mV you would need 2 V in 0.27 ns, which no analog mux produces.

**Probe ground-lead inductance does not either.** Two distinct mechanisms, both far short:

- *Series L·di/dt in the measurement path.* A 15 cm ground clip lead is ~150 nH. For 150 mV you
  need di/dt = 1×10⁶ A/s — 200 mA developing in 200 ns — in that lead. What actually flows there
  is the probe's own input current (10 MΩ ∥ ~15 pF, so microamps) plus common-mode return through
  the isolation barrier. The output node's capacitance to earth is a series chain: node to the
  isolated ground plane (tens of pF) through the CC3-0512DF-E interwinding and ISO77xx
  capacitance (tens of pF), of order 10 pF overall. 10 pF × 100 V/µs = **1 µA**. Five orders
  short.
- *Magnetic pickup into the tip-plus-ground-lead loop.* The fastest current in the circuit is the
  output node's own capacitance discharging through the mux: Q ≈ 30 pF × 2 V = 60 pC in ~50 ns,
  about 1 mA. Mutual inductance to a probe loop centimetres away is a few nH at most:
  5 nH × 2×10⁴ A/s ≈ **0.1 mV**. Four orders short.

Both fail for the same reason the conducted story fails: **there is no large current anywhere in
this circuit.** The total charge moved at the pulse end is tens of picocoulombs.

**What that leaves for setup 1: the scope's input stage.** If the trace hit the screen limit, then
150 mV and 500 mV were never amplitude measurements — they were "off screen at whatever range was
set". A genuine ≤14 mV feature fills a ±20 mV screen and looks dramatic. That needs no new
physics, explains the 150 → 500 mV spread and the dependence on connection, and matches the
recollection of capping. It is the leading explanation, and §7 test A decides it.

**Setup 2 is the trustworthy number.** A 2 V feature on an 8–10 V trace is well inside range and
linear. All quantitative work belongs there.

## 4. What survives: two candidates, one discriminator

A 4:1 mux has to be break-before-make — otherwise switching `V_OUT` → `GRND` would short the
voltage amplifier to ground — so there is a window in which `CHANNEL_OUT` is connected to
nothing. (Exact t_open / t_transition need the DG409 datasheet, which is not in the repo. Do not
quote a number for it until it is.) Both candidates live in that window.

| | Mechanism | Predicts |
| --- | --- | --- |
| **C1** | **Plateau read as a bump.** The node stops being driven and holds for the open window. Against a still-descending exponential, a few-hundred-ns hold looks like a positive excursion. | The trace **never passes its final resting value**. Feature width = the open window. |
| **C2** | **Real injection into a floating node.** The mux's own charge injection, plus coupling through the opened switch's off-capacitance from a pump that has just lost both its load and its sense connection, lands charge on a node carrying only tens of pF. ΔV = Q/C reaches volts easily. | The trace **does pass its final value** and comes back. |

Both explain the lead-out result: with the node already at zero there is nothing to hold and
nothing unsettled to inject.

**The discriminator is whether the trace crosses past its final value.** Everything in §7 is built
around measuring that one thing, signed relative to the settled level rather than to the baseline.

Worth confirming while there: `CHANNEL_OUT` is wired directly to a pin of U1 (OPA197), so the
output node is sensed by an amplifier as well as driven through the mux. Which pin decides what
happens to that loop when the mux takes `CHANNEL_OUT` to ground — read it off
[PCB/stimjim.pdf](../PCB/stimjim.pdf). It was not determined here.

## 5. The measurement constraint that bit setup 1

The PicoScope 2000 series is **8 bit**: on ±10 V one code is 78 mV (the figure
`tests/device/trigcomp.py` already works around). So measuring a small feature that sits near 0 V
immediately after an 8 V pulse forces a choice, and both options are bad:

- a range that holds the pulse — poor resolution on the feature;
- a range that resolves the feature — the input clips for 500 µs, and what follows 1 µs later is
  overload recovery, the very confound suspected in setup 1.

**The way out is to shrink the signal, not the range.** At 150 µA into 10 kΩ the pulse is 1.50 V,
the ±2 V range gives 16 mV codes, and nothing clips. So the amplitude sweep (§7 test D) is not
only a mechanism test — it is how a clean measurement becomes possible at all. If the feature
survives down to 150 µA it can be measured properly; if it scales away, that is itself the answer.

150 and not 200 µA because a range step doubles the code size and the headroom has to clear the
pulse: 200 µA gives 2.00 V, which does not fit ±2 V and is forced up to ±5 V and 39 mV codes. For
the same reason `range_for` in the script keeps only 10 % headroom — at 25 % the 8.008 V baseline
pulse lands on ±20 V and every figure loses a bit of resolution for nothing.

Corollary: the Pico is the right instrument for **amplitudes and sweeps** — repeatable, scripted,
and its driver reports an overflow flag. The 100 MHz Voltcraft is the right instrument for
**width and shape**. Do not ask either to do the other's job.

## 6. Configuration D — wiring

```
    setup 2 (default)                      setup 1 (test A only)
    -----------------                      ---------------------
    CH0(+) --+-- 10k --+-- 10R --+         CH0(+) -- 10k --+-- 10R --+
             |         |         |                         |         |
          scope A   (node M)  CH0(-)                    scope A   CH0(-)
                              scope gnd                           scope gnd

    IN1 (BNC) ------------------------> scope B       (the stimulus marker)
```

CH1 unused. `TRIG1,3,-1,-1,0` makes IN1 an output driven high during each stimulus;
`TRIG0,0,-1,-1,0` makes IN0 a plain input, so the boot default does not leave a second marker
driving into whatever is on that BNC.

The marker's **falling** edge is the mux write, trailing it by a handful of instructions (§2) —
the time reference asked for. Trigger scope B falling at 1.65 V with half the block pre-trigger.

This becomes configuration D in [bench-wiring.md](bench-wiring.md) once it has actually been run.
It is kept here until then, because that document records what has been measured.

## 7. Procedure

`tests/device/parktransient.py` implements A–E. Each test says what each outcome would mean, so a
result that comes out the other way is still a result.

### Test A — is setup 1 measuring anything? (`ranges`)

Scope A on node M. Capture the same event at ±0.05, 0.1, 0.2, 0.5, 1, 5 V, reporting the peak
excursion in volts and the driver's own overflow flag at each.

- Amplitude **consistent in volts** across ranges and no overflow → the feature is real and its
  size is that consistent value. If it is ≤14 mV, there was never an anomaly.
- Amplitude **growing as the range narrows**, or overflow set → the input stage is saturating and
  the 150–500 mV figures should be struck from the record.

Two manual controls belong with it:

- **Short the probe.** Leave everything connected and the train running, and clip the probe tip
  onto its own ground clip, both at CH0(−) — the probe then reads zero by construction. Keep the
  volts/div at which the spike appeared. Anything still on screen is pickup or overload. Then coil
  the ground lead short and repeat: a residual that changes with lead dressing is magnetic pickup.
- **Reverse the probe** across the 10 Ω. A genuine differential signal inverts and keeps its
  amplitude. Caveat: this also moves which node is pinned to the scope's earth, from CH0(−) to
  node M, so an artifact can change amplitude arbitrarily instead of simply inverting. Clean
  inversion at equal amplitude ⇒ real; anything else ⇒ not.

### Test B — lead-out sweep (`sweep`)

Setup 2. Lead-out 0, 1, 2, 5, 10, 20, 50, 100 µs:
`S40,1,3,10000,<dur>; 800,0,500; -800,0,500` plus `; 0,0,<lead>` when non-zero. Per lead-out, N
shots triggered on the marker fall; report the residual just before the switch and the overshoot
past the final value.

- Residual versus lead-out **is** the pump's settling curve into this load — the number that does
  not exist for this board. Fit τ.
- Overshoot falling with the residual → the mechanism needs an unsettled node, consistent with
  both C1 and C2, and a dwell is the fix.
- Overshoot **independent** of the residual → neither candidate; reopen §4.

### Test C — polarity (`mono`)

Biphasic, monophasic positive, monophasic negative, all with lead-out 0.

- Overshoot sign **follows the last phase**, never crossing the final value → C1.
- Overshoot sign **fixed** regardless of the last phase → C2, charge injection.

### Test D — amplitude at fixed load (`amp`)

10 kΩ at 150, 400, 800 µA, lead-out 0 — 1.50 V on ±2 V, 4.00 V on ±5 V, 8.01 V on ±10 V, so the
code size falls with the signal instead of tracking it. Scaling with amplitude ⇒ driven by the
residual. Constant in absolute volts ⇒ driven by injected charge. This is also the test that buys
resolution (§5).

### Test E — larger resistances (`load`, one point per invocation)

Hold the output **voltage** constant and change the resistance, so what varies is the loop's
settling and not the size of the step:

| R | Amplitude for ~8 V | Note |
| --- | --- | --- |
| 10 kΩ | 800 µA | the baseline; 8.008 V across 10.010 kΩ |
| 100 kΩ | 80 µA | inside the ±137 µA compliance at 100 kΩ (README) |
| 1 MΩ | 8 µA | **optional.** 8 µA is ~79 DAC codes at 0.1017 µA/code, so offset and noise are a real fraction of the command. Indicative only. |

Run one point at a time, swapping resistors between invocations:

```
python parktransient.py load --rtop 10000  --rbot 10 --amp 800
python parktransient.py load --rtop 100000 --rbot 10 --amp 80
```

Each invocation appends a row to the same CSV. Keep the 10 Ω in place throughout so the chain and
its stray capacitance are otherwise unchanged; only the top resistor moves.

Prediction to test: settling into a larger load is slower, so the residual at a fixed lead-out
grows and, if the overshoot tracks the residual, it grows with it. A charge-injection overshoot
would instead stay put in absolute volts once a long lead-out has zeroed the residual.

### Test F — width and shape (manual, Voltcraft)

With the marker on the second channel and the fastest sweep the 100 MHz scope offers, measure the
feature's **width** and whether it passes the final value. A few hundred ns centred on the marker
edge is the mux window; µs-wide is not. The Pico cannot settle this — that is what §5 says.

## 8. Decision rule for the firmware

If tests B–D confirm that the mux switches while the node is unsettled, the change is a **park
dwell** — not the settable park *mode* discussed before the evidence came in. A mode that holds
the drive mode at zero never grounds at all, which is worse: it leaves the node to leakage. What
is wanted is the lead-out's behaviour — discharge, then ground — done by the firmware so it also
covers `L` and `W` and costs no user stage.

Sketch, unchanged except in how its size is chosen:

- a new `EV_PARK` phase after `EV_OFF` in each type's state machine, deadline = the off deadline
  plus the dwell; `oeGround` moves there. One extra scheduled event per pulse.
- a parse-time check that the dwell fits before the next pulse's first latch, the same shape as
  the existing short-stage `WARN`.
- `PARK <ch>,<dwell_us>`, default **0** = today's behaviour, persisted → EEPROM v7.

**Its size comes from test B, not from arithmetic.** The dwell has to cover the pump's settling
into the load in front of it, which is a property of the bench and not of the board — hence a
runtime setting with a zero default, and hence a value correct for a 10 kΩ bench that will be
wrong for a 1 kΩ one. An earlier "measure τ = R·C and use 5–7 τ" rule was built on the discharge
story of §3 and is withdrawn.

If test B shows the overshoot does **not** track the residual, no dwell will help and this section
is void.

## 9. What is still unknown

- Whether setup 1 ever measured a real quantity (test A).
- Whether the feature passes its final value (tests C, F) — the C1/C2 discriminator.
- The DG409's open-window duration: no datasheet in the repo.
- Which U1 pin `CHANNEL_OUT` reaches, and therefore what the sense loop does at the switch.
- Current-mode settling into 10 kΩ, or into anything (test B produces the first figure).
- Whether any of this matters outside a bench resistor. The transient's charge is
  Q = C·ΔV ≈ 30 pF × 2 V ≈ 60 pC against a phase charge of 800 µA × 500 µs = 400 nC — about
  1.5×10⁻⁴ of what is delivered. It is a coupling and EMI concern next to recording electronics,
  not a stimulation one. Worth fixing; not worth alarm.
