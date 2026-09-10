"""Configuration D: the transient at the end of a current-mode pulse.

At the end of every pulse the firmware latches the calibrated zero code and
then, under a microsecond later, writes output mode 3 -- which shorts
CHANNEL_OUT to the channel's ground and parks the pump on the on-board 1 kOhm
dummy (Engine.cpp:519-524, Engine.cpp:294-298). Into a large load the node has
not finished settling when that happens, and a brief feature appears on the
settling arm. A 0 mA lead-out stage makes it go away.

This script measures the feature instead of arguing about it. The whole design
turns on one quantity:

    residual   the level shortly before the mux write, relative to where the
               trace finally rests -- how unsettled the node was
    overshoot  how far the trace goes PAST that resting level, on the far side

`overshoot` is the discriminator between the two surviving mechanisms
(docs/PLAN_park-transient.md section 4). If the node merely stops being driven
during the mux's break-before-make window, the trace holds and then drops: it
never passes its final value, and an apparent bump is the held level read
against a still-descending exponential. If charge is injected into the floating
node, the trace does pass its final value and comes back. Nothing else in the
firmware or the wiring separates those two.

Wiring (docs/PLAN_park-transient.md section 6):

    setup 2 (default here)                 setup 1 (`ranges` only)
    CH0(+) --+-- Rtop --+-- Rbot --+       CH0(+) -- Rtop --+-- Rbot --+
             |          |          |                        |          |
          scope A   (node M)    CH0(-)                   scope A    CH0(-)
                               scope gnd                            scope gnd

    IN1 (BNC) ------------------------> scope B      (the stimulus marker)

The marker is the time reference. `TRIG1,3,-1,-1,0` drives IN1 high during each
stimulus, and `Triggers::marker(false)` is called inside `oeGround` a handful of
instructions after the mux write -- so the marker's FALLING edge trails the
event by a few hundred nanoseconds at most, and the feature sits just before it.
IN0 is set to mode 0 so the boot default does not leave a second marker driving
into whatever is on that BNC.

    python parktransient.py check                     verify the wiring
    python parktransient.py sweep                     test B: lead-out sweep
    python parktransient.py mono                      test C: polarity
    python parktransient.py amp                       test D: amplitude
    python parktransient.py ranges --confirm-node-m   test A: is setup 1 real?
    python parktransient.py load --rtop 100000 --amp 80    test E, one point
    python parktransient.py all                       check + sweep + mono + amp

Slots 40/41 and trigger input 1 are borrowed and released at the end. Figures go
to figs/, per-shot and per-point data to tmp/.

The Pico is 8 bit: on +/-10 V one code is 78 mV. That is why `amp` matters for
more than the mechanism -- at 150 uA into 10 kOhm the whole pulse fits the
+/-2 V range with 16 mV codes and nothing clips. Widths and shapes need the
100 MHz scope, not this one.
"""

import argparse
import csv
import pathlib
import statistics
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import pico2000 as ps
from sjcon import StimJim

ROOT = pathlib.Path(__file__).resolve().parents[2]
FIGS = ROOT / "figs"
TMP = ROOT / "tmp"

SLOT = 40             # the biphasic train under test
SLOT_LOAD = 41        # the train that characterises the load
TRIG_IN = 1           # IN1 carries the marker; IN0 is parked as a plain input

PERIOD_US = 10000     # the user's train: 500 us per phase, 10 ms period
PHASE_US = 500
RUN_US = 20_000_000   # 20 s of train -- long enough for a shot series
AMP_UA = 800          # the working amplitude
RTOP, RBOT = 10_000.0, 10.0

MARKER_LEVEL = 1.65   # half of the Teensy's 3.3 V logic swing
B_RANGE = 5.0         # holds the marker with room for overshoot

N_SAMPLES = 4000      # per capture, both channels
WANT_DT = 20e-9       # fastest interval the driver offers with two channels
N_SHOTS = 10

# The residual is read from a window this far before the marker's falling edge,
# in samples. It has to clear the marker's own lag behind the mux write (a few
# hundred ns) without straying so far back that the settling has moved much.
PRE_LO, PRE_HI = 25, 20
# The excursion is looked for from just before the edge to this far after.
POST = 100


# ------------------------------------------------------------- edge finding

def cross_down(v, level, i0=1):
    """Index (fractional) of the first downward crossing of `level`."""
    for i in range(max(i0, 1), len(v)):
        if v[i - 1] >= level > v[i]:
            span = v[i - 1] - v[i]
            return (i - 1) + (0.0 if span == 0 else (v[i - 1] - level) / span)
    return None


def cross_up(v, level, i0=1):
    """Index (fractional) of the first upward crossing of `level`."""
    for i in range(max(i0, 1), len(v)):
        if v[i - 1] < level <= v[i]:
            span = v[i] - v[i - 1]
            return (i - 1) + (0.0 if span == 0 else (level - v[i - 1]) / span)
    return None


# ------------------------------------------------------------ the shot itself

def range_for(volts):
    """Smallest driver range that holds `volts` with 10 % headroom.

    The headroom is deliberately thin. On an 8-bit unit one range step doubles
    the code size, so 25 % headroom would push an 8.008 V pulse from +/-10 V
    (78 mV codes) to +/-20 V (156 mV) and halve the resolution of the thing
    being measured. 10 % still clears the pulse's own overshoot.
    """
    want = abs(volts) * 1.10
    for r in sorted(ps.RANGE):
        if r >= want:
            return r
    return max(ps.RANGE)


def arm_and_capture(scope, a_range, n=N_SAMPLES, tb=None):
    """One capture triggered on the marker's falling edge.

    Half the block is pre-trigger, so the settling arm that leads into the mux
    write is in the record along with what follows it.
    """
    scope.channel(ps.CHANNEL_A, volts=a_range)
    scope.channel(ps.CHANNEL_B, volts=B_RANGE)
    if tb is None:
        tb, _, _ = scope.pick_timebase(WANT_DT, n)
    scope.trigger(ps.CHANNEL_B, MARKER_LEVEL, ps.FALLING, delay_pct=-50,
                  auto_ms=0)
    dt, ch, overflow = scope.block(n, tb, timeout=8)
    return dt, ch[ps.CHANNEL_A], ch[ps.CHANNEL_B], overflow, tb


def analyse(a, b, dt):
    """Residual and overshoot around the marker's falling edge.

    Signs are relative to the trace's own resting level, not to zero, so the
    numbers mean the same thing whichever phase ran last:

      residual   > 0 when the node had not yet reached its resting level
      overshoot  > 0 when the trace passed that level on the far side

    With the residual at zero -- a long lead-out -- there is no "far side" and
    `overshoot` degenerates to whichever side the sign test happened to pick.
    `peak_from_final` is the two-sided excursion and stays meaningful there.

    Returns None when the marker edge is not in the record, which is how a
    mis-triggered shot is discarded rather than averaged in.
    """
    edge = cross_down(b, MARKER_LEVEL, i0=len(b) // 8)
    if edge is None:
        return None
    e = int(edge)
    if e - PRE_LO < 8 or e + POST >= len(a):
        return None

    # Where the trace ends up: the node is grounded by then, so this is the
    # zero the excursion is measured against. Taken from the tail, well clear
    # of the event.
    final = statistics.fmean(a[int(0.80 * len(a)):])
    before = statistics.fmean(a[e - PRE_LO:e - PRE_HI])
    residual = before - final

    # The window starts before the mux write, which leads the marker's fall by
    # a few hundred ns. Including the pre-event samples cannot corrupt the
    # overshoot: they lie on the residual's side of `final`, and the overshoot
    # is the extreme on the *other* side.
    win = a[e - PRE_LO:e + POST]
    if residual >= 0:
        # The node was above its resting level; an overshoot goes below it.
        overshoot = final - min(win)
    else:
        overshoot = max(win) - final
    return {
        "edge": edge, "dt": dt, "final": final, "before": before,
        "residual": residual, "abs_residual": abs(residual),
        "overshoot": overshoot,
        "peak_from_final": max(abs(x - final) for x in win),
        "peak_from_before": max(abs(x - before) for x in win),
    }


def shots(scope, a_range, n_shots=None, tb=None):
    """A series of captures. Returns (rows, one representative trace, tb).

    `n_shots` defaults to the module-level N_SHOTS at *call* time, not at def
    time, so --shots reaches it.
    """
    rows, keep = [], None
    for _ in range(n_shots or N_SHOTS):
        dt, a, b, overflow, tb = arm_and_capture(scope, a_range, tb=tb)
        m = analyse(a, b, dt)
        if m is None:
            continue
        m["overflow"] = overflow
        rows.append(m)
        if keep is None:
            keep = (dt, a, b)
    assert rows, "no shot caught the marker edge -- is scope B on IN1?"
    return rows, keep, tb


def summarise(rows):
    def med(key):
        return statistics.median([r[key] for r in rows])
    return {
        "n": len(rows),
        "residual": med("residual"),
        "overshoot": med("overshoot"),
        "overshoot_sd": (statistics.pstdev([r["overshoot"] for r in rows])
                         if len(rows) > 1 else 0.0),
        "peak_from_final": med("peak_from_final"),
        "peak_from_before": med("peak_from_before"),
        "final": med("final"),
        "overflow": any(r["overflow"] for r in rows),
    }


# Column order for the CSVs: whatever identifies the point, then the numbers.
CSV_TAIL = ["n", "residual", "overshoot", "overshoot_sd", "peak_from_final",
            "peak_from_before", "final", "overflow"]


def write_csv(path, rows, head, append=False):
    """Write summary rows with `head` columns first, then CSV_TAIL."""
    fields = head + CSV_TAIL
    new = append and not path.exists()
    with open(path, "a" if append else "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fields)
        if new or not append:
            w.writeheader()
        for s in rows:
            w.writerow({k: s[k] for k in fields})
    print(f"{'appended to' if append else 'wrote'} {path}")


# ----------------------------------------------------------- train plumbing

def define(sj, slot, amp_ua, lead_us, phases=("+", "-")):
    """Write the train. `phases` selects biphasic, mono-positive, mono-neg."""
    stages = []
    for p in phases:
        stages.append(f"{amp_ua if p == '+' else -amp_ua},0,{PHASE_US}")
    if lead_us:
        stages.append(f"0,0,{lead_us}")
    line = (f"S{slot},1,3,{PERIOD_US},{RUN_US};" + ";".join(stages))
    r = sj.cmd1(line)
    assert r.startswith(f"S{slot}"), f"{line!r} -> {r!r}"
    return line


def with_train(sj, slot, body):
    """Run `body()` with the train playing, and always stop it afterwards."""
    sj.cmd(f"T{slot}", quiet=0.06)
    try:
        return body()
    finally:
        sj.cmd("T-1", quiet=0.3)
        sj.drain(quiet=0.4, limit=4.0)


def borrow_triggers(sj):
    """Park IN0, make IN1 the marker. Returns the lines to restore."""
    saved = [sj.cmd1(f"TRIG{t}?") for t in (0, 1)]
    sj.cmd1("TRIG0,0,-1,-1,0")
    sj.cmd1(f"TRIG{TRIG_IN},3,-1,-1,0")
    return saved


def restore_triggers(sj, saved):
    for line in saved:
        sj.cmd(line, quiet=0.2)


# ------------------------------------------------------------------- check

def check(scope, sj, rtop, rbot):
    """Verify each wire separately, so a failure names what to look at."""
    rload = rtop + rbot
    expect_v = AMP_UA * 1e-6 * rload
    a_range = range_for(expect_v)
    print(f"expecting {expect_v:.3f} V across {rload:.0f} ohm "
          f"at {AMP_UA} uA; scope A on +/-{a_range} V")

    define(sj, SLOT, AMP_UA, lead_us=0)

    # 1  the marker reaches scope B, with the shape the train has
    def grab():
        scope.channel(ps.CHANNEL_A, volts=a_range)
        scope.channel(ps.CHANNEL_B, volts=B_RANGE)
        tb, dt_nom, _ = scope.pick_timebase(2e-6, N_SAMPLES)
        scope.trigger(ps.CHANNEL_B, MARKER_LEVEL, ps.RISING, delay_pct=-10,
                      auto_ms=4000)
        return scope.block(N_SAMPLES, tb, timeout=10)

    dt, ch, _ = with_train(sj, SLOT, grab)
    a, b = ch[ps.CHANNEL_A], ch[ps.CHANNEL_B]
    assert max(b) - min(b) > 2.0, \
        f"1 FAIL no marker on scope B (swing {max(b) - min(b):.2f} V) -- " \
        f"scope B on IN1? TRIG{TRIG_IN} mode 3?"
    rise = cross_up(b, MARKER_LEVEL)
    fall = cross_down(b, MARKER_LEVEL, i0=int(rise) + 2)
    high_us = (fall - rise) * dt * 1e6
    assert abs(high_us - 2 * PHASE_US) < 50, \
        f"1 FAIL marker high for {high_us:.1f} us, expected {2 * PHASE_US}"
    print(f"1 OK   marker on B, high {high_us:.1f} us")

    # 2  the output reaches scope A, right way up and full amplitude
    swing = max(a) - min(a)
    assert swing > 1.5 * expect_v, \
        f"2 FAIL scope A swing {swing:.2f} V, expected about " \
        f"{2 * expect_v:.2f} V -- is A on CH0(+) and gnd on CH0(-)? " \
        f"A tapping node M would read {expect_v * rbot / rload:.4f} V"
    print(f"2 OK   output on A, swing {swing:.2f} V (expect "
          f"{2 * expect_v:.2f})")

    # 3  the marker's falling edge really is the end of the pulse
    #    (this is the claim the whole measurement rests on)
    base = statistics.fmean(a[:max(8, int(0.05 * len(a)))])
    lo = min(a)
    back = cross_up(a, base - 0.5 * (base - lo), i0=int(rise) + 2)
    assert back is not None, "3 FAIL A never returns towards baseline"
    skew_us = (back - fall) * dt * 1e6
    assert abs(skew_us) < 5.0, \
        f"3 FAIL marker fall and output return differ by {skew_us:.2f} us"
    print(f"3 OK   marker fall tracks the pulse end to {skew_us:+.2f} us")

    # 4  the load is a plain resistor, anchored by the scope and not by the
    #    board's own readback -- current mode, because in voltage mode there is
    #    no shunt in the output path at all (docs/hardware-notes.md).
    define(sj, SLOT_LOAD, AMP_UA, lead_us=0, phases=("+",))

    def grab2():
        scope.channel(ps.CHANNEL_A, volts=a_range)
        scope.channel(ps.CHANNEL_B, volts=B_RANGE)
        tb, _, _ = scope.pick_timebase(2e-6, N_SAMPLES)
        scope.trigger(ps.CHANNEL_B, MARKER_LEVEL, ps.RISING, delay_pct=-10,
                      auto_ms=4000)
        return scope.block(N_SAMPLES, tb, timeout=10)

    dt2, ch2, _ = with_train(sj, SLOT_LOAD, grab2)
    a2, b2 = ch2[ps.CHANNEL_A], ch2[ps.CHANNEL_B]
    r2 = cross_up(b2, MARKER_LEVEL)
    f2 = cross_down(b2, MARKER_LEVEL, i0=int(r2) + 2)
    # the middle half of the pulse, clear of both edges and of settling
    i0, i1 = int(r2 + 0.25 * (f2 - r2)), int(r2 + 0.75 * (f2 - r2))
    plateau = statistics.fmean(a2[i0:i1])
    r_meas = plateau / (AMP_UA * 1e-6)
    err = (r_meas - rload) / rload
    assert abs(err) < 0.10, \
        f"4 FAIL load reads {r_meas:.0f} ohm, expected {rload:.0f} " \
        f"({err * 100:+.1f} %) -- contact, or the wrong resistor"
    print(f"4 OK   load {r_meas:.0f} ohm vs {rload:.0f} nominal "
          f"({err * 100:+.1f} %)")
    print("check: all four passed")


# --------------------------------------------------------- test B: lead-out

LEADS_US = (0, 1, 2, 5, 10, 20, 50, 100)


def sweep(scope, sj, rtop, rbot, amp_ua=AMP_UA, leads=LEADS_US, tag="sweep"):
    rload = rtop + rbot
    a_range = range_for(amp_ua * 1e-6 * rload)
    out, traces, tb = [], {}, None
    for lead in leads:
        define(sj, SLOT, amp_ua, lead_us=lead)
        rows, keep, tb = with_train(
            sj, SLOT, lambda: shots(scope, a_range, tb=tb))
        s = summarise(rows)
        s["lead_us"] = lead
        out.append(s)
        traces[lead] = keep
        print(f"lead {lead:4d} us   residual {s['residual'] * 1e3:+8.1f} mV   "
              f"overshoot {s['overshoot'] * 1e3:+8.1f} mV "
              f"(sd {s['overshoot_sd'] * 1e3:.1f})   n={s['n']}"
              + ("   OVERFLOW" if s["overflow"] else ""))

    write_csv(TMP / f"parktransient-{tag}.csv", out, ["lead_us"])

    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    leads_us = [s["lead_us"] for s in out]
    ax[0].plot(leads_us, [abs(s["residual"]) * 1e3 for s in out], "o-",
               label="|residual| before the mux write")
    ax[0].plot(leads_us, [s["overshoot"] * 1e3 for s in out], "s-",
               label="overshoot past the resting level")
    ax[0].set_xlabel("lead-out stage (us)")
    ax[0].set_ylabel("mV")
    ax[0].set_xscale("symlog", linthresh=1)
    ax[0].axhline(0, lw=0.6, color="k")
    ax[0].legend(fontsize=8)
    ax[0].set_title(f"{amp_ua} uA into {rload:.0f} ohm")

    for lead in (leads[0], leads[len(leads) // 2], leads[-1]):
        dt, a, b = traces[lead]
        t = [(i - len(a) / 2) * dt * 1e6 for i in range(len(a))]
        ax[1].plot(t, a, lw=0.8, label=f"lead-out {lead} us")
    ax[1].set_xlabel("us relative to the marker's falling edge")
    ax[1].set_ylabel("output (V)")
    ax[1].set_xlim(-20, 20)
    ax[1].axvline(0, lw=0.6, color="k")
    ax[1].legend(fontsize=8)
    ax[1].set_title("the settling arm and the event")
    fig.tight_layout()
    path = FIGS / f"parktransient-{tag}.png"
    fig.savefig(path, dpi=130)
    plt.close(fig)
    print(f"wrote {path}")
    return out


# --------------------------------------------------------- test C: polarity

def mono(scope, sj, rtop, rbot, amp_ua=AMP_UA):
    """Does the overshoot's sign follow the last phase, or is it fixed?"""
    rload = rtop + rbot
    a_range = range_for(amp_ua * 1e-6 * rload)
    cases = (("biphasic +-", ("+", "-")),
             ("mono +", ("+",)),
             ("mono -", ("-",)))
    out, tb = [], None
    for name, phases in cases:
        define(sj, SLOT, amp_ua, lead_us=0, phases=phases)
        rows, keep, tb = with_train(
            sj, SLOT, lambda: shots(scope, a_range, tb=tb))
        s = summarise(rows)
        s["case"] = name
        # The residual's own sign says which side the node was on; the
        # overshoot is already signed relative to the resting level.
        s["residual_sign"] = "+" if s["residual"] > 0 else "-"
        out.append(s)
        print(f"{name:12s} residual {s['residual'] * 1e3:+8.1f} mV   "
              f"overshoot {s['overshoot'] * 1e3:+8.1f} mV "
              f"(sd {s['overshoot_sd'] * 1e3:.1f})"
              + ("   OVERFLOW" if s["overflow"] else ""))

    # The floor is quantisation, not shot-to-shot spread: on +/-10 V one code
    # is 78 mV, and ten identical shots can have an sd near zero, so a
    # threshold built from the sd alone would call one code an injection.
    code = a_range / 128
    print(f"(one code is {code * 1e3:.1f} mV; an overshoot has to clear that "
          "to mean anything)")
    crosses = [s for s in out
               if s["overshoot"] > max(3 * s["overshoot_sd"], code)]
    if not crosses:
        print("verdict: no case passes its resting level -> C1 "
              "(the node holds during the mux's open window)")
    else:
        signs = {s["residual_sign"] for s in crosses}
        print(f"verdict: {len(crosses)} of {len(out)} cases pass the resting "
              f"level, residual signs {sorted(signs)} -> C2 "
              "(charge injected into the floating node)")

    write_csv(TMP / "parktransient-mono.csv", out, ["case", "residual_sign"])
    return out


# -------------------------------------------------------- test D: amplitude

def amp(scope, sj, rtop, rbot, amps=(150, 400, 800)):
    """Does the feature scale with the step, or sit at a fixed charge?

    Also the test that buys resolution: the small amplitudes fit a small range
    with nothing clipping, which is the only clean measurement on an 8-bit
    scope (docs/PLAN_park-transient.md section 5). 150 uA into 10 kOhm is
    1.50 V, which is what it takes to fit the +/-2 V range and its 16 mV
    codes -- 200 uA would give 2.00 V and be forced up to +/-5 V.
    """
    rload = rtop + rbot
    out = []
    for a_ua in amps:
        a_range = range_for(a_ua * 1e-6 * rload)
        define(sj, SLOT, a_ua, lead_us=0)
        rows, _, _ = with_train(
            sj, SLOT, lambda: shots(scope, a_range))
        s = summarise(rows)
        s["amp_ua"] = a_ua
        s["a_range"] = a_range
        s["code_mv"] = a_range / 128 * 1e3      # 8 bit over +/- full scale
        out.append(s)
        print(f"{a_ua:4d} uA (+/-{a_range} V, {s['code_mv']:.0f} mV codes)   "
              f"residual {s['residual'] * 1e3:+8.1f} mV   "
              f"overshoot {s['overshoot'] * 1e3:+8.1f} mV"
              + ("   OVERFLOW" if s["overflow"] else ""))

    write_csv(TMP / "parktransient-amp.csv", out,
              ["amp_ua", "a_range", "code_mv"])
    return out


# ------------------------------------------------------ test A: is it real?

RANGES = (0.05, 0.1, 0.2, 0.5, 1.0, 5.0)


def ranges(scope, sj, rtop, rbot, amp_ua=AMP_UA):
    """Same event, six vertical ranges. Scope A must be on node M.

    An amplitude that is consistent in volts across ranges, with no overflow,
    is a real feature of that node. One that grows as the range narrows -- or
    any range that sets the overflow flag -- is the input stage saturating, and
    then the number was never a measurement.
    """
    rload = rtop + rbot
    expect_mv = amp_ua * 1e-6 * rbot * 1e3
    bound_mv = 14.0 / rtop * rbot * 1e3   # rail/Rtop, dropped across Rbot
    print(f"node M carries {expect_mv:.1f} mV of signal; nothing conducted "
          f"through {rtop:.0f} ohm can exceed about {bound_mv:.0f} mV")

    define(sj, SLOT, amp_ua, lead_us=0)
    out, traces, tb = [], {}, None
    for r in RANGES:
        rows, keep, tb = with_train(
            sj, SLOT, lambda: shots(scope, r, tb=tb))
        s = summarise(rows)
        s["a_range"] = r
        s["code_mv"] = r / 128 * 1e3
        out.append(s)
        traces[r] = keep
        print(f"+/-{r:<5} V ({s['code_mv']:6.2f} mV codes)   "
              f"peak from baseline {s['peak_from_before'] * 1e3:8.1f} mV   "
              f"overshoot {s['overshoot'] * 1e3:+8.1f} mV"
              + ("   OVERFLOW" if s["overflow"] else ""))

    clean = [s for s in out if not s["overflow"]]
    if len(clean) >= 2:
        peaks = [s["peak_from_before"] for s in clean]
        spread = (max(peaks) - min(peaks)) / max(max(peaks), 1e-9)
        print(f"\nnon-overflowing ranges agree to {spread * 100:.0f} % "
              f"({min(peaks) * 1e3:.1f}-{max(peaks) * 1e3:.1f} mV)")
        if spread < 0.3:
            print("verdict: consistent -> the feature is real at "
                  f"{statistics.median(peaks) * 1e3:.1f} mV"
                  + ("  (inside the conduction bound)"
                     if statistics.median(peaks) * 1e3 <= bound_mv else
                     "  (ABOVE the conduction bound -- unexplained)"))
        else:
            print("verdict: range-dependent -> the input stage, not the "
                  "signal. The 150-500 mV figures are not measurements.")
    if any(s["overflow"] for s in out):
        bad = [s["a_range"] for s in out if s["overflow"]]
        print(f"overflow set on ranges {bad} -- clipped, so any amplitude "
              "read there is meaningless")

    write_csv(TMP / "parktransient-ranges.csv", out, ["a_range", "code_mv"])

    fig, ax = plt.subplots(figsize=(7, 4))
    for r in RANGES:
        dt, a, b = traces[r]
        t = [(i - len(a) / 2) * dt * 1e6 for i in range(len(a))]
        ax.plot(t, [x * 1e3 for x in a], lw=0.8, label=f"+/-{r} V")
    ax.set_xlabel("us relative to the marker's falling edge")
    ax.set_ylabel("node M (mV)")
    ax.set_xlim(-10, 10)
    ax.axhline(0, lw=0.6, color="k")
    ax.axvline(0, lw=0.6, color="k")
    ax.legend(fontsize=8, title="scope range")
    ax.set_title("the same event on six ranges (setup 1)")
    fig.tight_layout()
    path = FIGS / "parktransient-ranges.png"
    fig.savefig(path, dpi=130)
    plt.close(fig)
    print(f"wrote {path}")
    return out


# ----------------------------------------------------------- test E: loads

def load_point(scope, sj, rtop, rbot, amp_ua):
    """One load/amplitude point, appended to a CSV that spans invocations."""
    rload = rtop + rbot
    a_range = range_for(amp_ua * 1e-6 * rload)
    print(f"{rload:.0f} ohm at {amp_ua} uA -> "
          f"{amp_ua * 1e-6 * rload:.2f} V, scope A on +/-{a_range} V")
    out = []
    for lead in (0, 100):
        define(sj, SLOT, amp_ua, lead_us=lead)
        rows, _, _ = with_train(sj, SLOT, lambda: shots(scope, a_range))
        s = summarise(rows)
        s.update({"rtop": rtop, "rbot": rbot, "amp_ua": amp_ua,
                  "lead_us": lead, "volts": amp_ua * 1e-6 * rload})
        out.append(s)
        print(f"  lead {lead:3d} us   residual {s['residual'] * 1e3:+8.1f} mV"
              f"   overshoot {s['overshoot'] * 1e3:+8.1f} mV"
              + ("   OVERFLOW" if s["overflow"] else ""))

    write_csv(TMP / "parktransient-load.csv", out,
              ["rtop", "rbot", "amp_ua", "volts", "lead_us"], append=True)
    return out


# -------------------------------------------------------------------- main

def main(argv):
    global N_SHOTS
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("what", choices=["check", "sweep", "mono", "amp", "ranges",
                                    "load", "all"])
    p.add_argument("--port", default="COM4")
    p.add_argument("--rtop", type=float, default=RTOP,
                   help="resistor between CH0(+) and node M")
    p.add_argument("--rbot", type=float, default=RBOT,
                   help="resistor between node M and CH0(-)")
    p.add_argument("--amp", type=int, default=AMP_UA, help="microamps")
    p.add_argument("--shots", type=int, default=N_SHOTS)
    p.add_argument("--confirm-node-m", action="store_true",
                   help="required by `ranges`: scope A has been moved to "
                        "node M (setup 1), not CH0(+)")
    a = p.parse_args(argv)

    N_SHOTS = a.shots
    FIGS.mkdir(exist_ok=True)
    TMP.mkdir(exist_ok=True)

    if a.what == "ranges" and not a.confirm_node_m:
        p.error("`ranges` measures node M (setup 1). Move scope A from "
                "CH0(+) to the junction of the two resistors, leave the "
                "ground on CH0(-), then pass --confirm-node-m.")

    with ps.Scope() as scope, StimJim(a.port) as sj:
        print({k: v for k, v in scope.probe().items()
               if k in ("variant", "serial")})
        sj.reset()
        saved = borrow_triggers(sj)
        try:
            if a.what in ("check", "all"):
                check(scope, sj, a.rtop, a.rbot)
            if a.what in ("sweep", "all"):
                print("\n--- test B: lead-out sweep")
                sweep(scope, sj, a.rtop, a.rbot, a.amp)
            if a.what in ("mono", "all"):
                print("\n--- test C: polarity")
                mono(scope, sj, a.rtop, a.rbot, a.amp)
            if a.what in ("amp", "all"):
                print("\n--- test D: amplitude")
                amp(scope, sj, a.rtop, a.rbot)
            if a.what == "ranges":
                print("\n--- test A: is setup 1 measuring anything?")
                ranges(scope, sj, a.rtop, a.rbot, a.amp)
            if a.what == "load":
                print("\n--- test E: one load point")
                load_point(scope, sj, a.rtop, a.rbot, a.amp)
        finally:
            sj.reset()
            restore_triggers(sj, saved)
            scope.siggen_off()


if __name__ == "__main__":
    main(sys.argv[1:])
