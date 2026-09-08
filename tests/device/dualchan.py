"""Configuration C: the two channels compared against each other.

Wiring (docs/bench-wiring.md, configuration C):

        CH0(+) ---- 1k ---- CH0(-) ---+
          |                           |
       scope A                        +--- scope gnd
                                      |
        CH1(+) ---- 1k ---- CH1(-) ---+

    scope B on CH1(+)
    PicoScope AWG ---> StimJim trigger input IN0

Tying CH0(-) and CH1(-) together defeats the channel-to-channel isolation,
which is what makes a two-channel comparison possible and why it is a separate
configuration. Do not use this wiring with anything connected to a preparation.

Three questions live on this bench, and one application that exercises all of
them at once:

  check     each connection verified on its own, so a failure names a wire
  c1        do both channels of *one* train step together, and how far apart
            do two independently routed trains land?
  c2        how fast can the sine generator sample both channels before a
            latch misses its deadline?
  usecase   a biphasic current stimulus on CH0 and a gate burst on CH1,
            delayed half a second, started by one trigger edge -- with the
            gate driven first in voltage and then in current mode

Slots 30-36 and trigger input 0 are borrowed and released at the end.
`CAL STARTLAT` is raised for the two-engine routes and put back; nothing is
persisted, so a power cycle restores the board's EEPROM values regardless.

Figures go to figs/, raw samples and per-shot data to tmp/.
"""

import argparse
import csv
import pathlib
import statistics
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import pico2000 as ps
from sjcon import StimJim
from trigcomp import cross, step_times

ROOT = pathlib.Path(__file__).resolve().parents[2]
FIGS = ROOT / "figs"
TMP = ROOT / "tmp"

SLOT_JOINT = 30      # one train, both channels
SLOT_CH0 = 31        # single-channel train on CH0
SLOT_CH1 = 32        # single-channel train on CH1
SLOT_SINE = 33       # the sine sweep
SLOT_STIM = 34       # use case: biphasic current on CH0
SLOT_GATE = 35       # use case: the gate burst on CH1
SLOT_LOAD = 36       # the load characterisation

AWG_VPP = 2.0        # 0..2 V into IN0, the amplitude every other script uses
STARTLAT_TWO = 35    # what an independent (two-engine) route needs
# How far apart C1's control puts the two engines' latches. Far enough that
# neither waits for the other, near enough that both steps are in one 79 us
# capture with ~1000 samples of plateau on each side of the pair.
OFFSET_US = 20

# Which scope channel watches which StimJim channel.
SCOPE = {0: ps.CHANNEL_A, 1: ps.CHANNEL_B}
NAME = {ps.CHANNEL_A: "A: StimJim CH0", ps.CHANNEL_B: "B: StimJim CH1"}


# ------------------------------------------------------------------- helpers

def crossings(v, level, rising=True):
    """Every crossing of `level` in one direction, as fractional indices.

    After a hit the scan skips the whole run on the far side of the level, so
    a noisy plateau cannot produce a burst of crossings; the signals here step
    by volts against 39 mV codes, so a plain level needs no hysteresis.
    """
    out, i = [], 1
    while i < len(v):
        hit = (v[i - 1] < level <= v[i]) if rising else (v[i - 1] > level >= v[i])
        if not hit:
            i += 1
            continue
        span = v[i] - v[i - 1]
        out.append((i - 1) + (level - v[i - 1]) / span)
        j = i + 1
        while j < len(v) and ((v[j] > level) if rising else (v[j] < level)):
            j += 1
        i = j
    return out


def startlat(sj, us=None):
    """Read `CAL STARTLAT`, or set it, and return what it was before."""
    old = int(sj.cmd1("CAL,STARTLAT?").split(",")[2])
    if us is not None and us != old:
        sj.cmd1(f"CAL,STARTLAT,{us}")
    return old


def train_line(kind, slot, ch, mode, header, stages):
    """A one-channel definition line; `stages` are (amplitude, duration)."""
    m = [3, 3]
    m[ch] = mode
    body = ""
    for amp, dur in stages:
        a = [0, 0]
        a[ch] = amp
        body += f";{a[0]},{a[1]},{dur}"
    return f"{kind}{slot},{m[0]},{m[1]},{header}{body}"


def faults(lines):
    """The firmware's own verdict on a completed train."""
    return [ln.strip() for ln in lines
            if ln.startswith("WARN engine:") or ln.startswith("# engine:")]


def save_trace(name, dt, chans, title, marks=(), ylabel="output (V)",
               xscale=1e3, xlabel="time (ms)", t0=0.0):
    """CSV of the raw samples and a PNG of the trace, as capture.py writes."""
    TMP.mkdir(exist_ok=True)
    FIGS.mkdir(exist_ok=True)
    n = len(next(iter(chans.values())))
    t = [i * dt - t0 for i in range(n)]
    with open(TMP / f"{name}.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s"] + [f"ch{c}_V" for c in sorted(chans)])
        for i in range(n):
            w.writerow([f"{t[i]:.9g}"] + [f"{chans[c][i]:.4g}" for c in sorted(chans)])
    fig, ax = plt.subplots(figsize=(9, 3.4), dpi=130)
    for c in sorted(chans):
        ax.plot([x * xscale for x in t], chans[c], lw=0.9, label=NAME[c])
    for x_s, text in marks:
        ax.axvline(x_s * xscale, color="0.4", ls="--", lw=0.8)
        ax.annotate(text, (x_s * xscale, ax.get_ylim()[1]), fontsize=7,
                    ha="left", va="top", color="0.3")
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title, fontsize=10)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="upper right")
    fig.tight_layout()
    fig.savefig(FIGS / f"{name}.png")
    plt.close(fig)
    return FIGS / f"{name}.png"


# ---------------------------------------------------------------- the checks

def load_sweep(scope, sj, ch, i_amps=(500, 1000, 2000)):
    """Measure one channel's load in current mode, anchored by the scope.

    Current mode is the only mode that can: the 100 ohm sense shunt sits in
    the I_OUT branch, which the output mux ties to the output in current mode
    and parks on an on-board 1 kOhm dummy in every other mode (phase 018). The
    scope reads the voltage the forced current produces, so the resistance
    rests on nothing the board calibrates.
    """
    rows = []
    sc = SCOPE[ch]
    scope.channel(sc, volts=5.0)
    scope.channel(SCOPE[1 - ch], volts=5.0)
    for amp in i_amps:
        sj.cmd1(train_line("S", SLOT_LOAD, ch, 1, "20000,10000000",
                           [(amp, 4000), (-amp, 4000)]))
        sj.cmd(f"T{SLOT_LOAD}", quiet=0.06)
        time.sleep(0.4)
        # A free-running capture of a 25 Hz square: both levels are in the
        # window wherever it lands, so the plateaus are the sorted extremes.
        scope.no_trigger()
        _, chans, _ = scope.block(3968, 9, timeout=10)
        v = sorted(chans[sc])
        hi = statistics.fmean(v[-200:])
        lo = statistics.fmean(v[:200])
        sj.cmd("T-1", quiet=0.8)
        sj.drain(quiet=0.5, limit=5.0)
        rows.append({"ch": ch, "set_uA": amp, "scope_hi": hi, "scope_lo": lo,
                     "R_hi": hi / (amp * 1e-6), "R_lo": lo / (-amp * 1e-6)})
    sj.cmd1(train_line("S", SLOT_LOAD, ch, 3, "10000,500000", []))
    return rows


def cmd_check(scope, sj):
    print("\n[configuration C wiring]")
    bad = 0
    scope.channel(ps.CHANNEL_A, volts=10.0)
    scope.channel(ps.CHANNEL_B, volts=10.0)

    spans = {}
    for ch in (0, 1):
        # An 8 V bipolar square on one channel, watched on both scope inputs.
        sj.cmd1(train_line("S", SLOT_LOAD, ch, 90, "20000,10000000",
                           [(8000, 4000), (-8000, 4000)]))
        sj.cmd(f"T{SLOT_LOAD}", quiet=0.06)
        time.sleep(0.4)
        scope.no_trigger()
        _, chans, _ = scope.block(3968, 9, timeout=10)
        sj.cmd("T-1", quiet=0.3)
        sj.drain(quiet=0.4)
        spans[ch] = {c: (min(chans[c]), max(chans[c])) for c in chans}
    sj.cmd1(train_line("S", SLOT_LOAD, 0, 3, "10000,500000", []))

    for ch in (0, 1):
        lo, hi = spans[ch][SCOPE[ch]]
        span = hi - lo
        ok = span > 8.0
        bad += not ok
        print(f"  {2*ch+1}  CH{ch}(+) -> scope {'AB'[ch]}       {span:5.2f} V swing "
              f"({lo:+.2f} .. {hi:+.2f})"
              f"   {'ok' if ok else 'FAIL: no full-swing output there'}")
        # A ground at CH<ch>(-) makes that scope input read the whole output;
        # a ground elsewhere in a chain makes it read a divided fraction.
        ok = abs(hi) > 3.0 and abs(lo) > 3.0 and abs(abs(hi) - abs(lo)) < 0.25 * span
        bad += not ok
        print(f"  {2*ch+2}  scope gnd at CH{ch}(-)   |+peak| {abs(hi):.2f} V vs "
              f"|-peak| {abs(lo):.2f} V"
              f"    {'ok' if ok else 'FAIL: looks like a divider tap'}")

    # The two loads are separate: driving one must not move the other. In
    # configuration A the series chain couples them, and this is the check
    # that says which of the two benches is on the table.
    for ch in (0, 1):
        lo, hi = spans[ch][SCOPE[1 - ch]]
        span = hi - lo
        ok = span < 0.5
        bad += not ok
        print(f"  {5+ch}  CH{1-ch} quiet while CH{ch} drives  "
              f"{'AB'[1-ch]} moves {span:5.2f} V"
              f"   {'ok' if ok else 'FAIL: the two loads are not separate'}")

    # Each load is a plain resistor, measured through the mode that can.
    for ch in (0, 1):
        rows = load_sweep(scope, sj, ch)
        rs = [r["R_hi"] for r in rows] + [r["R_lo"] for r in rows]
        spread = (max(rs) - min(rs)) / statistics.fmean(rs)
        ok = spread < 0.10
        bad += not ok
        print(f"  {7+ch}  load across CH{ch}        {statistics.fmean(rs):.0f} ohm "
              f"(current mode, scope-anchored), spread {spread*100:.1f} %"
              f"   {'ok' if ok else 'FAIL: not a plain resistor'}")
        for r in rows:
            print(f"       {r['set_uA']:+5d} uA -> {r['scope_hi']:+6.3f} V "
                  f"({r['R_hi']:6.0f} ohm)   {-r['set_uA']:+5d} uA -> "
                  f"{r['scope_lo']:+6.3f} V ({r['R_lo']:6.0f} ohm)")

    # The AWG reaches IN0: c1's independent route and the whole use case need
    # a real edge, and nothing else on this bench would notice its absence.
    sj.reset()
    sj.cmd1(train_line("S", SLOT_CH0, 0, 90, "100000,1000", [(2000, 2000)]))
    sj.cmd1(f"TRIG0,1,{SLOT_CH0},-1,0")
    sj.drain(quiet=0.3)
    scope.square(2.0, AWG_VPP)
    lines, end = [], time.time() + 3.0
    while time.time() < end:
        lines += sj.drain(quiet=0.25, limit=1.0)
    scope.siggen_off()
    lines += sj.drain(quiet=0.5)
    trains = sum(1 for x in lines if x.startswith("Train #"))
    ok = trains >= 3
    bad += not ok
    print(f"  9  AWG -> IN0              {trains} train(s) in 3 s at 2 Hz"
          f"          {'ok' if ok else 'FAIL: the AWG does not reach IN0'}")
    sj.cmd1("TRIG0,0,-1,-1,0")
    sj.cmd1(train_line("S", SLOT_CH0, 0, 3, "10000,500000", []))
    return bad


# ------------------------------------------- C1: do the two channels step together?

def step_edge(v, frac=0.5, k=198):
    """Fractional index where a single rising step crosses `frac` of its size.

    The two levels come from the sorted extremes rather than from a fixed
    leading window, so the step may sit anywhere in the record -- which is
    what an unknown inter-channel skew needs. `k` samples from each end are
    averaged, so each run has to be at least that long: the captures below put
    the trigger half way into the record, leaving ~2000 samples either side.
    """
    s = sorted(v)
    base = statistics.fmean(s[:k])
    top = statistics.fmean(s[-k:])
    return cross(v, base + frac * (top - base)), base, top


def skew_shot(scope, tb, level=4.0, timeout=20.0, pre=-50, n=3968):
    """One capture of a rising step on both channels; A - B at two fractions.

    50 % is the fraction the two channels can be compared at without the
    output settling entering the answer: both steps are the same size into
    loads that differ by 2 %, so the settling is common mode there. 5 % is
    the foot, reported as the cross-check -- if the two fractions disagree,
    the difference is the settling and not the latch.
    """
    scope.trigger(ps.CHANNEL_A, level, ps.RISING, delay_pct=pre, auto_ms=0)
    dt, ch, _ = scope.block(n, tb, timeout=timeout)
    out = {"dt": dt, "ch": ch}
    for frac, name in ((0.05, "foot"), (0.50, "half")):
        ia, base_a, top_a = step_edge(ch[ps.CHANNEL_A], frac)
        ib, base_b, top_b = step_edge(ch[ps.CHANNEL_B], frac)
        if ia is None or ib is None:
            return None
        out[name] = (ib - ia) * dt
        out[f"step_{name}"] = (top_a - base_a, top_b - base_b)
    return out


def series(scope, sj, tb, shots, before=None, after=None):
    """`shots` skew captures, with a hook run before and after each."""
    vals = []
    trace = None
    for _ in range(shots):
        if before is not None:
            before()
        s = skew_shot(scope, tb)
        if after is not None:
            after()
        if s is None:
            continue
        vals.append(s)
        if trace is None:
            trace = s
    return vals, trace


def report(tag, vals, key="half"):
    xs = [v[key] * 1e6 for v in vals]
    if not xs:
        print(f"  {tag:34s} no usable shot")
        return None
    m = statistics.fmean(xs)
    sd = statistics.pstdev(xs) if len(xs) > 1 else 0.0
    print(f"  {tag:34s} {m:+8.3f} us  sd {sd*1e3:6.1f} ns  "
          f"[{min(xs):+.3f} .. {max(xs):+.3f}]  n={len(xs)}")
    return m, sd, min(xs), max(xs), len(xs)


def cmd_c1(scope, sj, shots):
    print("\n[C1] CH1 minus CH0, at the same fraction of each channel's step")
    scope.channel(ps.CHANNEL_A, volts=10.0)
    scope.channel(ps.CHANNEL_B, volts=10.0)
    tb, dt, _ = scope.pick_timebase(19e-9, 3968)
    print(f"  timebase {tb}: {dt*1e9:.0f} ns/sample, "
          f"{3968*dt*1e6:.1f} us window, trigger half way in")

    rows, traces = {}, {}

    # (a) one train drives both channels: one dacProgramBoth, one NLDAC pulse.
    sj.cmd1(f"S{SLOT_JOINT},90,90,20000,3000000;8000,8000,2000;0,0,2000")
    sj.cmd(f"T{SLOT_JOINT}", quiet=0.06)
    time.sleep(0.3)
    vals, traces["joint"] = series(scope, sj, tb, shots)
    sj.cmd("T-1", quiet=0.4)
    sj.drain(quiet=0.5)
    rows["joint (one train, T start)"] = vals

    # (b) two single-channel trains, one trigger edge, one pulse each: the
    #     first latch after the edge, which is the case configuration A
    #     measured indirectly as latch contention.
    sj.cmd1(train_line("S", SLOT_CH0, 0, 90, "20000,5000", [(8000, 2000)]))
    sj.cmd1(train_line("S", SLOT_CH1, 1, 90, "20000,5000", [(8000, 2000)]))
    sj.cmd1(f"TRIG0,2,{SLOT_CH0},{SLOT_CH1},0")
    old = startlat(sj, STARTLAT_TWO)
    print(f"  CAL STARTLAT {old} -> {STARTLAT_TWO} for the two-engine route")
    sj.drain(quiet=0.3)
    scope.square(2.0, AWG_VPP)
    time.sleep(0.5)
    vals, traces["indep"] = series(scope, sj, tb, shots)
    scope.siggen_off()
    warn = faults(sj.drain(quiet=0.6, limit=6.0))
    rows["independent route, first pulse"] = vals

    # (c) the same route, but caught tens of pulses into a 3 s train, where
    #     both arms are long finished and only the two PIT wakes collide.
    sj.cmd1(train_line("S", SLOT_CH0, 0, 90, "20000,3000000", [(8000, 2000)]))
    sj.cmd1(train_line("S", SLOT_CH1, 1, 90, "20000,3000000", [(8000, 2000)]))
    sj.drain(quiet=0.3)
    later = []
    for _ in range(max(4, shots // 2)):
        scope.square(0.5, AWG_VPP)
        # 2.2 s at 0.5 Hz guarantees an edge has passed and the 3 s train is
        # still running, so the capture lands somewhere past pulse 10.
        time.sleep(2.2)
        s = skew_shot(scope, tb)
        scope.siggen_off()
        sj.cmd("T-1", quiet=0.2)
        sj.cmd("U-1", quiet=0.2)
        sj.drain(quiet=0.4)
        if s is not None:
            later.append(s)
    traces["later"] = later[0] if later else None
    rows["independent route, later pulse"] = later

    # (d) the control the other three need: the same two engines, the same
    #     route, but latches that never fall on the same instant. CH1 is
    #     delayed 30 us, which is inside the capture window, so what is left
    #     of the offset here is everything *except* contention -- the scope's
    #     own inter-channel skew and any constant lead of one engine.
    sj.cmd1(train_line("S", SLOT_CH0, 0, 90, "20000,5000", [(8000, 2000)]))
    sj.cmd1(train_line("S", SLOT_CH1, 1, 90, f"20000,5000,{OFFSET_US}",
                       [(8000, 2000)]))
    sj.drain(quiet=0.3)
    scope.square(2.0, AWG_VPP)
    time.sleep(0.5)
    vals, traces["offset"] = series(scope, sj, tb, shots)
    scope.siggen_off()
    sj.drain(quiet=0.6, limit=6.0)
    for v in vals:                    # report the residual, not the 30 us
        v["half"] -= OFFSET_US * 1e-6
        v["foot"] -= OFFSET_US * 1e-6
    rows[f"independent, CH1 delayed {OFFSET_US} us"] = vals

    startlat(sj, old)
    sj.cmd1("TRIG0,0,-1,-1,0")

    stats = {}
    for tag, vals in rows.items():
        stats[tag] = {"half": report(tag, vals, "half"),
                      "foot": report(f"    (at the foot)", vals, "foot")}
    if warn:
        print("  firmware verdict on the two-engine route:")
        for w in warn:
            print("   ", w)

    tag_j = "joint (one train, T start)"
    tag_i = "independent route, first pulse"
    tag_c = f"independent, CH1 delayed {OFFSET_US} us"

    # The control carries the scope's own inter-channel skew and whatever
    # constant lead one engine has over the other, so subtracting it from the
    # coincident case leaves the contention and nothing else. The two
    # fractions are independent estimates of it: they agree only if the
    # difference really is a delay and not a difference in settling.
    if stats.get(tag_i) and stats.get(tag_c):
        print()
        for frac in ("half", "foot"):
            c = stats[tag_i][frac][0] - stats[tag_c][frac][0]
            print(f"  contention = coincident - control, at the {frac}: "
                  f"{c:+.3f} us")

    # ------------------------------------------------------------- figure
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 3.6), dpi=130)
    for ax, key, title in (
            (axes[0], "joint", "one train drives both channels"),
            (axes[1], "indep", "two trains, one edge, same instant"),
            (axes[2], "offset",
             f"two trains, one edge, {OFFSET_US} us apart (control)")):
        s = traces.get(key)
        if s is None:
            continue
        n = len(s["ch"][ps.CHANNEL_A])
        t = [(i - n / 2) * s["dt"] * 1e6 for i in range(n)]
        for c in (ps.CHANNEL_A, ps.CHANNEL_B):
            ax.plot(t, s["ch"][c], lw=1.0, label=NAME[c])
        ax.set_xlim(-4, 8 + (OFFSET_US if key == "offset" else 0))
        ax.set_xlabel("time relative to the CH0 step (us)")
        ax.set_ylabel("output (V)")
        ax.set_title(title, fontsize=10)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, loc="lower right")
    for ax, tag, extra in ((axes[0], tag_j, ""), (axes[1], tag_i, ""),
                           (axes[2], tag_c, f" - {OFFSET_US} us")):
        s = stats.get(tag, {}).get("half")
        if s:
            ax.annotate(f"CH1 - CH0{extra} = {s[0]:+.3f} us\n"
                        f"sd {s[1]*1e3:.0f} ns, n={s[4]}",
                        (0.03, 0.95), xycoords="axes fraction", fontsize=8,
                        va="top")
    fig.suptitle("One train latches both channels together; two engines that "
                 "want the same instant do not", fontsize=11)
    fig.tight_layout()
    FIGS.mkdir(exist_ok=True)
    fig.savefig(FIGS / "stimjim-dual-latch.png")
    plt.close(fig)

    TMP.mkdir(exist_ok=True)
    with open(TMP / "stimjim-dual-latch.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["case", "shot", "skew_half_us", "skew_foot_us",
                    "step_ch0_V", "step_ch1_V"])
        for tag, vals in rows.items():
            for i, v in enumerate(vals):
                w.writerow([tag, i, f"{v['half']*1e6:.4f}",
                            f"{v['foot']*1e6:.4f}",
                            f"{v['step_half'][0]:.3f}",
                            f"{v['step_half'][1]:.3f}"])
    print("  -> figs/stimjim-dual-latch.png, tmp/stimjim-dual-latch.csv")

    # What the claim actually is: one dacProgramBoth and one NLDAC pulse serve
    # both channels, so they step together rather than a DAC programming time
    # apart. The threshold is therefore what programming a second channel
    # costs -- CAL DACPROG2 minus DACPROG1 -- and not the scope's sample
    # interval, which is a property of the instrument watching. Sub-sample
    # agreement was never on offer: the two channels drive different loads
    # through different output stages, and the control above shows the
    # measurement floor is already ~40 ns.
    bad = 0
    prog1 = int(sj.cmd1("CAL,DACPROG1?").split(",")[2])
    prog2 = int(sj.cmd1("CAL,DACPROG2?").split(",")[2])
    limit = prog2 - prog1
    if stats.get(tag_j, {}).get("half"):
        m = abs(stats[tag_j]["half"][0])
        ok = m < limit
        bad += not ok
        print(f"\n  verdict: one train's two channels step {m*1e3:.0f} ns "
              f"apart, against the {limit} us a separately programmed second "
              f"channel would cost (CAL DACPROG2 - DACPROG1) -- "
              f"{'ok, they are latched together' if ok else 'FAIL'}")
    return bad


# --------------------------------------------- C2: the sine sample-rate ceiling

F_CPU_HZ = 120e6
C2_DURATION_US = 1000000
C2_BURST_PERIOD_US = 20000


def sine_run(sj, f, period_us, burst_us):
    """Play one sine train and return what its completion reported."""
    sj.cmd1(f"W{SLOT_SINE},90,90,{period_us},{C2_DURATION_US};"
            f"4000,4000,{burst_us};{f},{f},0;0,0,0")
    sj.drain(quiet=0.2)
    sj.cmd(f"T{SLOT_SINE}", quiet=0.06)
    time.sleep(C2_DURATION_US / 1e6 + 0.4)
    out = sj.drain(quiet=1.0, limit=10.0)
    over = due = 0
    worst_over = worst_due = 0
    for ln in out:
        if "overran their deadline" in ln:
            over = int(ln.split()[2])
            worst_over = int(ln.split("by up to")[1].split()[0])
        elif "already due" in ln:
            due = int(ln.split()[2])
            worst_due = int(ln.split("worst by")[1].split()[0])
    bursts = next((int(x.split("Delivered")[1].split()[0])
                   for x in out if x.startswith("Train #")), -1)
    return {"overrun": over, "worst_overrun_ns": worst_over,
            "due": due, "worst_due_ns": worst_due, "bursts": bursts}


def cmd_c2(scope, sj, freqs, fs_max=50000.0):
    print("\n[C2] sine playback, both channels, sample rate walked up")
    print(f"  Fs = 64 * f clamped to {fs_max/1000:g} kHz, then rounded to a "
          f"whole number of CPU cycles at {F_CPU_HZ/1e6:g} MHz")
    print("  Each frequency is played twice for "
          f"{C2_DURATION_US/1e6:g} s: once as one continuous burst, which "
          "measures the\n  sustained sample rate, and once as "
          f"{C2_DURATION_US//C2_BURST_PERIOD_US} bursts of "
          f"{C2_BURST_PERIOD_US} us, which adds a burst boundary "
          f"{C2_DURATION_US//C2_BURST_PERIOD_US} times.")
    print(f"\n  {'f (Hz)':>8} {'Fs (kHz)':>9} {'cyc/smp':>8} {'dt (us)':>8} "
          f"{'samples':>9} | {'continuous':>21} | {'in 20 ms bursts':>21}")
    print(f"  {'':>8} {'':>9} {'':>8} {'':>8} {'':>9} | "
          f"{'play':>4} {'late':>6} {'due':>6} {'worst ns':>8} {'/1k':>6} | "
          f"{'play':>4} {'late':>6} {'due':>6} {'worst ns':>8} {'/1k':>6}")
    rows = []
    for f in freqs:
        fs_want = min(64.0 * f, fs_max)
        # The engine puts samples on an exact CPU-cycle grid, so the rate it
        # actually plays is F_CPU over a whole number of cycles -- which is
        # why a "50 kHz" and a "57.6 kHz" train are not equally well behaved.
        cyc = round(F_CPU_HZ / fs_want)
        fs = F_CPU_HZ / cyc
        n_smp = fs * C2_DURATION_US / 1e6
        cont = sine_run(sj, f, C2_DURATION_US, C2_DURATION_US)
        brst = sine_run(sj, f, C2_BURST_PERIOD_US, C2_BURST_PERIOD_US)
        row = {"f_hz": f, "fs_max_hz": fs_max,
               "fs_hz": fs, "cycles_per_sample": cyc,
               "dt_us": 1e6 / fs, "samples": int(n_smp),
               "cont_overrun": cont["overrun"],
               "cont_worst_ns": cont["worst_overrun_ns"],
               "cont_due": cont["due"], "cont_bursts": cont["bursts"],
               "burst_overrun": brst["overrun"],
               "burst_worst_ns": brst["worst_overrun_ns"],
               "burst_due": brst["due"], "bursts": brst["bursts"]}
        rows.append(row)
        print(f"  {f:8.1f} {fs/1000:9.3f} {cyc:8d} {1e6/fs:8.3f} "
              f"{int(n_smp):9d} | "
              f"{cont['bursts']:4d} {cont['overrun']:6d} {cont['due']:6d} "
              f"{cont['worst_overrun_ns']:8d} "
              f"{1000*cont['overrun']/n_smp:6.1f} | "
              f"{brst['bursts']:4d} {brst['overrun']:6d} {brst['due']:6d} "
              f"{brst['worst_overrun_ns']:8d} "
              f"{1000*brst['overrun']/n_smp:6.1f}"
              f"{'   <-- clamped' if 64.0*f > fs_max else ''}")
    sj.cmd1(f"S{SLOT_SINE},3,3,10000,500000")

    TMP.mkdir(exist_ok=True)
    # Not "stimjim-sine-ceiling.csv": save_trace writes that name for the
    # capture below, and the sweep is the more valuable of the two.
    with open(TMP / "stimjim-sine-sweep.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    # The scope confirms that the fastest frequency that reports no overrun
    # still draws a sine rather than a waveform losing samples.
    # The ceiling is the fastest rate a *continuous* sine sustains with no
    # late latch. Burst mode is reported beside it but does not define the
    # ceiling: its extra late latches come one or two per burst boundary, not
    # one per sample, so they scale with the burst count and not with Fs.
    rows = [r for r in rows if r["cont_bursts"] > 0]
    clean = [r for r in rows if r["cont_overrun"] == 0]
    top = max(clean, key=lambda r: r["fs_hz"]) if clean else rows[0]
    first_bad = min((r for r in rows if r["cont_overrun"] > 0),
                    key=lambda r: r["fs_hz"], default=None)
    print(f"\n  fastest continuous sine with no late latch: "
          f"Fs = {top['fs_hz']/1000:.3f} kHz (f = {top['f_hz']:g} Hz, "
          f"{top['dt_us']:.3f} us per sample)")
    if first_bad:
        print(f"  first continuous rate that is late:          "
              f"Fs = {first_bad['fs_hz']/1000:.3f} kHz "
              f"({first_bad['cont_overrun']} of {first_bad['samples']} "
              f"samples, worst {first_bad['cont_worst_ns']} ns)")
        print(f"  -> SJ_FS_MAX_HZ at 70 % of that is "
              f"{int(round(0.7*first_bad['fs_hz']/1000))} kHz")

    sine_figure(rows, fs_max)
    scope.channel(ps.CHANNEL_A, volts=5.0)
    scope.channel(ps.CHANNEL_B, volts=5.0)
    sj.cmd1(f"W{SLOT_SINE},90,90,20000,3000000;4000,4000,20000;"
            f"{top['f_hz']},{top['f_hz']},0;0,0,0")
    sj.cmd(f"T{SLOT_SINE}", quiet=0.06)
    time.sleep(0.3)
    want = 2.5 / top["f_hz"] / 3968          # ~2.5 periods across the record
    tb, dt, _ = scope.pick_timebase(want, 3968)
    scope.trigger(ps.CHANNEL_A, 1.0, ps.RISING, delay_pct=-10, auto_ms=2000)
    dt, ch, _ = scope.block(3968, tb, timeout=10)
    sj.cmd("T-1", quiet=0.4)
    sj.drain(quiet=0.5)
    save_trace("stimjim-sine-ceiling", dt, ch,
               f"Both channels at the fastest clean sine: f = {top['f_hz']:g} Hz, "
               f"Fs = {top['fs_hz']/1000:.1f} kHz "
               f"({top['dt_us']:.2f} us per sample)")
    sj.drain(quiet=0.6, limit=6.0)
    sj.cmd(f"S{SLOT_SINE},3,3,10000,500000", quiet=0.4)
    print(f"  -> figs/stimjim-sine-ceiling.png, tmp/stimjim-sine-ceiling.csv")
    return rows


def sine_figure(rows, fs_max, name="stimjim-sine-overruns"):
    """Late latches against sample rate, for both burst shapes.

    Two counters, because past about 175 kHz they swap roles. `overrun` counts
    events whose DAC programming finished after a deadline that was still in
    the future, so it needs the player to be keeping up at all; `already due`
    counts events the player reached after their instant had passed. When the
    generator collapses the first falls back to almost nothing and the second
    goes to one per sample -- plotting only the first would read as recovery.
    """
    FIGS.mkdir(exist_ok=True)
    fig, ax = plt.subplots(figsize=(9.5, 4.0), dpi=130)
    x = [r["fs_hz"] / 1000 for r in rows]
    # 0.3 is a floor for the log axis: a count of zero has no place on one,
    # and a marker below 1 reads as "none" against the gridline at 1.
    ax.plot(x, [max(r["cont_overrun"], 0.3) for r in rows], "o-", lw=1.4,
            color="C0", label="continuous: latch late")
    ax.plot(x, [max(r["cont_due"], 0.3) for r in rows], "o--", lw=1.2,
            color="C0", alpha=0.55, label="continuous: event already due")
    ax.plot(x, [max(r["burst_overrun"], 0.3) for r in rows], "s-", lw=1.4,
            color="C1", label=f"{rows[0]['bursts']} bursts: latch late")
    ax.plot(x, [r["samples"] for r in rows], ":", color="0.4", lw=1.0,
            label="one per sample (total failure)")
    ax.axvline(100, color="C2", lw=1.2)
    ax.annotate("SJ_FS_MAX_HZ set here\n(100 kHz, zero late latches)",
                (100, 3e4), fontsize=8, ha="right", va="top", color="C2",
                xytext=(-6, 0), textcoords="offset points")
    ax.set_yscale("log")
    ax.set_xlabel("sine sample rate Fs (kHz), both channels driven, 1 s of play")
    ax.set_ylabel("latches that missed their deadline")
    ax.set_title("Where the two-channel sine generator stops making its "
                 f"deadlines (built with SJ_FS_MAX_HZ = {fs_max/1000:g} kHz)",
                 fontsize=10)
    ax.grid(alpha=0.3, which="both")
    ax.legend(fontsize=7.5, loc="center left")
    fig.tight_layout()
    fig.savefig(FIGS / f"{name}.png")
    plt.close(fig)


# ----------------------------- the use case: stimulate on CH0, gate on CH1

STIM_UA = 2000            # biphasic current amplitude on CH0
STIM_PHASE_US = 500       # each phase of the biphasic pulse
STIM_PERIOD_US = 10000    # one stimulus pulse every 10 ms
STIM_DURATION_US = 1000000
GATE_AMP = 2500           # mV in voltage mode, uA in current mode
GATE_WIDTH_US = 50
GATE_PERIOD_US = 500
GATE_DURATION_US = 2000   # four gate pulses
GATE_DELAY_US = 500000    # half a second after the same edge
GATE_STAGGER_US = 250     # what it takes to step off the stimulus grid


def define_usecase(sj, gate_mode, gate_delay):
    """The two slots the use case is made of, and the route that starts them.

    CH0 carries the stimulus: a charge-balanced biphasic current pulse, one
    positive and one negative 500 us phase, every 10 ms for a second. CH1
    carries a gate: 50 us high every 500 us, four times, half a second after
    the same trigger edge. Two rhythms means two trains, two trains means two
    engines, and two engines from one edge means an independent route -- the
    one construct on this instrument that cannot sample-align its channels.
    """
    sj.cmd1(f"S{SLOT_STIM},1,3,{STIM_PERIOD_US},{STIM_DURATION_US};"
            f"{STIM_UA},0,{STIM_PHASE_US};{-STIM_UA},0,{STIM_PHASE_US}")
    sj.cmd1(f"S{SLOT_GATE},3,{gate_mode},{GATE_PERIOD_US},{GATE_DURATION_US},"
            f"{gate_delay};0,{GATE_AMP},{GATE_WIDTH_US}")
    sj.cmd1(f"TRIG0,2,{SLOT_STIM},{SLOT_GATE},0")


def triggered_capture(scope, sj, want_dt, trig_ch, level, pre,
                      n=3968, timeout=25.0, rate=0.5):
    """Arm the scope, then start the generator, and capture one whole pair.

    Both engines have to be stopped first, or the capture is not of the run
    it looks like. A free-running AWG put edges on IN0 while the previous
    stimulus train still held engine 0; `fireRoute` arms the two engines
    separately, so such an edge starts the gate alone, and a capture triggered
    on the gate then shows a gate burst with no stimulus anywhere near it.
    With both engines idle every edge starts a complete pair, so it no longer
    matters which edge the scope catches -- and the AWG period only has to
    exceed the train length for the pair to be complete by the next one.

    The generator cannot be started from `on_armed`: the driver refuses
    `ps2000_set_sig_gen_built_in` while a block is running. So it starts
    first, and a first edge missed during arming costs one AWG period.
    """
    sj.cmd("T-1", quiet=0.15)
    sj.cmd("U-1", quiet=0.15)
    sj.drain(quiet=0.3)
    tb, _, _ = scope.pick_timebase(want_dt, n)
    scope.square(rate, AWG_VPP)
    scope.trigger(trig_ch, level, ps.RISING, delay_pct=pre, auto_ms=0)
    dt, ch, _ = scope.block(n, tb, timeout=timeout)
    scope.siggen_off()
    return dt, ch


def serial_run(sj, scope, tries=4):
    """One trigger edge with no scope involved; everything the pair prints."""
    out = []
    for _ in range(tries):
        sj.cmd("T-1", quiet=0.15)
        sj.cmd("U-1", quiet=0.15)
        sj.drain(quiet=0.3)
        # 0.2 Hz: the generator starts its square high, so the edge lands at
        # once and the next one is 5 s away -- one train, not two.
        scope.square(0.2, AWG_VPP)
        time.sleep(1.6)
        scope.siggen_off()
        out = sj.drain(quiet=1.0, limit=10.0)
        if any(x.startswith("Train #") for x in out):
            return out
    return out


def gate_pulses(v, dt, k_top=80, k_base=1200):
    """Rise, fall and width of every gate pulse in a burst capture.

    The gate is high for 50 us in every 500 us, so the levels cannot come from
    a symmetric pair of tails: the top is the mean of the highest `k_top`
    samples and the baseline the mean of the lowest `k_base`.
    """
    s = sorted(v)
    base = statistics.fmean(s[:k_base])
    top = statistics.fmean(s[-k_top:])
    lvl = base + 0.5 * (top - base)
    ups = crossings(v, lvl, True)
    dns = crossings(v, lvl, False)
    out = []
    for u in ups:
        f = next((d for d in dns if d > u), None)
        if f is None:
            continue
        out.append({"rise_s": u * dt, "width_s": (f - u) * dt})
    return out, base, top


def cmd_usecase(scope, sj):
    print("\n[use case] a stimulus on CH0 and a delayed gate on CH1, "
          "one trigger edge")
    print(f"  CH0  {STIM_UA} uA biphasic, {STIM_PHASE_US}+{STIM_PHASE_US} us, "
          f"every {STIM_PERIOD_US} us for {STIM_DURATION_US/1e6:g} s "
          f"(current mode, measured)")
    print(f"  CH1  {GATE_AMP} mV/uA gate, {GATE_WIDTH_US} us every "
          f"{GATE_PERIOD_US} us x {GATE_DURATION_US//GATE_PERIOD_US}, "
          f"{GATE_DELAY_US/1000:g} ms after the edge")
    old = startlat(sj, STARTLAT_TWO)
    print(f"  CAL STARTLAT {old} -> {STARTLAT_TWO}: an independent route arms "
          f"two engines from one edge and does not fit {old}")

    res = {}
    for mode, mname in ((0, "voltage"), (1, "current")):
        print(f"\n  --- CH1 gate in {mname} mode "
              f"({GATE_AMP} {'mV' if mode == 0 else 'uA'}) ---")
        r = {"mode": mode}
        scope.channel(ps.CHANNEL_A, volts=5.0)
        scope.channel(ps.CHANNEL_B, volts=5.0)

        define_usecase(sj, mode, GATE_DELAY_US)
        sj.drain(quiet=0.3)

        # 650 ms of the second: the delay, the gate burst and 65 stimulus
        # pulses. Each 1 ms pulse gets four samples here -- an overview, not
        # a measurement.
        r["overview"] = triggered_capture(scope, sj, 160e-6, ps.CHANNEL_A, 1.0, 0)
        # One stimulus pulse, both phases.
        r["stim"] = triggered_capture(scope, sj, 600e-9, ps.CHANNEL_A, 1.0, -10)
        # The gate burst. Its first pulse falls on a stimulus pulse: the
        # delay is an exact multiple of the stimulus period.
        r["gate"] = triggered_capture(scope, sj, 600e-9, ps.CHANNEL_B, 1.0, -10)
        # That coincidence at 20 ns.
        r["collide"] = triggered_capture(scope, sj, 19e-9, ps.CHANNEL_B, 1.0, -25)
        r["lines"] = serial_run(sj, scope)

        # The same burst moved a quarter period off the stimulus grid.
        define_usecase(sj, mode, GATE_DELAY_US + GATE_STAGGER_US)
        sj.drain(quiet=0.3)
        r["gate_staggered"] = triggered_capture(scope, sj, 600e-9,
                                                ps.CHANNEL_B, 1.0, -10)
        r["lines_staggered"] = serial_run(sj, scope)
        res[mname] = r

        # ------------------------------------------------------- the numbers
        dt, ch = r["stim"]
        a = ch[ps.CHANNEL_A]
        s = sorted(a)
        pos, neg = statistics.fmean(s[-300:]), statistics.fmean(s[:300])
        print(f"  stimulus into the load: {pos:+.3f} V / {neg:+.3f} V "
              f"for {STIM_UA:+d} / {-STIM_UA:+d} uA "
              f"-> {pos/(STIM_UA*1e-6):.0f} / {neg/(-STIM_UA*1e-6):.0f} ohm; "
              f"charge imbalance {100*(pos+neg)/(pos-neg):+.1f} %")

        for key, label in (("gate", "on the stimulus grid"),
                           ("gate_staggered",
                            f"{GATE_STAGGER_US} us off the grid")):
            dt, ch = r[key]
            pulses, base, top = gate_pulses(ch[ps.CHANNEL_B], dt)
            r[key + "_pulses"] = pulses
            print(f"  gate burst, {label}: {len(pulses)} pulses, "
                  f"{top - base:.3f} V amplitude for {GATE_AMP} "
                  f"{'mV' if mode == 0 else 'uA'}")
            if not pulses:
                continue
            widths = [p["width_s"] * 1e6 for p in pulses]
            gaps = [(pulses[i + 1]["rise_s"] - pulses[i]["rise_s"]) * 1e6
                    for i in range(len(pulses) - 1)]
            print("        width (us)   " +
                  "  ".join(f"{w:7.2f}" for w in widths))
            print("        rise-to-rise (us)      " +
                  "  ".join(f"{g:7.2f}" for g in gaps))

        dt, ch = r["collide"]
        ia, base_a, top_a = step_edge(ch[ps.CHANNEL_A], 0.5)
        ib, base_b, top_b = step_edge(ch[ps.CHANNEL_B], 0.5)
        # Only a real step on A is a stimulus edge. Without this the sorted
        # extremes of a flat trace still yield a "level" and cross() returns
        # the first noise sample above it, which reads as a plausible skew.
        if top_a - base_a < 1.0:
            print("  at the coincidence: no stimulus edge inside the window "
                  f"(CH0 moves only {top_a - base_a:.3f} V) -- not reported")
        elif ia is not None and ib is not None:
            r["collide_skew"] = (ib - ia) * dt
            print(f"  at the coincidence: the gate edge is "
                  f"{(ib - ia) * dt * 1e6:+.3f} us from the stimulus edge")

        for tag, key in (("aligned", "lines"), ("staggered", "lines_staggered")):
            v = faults(r[key])
            msum = [x for x in r[key] if x.startswith(("MSUM", "MRANGE"))]
            print(f"  firmware verdict, {tag}: "
                  f"{'; '.join(v) if v else 'no timing fault'}")
            for m in msum:
                print(f"        {m}")

    startlat(sj, old)
    sj.cmd1("TRIG0,0,-1,-1,0")
    usecase_figures(res)
    return res


def usecase_figures(res):
    FIGS.mkdir(exist_ok=True)
    TMP.mkdir(exist_ok=True)
    modes = [m for m in ("voltage", "current") if m in res]

    # 1. what the second looks like.
    fig, axes = plt.subplots(len(modes), 1, figsize=(9, 2.4 * len(modes)),
                             dpi=130, squeeze=False)
    for ax, m in zip(axes[:, 0], modes):
        dt, ch = res[m]["overview"]
        t = [i * dt * 1e3 for i in range(len(ch[ps.CHANNEL_A]))]
        ax.plot(t, ch[ps.CHANNEL_A], lw=0.6, label=NAME[ps.CHANNEL_A])
        ax.plot(t, ch[ps.CHANNEL_B], lw=0.9, label=NAME[ps.CHANNEL_B])
        ax.set_ylabel("output (V)")
        ax.set_title(f"gate in {m} mode", fontsize=9)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, loc="upper right")
    axes[-1, 0].set_xlabel("time from the trigger edge (ms)")
    fig.suptitle("One trigger edge: a 1 s biphasic current train on CH0 and a "
                 "gate burst on CH1 half a second later", fontsize=10)
    fig.tight_layout()
    fig.savefig(FIGS / "stimjim-usecase-overview.png")
    plt.close(fig)

    # 2. the gate burst, on and off the stimulus grid.
    fig, axes = plt.subplots(len(modes), 2, figsize=(11, 2.6 * len(modes)),
                             dpi=130, squeeze=False)
    for row, m in enumerate(modes):
        for col, (key, label) in enumerate(
                (("gate", "first pulse on a stimulus pulse"),
                 ("gate_staggered",
                  f"burst moved {GATE_STAGGER_US} us off the grid"))):
            ax = axes[row, col]
            dt, ch = res[m][key]
            t = [i * dt * 1e6 for i in range(len(ch[ps.CHANNEL_B]))]
            ax.plot(t, ch[ps.CHANNEL_A], lw=0.6, label=NAME[ps.CHANNEL_A])
            ax.plot(t, ch[ps.CHANNEL_B], lw=0.9, label=NAME[ps.CHANNEL_B])
            for p in res[m].get(key + "_pulses", []):
                ax.annotate(f"{p['width_s']*1e6:.1f} us",
                            (p["rise_s"] * 1e6, ax.get_ylim()[1]),
                            fontsize=6, ha="center", va="top", color="0.3")
            ax.set_title(f"{m} gate, {label}", fontsize=9)
            ax.set_ylabel("output (V)")
            ax.grid(alpha=0.3)
            if row == 0 and col == 0:
                ax.legend(fontsize=7, loc="lower right")
    for ax in axes[-1]:
        ax.set_xlabel("time in the capture (us)")
    fig.suptitle("The gate burst: four 50 us pulses, and what the coinciding "
                 "stimulus pulse does to the first of them", fontsize=10)
    fig.tight_layout()
    fig.savefig(FIGS / "stimjim-usecase-gate.png")
    plt.close(fig)

    # 3. the coincidence at 20 ns.
    fig, axes = plt.subplots(1, len(modes), figsize=(5.2 * len(modes), 3.4),
                             dpi=130, squeeze=False)
    for ax, m in zip(axes[0], modes):
        dt, ch = res[m]["collide"]
        n = len(ch[ps.CHANNEL_A])
        t = [(i - n / 4) * dt * 1e6 for i in range(n)]
        ax.plot(t, ch[ps.CHANNEL_A], lw=1.0, label=NAME[ps.CHANNEL_A])
        ax.plot(t, ch[ps.CHANNEL_B], lw=1.0, label=NAME[ps.CHANNEL_B])
        ax.set_xlim(-6, 14)
        ax.set_xlabel("time relative to the gate edge (us)")
        ax.set_ylabel("output (V)")
        sk = res[m].get("collide_skew")
        ax.set_title(f"{m} gate" +
                     (f": gate edge {sk*1e6:+.2f} us from the stimulus edge"
                      if sk is not None else ""), fontsize=9)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, loc="lower right")
    fig.suptitle("Two engines asked for the same instant: the second one waits",
                 fontsize=10)
    fig.tight_layout()
    fig.savefig(FIGS / "stimjim-usecase-collision.png")
    plt.close(fig)

    with open(TMP / "stimjim-usecase.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["gate_mode", "burst", "pulse", "rise_us", "width_us"])
        for m in modes:
            for key in ("gate", "gate_staggered"):
                for i, p in enumerate(res[m].get(key + "_pulses", [])):
                    w.writerow([m, key, i, f"{p['rise_s']*1e6:.3f}",
                                f"{p['width_s']*1e6:.3f}"])
    print("\n  -> figs/stimjim-usecase-overview.png, "
          "figs/stimjim-usecase-gate.png, figs/stimjim-usecase-collision.png")
    print("  -> tmp/stimjim-usecase.csv")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("what", nargs="?", default="all",
                    choices=["check", "c1", "c2", "usecase", "all"])
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--shots", type=int, default=20)
    ap.add_argument("--freqs", type=float, nargs="*",
                    default=[100, 200, 400, 600, 700, 781.25, 900],
                    help="sine frequencies for C2; Fs is 64x each, and "
                         "anything past SJ_FS_MAX_HZ/64 lands on the clamp")
    ap.add_argument("--fs-max", type=float, default=50000.0,
                    help="the SJ_FS_MAX_HZ the running firmware was built "
                         "with; only labels the table, the firmware does the "
                         "clamping")
    a = ap.parse_args()

    bad = 0
    with StimJim(a.port) as sj, ps.Scope() as scope:
        sj.drain(quiet=0.3)
        sj.reset()
        print("scope:  ", scope.info(3), "serial", scope.info(4))
        print("stimjim:", sj.cmd("IDN")[0])
        # Captured here rather than inside each step: a step that raises must
        # not leave the board on the raised two-engine budget, which is what
        # happened when the generator refused to start from a running block.
        was = startlat(sj)
        try:
            if a.what in ("check", "all"):
                bad += cmd_check(scope, sj)
            if a.what in ("c1", "all"):
                bad += cmd_c1(scope, sj, a.shots)
            if a.what in ("c2", "all"):
                cmd_c2(scope, sj, a.freqs, a.fs_max)
            if a.what in ("usecase", "all"):
                cmd_usecase(scope, sj)
        finally:
            scope.siggen_off()
            sj.cmd("T-1", quiet=0.2)
            sj.cmd("U-1", quiet=0.2)
            startlat(sj, was)
            sj.cmd1("TRIG0,0,-1,-1,0")
            for s in (SLOT_JOINT, SLOT_CH0, SLOT_CH1, SLOT_SINE,
                      SLOT_STIM, SLOT_GATE, SLOT_LOAD):
                sj.cmd1(f"S{s},3,3,10000,500000")
            sj.drain(quiet=0.4)
    print(f"\n{'ALL OK' if bad == 0 else str(bad) + ' PROBLEM(S)'}")
    return bad


if __name__ == "__main__":
    sys.exit(main())
