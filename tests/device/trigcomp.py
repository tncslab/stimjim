"""Configuration B: check the wiring, then measure `CAL TRIGCOMP`.

`CAL TRIGCOMP` is the delay from the physical rising edge at trigger input IN0
to the trigger ISR's first instruction. It is the one term of the delivered
start latency the firmware cannot observe, because software only ever sees the
instant it is already running. A scope that watches the edge and the output at
the same time can see both, which is what this bench configuration is for
(docs/bench-wiring.md, configuration B):

    PicoScope AWG ---+---> StimJim trigger input IN0
                     |
                     +---> scope channel B          (the edge itself)

        CH0(+) ---- R ---- CH0(-)
          |                   |
       scope A            scope gnd

    CH1 unused and grounded; no LEDs anywhere.

`check` verifies each of those connections separately, so a failure names the
wire to look at rather than reporting a number that happens to be wrong:

  1  the AWG reaches scope B          (an edge is visible on B)
  2  the AWG reaches IN0              (the board fires trains from it)
  3  scope A is on CH0(+)             (a train appears on A)
  4  scope gnd is at CH0(-)           (A is single-ended, not a divider tap)
  5  CH1 and the LED chain are gone   (a CH0 pulse does not appear on B)
  6  the load is a plain resistor     (the same value through both modes)

Check 6 measures the load twice, once through each output mode, because one
route on its own cannot tell a bad contact from a miscalibrated readback.
Voltage mode drives a known voltage and reads the current back; current mode
drives a known current and reads the voltage back; the scope watches the load
throughout, so the voltage across it is known independently of the board. A
resistor gives the same value both ways, in both polarities, at every
amplitude. A bad contact gives an unstable value both ways. A readback that is
out of calibration gives a stable, linear, wrong value one way only -- which is
what this board does: its voltage-mode current readback on channel 0 is low by
a factor of 4, so `V/I` in voltage mode reports a 1 kOhm load as 4 kOhm.

`measure` then takes the number. The delivered latency is
`TRIGCOMP + STARTLAT`, so with `CAL TRIGCOMP` at 0 the edge-to-output time
reads `STARTLAT` plus the hardware part, and the hardware part is what is
wanted. Three estimators of "when the output moved" are reported, because they
disagree by more than the quantity being measured:

  foot     the first departure from baseline -- closest to the latch instant,
           and the one TRIGCOMP is derived from
  50 %     the doc's prescription; late by roughly half the output settling
  10-90    the 10-90 % chord extrapolated back to baseline

The difference between them is the output's own settling time, which is
reported too and cross-checks `BENCHSETTLE`.

    python trigcomp.py check
    python trigcomp.py measure
    python trigcomp.py all

Slot 20 and trigger input 0 are borrowed and released at the end; `CAL` is left
as it was found unless --set is given. Figures go to figs/, per-shot data to
tmp/.
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

ROOT = pathlib.Path(__file__).resolve().parents[2]
FIGS = ROOT / "figs"
TMP = ROOT / "tmp"

SLOT = 20            # the slot docs/bench-wiring.md configuration B names
SLOT_LOAD = 21       # the bipolar train that characterises the load
TRIG_HZ = 5.0        # AWG rate during the shot series; 1 Hz makes it 5x longer
AWG_VPP = 2.0        # 0..2 V into IN0, the amplitude every other script uses

# Ranges the scope needs. B carries the 0..2 V AWG edge, so +/-5 V is the
# smallest that holds it without clipping the overshoot.
B_RANGE = 5.0


# --------------------------------------------------------------- edge finding

def cross(v, level, i0=1):
    """Index (fractional) of the first upward crossing of `level`."""
    for i in range(max(i0, 1), len(v)):
        if v[i - 1] < level <= v[i]:
            span = v[i] - v[i - 1]
            return (i - 1) + (0.0 if span == 0 else (level - v[i - 1]) / span)
    return None


def step_times(v, pre_n):
    """Describe a rising step: where it starts, and by which definition.

    `pre_n` is how many leading samples are baseline. Returns a dict of
    fractional sample indices plus the baseline, plateau and noise in volts.
    Everything is in samples; the caller multiplies by dt.
    """
    # Leave a margin before the trigger point: the scope's own trigger
    # interpolation puts the edge a sample or two either side of pre_n.
    base_n = max(8, pre_n - 4)
    base = statistics.fmean(v[:base_n])
    noise = statistics.pstdev(v[:base_n])
    # The plateau, from the last quarter of the record -- the pulse is far
    # longer than the window, so the record ends inside it.
    top = statistics.fmean(v[int(0.75 * len(v)):])
    step = top - base

    # The foot threshold is 5 % of the step, but never inside the noise: on the
    # +/-10 V range one 8-bit code is 78 mV, so a small step needs the noise
    # floor to set the threshold instead.
    foot_level = base + max(0.05 * step, 5 * noise)
    out = {"base": base, "top": top, "step": step, "noise": noise,
           "foot_level": foot_level}
    for name, level in (("foot", foot_level),
                        ("t10", base + 0.10 * step),
                        ("t50", base + 0.50 * step),
                        ("t90", base + 0.90 * step)):
        out[name] = cross(v, level)
    if out["t10"] is not None and out["t90"] is not None:
        # Extrapolate the 10-90 chord back to the baseline. Exact for a
        # slew-limited edge; early by ~0.24 tau for a single-pole one.
        slope = (0.8 * step) / (out["t90"] - out["t10"])
        out["extrap"] = out["t10"] - (0.10 * step) / slope
    else:
        out["extrap"] = None
    return out


# ------------------------------------------------------------ timebase scale

def timebase_scale(scope, tb, ref_tb=6, freq=50000.0, n=3968):
    """How much the working timebase's interval differs from the truth.

    The unit's AWG is the reference: the slower timebases reproduce its period
    to a few parts in 1e5 while the fastest are good to about 1e-3
    (tests/device/scope_timebase.py), so the period measured at `ref_tb` is the
    truth against which the faster working timebase is scaled.
    Returns (scale, period_ref, period_work); true dt = nominal dt * scale.
    """
    def period(tbx):
        dt, _ = scope.timebase(tbx, n)
        scope.trigger(ps.CHANNEL_B, 1.0, ps.RISING, delay_pct=-5, auto_ms=3000)
        _, ch, _ = scope.block(n, tbx, timeout=10)
        b = ch[ps.CHANNEL_B]
        xs, i = [], 1
        while i < len(b):
            x = cross(b, 1.0, i)
            if x is None:
                break
            xs.append(x)
            # step past the high run so the next search starts on a low one
            i = int(x) + 2
            while i < len(b) and b[i] > 1.0:
                i += 1
        assert len(xs) >= 3, f"timebase {tbx}: only {len(xs)} edges at {freq} Hz"
        return (xs[-1] - xs[0]) / (len(xs) - 1) * dt

    scope.square(freq, AWG_VPP)
    time.sleep(0.8)
    p_ref, p_work = period(ref_tb), period(tb)
    # Stop it before returning. The same wire feeds IN0, so leaving 50 kHz on
    # it means the next `TRIG` command arms a route that immediately fires
    # tens of thousands of trains a second and buries its own reply.
    scope.siggen_off()
    time.sleep(0.2)
    return p_ref / p_work, p_ref, p_work


# ------------------------------------------------------------------- the load

def load_sweep(scope, sj, v_amps=(1000, 2000, 4000), i_amps=(500, 1000, 1500)):
    """Measure the load twice over, once through each output mode.

    Voltage mode drives a known voltage and reads the current back; current
    mode drives a known current and reads the voltage back. The scope watches
    the load in both, so the voltage across it is known independently of the
    board. Two routes to the same resistor is what separates a bad contact
    (both routes wrong, and unstable) from a readback that is miscalibrated
    (one route wrong, both stable).
    """
    rows = []
    # Range per mode, not per bench: an 8-bit scope on +/-10 V has 78 mV codes,
    # which is a quarter of the whole signal when 250 uA crosses 1 kOhm.
    for mode, amps, unit, rng in ((0, v_amps, "mV", 5.0), (1, i_amps, "uA", 2.0)):
        scope.channel(ps.CHANNEL_A, volts=rng)
        for amp in amps:
            sj.cmd1(f"S{SLOT_LOAD},{mode},3,20000,10000000;"
                    f"{amp},0,4000;{-amp},0,4000")
            sj.cmd1(f"MEAS{SLOT_LOAD},3,0,0,-1,0,1")
            sj.cmd(f"T{SLOT_LOAD}", quiet=0.06)
            time.sleep(0.5)
            # A free-running capture of a 25 Hz square: both levels are in the
            # window wherever it lands, so the plateaus are the sorted extremes.
            scope.no_trigger()
            _, ch, _ = scope.block(3968, 9, timeout=10)
            a = sorted(ch[ps.CHANNEL_A])
            scope_hi = statistics.fmean(a[-200:])
            scope_lo = statistics.fmean(a[:200])
            out = sj.cmd("T-1", quiet=0.8) + sj.drain(quiet=0.6, limit=5.0)
            for ln in out:
                if not ln.startswith("MSUM"):
                    continue
                f = ln.split(",")
                point, v, i = int(f[3]), float(f[4]), float(f[6])
                rows.append({
                    "mode": "V" if mode == 0 else "I",
                    "set": amp if point == 0 else -amp, "unit": unit,
                    "n": int(f[2]), "V_mV": v, "I_uA": i,
                    "scope_V": scope_hi if point == 0 else scope_lo,
                    "R_board": v / i * 1000 if i else float("nan"),
                })
    for r in rows:
        # The scope is the only voltage in the chain nothing on the board
        # calibrated, so it anchors the resistance in current mode.
        r["R_scope"] = (r["scope_V"] / (r["set"] * 1e-6) if r["mode"] == "I"
                        else float("nan"))
    return rows


# ----------------------------------------------------------------- the checks

def cmd_check(scope, sj):
    print("\n[configuration B wiring]")
    bad = 0
    scope.channel(ps.CHANNEL_A, volts=10.0)
    scope.channel(ps.CHANNEL_B, volts=10.0)

    # 1. the AWG reaches scope B. 1 kHz rather than the 1 Hz the measurement
    #    uses, so a single 20 ms window holds many whole periods.
    scope.square(1000.0, AWG_VPP)
    time.sleep(0.8)
    scope.no_trigger()
    _, ch, _ = scope.block(3968, 9, timeout=10)
    b = ch[ps.CHANNEL_B]
    span = max(b) - min(b)
    ok = span > 1.5
    bad += not ok
    print(f"  1  AWG -> scope B          {span:5.2f} V swing "
          f"({min(b):+.2f} .. {max(b):+.2f})"
          f"   {'ok' if ok else 'FAIL: tee the AWG to channel B'}")

    # 2. the AWG reaches IN0.
    scope.siggen_off()
    sj.reset()
    sj.cmd1(f"S{SLOT},90,3,100000,1000;2000,0,2000")
    sj.cmd1(f"TRIG0,1,{SLOT},-1,0")
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
    print(f"  2  AWG -> IN0              {trains} train(s) in 3 s at 2 Hz"
          f"          {'ok' if ok else 'FAIL: the tee does not reach IN0'}")
    sj.cmd1("TRIG0,0,-1,-1,0")

    # 3/4/5. a train on CH0, watched on both scope channels at once.
    sj.cmd1(f"S{SLOT},90,3,20000,10000000;8000,0,4000;-8000,0,4000")
    sj.cmd(f"T{SLOT}", quiet=0.06)
    time.sleep(0.4)
    scope.no_trigger()
    _, ch, _ = scope.block(3968, 9, timeout=10)
    sj.cmd("T-1", quiet=0.3)
    sj.drain(quiet=0.4)
    a, b = ch[ps.CHANNEL_A], ch[ps.CHANNEL_B]
    a_span, b_span = max(a) - min(a), max(b) - min(b)

    ok = a_span > 4.0
    bad += not ok
    print(f"  3  CH0(+) -> scope A       {a_span:5.2f} V swing "
          f"({min(a):+.2f} .. {max(a):+.2f})"
          f"   {'ok' if ok else 'FAIL: no output on channel A'}")

    # A ground at CH0(-) makes channel A read the whole output; a ground
    # anywhere else in a chain makes it read a divided fraction of it, which
    # is what configuration A does deliberately.
    ok = (abs(max(a)) > 3.0 and abs(min(a)) > 3.0
          and abs(abs(max(a)) - abs(min(a))) < 0.25 * a_span)
    bad += not ok
    print(f"  4  scope gnd at CH0(-)     |+peak| {abs(max(a)):.2f} V vs "
          f"|-peak| {abs(min(a)):.2f} V"
          f"    {'ok' if ok else 'FAIL: A looks like a divider tap'}")

    ok = b_span < 0.5
    bad += not ok
    print(f"  5  CH1 / LED chain absent  B moves {b_span:5.2f} V while CH0 pulses"
          f"   {'ok' if ok else 'FAIL: CH0 couples into B, the chain is wired'}")

    # 6. the load itself, measured through both output modes.
    rows = load_sweep(scope, sj)
    r_i = [abs(r["R_scope"]) for r in rows if r["mode"] == "I"]
    r_v = [abs(r["R_board"]) for r in rows if r["mode"] == "V"]
    spread = (max(r_i) - min(r_i)) / statistics.fmean(r_i)
    ok = spread < 0.10
    bad += not ok
    print(f"  6  load across CH0         {statistics.fmean(r_i):.0f} ohm "
          f"(current mode, scope-anchored), spread {spread*100:.1f} %"
          f"   {'ok' if ok else 'FAIL: not a plain resistor'}")
    print("       mode  set        board V     board I     scope V    R board"
          "   R scope")
    for r in rows:
        rs_ = f"{r['R_scope']:8.0f}" if r["R_scope"] == r["R_scope"] else "      --"
        print(f"       {r['mode']:>4s} {r['set']:+6d} {r['unit']:<3s} "
              f"{r['V_mV']:+9.2f} mV {r['I_uA']:+9.1f} uA {r['scope_V']:+8.3f} V "
              f"{r['R_board']:8.0f} {rs_}")
    # The same resistor through the two modes. Current mode is anchored by the
    # scope, so a ratio far from 1 indicts the voltage-mode current readback,
    # not the load: a bad contact cannot be selective about the output mode.
    ratio = statistics.fmean(r_v) / statistics.fmean(r_i)
    if abs(ratio - 1) > 0.10:
        bad += 1
        print(f"       FAIL: voltage mode reports {statistics.fmean(r_v):.0f} ohm "
              f"for the same load, {ratio:.2f}x the current-mode figure.")
        print(f"             The board V readback agrees with the scope, so it "
              f"is the voltage-mode")
        print(f"             CURRENT readback that is low by {ratio:.2f}x. The "
              f"load and the contacts are fine.")
    sj.cmd1(f"S{SLOT_LOAD},3,3,10000,500000")
    write_load_csv(rows)
    return bad


def write_load_csv(rows):
    TMP.mkdir(exist_ok=True)
    with open(TMP / "stimjim-load-modes.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print("       -> tmp/stimjim-load-modes.csv")


# ---------------------------------------------------------- the input threshold

def cmd_threshold(scope, sj, step=0.05, lo=0.5, hi=2.0):
    # hi is 2.0 because the 2204A generator spans +/-2 V: a 0..V square is
    # offset V/2, so its peak is V and V > 2.0 is refused by the driver.
    """Find the voltage at which IN0 stops seeing the edge.

    The reference instant of the whole measurement is "when the input pin
    crossed its threshold", and the AWG's edge is slow enough -- about
    0.8 V/us -- that guessing the threshold half a volt wrong moves the answer
    by two thirds of a microsecond, which is more than the quantity being
    measured. So measure it: `Triggers::begin` sets the pin to plain `INPUT`
    with no pull, so the pin sees the BNC directly and the lowest square-wave
    peak that still fires trains is the threshold.

    Returns (v_fail, v_pass, table): the highest peak that failed, the lowest
    that worked, and the rows. Peaks are the amplitudes *measured on scope B*,
    not the ones commanded, so the AWG's own gain drops out.
    """
    print("\n[IN0 input threshold]")
    scope.channel(ps.CHANNEL_B, volts=B_RANGE)
    sj.reset()
    sj.cmd1(f"S{SLOT},90,3,100000,1000;2000,0,2000")
    sj.cmd1(f"TRIG0,1,{SLOT},-1,0")
    sj.drain(quiet=0.3)

    rows = []
    v = hi
    while v >= lo - 1e-9:
        scope.square(TRIG_HZ, v)
        time.sleep(0.4)
        # What the AWG actually delivers at this setting. The window has to
        # hold a whole period: an untriggered 20 ms capture of a 5 Hz square
        # lands on a flat half nine times out of ten and reads the baseline.
        # Untriggered rather than triggered, because near the threshold there
        # is no level left to put a scope trigger on.
        tb_slow, _, _ = scope.pick_timebase(1.5 / TRIG_HZ / 3968, 3968)
        scope.no_trigger()
        _, ch, _ = scope.block(3968, tb_slow, timeout=10)
        peak = max(ch[ps.CHANNEL_B])
        sj.drain(quiet=0.3)
        lines, end = [], time.time() + 2.0
        while time.time() < end:
            lines += sj.drain(quiet=0.2, limit=1.0)
        trains = sum(1 for x in lines if x.startswith("Train #"))
        rows.append({"set_V": v, "peak_V": peak, "trains": trains})
        print(f"       peak {v:4.2f} V set, {peak:5.3f} V measured "
              f"-> {trains:2d} train(s) in 2 s at 5 Hz")
        if trains == 0 and len(rows) >= 2:
            break
        v -= step
    scope.siggen_off()
    sj.cmd1("TRIG0,0,-1,-1,0")
    sj.reset()

    passed = [r for r in rows if r["trains"] >= 5]
    failed = [r for r in rows if r["trains"] == 0]
    v_pass = min(r["peak_V"] for r in passed) if passed else float("nan")
    v_fail = max(r["peak_V"] for r in failed) if failed else float("nan")
    print(f"       threshold is between {v_fail:.3f} V (never fires) and "
          f"{v_pass:.3f} V (fires every edge) -> {(v_fail+v_pass)/2:.3f} V")
    TMP.mkdir(exist_ok=True)
    with open(TMP / "stimjim-in0-threshold.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print("       -> tmp/stimjim-in0-threshold.csv")
    return v_fail, v_pass, rows


# ------------------------------------------------------------ the measurement

def one_shot(scope, tb, n, pre_frac):
    scope.trigger(ps.CHANNEL_B, AWG_VPP / 2, ps.RISING,
                  delay_pct=-int(pre_frac * 100), auto_ms=0)
    dt, ch, overflow = scope.block(n, tb, timeout=8)
    return dt, ch, overflow


def cmd_measure(scope, sj, shots, amps, set_cal, ref_levels, threshold):
    print("\n[trigger edge -> first output sample]")
    startlat = int(sj.cmd1("CAL,STARTLAT?").split(",")[2])
    trigcomp0 = int(sj.cmd1("CAL,TRIGCOMP?").split(",")[2])
    print(f"       budget in force: CAL STARTLAT = {startlat} us, "
          f"CAL TRIGCOMP = {trigcomp0} us")
    if trigcomp0 != 0:
        print("       WARNING: TRIGCOMP is not 0, so the board is already "
              "compensating; the number below is a residual, not the total")

    # The instant that counts is when the pin crossed its own threshold, not
    # when the edge passed half its amplitude: on a 0.75 V/us edge those are
    # a microsecond apart. `threshold` is what the `threshold` step measured.
    primary = threshold if threshold else AWG_VPP / 2
    ref_levels = sorted(set(list(ref_levels) + [primary]))
    print(f"       reference instant: input edge crossing {primary:.3f} V"
          + ("  (measured pin threshold)" if threshold else
             "  (half amplitude -- run `threshold` for the real one)"))

    n, pre_frac = 3968, 0.10
    scope.channel(ps.CHANNEL_B, volts=B_RANGE)
    scope.channel(ps.CHANNEL_A, volts=10.0)
    tb, dt_nom, _ = scope.pick_timebase(0, n)      # the fastest honest timebase
    scale, p_ref, p_work = timebase_scale(scope, tb)
    dt = dt_nom * scale
    print(f"       timebase {tb}: {dt_nom*1e9:.0f} ns nominal, "
          f"{dt*1e9:.2f} ns measured against the AWG "
          f"({p_ref*1e6:.4f} us reference period vs {p_work*1e6:.4f} us) "
          f"-> window {n*dt*1e6:.0f} us")

    results = {}
    for amp in amps:
        # Smallest range that still holds the output. It comes out ~13 % below
        # the commanded amplitude on this board, so ask for the command itself
        # with a little margin and let the range table round up.
        want = abs(amp) / 1000.0 * 1.05
        rng = min(r for r in sorted(ps.RANGE) if r >= want)
        scope.channel(ps.CHANNEL_A, volts=rng)

        sj.reset()
        sj.cmd1(f"S{SLOT},90,3,100000,1000;{amp},0,2000")
        sj.cmd1(f"TRIG0,1,{SLOT},-1,0")
        sj.drain(quiet=0.3)
        scope.square(TRIG_HZ, AWG_VPP)
        time.sleep(0.5)

        rows, keep = [], None
        for k in range(shots):
            _, ch, overflow = one_shot(scope, tb, n, pre_frac)
            a, b = ch[ps.CHANNEL_A], ch[ps.CHANNEL_B]
            pre_n = int(pre_frac * n)
            A, B = step_times(a, pre_n), step_times(b, pre_n)
            if None in (A["foot"], A["t50"], A["extrap"],
                        B["t50"], B["t10"], B["t90"]):
                continue
            # The board's input is a logic pin, so where on the AWG's own slope
            # it actually fires is unknown. Reference the edge at its 50 %
            # point (what the doc prescribes) and record the slope beside it,
            # so the reader can move the threshold and see the cost.
            row = {
                "shot": k, "overflow": int(overflow),
                "step_V": A["step"], "awg_step_V": B["step"],
                "foot_us": (A["foot"] - B["t50"]) * dt * 1e6,
                "t50_us": (A["t50"] - B["t50"]) * dt * 1e6,
                "extrap_us": (A["extrap"] - B["t50"]) * dt * 1e6,
                "settle_us": (A["t90"] - A["foot"]) * dt * 1e6,
                "awg_rise_ns": (B["t90"] - B["t10"]) * dt * 1e9,
            }
            # The same output foot referenced to a range of levels on the input
            # edge, so the reader can see what the unknown pin threshold costs
            # instead of having to trust one choice of it.
            for lv in ref_levels:
                x = cross(b, lv)
                row[f"foot_at_{lv:.2f}V_us"] = (
                    (A["foot"] - x) * dt * 1e6 if x is not None else float("nan"))
            rows.append(row)
            if keep is None:
                keep = (a, b, A, B)
            if k % 10 == 0:
                sj.drain(quiet=0.05, limit=0.3)   # keep the CDC buffer moving
        scope.siggen_off()
        sj.cmd1("TRIG0,0,-1,-1,0")
        sj.reset()

        assert rows, f"{amp} mV: no usable shot out of {shots}"
        results[amp] = (rows, keep)

        def col(name):
            xs = [r[name] for r in rows]
            return statistics.fmean(xs), (statistics.stdev(xs) if len(xs) > 1 else 0.0)

        print(f"\n       {amp} mV commanded, scope A on +/-{rng} V, "
              f"{len(rows)}/{shots} shots, output step "
              f"{statistics.fmean([r['step_V'] for r in rows]):.3f} V")
        print(f"       {'estimator':<24s} {'latency (us)':>14s} {'sd (ns)':>9s}"
              f" {'implied TRIGCOMP (us)':>22s}")
        for name, label in (("foot_us", "foot (first movement)"),
                            ("t50_us", "50 % of step"),
                            ("extrap_us", "10-90 % extrapolated")):
            m, sd = col(name)
            print(f"       {label:<24s} {m:14.3f} {sd*1000:9.0f} "
                  f"{m - startlat:22.3f}")
        m, sd = col("settle_us")
        print(f"       output settling, foot to 90 %: {m:.2f} +- {sd:.2f} us")
        m, sd = col("awg_rise_ns")
        step_V = statistics.fmean([r["awg_step_V"] for r in rows])
        slew = 0.8 * step_V / (m * 1e-9)          # 10-90 % chord, V/s
        print(f"       AWG edge 10-90 % rise:         {m:.0f} +- {sd:.0f} ns "
              f"over {0.8*step_V:.2f} V  ({slew/1e6:.2f} V/us)")
        print(f"       the same output foot, referenced to the input edge at:")
        for lv in ref_levels:
            mm, ss = col(f"foot_at_{lv:.2f}V_us")
            print(f"         {lv:4.2f} V  ->  {mm:7.3f} us latency, "
                  f"implied TRIGCOMP {mm - startlat:6.3f} us  (sd {ss*1000:.0f} ns)")

    write_outputs(results, dt, startlat)

    if set_cal:
        rows, _ = results[max(amps)]
        cand = round(statistics.fmean(
            [r[f"foot_at_{primary:.2f}V_us"] for r in rows]) - startlat)
        r = sj.cmd1(f"CAL,TRIGCOMP,{cand}")
        print(f"\n       set: {r}   (P persists it)")
    return 0


def write_outputs(results, dt, startlat):
    TMP.mkdir(exist_ok=True)
    FIGS.mkdir(exist_ok=True)
    with open(TMP / "stimjim-trigcomp.csv", "w", newline="") as f:
        w = None
        for amp, (rows, _) in results.items():
            for r in rows:
                r = dict(r, amp_mV=amp)
                if w is None:
                    w = csv.DictWriter(f, fieldnames=list(r))
                    w.writeheader()
                w.writerow(r)

    # One representative shot's raw samples per amplitude, so a figure can show
    # the two edges themselves rather than only the numbers derived from them.
    for amp, (_, keep) in results.items():
        a, b, A, B = keep
        with open(TMP / f"stimjim-trigcomp-shot-{amp}mV.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_us_from_edge50", "input_V", "output_V"])
            for i in range(len(a)):
                w.writerow([f"{(i - B['t50']) * dt * 1e6:.6f}",
                            f"{b[i]:.5g}", f"{a[i]:.5g}"])

    fig, axes = plt.subplots(len(results), 1, figsize=(9, 3.2 * len(results)),
                             dpi=130, squeeze=False)
    for ax, (amp, (rows, keep)) in zip(axes[:, 0], results.items()):
        a, b, A, B = keep
        t = [(i - B["t50"]) * dt * 1e6 for i in range(len(a))]
        ax.plot(t, b, lw=0.9, label="B: AWG edge into IN0")
        ax.plot(t, a, lw=0.9, label=f"A: StimJim CH0, {amp} mV commanded")
        for key, lbl, c in (("foot", "foot", "0.35"), ("t50", "50 %", "0.6")):
            x = (A[key] - B["t50"]) * dt * 1e6
            ax.axvline(x, color=c, ls="--", lw=0.8)
            ax.annotate(f"{lbl}: {x:.2f} us", (x, max(a)), fontsize=7,
                        ha="left", va="top", color="0.3")
        ax.axvline(0, color="0.35", ls=":", lw=0.8)
        ax.set_xlim(-3, 60)
        ax.set_xlabel("time from the AWG edge's 50 % point (us)")
        ax.set_ylabel("volts")
        ax.set_title(f"Trigger edge to first output sample, {amp} mV "
                     f"(CAL STARTLAT = {startlat} us)", fontsize=10)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc="lower right")
    fig.tight_layout()
    fig.savefig(FIGS / "stimjim-trigcomp.png")
    plt.close(fig)
    print("\n       -> figs/stimjim-trigcomp.png, tmp/stimjim-trigcomp.csv")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("what", nargs="?", default="all",
                    choices=["check", "threshold", "measure", "all"])
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--shots", type=int, default=25)
    ap.add_argument("--amps", type=int, nargs="*", default=[8000, 2000],
                    help="commanded CH0 amplitudes in mV; the second is a "
                         "control -- the same latency at a different slew rate")
    ap.add_argument("--ref-levels", type=float, nargs="*",
                    default=[0.75, 1.00, 1.25, 1.50],
                    help="levels on the input edge to reference the latency "
                         "to; the pin's own threshold is measured by the "
                         "`threshold` step and should be among them")
    ap.add_argument("--threshold", type=float, default=None,
                    help="the IN0 input threshold in volts, from the "
                         "`threshold` step; the latency is referenced to the "
                         "edge crossing it, and --set uses that value")
    ap.add_argument("--set", action="store_true",
                    help="write the measured value to CAL TRIGCOMP")
    a = ap.parse_args()

    bad = 0
    with StimJim(a.port) as sj, ps.Scope() as scope:
        sj.drain(quiet=0.3)
        sj.reset()
        print("scope:  ", scope.info(3), "serial", scope.info(4))
        print("stimjim:", sj.cmd("IDN")[0])
        try:
            if a.what in ("check", "all"):
                bad += cmd_check(scope, sj)
            if a.what in ("threshold", "all"):
                cmd_threshold(scope, sj)
            if a.what in ("measure", "all"):
                bad += cmd_measure(scope, sj, a.shots, a.amps, a.set,
                                   a.ref_levels, a.threshold)
        finally:
            scope.siggen_off()
            sj.cmd("T-1", quiet=0.2)
            sj.cmd("U-1", quiet=0.2)
            sj.cmd1("TRIG0,0,-1,-1,0")
            sj.cmd1(f"S{SLOT},3,3,10000,500000")
            sj.drain(quiet=0.4)
    print(f"\n{'ALL OK' if bad == 0 else str(bad) + ' PROBLEM(S)'}")
    return bad


if __name__ == "__main__":
    sys.exit(main())
