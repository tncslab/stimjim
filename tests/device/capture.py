"""Oscilloscope acceptance captures for stimjimAWG (PicoScope 2204A).

Wiring this assumes (the bench as built):

    PicoScope AWG  --> StimJim trigger input IN0
    StimJim CH0(+) --> PicoScope channel A ("channel 1")
    StimJim CH1(+) --> PicoScope channel B ("channel 2")
    StimJim CH0(-) --> PicoScope ground

    load chain:  CH1(+) -1k- CH0(+) -1k- CH0(-) -[antiparallel LEDs]- CH1(-)
                    |           |           |
                 scope B     scope A    scope gnd

The chain couples the channels, so a quiet channel's trace is not flat -- see
README.md for what each direction looks like and why.

The AWG wire is not teed to a scope input, so trigger-to-output delay is
measured differentially instead: the same trigger edge starts a zero-delay
reference pulse on CH1 (engine 1) and the delayed pulse under test on CH0
(engine 0), and the scope triggers on the reference. Subtracting the two
engines' arming skew -- measured in the same way with the delay set to 0 --
leaves the delay itself.

    python capture.py delay      # trigger edge to first output sample
    python capture.py shapes     # S / L / W waveform shapes
    python capture.py all

Figures go to figs/, raw samples to tmp/.
"""

import argparse
import csv
import pathlib
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

# Slots reserved for the bench so nothing the user stored gets overwritten.
SLOT_DUT, SLOT_REF = 10, 11


def save(name, dt, chans, title, ylabel="output (V)", marks=()):
    """Write a CSV of the raw samples and a PNG of the trace."""
    TMP.mkdir(exist_ok=True)
    FIGS.mkdir(exist_ok=True)
    n = len(next(iter(chans.values())))
    t = [i * dt for i in range(n)]

    with open(TMP / f"{name}.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s"] + [f"ch{c}_V" for c in sorted(chans)])
        for i in range(n):
            w.writerow([f"{t[i]:.9g}"] + [f"{chans[c][i]:.4g}" for c in sorted(chans)])

    fig, ax = plt.subplots(figsize=(9, 3.6), dpi=130)
    label = {ps.CHANNEL_A: "A: StimJim CH0", ps.CHANNEL_B: "B: StimJim CH1"}
    for c in sorted(chans):
        ax.plot([x * 1e3 for x in t], chans[c], lw=0.9, label=label[c])
    for x_s, text in marks:
        ax.axvline(x_s * 1e3, color="0.4", ls="--", lw=0.8)
        ax.annotate(text, (x_s * 1e3, ax.get_ylim()[1]), fontsize=7,
                    ha="left", va="top", color="0.3")
    ax.set_xlabel("time (ms)")
    ax.set_ylabel(ylabel)
    ax.set_title(title, fontsize=10)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="upper right")
    fig.tight_layout()
    fig.savefig(FIGS / f"{name}.png")
    plt.close(fig)
    print(f"       -> figs/{name}.png, tmp/{name}.csv")


def first_cross(samples, dt, level, start=0):
    """Time of the first upward crossing of `level`, linearly interpolated."""
    for i in range(max(start, 1), len(samples)):
        if samples[i - 1] < level <= samples[i]:
            span = samples[i] - samples[i - 1]
            frac = 0.0 if span == 0 else (level - samples[i - 1]) / span
            return (i - 1 + frac) * dt
    return None


def measure_pair(scope, sj, delay_us, n=3900, want_dt=2e-6, settle=1.5):
    """Arm the trigger route, capture one shot, return (dt, chans, dA, dB)."""
    # DUT: CH0 voltage 5 V, 2 ms, one pulse per trigger (duration < period).
    sj.cmd1(f"S{SLOT_DUT},0,3,50000,1000,{delay_us};5000,0,2000")
    # Reference: CH1 voltage 8 V, no delay -- it marks the trigger instant.
    # 8 V because CH1 drives through 2 k plus the antiparallel LEDs.
    sj.cmd1(f"S{SLOT_REF},3,0,50000,1000,0;0,8000,2000")
    # Independent routing: slot0 on engine 0, slot1 on engine 1, rising edge.
    sj.cmd1(f"TRIG0,2,{SLOT_DUT},{SLOT_REF},0")

    tb, dt, _ = scope.pick_timebase(want_dt, n)
    # 5 % pre-trigger so the baseline before the reference edge is visible.
    scope.trigger(ps.CHANNEL_B, 1.0, ps.RISING, delay_pct=-5, auto_ms=0)
    time.sleep(settle)
    dt, chans, overflow = scope.block(n, tb, timeout=15)
    if overflow:
        print("       WARNING: input overflow, a channel clipped its range")

    tB = first_cross(chans[ps.CHANNEL_B], dt, 4.0)      # reference edge
    tA = first_cross(chans[ps.CHANNEL_A], dt, 2.5)      # delayed pulse
    return dt, chans, tA, tB


def cmd_delay(scope, sj):
    print("\n[trigger -> output delay]")
    scope.channel(ps.CHANNEL_A, volts=10.0)
    scope.channel(ps.CHANNEL_B, volts=10.0)
    scope.square(1.0, 2.0)              # 1 Hz, 0..2 V into IN0
    time.sleep(0.5)

    # 1) Arming skew between the two engines, with the delay set to zero.
    dt, chans, tA, tB = measure_pair(scope, sj, 0, want_dt=0.5e-6)
    if tA is None or tB is None:
        print(f"       FAIL: no edges found (A={tA}, B={tB})")
        return 1
    skew = tA - tB
    print(f"       arming skew (delay=0): {skew*1e6:+.1f} us  [dt={dt*1e6:.2f} us]")
    save("stimjim-trigger-skew", dt, chans,
         f"Trigger with delay = 0: engine-to-engine arming skew {skew*1e6:+.1f} us",
         marks=[(tB, "ref edge"), (tA, "DUT edge")])

    # 2) The delay itself.
    failures = 0
    for set_us in (2000, 5000, 20000):
        want_dt = max(0.5e-6, (set_us * 1e-6 + 4e-3) / 3900)
        dt, chans, tA, tB = measure_pair(scope, sj, set_us, want_dt=want_dt)
        if tA is None or tB is None:
            print(f"       {set_us:6d} us set: FAIL, no edges (A={tA}, B={tB})")
            failures += 1
            continue
        measured = (tA - tB) - skew
        errr = measured - set_us * 1e-6
        ok = abs(errr) < max(3 * dt, 2e-6)
        print(f"       {set_us:6d} us set -> {measured*1e6:9.1f} us measured, "
              f"error {errr*1e6:+.1f} us (dt={dt*1e6:.2f} us) {'ok' if ok else 'FAIL'}")
        if not ok:
            failures += 1
        save(f"stimjim-trigger-delay-{set_us}us", dt, chans,
             f"Post-trigger delay {set_us} us set, {measured*1e6:.1f} us measured "
             f"(skew-corrected)",
             marks=[(tB, "trigger"), (tA, "delayed pulse")])
    return failures


def cmd_shapes(scope, sj):
    """S, L and W on CH0, each triggered on its own rising edge."""
    print("\n[waveform shapes on CH0]")
    scope.channel(ps.CHANNEL_A, volts=10.0)
    scope.channel(ps.CHANNEL_B, volts=10.0)
    sj.cmd1("TRIG0,0,-1,-1,0")          # no trigger routing; start over serial
    failures = 0

    # Each train runs for 3 s and repeats every 20 ms, so the capture can be
    # armed calmly inside it and still catch a whole pulse. (Arming after a
    # one-shot start does not work here: the serial round trip that starts the
    # train is itself tens of milliseconds.)
    cases = [
        ("S", "stimjim-shape-S-biphasic",
         "S12,0,3,20000,3000000;5000,0,2000;-5000,0,2000",
         "S: biphasic rectangular, +/-5 V, 2 ms per phase, 20 ms period",
         4e-6, 2.0),
        ("L", "stimjim-shape-L-ramp",
         "L13,0,3,20000,3000000;5000,0,4000;0,0,4000",
         "L: linear ramp 0 -> 5 V over 4 ms, back to 0 over 4 ms",
         4e-6, 1.5),
        ("W", "stimjim-shape-W-sine",
         "W14,0,3,20000,3000000;5000,0,10000;500,0,0;0,0,0;0,0,0",
         "W: 500 Hz sine burst, 5 V amplitude, 10 ms burst per 20 ms period",
         2e-6, 2.0),
    ]
    for letter, name, line, title, want_dt, level in cases:
        r = sj.cmd1(line)
        if r.startswith("ERR"):
            print(f"       {letter}: FAIL to define: {r}")
            failures += 1
            continue
        slot = "".join(c for c in line.split(",")[0][1:] if c.isdigit())
        n = 3900
        tb, dt, _ = scope.pick_timebase(want_dt, n)
        sj.cmd(f"T{slot}", quiet=0.06)
        time.sleep(0.2)
        # Rising-edge trigger on the stimulus itself, with pre-trigger baseline.
        # Levels are kept ~1 V clear of ground: the 2204A is 8-bit, so on the
        # +/-10 V range one code is 78 mV and a threshold within a few codes of
        # the baseline sits inside the trigger hysteresis and never fires.
        scope.trigger(ps.CHANNEL_A, level, ps.RISING, delay_pct=-10, auto_ms=0)
        try:
            dt, chans, overflow = scope.block(n, tb, timeout=10)
        except ps.PicoError as e:
            print(f"       {letter}: FAIL capture: {e}")
            failures += 1
            sj.cmd("T-1")
            continue
        if overflow:
            print(f"       {letter}: WARNING input overflow")
        span = max(chans[ps.CHANNEL_A]) - min(chans[ps.CHANNEL_A])
        print(f"       {letter}: captured {len(chans[ps.CHANNEL_A])} samples "
              f"at {dt*1e6:.2f} us, CH0 span {span:.2f} V")
        if span < 1.0:
            print(f"       {letter}: FAIL, no signal on CH0")
            failures += 1
        save(name, dt, chans, title)
        sj.cmd("T-1", quiet=0.2)
        sj.drain(quiet=0.3)
    return failures


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("what", nargs="?", default="all",
                    choices=["delay", "shapes", "all"])
    ap.add_argument("--port", default="COM4")
    a = ap.parse_args()

    failures = 0
    with StimJim(a.port) as sj, ps.Scope() as scope:
        sj.reset()
        print("scope:", scope.info(3), "serial", scope.info(4))
        print("stimjim:", sj.cmd("IDN")[0])
        try:
            if a.what in ("delay", "all"):
                failures += cmd_delay(scope, sj)
            if a.what in ("shapes", "all"):
                failures += cmd_shapes(scope, sj)
        finally:
            scope.siggen_off()
            sj.cmd("T-1")
            sj.cmd("U-1")
            sj.cmd1("TRIG0,3,-1,-1,0")
            for s in (SLOT_DUT, SLOT_REF, 12, 13, 14):
                sj.cmd1(f"S{s},3,3,10000,500000")

    print(f"\n{'ALL CAPTURES OK' if failures == 0 else str(failures) + ' FAILURE(S)'}")
    return failures


if __name__ == "__main__":
    sys.exit(main())
