"""Check which of the PicoScope's timebases deliver the interval they report.

`ps2000_get_timebase` answers with a sample interval and a `time_units` code,
and the two do not go together: the interval is in nanoseconds whatever the
code says, because the code describes the timestamp array of a different call.
Multiplying the two -- which pico2000.py used to do -- turns the fast
timebases into intervals near zero, and `pick_timebase` then skips every one of
them and settles for 640 ns when 20 ns was available.

The unit's own AWG is the reference for whether an index is honest. A square
wave of known frequency is captured at each timebase and its period measured
from the level crossings; a timebase that delivers what it claims reproduces
the period.

**Count the crossings with hysteresis.** The AWG's edge takes about 2 us to
cross 2 V, so on a coarse vertical range the samples dither across any fixed
level and one real edge yields several crossings. Counting them all makes the
average spacing come out short -- by a third, by a quarter -- which reads
exactly like a timebase running slow. Skipping to the end of each high run
before looking for the next edge is what makes the result mean anything; this
script does that, and every timebase this unit accepts then passes.

This needs only the AWG wired to a scope channel, which every bench
configuration in docs/bench-wiring.md has. It writes tmp/pico-timebase.csv.

    python scope_timebase.py
    python scope_timebase.py --freq 20000 --channels 1
"""

import argparse
import csv
import pathlib
import sys
import time

import pico2000 as ps

TMP = pathlib.Path(__file__).resolve().parents[2] / "tmp"


def raw_units(scope, tb, n):
    """The `time_units` code the driver pairs with the interval, for the record.

    It is what pico2000.py used to multiply the interval by; keeping it in the
    CSV lets a figure show how far that put each timebase off.
    """
    import ctypes
    iv, un, mx = ctypes.c_int32(), ctypes.c_int16(), ctypes.c_int32()
    rc = scope.lib.ps2000_get_timebase(
        scope.h, ctypes.c_int16(tb), ctypes.c_int32(n), ctypes.byref(iv),
        ctypes.byref(un), ctypes.c_int16(1), ctypes.byref(mx))
    return (un.value if rc else None)


def period_at(scope, tb, n, level, freq):
    """Measured period at timebase `tb`, or None if it holds too few edges."""
    r = scope.timebase(tb, n)
    if r is None:
        return None, None
    dt, _ = r
    scope.trigger(ps.CHANNEL_B, level, ps.RISING, delay_pct=-5, auto_ms=3000)
    _, ch, _ = scope.block(n, tb, timeout=10)
    b = ch[ps.CHANNEL_B]
    xs, i = [], 1
    while i < len(b):
        x = ps_cross(b, level, i)
        if x is None:
            break
        xs.append(x)
        i = int(x) + 2
        while i < len(b) and b[i] > level:   # skip the high run
            i += 1
    if len(xs) < 3:
        return dt, None
    return dt, (xs[-1] - xs[0]) / (len(xs) - 1) * dt


def ps_cross(v, level, i0):
    for i in range(max(i0, 1), len(v)):
        if v[i - 1] < level <= v[i]:
            span = v[i] - v[i - 1]
            return (i - 1) + (0.0 if span == 0 else (level - v[i - 1]) / span)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--freq", type=float, default=50000.0)
    ap.add_argument("--vpp", type=float, default=2.0)
    ap.add_argument("--samples", type=int, default=3968)
    ap.add_argument("--max-tb", type=int, default=9)
    ap.add_argument("--channels", type=int, default=2, choices=[1, 2],
                    help="how many scope channels are enabled; the driver's "
                         "fastest usable timebase can depend on it")
    a = ap.parse_args()

    truth = 1.0 / a.freq
    rows = []
    with ps.Scope() as scope:
        print("scope:", scope.info(3), "serial", scope.info(4))
        if a.channels == 2:
            scope.channel(ps.CHANNEL_A, volts=10.0)
        else:
            scope.channel(ps.CHANNEL_A, enabled=False)
        scope.channel(ps.CHANNEL_B, volts=5.0)
        scope.square(a.freq, a.vpp)
        time.sleep(1.0)
        print(f"\nreference: AWG square at {a.freq:.0f} Hz, "
              f"period {truth*1e6:.4f} us, {a.channels} channel(s) enabled\n")
        print("  tb   claimed dt   measured period      error   verdict")
        for tb in range(0, a.max_tb + 1):
            dt, per = period_at(scope, tb, a.samples, a.vpp / 2, a.freq)
            if dt is None:
                print(f"  {tb:2d}   rejected by the driver")
                rows.append({"timebase": tb, "claimed_dt_ns": "", "units": "",
                             "period_us": "", "error_pct": "", "usable": 0})
                continue
            if per is None:
                print(f"  {tb:2d}   {dt*1e9:8.0f} ns   too few edges in the window")
                rows.append({"timebase": tb, "claimed_dt_ns": dt * 1e9,
                             "units": raw_units(scope, tb, a.samples),
                             "period_us": "", "error_pct": "", "usable": ""})
                continue
            err = per / truth - 1
            # A timebase that samples slower than it claims reads every interval
            # short by the same factor, so the error is the whole verdict.
            ok = abs(err) < 0.01
            print(f"  {tb:2d}   {dt*1e9:8.0f} ns   {per*1e6:10.4f} us   "
                  f"{err*100:+7.2f} %   {'delivers it' if ok else 'DOES NOT'}"
                  + ("" if ok else f"  (really {dt/(1+err)*1e9:.1f} ns)"))
            rows.append({"timebase": tb, "claimed_dt_ns": dt * 1e9,
                         "units": raw_units(scope, tb, a.samples),
                         "period_us": per * 1e6, "error_pct": err * 100,
                         "usable": int(ok)})
        scope.siggen_off()

    TMP.mkdir(exist_ok=True)
    out = TMP / f"pico-timebase-{a.channels}ch.csv"
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(f"\n-> {out.relative_to(TMP.parent)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
