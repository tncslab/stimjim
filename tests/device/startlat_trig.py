"""Qualify a candidate `CAL STARTLAT` with real trigger edges.

`bench_arm.py` measures the arm; this checks the consequence. A train whose arm
ran past its start latency says so at train end -- `WARN engine: arming this
train took longer than CAL STARTLAT ... set CAL STARTLAT >= <n> us` -- so
setting a candidate and starting the worst trains through a trigger is a direct
test of the number, and the firmware names the value it wanted.

**It has to be a trigger.** With no anchor, `Engine::startTrain` reads its own
clock at the end, so `t0` is one STARTLAT after the arm finished and a `T`/`U`
start is never charged for the arm at all: every candidate passes. A trigger
edge is timestamped at ISR entry and `t0` is measured from there, which is what
makes the latency deterministic and what makes the arm come out of it.

The edge comes from the PicoScope AWG wired to IN0 (docs/bench-wiring.md
configuration A), 1 Hz 0 to 2 V, the same source capture.py uses. Nothing is
captured; the scope is only a signal generator here. The trains drive the
outputs, so the bench load or nothing at all should be connected.

    python startlat_trig.py COM4 35            # is 35 us enough?
    python startlat_trig.py COM4 30 27 25 23   # sweep down to the break point

Slots 96 and 98 are used and left defined, trigger input 0 is borrowed and
disabled at the end, and `CAL STARTLAT` is left at the last candidate tried --
`CALDEF` or `CAL,STARTLAT,<us>` restores it. Slots 0-9 are untouched.
"""

import argparse
import sys
import time

from sjcon import StimJim
import pico2000 as ps

STAGES = "".join(f";{400 * (i + 1)},{-400 * (i + 1)},1000" for i in range(10))
# The two heaviest arms bench_arm.py finds: the ten-stage ramp (the worst warmed
# arm of any shape) and the ten-point measured train (the worst arm when loop()
# has been starved and the plan is compiled inside it).
CASES = [
    ("L 10 stages", 96, f"L96,90,90,20000,100000{STAGES}", None),
    ("S 10 stages + 10 MEAS", 98, f"S98,0,0,20000,100000{STAGES}", "MEAS98,3,3,0,-1,0,1"),
]
EDGE_HZ = 1.0
SECONDS = 3.6


def trial(sj, scope, label, slot, define, meas, cand):
    scope.siggen_off()
    sj.cmd1("TRIG0,0,-1,-1,0")        # no route while the budget changes
    sj.reset()
    sj.cmd1(f"CAL,STARTLAT,{cand}")
    sj.cmd1(define)
    if meas:
        sj.cmd1(meas)
    sj.cmd1(f"TRIG0,1,{slot},-1,0")
    sj.drain(quiet=0.3)

    scope.square(EDGE_HZ, 2.0)
    lines, end = [], time.time() + SECONDS
    while time.time() < end:
        lines += sj.drain(quiet=0.25, limit=1.0)
    scope.siggen_off()
    lines += sj.drain(quiet=0.5)

    trains = sum(1 for x in lines if x.startswith("Train #"))
    bad = [x for x in lines if "took longer than CAL STARTLAT" in x
           or "overran their deadline" in x or "already due when the player" in x]
    if trains == 0:
        verdict = "NO TRAINS — is the AWG wired to IN0?"
    else:
        verdict = "ok" if not bad else "FAIL"
    print(f"  STARTLAT {cand:3d}  {label:26s} {trains} train(s)  {verdict}")
    for x in dict.fromkeys(bad):          # the same line once per train, collapse
        print("       " + x)
    return 0 if verdict == "ok" else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM4")
    ap.add_argument("candidates", nargs="*", type=int, default=[35],
                    help="CAL STARTLAT values to try, in order")
    a = ap.parse_args()

    bad = 0
    with StimJim(a.port) as sj, ps.Scope() as scope:
        sj.drain(quiet=0.3)
        sj.reset()
        print(sj.cmd("IDN")[0])
        try:
            for cand in a.candidates:
                for label, slot, define, meas in CASES:
                    bad += trial(sj, scope, label, slot, define, meas, cand)
        finally:
            scope.siggen_off()
            sj.cmd1("TRIG0,0,-1,-1,0")
            sj.reset()
    print("every candidate fits" if bad == 0 else f"{bad} failing trial(s)")
    return bad


if __name__ == "__main__":
    sys.exit(main())
