"""Run the trains that stress the per-stage machinery and check every deadline was met.

The player derives stage i+1 while stage i plays, instead of the arm deriving
every stage before it takes t0 (docs/PLAN_lazy-stages.md). That trades start
latency for work inside the player ISR, so the question this script answers is
whether the work still fits: each case runs to completion and the firmware's own
per-latch counters say whether any latch overran its deadline (`WARN engine: ...
overran`) or was already due when the player reached it (`# engine: ... already
due`). Those counters are the timing design's acceptance measurement and need no
oscilloscope.

It is also a before/after regression on the *values*: every case measures its
own output and prints the `MSUM` rows, so running the script on two firmware
builds and diffing the output shows whether a stage's DAC codes moved. The
absolute numbers depend on what is connected to the outputs; the comparison does
not.

    python stage_timing.py COM4                      # run every case
    python stage_timing.py COM4 --out ../../tmp/p13.txt

Cases, and what each one is for:

  S 10 stages, 1 ms    the plain per-stage path, amplitudes all different so a
                       stage derived from the wrong neighbour shows up, with a
                       measurement point on each stage
  S 10 stages, 12 us   stage boundaries one PRELOAD+DACPROG2+3 apart, the
                       closest a two-channel latch can be scheduled, so every
                       derivation lands between two back-to-back latches
  L 10 stages          the ramp path at the default 20 us sample interval
  L 10 stages, dt=12   the ramp interval at its floor, where samples run inline
                       in one ISR pass and the look-ahead never gets a gap
  L jump chain         0-duration stages, whose single sample shares a deadline
                       with the previous stage's last one — the case that needs
                       more than one stage of look-ahead
  L 10 stages, dt=50   a ramp wide enough to carry a measurement point per stage

Slots 80-85 are used and left defined; slots 0-9 are untouched. The outputs are
driven, so connect the bench load (docs/bench-wiring.md) or nothing at all.
"""

import argparse
import re
import sys
import time

from sjcon import StimJim

# One stage triplet per level, amplitudes all different so that a stage taking
# its predecessor's value (or its own from the wrong index) cannot look right.
def stages(dur_us, n=10, step=400):
    return "".join(f";{step * (i + 1)},{-step * (i + 1)},{dur_us}" for i in range(n))


CASES = [
    # Modes 0/0 measure, modes 90/91 are the same drive without measurement --
    # the timing-only cases use 90 so a measurement point cannot be what runs
    # long. The two measured cases are the value regression, and their sample
    # interval is wide enough for a point to fit (a 20 us ramp interval has 20 us
    # free and one V+I point on two channels needs 26).
    (80, "S", "S 10 stages, 1 ms, measured",
     "0,0,20000,100000" + stages(1000), "3,3,0,-1,0,1"),
    (81, "S", "S 10 stages, 12 us",
     "90,90,20000,100000" + stages(12), None),
    (82, "L", "L 10 stages, dt default (20 us)",
     "90,90,20000,100000" + stages(1000), None),
    (83, "L", "L 10 stages, dt=12 us (the floor)",
     "90,90,20000,100000,0,12" + stages(1000), None),
    # A 0-duration stage is an instant jump; every other stage is one, which is
    # the densest jump chain the parser allows (two in a row are refused).
    (84, "L", "L jump chain (alternating 0-duration)",
     "90,90,20000,100000" + "".join(
         f";{400 * (i + 1)},{-400 * (i + 1)},{0 if i % 2 else 1000}" for i in range(10)), None),
    (85, "L", "L 10 stages, dt=50 us, measured",
     "0,0,20000,100000,0,50" + stages(1000), "3,3,0,-1,0,1"),
]

LATE = re.compile(r"WARN engine: (\d+) latch")
OVERDUE = re.compile(r"# engine: (\d+) event")
NEED = re.compile(r"CAL STARTLAT \((\d+) us\)")


def run_case(sj, slot, letter, label, body, meas, out):
    reply = sj.cmd1(f"{letter}{slot},{body}")
    if reply.startswith("ERR"):
        out(f"{label}: DEFINITION REFUSED: {reply}")
        return 1
    if meas is not None:
        sj.cmd1(f"MEAS{slot},{meas}")
    sj.drain(quiet=0.2)
    # The train is 100 ms long, so its completion can land inside the reply to
    # `T` itself -- keep that reply rather than discarding it.
    lines = sj.cmd(f"T{slot}", quiet=0.4)
    deadline = time.time() + 6.0
    while time.time() < deadline:
        lines += sj.drain(quiet=0.4, limit=2.0)
        if any(x.startswith("Train #") for x in lines):
            break
    faults = 0
    out(f"--- {label}")
    for x in lines:
        if x.startswith(("MSUM,", "Train #", "WARN", "ERR")) or OVERDUE.match(x):
            out(f"    {x}")
        if LATE.match(x) or OVERDUE.match(x) or NEED.search(x):
            faults += 1
    if not any(x.startswith("Train #") for x in lines):
        out("    NO COMPLETION LINE")
        faults += 1
    return faults


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM4")
    ap.add_argument("--out", default=None, help="also write everything to this file")
    a = ap.parse_args()

    fh = open(a.out, "w", encoding="utf-8") if a.out else None

    def out(s):
        print(s)
        if fh:
            fh.write(s + "\n")

    faults = 0
    with StimJim(a.port) as sj:
        sj.drain(quiet=0.3)
        sj.reset()
        for line in sj.cmd("IDN"):
            out(line)
        for line in sj.cmd("CAL?"):
            if line.startswith("CAL,"):
                out(line)
        for slot, letter, label, body, meas in CASES:
            faults += run_case(sj, slot, letter, label, body, meas, out)
        sj.reset()

    out("")
    out("no timing faults" if faults == 0 else f"{faults} timing fault line(s)")
    if fh:
        fh.close()
    return faults


if __name__ == "__main__":
    sys.exit(main())
