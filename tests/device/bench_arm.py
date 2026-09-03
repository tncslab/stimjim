"""Sweep BENCHARM over representative waveform slots.

`CAL STARTLAT` is the trigger-to-output latency, and what sizes it is the cost
of `Engine::startTrain` -- every microsecond the arm spends comes out of the
latency rather than being added to it. This script defines one slot per shape
the arm treats differently, runs `BENCHARM` on each, and prints the table plus
what `STARTLAT` each case needs (`arm + PRELOAD + DACPROG2 + 3 us`, read from
the board's own `CAL` set).

It measures only; nothing is played and the outputs stay parked, because
`BENCHARM` stops the train inside the same bus lock it armed it in. Safe to run
with a load connected and with nothing connected.

    python bench_arm.py COM4                 # the full sweep
    python bench_arm.py COM4 --reps 500      # more repetitions per case
    python bench_arm.py COM4 --csv arm.csv   # also write the table

Slots 90-97 are used and left holding these definitions; slots 0-9 (the ones
`P` persists) are untouched.
"""

import argparse
import csv
import sys

from sjcon import StimJim

# One case per shape the arm treats differently. `meas` is the MEAS line body
# (None = leave the slot's default, which measures V+I on both channels).
# Mode 90/91 mean voltage/current with measurement off, which is how a case
# isolates the plan cost from the geometry cost.
CASES = [
    # slot, letter, label,                     train line body,                                  meas
    (90, "S", "empty, drives nothing",      "3,3,10000,500000",                                   None),
    (91, "S", "S 1 stage, 1 ch, no meas",   "90,3,10000,500000;5000,0,1000",                      None),
    (92, "S", "S 1 stage, 2 ch, no meas",   "90,90,10000,500000;5000,5000,1000",                  None),
    (93, "S", "S 1 stage, 2 ch, V+I both",  "0,0,10000,500000;5000,5000,1000",                    "3,3,0,-1,0,1"),
    (94, "S", "S 10 stages, 2 ch, no meas", "90,90,10000,500000" + ";1000,1000,100" * 10,         None),
    (95, "L", "L 1 stage, 2 ch, no meas",   "90,90,10000,500000,0,50;5000,5000,1000",             None),
    (96, "L", "L 10 stages, 2 ch, no meas", "90,90,10000,500000,0,50" + ";1000,1000,100" * 10,    None),
    (97, "W", "W sine, 2 ch, no meas",      "90,90,10000,500000;5000,5000,5000;1000,1000,0;0,0,0", None),
]


def parse_stat(line):
    """Parse a BENCH stat line into (n, min_us, avg_us, max_us).

    BENCH,ARM,n=200,cycles(min/avg/max)=1944/1955/1990,ns=16200/16292/16583
    The ns field is used rather than the cycles one, so the result does not
    depend on knowing the board's clock.
    """
    fields = {}
    for part in line.split(",")[2:]:
        k, _, v = part.partition("=")
        fields[k] = v
    mn, avg, mx = (int(x) / 1000.0 for x in fields["ns"].split("/"))
    return int(fields["n"]), mn, avg, mx


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM4")
    ap.add_argument("--reps", type=int, default=200, help="BENCHARM repetitions per case")
    ap.add_argument("--csv", default=None, help="also write the table to this file")
    a = ap.parse_args()

    with StimJim(a.port) as sj:
        sj.drain(quiet=0.3)
        sj.reset()

        # The board's own budget decides what STARTLAT each arm needs; read it
        # rather than assuming the Config.h defaults are still in force.
        cal = {}
        for line in sj.cmd("CAL?"):
            f = line.split(",")
            if len(f) == 3 and f[0] == "CAL":
                cal[f[1]] = int(f[2])
        overhead = cal["PRELOAD"] + cal["DACPROG2"] + 3
        print(f"CAL PRELOAD={cal['PRELOAD']} DACPROG2={cal['DACPROG2']} "
              f"STARTLAT={cal['STARTLAT']} us  ->  needed STARTLAT = arm + {overhead} us")

        rows = []
        for slot, letter, label, body, meas in CASES:
            reply = sj.cmd1(f"{letter}{slot},{body}")
            if reply.startswith("ERR"):
                print(f"  {label:32s} SKIPPED: {reply}")
                continue
            if meas is not None:
                sj.cmd1(f"MEAS{slot},{meas}")
            stat = None
            for line in sj.cmd(f"BENCHARM,{slot},{a.reps}", limit=30.0):
                if line.startswith("BENCH,ARM"):
                    stat = line
            if stat is None:
                print(f"  {label:32s} FAILED: no BENCH,ARM line")
                continue
            n, mn, avg, mx = parse_stat(stat)
            need = mx + overhead
            rows.append((label, n, mn, avg, mx, need))
            flag = "" if need <= cal["STARTLAT"] else "  <-- exceeds CAL STARTLAT"
            print(f"  {label:32s} {mn:6.2f} /{avg:6.2f} /{mx:6.2f} us   "
                  f"needs {need:6.1f}{flag}")

        # A TRIG independent route arms two engines from one edge, so it pays
        # the arm twice before the second engine's first latch is due.
        if rows:
            worst = max(r[4] for r in rows)
            print(f"\nworst single arm {worst:.2f} us -> STARTLAT {worst + overhead:.0f} us; "
                  f"an independent two-engine route of the same slot needs "
                  f"{2 * worst + overhead:.0f} us")

        if a.csv:
            with open(a.csv, "w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(["case", "n", "min_us", "avg_us", "max_us", "startlat_needed_us"])
                w.writerows(rows)
            print(f"wrote {a.csv}")

        sj.reset()
    return 0


if __name__ == "__main__":
    sys.exit(main())
