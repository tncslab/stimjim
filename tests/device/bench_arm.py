"""Sweep BENCHARM over representative waveform slots.

`CAL STARTLAT` is the trigger-to-output latency, and what sizes it is the cost
of `Engine::startTrain` -- every microsecond the arm spends comes out of the
latency rather than being added to it. This script defines one slot per shape
the arm treats differently, runs `BENCHARM` on each, and prints the table plus
what `STARTLAT` each case needs (`arm + PRELOAD + DACPROG2 + 3 us`, read from
the board's own `CAL` set).

Each case is measured in three columns, because the arm has three regimes:

  worst    BENCHARM's max over `--reps` back-to-back arms inside one bus lock.
           loop() never runs between them, so this is the arm of an engine
           re-triggered before loop() could prepare anything -- the fallback
           path, and the only one that still compiles a plan.
  typical  BENCHARM's min over the same run.
  warmed   one arm per BENCHARM invocation, so a loop() pass falls between
           them and prepares the buffer the next arm takes. This is what a
           running board delivers, and the column the firmware is built around.

The warmed column has to be measured with separate invocations: BENCHARM's own
repetitions run back to back inside one bus lock, alternating between the two
plan buffers with no loop() pass to warm the one that is not live. It also sets
a trigger route, because loop() prepares the slot a `TRIG` route names before
it falls back to the last slot written. Input 1 is used and restored to its
previous entry at the end -- an edge on it while the route is set would start
the case's train, so leave input 1 undriven for the duration.

It measures only; nothing is played and the outputs stay parked, because
`BENCHARM` stops the train inside the same bus lock it armed it in. Safe to run
with a load connected and with nothing connected.

    python bench_arm.py COM4                 # the full sweep
    python bench_arm.py COM4 --reps 500      # more repetitions per case
    python bench_arm.py COM4 --csv arm.csv   # also write the table
    python bench_arm.py COM4 --no-warm       # skip the routed column

Slots 90-98 are used and left holding these definitions; slots 0-9 (the ones
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
    # The heaviest plan the firmware can be given: one measurement point per
    # stage, V and I on both channels. This is the case that sizes STARTLAT.
    (98, "S", "S 10 stages, 10 MEAS points",  "0,0,10000,500000" + ";1000,1000,100" * 10,  "3,3,0,-1,0,1"),
]


# Separate BENCHARM invocations for the warmed column, one arm each, so a loop()
# pass falls between them. Enough of them that both plan buffers get warmed and
# the worst of the series is what a triggered board would deliver.
WARM_ARMS = 6


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


def bench(sj, slot, reps):
    """One BENCHARM run -> (min_us, max_us), or None when the board refused."""
    stat = None
    for line in sj.cmd(f"BENCHARM,{slot},{reps}", limit=30.0):
        if line.startswith("BENCH,ARM"):
            stat = line
    if stat is None:
        return None
    _, mn, _, mx = parse_stat(stat)
    return mn, mx


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM4")
    ap.add_argument("--reps", type=int, default=200, help="BENCHARM repetitions per case")
    ap.add_argument("--csv", default=None, help="also write the table to this file")
    ap.add_argument("--no-warm", action="store_true",
                    help="skip the routed column (leaves the trigger table untouched)")
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

        # Input 1's entry, restored at the end -- the warmed column has to route
        # a slot at engine 0 to make loop() prepare it.
        saved = None
        if not a.no_warm:
            # TRIG1? answers `TRIG1,<mode>,<slot0>,<slot1>,<edge>`; everything
            # after the command word is what TRIG1,... takes back.
            for line in sj.cmd("TRIG1?"):
                f = line.split(",")
                if len(f) == 5 and f[0] == "TRIG1":
                    saved = ",".join(f[1:])
            if saved is None:
                print("  (could not read TRIG input 1 — warmed column skipped)")
                a.no_warm = True

        print(f"  {'case':32s} {'worst':>7} {'typical':>7} {'warmed':>7}   needs")
        rows = []
        for slot, letter, label, body, meas in CASES:
            if not a.no_warm:
                sj.cmd1("TRIG1,0,-1,-1,0")        # unrouted for the back-to-back run
            reply = sj.cmd1(f"{letter}{slot},{body}")
            if reply.startswith("ERR"):
                print(f"  {label:32s} SKIPPED: {reply}")
                continue
            if meas is not None:
                sj.cmd1(f"MEAS{slot},{meas}")
            r = bench(sj, slot, a.reps)
            if r is None:
                print(f"  {label:32s} FAILED: no BENCH,ARM line")
                continue
            typical, worst = r

            warmed = ""
            if not a.no_warm:
                # Route the slot at engine 0, then bump the definition epoch so
                # the plan is stale again, and give loop() a pass to warm it.
                sj.cmd1(f"TRIG1,1,{slot},-1,0")
                sj.cmd1(f"{letter}{slot},{body}")
                if meas is not None:
                    sj.cmd1(f"MEAS{slot},{meas}")
                # One arm per BENCHARM, so loop() runs in between and gets to
                # prepare the buffer the next arm takes. A single BENCHARM with
                # many repetitions cannot show this: its arms run back to back
                # inside one bus lock, alternating between the two plan buffers
                # with no loop() pass to warm the one that is not live.
                warm = [bench(sj, slot, 1) for _ in range(WARM_ARMS)]
                warm = [w[1] for w in warm if w is not None]
                if warm:
                    warmed = max(warm)

            # The number STARTLAT has to cover is the worst arm the board can
            # actually deliver: the warmed one where a route exists, the cold
            # one where it does not.
            need = (warmed if warmed != "" else worst) + overhead
            rows.append((label, typical, worst, warmed, need))
            flag = "" if need <= cal["STARTLAT"] else "  <-- exceeds CAL STARTLAT"
            ws = f"{warmed:7.2f}" if warmed != "" else f"{'-':>7}"
            print(f"  {label:32s} {worst:7.2f} {typical:7.2f} {ws}   {need:6.1f}{flag}")

        if saved is not None:
            sj.cmd1(f"TRIG1,{saved}")

        # A TRIG independent route arms two engines from one edge, so it pays
        # the arm twice before the second engine's first latch is due.
        if rows:
            worst = max(r[4] - overhead for r in rows)
            print(f"\nworst arm {worst:.2f} us -> STARTLAT {worst + overhead:.0f} us; "
                  f"an independent two-engine route of the same slot needs "
                  f"{2 * worst + overhead:.0f} us")

        if a.csv:
            with open(a.csv, "w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(["case", "typical_us", "worst_us", "warmed_us", "startlat_needed_us"])
                w.writerows(rows)
            print(f"wrote {a.csv}")

        sj.reset()
    return 0


if __name__ == "__main__":
    sys.exit(main())
