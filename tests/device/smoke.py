"""On-target smoke test for stimjimAWG over the USB serial port.

Covers what can be checked without an oscilloscope: identity, backward
compatibility of the command grammar, the new per-slot start delay (set both
ways, queried, and timed end-to-end against STAT), and the OLED framebuffer in
each of its three views. Waveform shape and microsecond-accurate delay timing
need the scope -- see capture.py.

    python smoke.py [COM4] [--screens DIR]

Exit code is the number of failed checks.
"""

import argparse
import pathlib
import sys
import time

from sjcon import StimJim

failures = 0
notes = []


def check(cond, what):
    global failures
    if cond:
        print(f"  ok   {what}")
    else:
        failures += 1
        print(f"  FAIL {what}")
    return cond


def eq(got, want, what):
    return check(got == want, f"{what}: {got!r}" + ("" if got == want else f" != {want!r}"))


def screen(sj, name, outdir):
    """Capture the OLED framebuffer as ASCII art."""
    lines = sj.cmd("SCREEN", quiet=0.6, limit=15)
    art = [l for l in lines if l.startswith("|")]
    # The firmware also reports the composed row text, so a capture is readable
    # without decoding the bitmap font.
    text = [l for l in lines if l.startswith("# row") or l.startswith("# bar")]
    if not check(len(art) == 32 and lines[-1] == "OK", f"SCREEN {name}: 32 rows + OK"):
        print("   ", lines[:4])
        return None
    for t in text:
        print("       ", t)
    if outdir:
        p = pathlib.Path(outdir) / f"screen-{name}.txt"
        p.write_text("\n".join(text + art) + "\n", encoding="ascii")
        print(f"       -> {p}")
    return art


def stat(sj):
    # STAT answers with a single line, so a short quiet window is enough --
    # the default 0.3 s would put a floor under the delay measurement below.
    f = sj.cmd1("STAT", quiet=0.06).split(",")
    assert f[0] == "STAT", f
    v = [int(x) for x in f[1:]]
    return {"slot0": v[0], "n0": v[1], "el0": v[2], "dur0": v[3],
            "slot1": v[4], "n1": v[5], "el1": v[6], "dur1": v[7]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM4")
    ap.add_argument("--screens", default=None, help="directory for SCREEN captures")
    a = ap.parse_args()

    if a.screens:
        pathlib.Path(a.screens).mkdir(parents=True, exist_ok=True)

    with StimJim(a.port) as sj:
        sj.drain(quiet=0.3)                    # discard anything in flight

        print("\n[identity]")
        idn = sj.cmd("IDN")
        eq(idn[0].split(",")[0], "IDN", "IDN tag")
        check(any(l.startswith("# build:") for l in idn), "build line present")
        for line in idn:
            print("      ", line)

        print("\n[backward compatibility: a legacy 5-field header]")
        # Exactly the README example line, which predates the delay field.
        r = sj.cmd1("S0,0,1,2000,1000000;100,0,150;-100,-100,200")
        eq(r, "S0,0,1,2000,1000000;100,0,150;-100,-100,200", "legacy line echoes unchanged")
        eq(sj.cmd1("S0?"), "S0,0,1,2000,1000000;100,0,150;-100,-100,200",
           "query returns no delay field")
        eq(sj.cmd1("DELAY0?"), "DELAY0,0", "delay defaults to 0")

        print("\n[delay via the optional 6th header field]")
        r = sj.cmd1("S1,0,3,10000,500000,250000;5000,0,2000")
        eq(r, "S1,0,3,10000,500000,250000;5000,0,2000", "6th field echoed")
        eq(sj.cmd1("S1?"), "S1,0,3,10000,500000,250000;5000,0,2000", "round-trip")
        eq(sj.cmd1("DELAY1?"), "DELAY1,250000", "DELAY query agrees")

        print("\n[delay via the DELAY setter]")
        eq(sj.cmd1("DELAY1,400000"), "DELAY1,400000", "DELAY set echoes")
        eq(sj.cmd1("S1?"), "S1,0,3,10000,500000,400000;5000,0,2000", "S line follows")
        # Documented asymmetry: an S/L/W line fully defines its header, so a
        # line without the field clears the delay.
        sj.cmd1("S1,0,3,10000,500000;5000,0,2000")
        eq(sj.cmd1("DELAY1?"), "DELAY1,0", "waveform line without the field clears it")

        print("\n[delay rejection]")
        check(sj.cmd1("DELAY1,-1").startswith("ERR"), "negative delay refused")
        check(sj.cmd1("S1,0,3,10000,500000,2000000001;5000,0,2000").startswith("ERR"),
              "over-cap delay refused")
        eq(sj.cmd1("DELAY1?"), "DELAY1,0", "refused line left the slot untouched")

        print("\n[screens: browse view]")
        screen(sj, "1-browse", a.screens)

        print("\n[timed delay, end to end]")
        # 1.5 s delay then a 0.5 s train of 2 ms pulses every 10 ms on channel 0
        # in voltage mode. Nothing slow happens between T2 and the poll loop --
        # a SCREEN capture in that window would dominate the measurement.
        eq(sj.cmd1("S2,0,3,10000,500000,1500000;3000,0,2000"),
           "S2,0,3,10000,500000,1500000;3000,0,2000", "test train defined")
        eq(stat(sj)["slot0"], -1, "engine 0 idle before start")

        t_start = time.time()
        r = sj.cmd("T2", quiet=0.06)
        check(any("Started" in l for l in r), f"T2 accepted: {r}")

        t_first = None
        saw_waiting = False
        while time.time() - t_start < 4.0:
            s = stat(sj)
            if s["slot0"] == 2 and s["n0"] == 0 and s["el0"] == 0:
                saw_waiting = True          # armed, attached, nothing emitted yet
            if s["el0"] > 0 or s["n0"] > 0:
                t_first = time.time()
                break
        check(saw_waiting, "slot attached with no pulses while the delay runs")
        if check(t_first is not None, "train started within 4 s"):
            measured = t_first - t_start
            print(f"       measured start delay: {measured*1000:.0f} ms (set 1500 ms)")
            # Deliberately loose: the reference is host-side serial round-trip
            # timing, and a STAT poll costs 60-150 ms of that, so the measured
            # value overshoots by a few poll intervals and varies run to run.
            # This only has to prove the delay is real and about right --
            # capture.py measures it on the scope to better than a microsecond.
            check(1.4 < measured < 2.2, "delay is ~1.5 s (host timing, coarse)")
            notes.append(f"host-timed start delay {measured*1000:.0f} ms for a 1500 ms setting")

        run = stat(sj)
        check(run["n0"] > 0, f"pulses delivered while running (n={run['n0']})")
        eq(run["dur0"], 500000, "duration reported")

        print("\n[completion]")
        deadline = time.time() + 3.0
        done = []
        while time.time() < deadline:
            done += sj.drain(quiet=0.2, limit=1.0)
            if any("complete" in l for l in done):
                break
        check(any("complete" in l for l in done), f"completion line printed: {done}")
        eq(stat(sj)["slot0"], -1, "engine idle again after the train")

        print("\n[screens: waiting and running views]")
        # A long train, so each ~0.6 s framebuffer dump lands well inside the
        # phase it is meant to show.
        sj.cmd1("S3,0,1,20000,8000000,3000000;3000,2000,4000")
        sj.cmd("T3", quiet=0.06)
        time.sleep(1.0)                                  # inside the 3 s delay
        screen(sj, "2-waiting", a.screens)
        while stat(sj)["el0"] == 0 and stat(sj)["n0"] == 0:
            pass                                         # wait out the delay
        time.sleep(0.3)
        screen(sj, "3-running", a.screens)
        sj.cmd("T-1", quiet=0.3)

        print("\n[cleanup]")
        sj.cmd("T-1")
        for slot in (0, 1, 2, 3):
            sj.cmd1(f"S{slot},3,3,10000,500000")

    print()
    for n in notes:
        print("note:", n)
    print(f"\n{'ALL CHECKS PASSED' if failures == 0 else str(failures) + ' CHECK(S) FAILED'}")
    return failures


if __name__ == "__main__":
    sys.exit(main())
