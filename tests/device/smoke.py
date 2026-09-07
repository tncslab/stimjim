"""On-target smoke test for stimjimAWG over the USB serial port.

Covers what can be checked without an oscilloscope: identity, backward
compatibility of the command grammar, the per-slot start delay (set both ways,
queried, and timed end-to-end against STAT), the runtime timing budget (CAL:
query, set, the invariant refusals, CALDEF), the per-slot ramp sample interval
(DT), in-train measurement (MSUM/MDATA, the refusal of a point whose ADC window
does not fit, and the rotation that measures it anyway), SD logging with
length-framed retrieval over the serial port, the wall clock (CLK), the display
pages (PAGE) and the OLED framebuffer on each of them. Waveform shape and
microsecond-accurate delay timing need the scope -- see capture.py.

The SD section is skipped, with a note, when no card is in the socket. Nothing
here needs a panel: SCREEN renders into the framebuffer whether or not one is
connected, so a headless board passes the whole suite.

    python smoke.py [COM4] [--screens DIR]

Exit code is the number of failed checks.
"""

import argparse
import pathlib
import re
import sys
import time
import zlib

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
    text = [l for l in lines if l.startswith("# row") or l.startswith("# bar")
            or l.startswith("# page")]
    if not check(len(art) == 32 and lines[-1] == "OK", f"SCREEN {name}: 32 rows + OK"):
        print("   ", lines[:4])
        return None
    for t in text:
        print("       ", t)
    if outdir:
        p = pathlib.Path(outdir) / f"screen-{name}.txt"
        p.write_text("\n".join(text + art) + "\n", encoding="ascii")
        print(f"       -> {p}")
    # The composed text, not the pixels: that is what a layout check reads.
    return text


def run_train(sj, cmd, wait):
    """Start a train and collect everything it prints, completion included."""
    r = sj.cmd(cmd, quiet=min(wait, 1.0))
    time.sleep(wait)
    r += sj.drain(quiet=0.6)
    return r


def sdget(sj, name, offset=None, length=None):
    """Read a file through SDGET's length-framed transfer.

    Returns (payload, crc32_from_device, file_size). The byte count in the
    header is what makes the payload unambiguous whatever the file contains, so
    the transfer runs on the raw port rather than through the line helper.
    """
    line = "SDGET," + name
    if offset is not None:
        line += f",{offset}"
        if length is not None:
            line += f",{length}"
    ser = sj.ser
    ser.reset_input_buffer()
    ser.write((line + "\n").encode())
    ser.flush()
    hdr = b""
    deadline = time.time() + 5.0
    while not hdr.endswith(b"\n") and time.time() < deadline:
        hdr += ser.read(1) or b""
    fields = hdr.strip().decode("utf-8", "replace").split(",")
    if len(fields) != 5 or fields[0] != "SDGET":
        return None, None, None
    want, total = int(fields[3]), int(fields[4])
    body = b""
    deadline = time.time() + 30.0
    while len(body) < want and time.time() < deadline:
        body += ser.read(want - len(body))
    tail = ser.read(400).decode("utf-8", "replace")
    m = re.search(r"crc32=([0-9a-f]{8})", tail)
    if len(body) != want or not m:
        return None, None, total
    return body, int(m.group(1), 16), total


def stat(sj):
    # STAT answers with a single line, so a short quiet window is enough --
    # the default 0.3 s would put a floor under the delay measurement below.
    f = sj.cmd1("STAT", quiet=0.06).split(",")
    assert f[0] == "STAT", f
    v = [int(x) for x in f[1:]]
    return {"slot0": v[0], "n0": v[1], "el0": v[2], "dur0": v[3],
            "slot1": v[4], "n1": v[5], "el1": v[6], "dur1": v[7]}


def page(sj, quiet=0.3):
    """Current page as (index, count, name), from the PAGE status line."""
    f = sj.cmd1("PAGE?", quiet=quiet).split(",")
    assert f[0] == "PAGE", f
    return int(f[1]), int(f[2]), f[3]


def page_next(sj):
    """Advance one page -- what the page button does -- and report the new state."""
    f = sj.cmd1("PAGE", quiet=0.3).split(",")
    assert f[0] == "PAGE", f
    return int(f[1]), int(f[2]), f[3]


def clk(sj):
    """CLK? as (unix, ms, us_since_boot, src)."""
    lines = sj.cmd("CLK?", quiet=0.3, limit=4)
    rec = [l for l in lines if l.startswith("CLK,")]
    assert rec, lines
    f = rec[0].split(",")
    return int(f[1]), int(f[2]), int(f[3]), f[4]


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

        print("\n[CAL: the runtime timing budget]")
        cal = sj.cmd("CAL?", quiet=0.6, limit=15)
        eq(len(cal), 10, "CAL? prints nine parameters and OK")
        eq(cal[-1], "OK", "CAL? ends with OK")
        for line in cal[:-1]:
            print("      ", line)
        names = [l.split(",")[1] for l in cal[:-1]]
        eq(names[0], "PRELOAD", "first parameter is PRELOAD")
        check("TRIGCOMP" in names, "TRIGCOMP is in the set")
        eq(sj.cmd1("CAL,TRIGCOMP?"), "CAL,TRIGCOMP,0",
           "trigger compensation is 0 until someone measures it")
        settle = sj.cmd1("CAL,SETTLE?")
        eq(sj.cmd1("CAL,SETTLE,6"), "CAL,SETTLE,6", "one parameter set echoes back")
        eq(sj.cmd1("CAL,SETTLE?"), "CAL,SETTLE,6", "and is what the board now uses")
        check(any("cal=custom" in l for l in sj.cmd("IDN")), "IDN reports a custom budget")
        # A preload of 100 us cannot fit inside a 20 us start latency, so the
        # set is refused whole rather than arming an inconsistent budget.
        check(sj.cmd1("CAL,PRELOAD,100").startswith("ERR"),
              "a value breaking the STARTLAT invariant is refused")
        check(sj.cmd1("CAL,NONSENSE,4").startswith("ERR"), "unknown parameter refused")
        check(sj.cmd1("CAL,SETTLE,-1").startswith("ERR"), "negative value refused")
        check(sj.cmd1("CAL,SETTLE,1001").startswith("ERR"), "value above 1000 us refused")
        dump = sj.cmd("DUMP", quiet=0.8, limit=25)
        check(any(l == "CAL,SETTLE,6" for l in dump), "DUMP carries the changed parameter")
        eq(sj.cmd("CALDEF", quiet=0.6, limit=15)[-1], "OK", "CALDEF ends with OK")
        eq(sj.cmd1("CAL,SETTLE?"), settle, "CALDEF restored the build default")
        check(any("cal=default" in l for l in sj.cmd("IDN")), "IDN reports the default budget")

        print("\n[DT: the per-slot ramp sample interval]")
        eq(sj.cmd1("L8,0,1,10000,200000,0,25;4095,1000,1000"),
           "L8,0,1,10000,200000,0,25;4095,1000,1000", "L line with the 7th field echoes")
        eq(sj.cmd1("DT8?"), "DT8,25", "DT query agrees")
        eq(sj.cmd1("DT8,50"), "DT8,50", "DT set echoes")
        eq(sj.cmd1("L8?"), "L8,0,1,10000,200000,0,50;4095,1000,1000", "L line follows")
        # Same asymmetry as DELAY: a line without the field resets the interval.
        sj.cmd1("L8,0,1,10000,200000;4095,1000,1000")
        eq(sj.cmd1("DT8?"), "DT8,0", "waveform line without the field clears it")
        check(sj.cmd1("DT0,25").startswith("ERR"), "DT refused on an S slot")
        check(sj.cmd1("S0,0,1,2000,1000000,0,25;100,0,150").startswith("ERR"),
              "a 7th header field is refused on an S line")
        check(sj.cmd1("L8,0,1,10000,200000,0,1;4095,1000,1000").startswith("ERR"),
              "dt below the parse floor refused")
        # Below what one latch costs on this board: refused at start, since that
        # budget is runtime state and the parser cannot know it.
        sj.cmd1("L8,0,1,10000,200000,0,2;4095,1000,1000")
        r = sj.cmd("T8", quiet=0.4)
        check(any("ramp interval" in l for l in r), f"2 us ramp interval refused at start: {r}")
        sj.cmd("T-1", quiet=0.2)

        print("\n[L: 0-duration stages are level shifts, and only one at a time]")
        eq(sj.cmd1("L8,0,1,10000,200000;1000,0,0;2000,0,500"),
           "L8,0,1,10000,200000;1000,0,0;2000,0,500", "a single 0-duration jump is accepted")
        check(sj.cmd1("L8,0,1,10000,200000;1000,0,0;2000,0,0;3000,0,500").startswith("ERR"),
              "two 0-duration L stages in a row refused")
        # S stages are rectangular steps, not jumps: any number of 0-duration
        # ones stays legal there (bit-exact legacy semantics).
        eq(sj.cmd1("S8,0,1,10000,200000;1000,0,0;2000,0,0;3000,0,500"),
           "S8,0,1,10000,200000;1000,0,0;2000,0,0;3000,0,500", "S keeps 0-duration steps")

        print("\n[screens: status page, and the page walk]")
        # Page 0 is always STATUS and the last page is always SYSTEM; how many
        # sit between them depends on what the last train measured, which on a
        # re-run is whatever the previous run left behind. So the suite selects
        # rather than assuming, and checks the two ends of the list.
        _, count, name = page(sj)
        check(count >= 2, f"at least STATUS and SYSTEM exist: {count}")
        f = sj.cmd1("PAGE,0", quiet=0.3).split(",")
        eq(f[3], "STATUS", "page 0 is STATUS")
        idx = int(f[1])
        eq(idx, 0, "PAGE,0 selects page 0")
        first = screen(sj, "1-status", a.screens) or []
        # SCREEN works with or without a panel -- the framebuffer is composed
        # either way -- so the suite runs headless. Which of the two this is
        # goes in the notes rather than in a check.
        pline = [l for l in first if l.startswith("# page")]
        check(bool(pline), f"SCREEN names the page and the panel state: {pline}")
        if pline and "panel absent" in pline[0]:
            notes.append("no OLED panel connected: SCREEN rendered the framebuffer only")
        # Advancing wraps, and every page has to render.
        eq(sj.cmd1(f"PAGE,{count - 1}", quiet=0.3).split(",")[3], "SYSTEM",
           "the last page is SYSTEM")
        screen(sj, "2-system", a.screens)
        eq(page_next(sj)[0], 0, "PAGE wraps from the last page back to 0")
        check(sj.cmd1(f"PAGE,{count}").startswith("ERR"), "PAGE past the end is refused")
        eq(page(sj)[0], 0, "a refused PAGE leaves the page alone")

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

        print("\n[measurement: MSUM, MDATA and the fit refusal]")
        # 2 ms stages, V+I on both channels: the widest measurement point there
        # is, and it must fit without pushing a latch late.
        sj.cmd1("S4,0,1,10000,50000;5000,1000,2000;-5000,-1000,2000")
        r = run_train(sj, "T4", 1.2)
        msum = [l for l in r if l.startswith("MSUM,")]
        check(len(msum) == 2, f"one MSUM line per stage: {len(msum)}")
        check(not any(l.startswith("WARN engine") for l in r),
              "no latch missed its deadline while measuring")
        if len(msum) == 2:
            f0 = msum[0].split(",")
            eq(len(f0), 12, "MSUM field count")
            eq(f0[3], "0", "first MSUM point is stage 0")
            eq(msum[1].split(",")[3], "1", "second MSUM point is stage 1")
            eq(f0[2], "5", "MSUM n = pulses delivered")
            # 5 V into the bench divider reads a few hundred mV low, so only the
            # sign and the order of magnitude are checked here.
            check(1000.0 < float(f0[4]) < 6000.0, f"stage 0 V0 plausible: {f0[4]}")
            check(float(msum[1].split(",")[4]) < -1000.0,
                  "stage 1 V0 is the negative phase")

            # The panel reads the same accumulators, so the RESULT pages appear
            # as soon as a measured train completes and the firmware jumps to
            # the first of them.
            idx, count, name = page(sj)
            eq(name, "RESULT", "a measured completion jumps the panel to a RESULT page")
            eq(idx, 1, "... to the first one")
            eq(count, 4, "two measurement points give STATUS + 2 RESULT + SYSTEM")
            rowtext = [l for l in (screen(sj, "4-result", a.screens) or [])
                       if l.startswith("# row")]
            check(len(rowtext) == 4, f"RESULT page composes four rows: {len(rowtext)}")
            check(any(l.startswith('# row1: "V') for l in rowtext), "row 1 is the V row")
            check(any(l.startswith('# row2: "I') for l in rowtext), "row 2 is the I row")
            check(any(l.startswith('# row3: "R') for l in rowtext), "row 3 is the R row")
            # The panel prints millivolts to one decimal below 1000 and none
            # above, so a value matching MSUM to a few counts is what to expect.
            m = re.search(r'row1: "V'+r"\s*(-?[0-9.]+)", "".join(rowtext))
            if check(m is not None, "the V row carries a number for channel 0"):
                check(abs(float(m.group(1)) - float(f0[4])) < 1.0,
                      f"RESULT V0 {m.group(1)} agrees with MSUM {f0[4]}")
            eq(page_next(sj)[0], 2, "the second measurement point is the next page")
            screen(sj, "5-result2", a.screens)

        # report bit 0 streams one MDATA line per point per pulse.
        sj.cmd1("MEAS4,3,3,0,-1,1")
        r = run_train(sj, "T4", 1.2)
        mdata = [l for l in r if l.startswith("MDATA,")]
        eq(len(mdata), 10, "MDATA lines = pulses x points")
        if mdata:
            eq(len(mdata[0].split(",")), 8, "MDATA field count")

        # A stage too short for its measurement window (20 us against the 47 us
        # four reads need) is refused, not squeezed -- with fit = 0, which is
        # what the 7th MEAS field selects.
        sj.cmd1("MEAS4,3,3,0,-1,0,0")
        sj.cmd1("S5,0,1,10000,30000;5000,1000,20;0,0,1000")
        sj.cmd1("MEAS5,3,3,0,-1,0,0")
        r = run_train(sj, "T5", 1.0)
        check(any(l.startswith("WARN MEAS:") and "not measured" in l for l in r),
              "short stage, fit=0: measurement refused with a reason")
        skipped = [l for l in r if l.startswith("MSUM,5,0,0,")]
        check(bool(skipped), f"refused point still reports an MSUM line: {skipped}")

        print("\n[measurement coverage: reads rotated over repetitions]")
        # Stage 0 is 30 us: too short for four reads in one gap (47 us needed
        # with the measured 9 us SETTLE) but long enough for one (26 us), so
        # fit = 1 measures all four lines over four consecutive pulses. Stage 1
        # is long and does not rotate. 20 pulses, so each line of stage 0
        # accumulates about five samples.
        sj.cmd1("S6,0,1,10000,200000;5000,1000,30;-5000,-1000,1000")
        sj.cmd1("MEAS6,3,3,0,-1,0,0")
        r = run_train(sj, "T6", 1.6)
        check(any(l.startswith("WARN MEAS:") and "not measured" in l for l in r),
              "fit=0: the 30 us stage is refused")
        sj.cmd1("MEAS6,3,3,0,-1,0,1")
        r = run_train(sj, "T6", 1.6)
        check(not any(l.startswith("WARN MEAS:") for l in r),
              "fit=1: nothing refused any more")
        rot = [l for l in r if l.startswith("# MEAS:") and "rotate" in l]
        check(bool(rot), f"rotation is announced: {rot}")
        if rot:
            print("      ", rot[0])
        msum6 = [l for l in r if l.startswith("MSUM,6,")]
        check(len(msum6) == 2, f"both stages summarised: {len(msum6)}")
        if len(msum6) == 2:
            f0 = msum6[0].split(",")
            # 20 pulses over four groups: the field carries the largest of the
            # four lines' counts, so 5 or 6.
            check(4 <= int(f0[2]) <= 6, f"rotated point n is about nPulses/4: {f0[2]}")
            check(all(f0[i] for i in (4, 6, 8, 10)),
                  f"all four lines got a reading: {f0[4:]}")
            eq(msum6[1].split(",")[2], "20", "the long stage still measures every pulse")
        check(any(l.startswith("# MSUM point 0:") and "rotated" in l for l in r),
              "the summary repeats how the point was measured")

        # The same coverage on an L ramp, where the free gap is the sample
        # interval: refused at the default 20 us, measured at 30 us.
        sj.cmd1("L7,0,1,10000,100000;4095,1000,1000")
        sj.cmd1("MEAS7,3,3,0,-1,0,1")
        r = run_train(sj, "T7", 1.0)
        check(any(l.startswith("WARN MEAS:") for l in r),
              "L at the default 20 us interval: not even one read fits")
        sj.cmd1("DT7,30")
        r = run_train(sj, "T7", 1.0)
        check(not any(l.startswith("WARN MEAS:") for l in r),
              "L at a 30 us interval: rotated and measured")
        check(any(l.startswith("MSUM,7,") for l in r), "ramp point summarised")

        print("\n[SD: log, listing and framed retrieval]")
        info = sj.cmd1("SDINFO").split(",")
        eq(info[0], "SD", "SDINFO record word")
        if info[1] != "1":
            notes.append("no SD card in the socket - SD checks skipped")
        else:
            name = "SMOKE.CSV"
            sj.cmd("SDDEL," + name)                     # ignore "does not exist"
            opened = sj.cmd1("LOG1," + name).split(",")
            eq(opened[0], "LOG", "LOG record word")
            eq(opened[1], "1", "log reports itself open")
            eq(opened[2], name, "log file name")
            check(int(opened[3]) > 0, f"header written: {opened[3]} bytes")

            sj.cmd1("MEAS4,3,3,0,-1,2")                 # report bit 1: rows to SD
            run_train(sj, "T4", 1.2)
            closed = sj.cmd1("LOG0").split(",")
            eq(closed[1], "0", "log reports itself closed")
            rows_bytes = int(closed[3]) - int(opened[3])
            check(rows_bytes > 0, f"rows appended: {rows_bytes} bytes")

            listing = sj.cmd("SDLIST", quiet=0.6, limit=15)
            check(any(l.startswith("SDLIST," + name + ",") for l in listing),
                  f"{name} appears in SDLIST")
            eq(listing[-1], "OK", "SDLIST ends with OK")

            body, crc, total = sdget(sj, name)
            check(body is not None, "SDGET returned its announced byte count")
            if body is not None:
                eq(zlib.crc32(body) & 0xFFFFFFFF, crc, "SDGET crc32 matches the payload")
                eq(len(body), total, "default SDGET length is the whole file")
                text = body.decode("utf-8", "replace")
                check(text.startswith("# stimjimAWG log"), "log starts with its own header")
                check("IDN,stimjimAWG" in text, "log header carries the identity line")
                check("# train: S4," in text, "log records the waveform each train played")
                check("# columns: timestamp_us,slot,pulse,point," in text,
                      "log header carries the column names")
                data = [l for l in text.splitlines() if l and l[0].isdigit()]
                check(len(data) == 10, f"one CSV row per point per pulse: {len(data)}")
                if data:
                    eq(len(data[0].split(",")), 8, "CSV row field count")

                # A partial read must frame and check the chunk, not the file.
                chunk, crc2, total2 = sdget(sj, name, 0, 40)
                eq(len(chunk or b""), 40, "40-byte chunk length honoured")
                eq(total2, total, "chunk header still reports the full size")
                if chunk:
                    eq(zlib.crc32(chunk) & 0xFFFFFFFF, crc2, "chunk crc32 matches")

            eq(sj.cmd("SDDEL," + name)[-1], "OK", "SDDEL removes the file")
            check(not any(l.startswith("SDLIST," + name + ",")
                          for l in sj.cmd("SDLIST", quiet=0.6, limit=15)),
                  f"{name} is gone from SDLIST")
        sj.cmd1("MEAS4,3,3,0,-1,0")

        print("\n[screens: waiting and running views]")
        # A long train, so each ~0.6 s framebuffer dump lands well inside the
        # phase it is meant to show.
        sj.cmd1("S3,0,1,20000,8000000,3000000;3000,2000,4000")
        sj.cmd("T3", quiet=0.06)
        time.sleep(1.0)                                  # inside the 3 s delay
        # The engine copies the budget at arm time, so changing it mid-train
        # would describe a board the running waveform is not using.
        check(sj.cmd1("CAL,SETTLE,5").startswith("ERR"), "CAL refused while a train runs")
        check(sj.cmd1("CALDEF").startswith("ERR"), "CALDEF refused while a train runs")
        sj.cmd1("PAGE,0", quiet=0.3)   # a running train lives on STATUS; the page is sticky
        screen(sj, "2-waiting", a.screens)
        while stat(sj)["el0"] == 0 and stat(sj)["n0"] == 0:
            pass                                         # wait out the delay
        time.sleep(0.3)
        screen(sj, "3-running", a.screens)
        sj.cmd("T-1", quiet=0.3)

        print("\n[clock]")
        sec0, ms0, us0, src0 = clk(sj)
        check(src0 in ("build", "batt", "host"), f"CLK reports a source: {src0}")
        check(us0 > 0, "CLK carries the microsecond timebase, not only the RTC")
        iso = [l for l in sj.cmd("CLK?", quiet=0.3, limit=4) if l.startswith("# clock:")]
        check(bool(iso), f"CLK? explains its source: {iso}")
        # Setting it moves the source to host and the reading to what was sent;
        # the microsecond column is monotonic across the change, because it is a
        # different clock and nothing here touches it.
        host_unix = int(time.time())
        sj.cmd("CLK,%d,250" % host_unix, quiet=0.3)
        sec1, ms1, us1, src1 = clk(sj)
        eq(src1, "host", "CLK,<unix> moves the source to host")
        check(abs(sec1 - host_unix) <= 2, f"the RTC now reads what was sent: {sec1} vs {host_unix}")
        check(us1 > us0, "the microsecond timebase kept running across the set")
        check(sj.cmd1("CLK,%d,1000" % host_unix).startswith("ERR"), "ms > 999 refused")

        print("\n[cleanup]")
        sj.cmd("T-1")
        for slot in (0, 1, 2, 3, 4, 5, 6, 7, 8):
            sj.cmd1(f"S{slot},3,3,10000,500000")

    print()
    for n in notes:
        print("note:", n)
    print(f"\n{'ALL CHECKS PASSED' if failures == 0 else str(failures) + ' CHECK(S) FAILED'}")
    return failures


if __name__ == "__main__":
    sys.exit(main())
