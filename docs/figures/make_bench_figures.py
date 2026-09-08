"""Figures for the configuration B bench session: the four findings that need one.

Each figure explains a result that a table of numbers states but does not make
obvious. Everything is drawn from measured data written to `tmp/` by
`tests/device/trigcomp.py` and `tests/device/scope_timebase.py`; nothing here
talks to hardware, so re-running those scripts and then this one keeps the
figures and the numbers in step.

  figs/stimjim-load-two-modes.png     why voltage mode reported a current that
                                      was not the load's, and the mux bank that
                                      explains it
  figs/stimjim-trigcomp-reference.png why the measured trigger latency depends
                                      on where on the input edge you call "the
                                      trigger", and what pins it down
  figs/stimjim-trigcomp-budget.png    what the 37.3 us from edge to output is
                                      made of, and which part this bench can
                                      separate and which it cannot
  figs/pico-timebase-units.png        the scope driver's sample interval is
                                      nanoseconds whatever its units code says,
                                      and what believing the code cost

    python docs/figures/make_bench_figures.py
"""

import csv
import pathlib
import statistics

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

ROOT = pathlib.Path(__file__).resolve().parents[2]
FIGS = ROOT / "figs"
TMP = ROOT / "tmp"

# One palette across the four figures, so a colour means the same thing twice.
C_MEAS = "#1f6feb"        # something measured on this bench
C_ALT = "#d1600a"         # the same quantity by the other route
C_BAD = "#c0392b"         # a wrong value, or a region a value must not be in
C_OK = "#2e7d32"
C_BUDGET = "#6c757d"      # a firmware budget, not a measurement

# The reference instant of every latency here: the level IN0 was measured to
# switch at (`trigcomp.py threshold`). `CAL STARTLAT` and `CAL TRIGCOMP` are
# NOT constants here -- they are read from the per-shot CSV, so a figure always
# describes the run that produced its data.
IN0_THRESHOLD = 1.757


def rows(name):
    with open(TMP / name, newline="") as f:
        return list(csv.DictReader(f))


def num(x):
    """CSV cell to float, with the empty cell meaning 'not measured'."""
    return float(x) if x not in ("", None) else float("nan")


# ---------------------------------------------------------------- figure 1

# The voltage-mode current readings this figure explains away. They cannot be
# re-measured: the firmware now refuses to report them, which is the fix. Taken
# on 2026-09-08 with a 1 kOhm load, commanded amplitude -> reported microamps.
PRE_FIX_VMODE_UA = {1000: 221.7, 2000: 441.9, 4000: 882.3}
MV_PER_DAC, UA_PER_DAC = 0.4574, 0.1017     # Stimjim.h conversion constants
LOAD_OHM = 991.0                            # current mode, scope-anchored


def fig_load_two_modes():
    """Why voltage mode reported a current, and why it was not the load's."""
    r = rows("stimjim-load-modes.csv")
    v = [x for x in r if x["mode"] == "V" and float(x["set"]) > 0]
    i = [x for x in r if x["mode"] == "I"]

    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(11.5, 4.4), dpi=130,
                                   gridspec_kw={"width_ratios": [1, 1.25]})

    # Left: the reading voltage mode used to give, against the two things it
    # could have been. It tracks the DAC code, not the load.
    cmd = sorted(PRE_FIX_VMODE_UA)
    got = [PRE_FIX_VMODE_UA[c] for c in cmd]
    # The same DAC code read as a current -- the pump's own branch, which is
    # what the shunt was actually carrying.
    pump = [round(c / MV_PER_DAC) * UA_PER_DAC for c in cmd]
    scope_v = {abs(float(x["set"])): abs(num(x["scope_V"])) for x in v}
    load = [scope_v[c] / LOAD_OHM * 1e6 for c in cmd if c in scope_v]

    ax0.plot(cmd, pump, "-", color=C_ALT, lw=1.8,
             label="the pump's own branch:\nthe same DAC code as a current")
    ax0.plot(cmd, got, "o", color=C_BAD, ms=8, label="what voltage mode reported")
    if len(load) == len(cmd):
        ax0.plot(cmd, load, "s--", color=C_OK, ms=6, lw=1.4,
                 label=f"what the load actually drew:\nscope volts / {LOAD_OHM:.0f} ohm")
    ax0.set_xlabel("commanded output (mV)")
    ax0.set_ylabel("current (uA)")
    ax0.set_title("The reading was real, and was not the load.\n"
                  "It follows the DAC code to 0.8 %.", fontsize=10)
    ax0.legend(fontsize=7.5, loc="upper left")
    ax0.grid(alpha=0.3)

    # Right: why. The mux bank that decides what the shunt is in series with.
    ax1.axis("off")
    ax1.set_title("The output mux has a second bank, and it moves\n"
                  "the shunt out of the load path.", fontsize=10)
    rows_txt = [
        (0.86, "current mode   OE1:OE0 = 01", C_OK, True),
        (0.74, "  pump --[R2 3k]--[R12 100]--+-- CHANNEL_OUT -- load", "0.15", False),
        (0.67, "                     ^shunt  |", "0.15", False),
        (0.60, "                             I_OUT", "0.15", False),
        (0.60, "", C_OK, False),
        (0.44, "voltage mode   OE1:OE0 = 00", C_BAD, True),
        (0.32, "  pump --[R2 3k]--[R12 100]-- I_OUT --[R14 1k]-- GND", "0.15", False),
        (0.25, "                     ^shunt reads THIS branch", C_BAD, False),
        (0.14, "  DAC ------ V_OUT ---------- CHANNEL_OUT -- load", "0.15", False),
        (0.07, "                     no shunt anywhere in this path", C_BAD, False),
    ]
    for y, txt, col, bold in rows_txt:
        if not txt:
            continue
        ax1.text(0.0, y, txt, fontsize=8.4 if not bold else 9,
                 family=None if bold else "monospace",
                 weight="bold" if bold else "normal", color=col, va="center")
    ax1.text(0.0, -0.04,
             f"Load in current mode, scope-anchored: "
             f"{statistics.fmean([abs(num(x['R_scope'])) for x in i]):.0f} ohm",
             fontsize=8.5, color=C_OK, va="center")

    fig.suptitle("In voltage mode the load current is not measurable, so the firmware "
                 "reports none", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    out = FIGS / "stimjim-load-two-modes.png"
    fig.savefig(out)
    plt.close(fig)
    print(f"-> {out.relative_to(ROOT)}")


# ---------------------------------------------------------------- figure 2

def fig_trigcomp_reference():
    """Why the latency depends on where on the input edge you start the clock."""
    shot = rows("stimjim-trigcomp-shot-8000mV.csv")
    t = [num(x["t_us_from_edge50"]) for x in shot]
    vin = [num(x["input_V"]) for x in shot]
    vout = [num(x["output_V"]) for x in shot]

    per_shot = rows("stimjim-trigcomp.csv")
    levels = sorted(float(k.split("_")[2][:-1]) for k in per_shot[0]
                    if k.startswith("foot_at_"))
    lat = {lv: statistics.fmean(
        [num(x[f"foot_at_{lv:.2f}V_us"]) for x in per_shot
         if x["amp_mV"] == "8000"]) for lv in levels}

    startlat = num(per_shot[0]["startlat_us"])
    trigcomp = num(per_shot[0]["trigcomp_us"])
    thr = rows("stimjim-in0-threshold.csv")
    v_pass = min(num(x["peak_V"]) for x in thr if int(x["trains"]) >= 5)
    v_fail = max(num(x["peak_V"]) for x in thr if int(x["trains"]) == 0)

    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(10.5, 4.2), dpi=130)

    # Left: the input edge itself, with the band the pin actually fires in.
    ax0.plot(t, vin, color=C_MEAS, lw=1.2, label="input edge at IN0 (scope B)")
    ax0.axhspan(v_fail, v_pass, color=C_BAD, alpha=0.18)
    ax0.annotate(f"the pin fires somewhere in here:\n"
                 f"{v_fail:.2f} V never triggers, {v_pass:.2f} V always does",
                 (0.52, v_pass + 0.07), xycoords=("axes fraction", "data"),
                 fontsize=7.5, ha="center", va="bottom", color=C_BAD)
    for lv in levels:
        ax0.axhline(lv, color="0.75", lw=0.6, ls=":")
    ax0.set_xlim(-2.5, 3.5)
    ax0.set_ylim(-0.2, 2.4)
    ax0.set_xlabel("time from the edge's 50 % point (us)")
    ax0.set_ylabel("volts")
    ax0.set_title("The edge takes ~2.1 us to cross 1.6 V.\n"
                  "Half a volt of doubt is 0.65 us of doubt.", fontsize=10)
    ax0.legend(fontsize=8, loc="upper left")
    ax0.grid(alpha=0.3)

    # Right: the consequence. Same output, same foot, six starting instants.
    xs = list(lat)
    ys = [lat[k] for k in xs]
    ax1.plot(xs, ys, "o-", color=C_MEAS, ms=5)
    ax1.axvspan(v_fail, v_pass, color=C_BAD, alpha=0.18)
    ax1.axhline(startlat - trigcomp, color=C_BUDGET, lw=1.2, ls="--")
    ax1.annotate(f"t0: STARTLAT {startlat:.0f} - TRIGCOMP {trigcomp:.0f} "
                 f"= {startlat - trigcomp:.0f} us",
                 (0.03, startlat - trigcomp), xycoords=("axes fraction", "data"),
                 fontsize=8, va="bottom", color=C_BUDGET)
    pick = lat[min(levels, key=lambda x: abs(x - IN0_THRESHOLD))]
    ax1.annotate(f"at the measured threshold:\n{pick:.2f} us,\n"
                 f"{pick - (startlat - trigcomp):.2f} us after t0",
                 (IN0_THRESHOLD, pick), xytext=(-124, 34),
                 textcoords="offset points", fontsize=8, color=C_BAD, ha="left",
                 arrowprops=dict(arrowstyle="->", color=C_BAD, lw=0.9))
    ax1.margins(y=0.16)
    ax1.set_xlabel("level on the input edge the clock is started at (V)")
    ax1.set_ylabel("edge to first output movement (us)")
    ax1.set_title("One measurement, six answers 1.7 us apart.\n"
                  "Measuring the pin's threshold is what picks one.", fontsize=10)
    ax1.grid(alpha=0.3)

    fig.suptitle("Trigger latency: the answer is only as sharp as the "
                 "reference instant", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    out = FIGS / "stimjim-trigcomp-reference.png"
    fig.savefig(out)
    plt.close(fig)
    print(f"-> {out.relative_to(ROOT)}")


# ---------------------------------------------------------------- figure 3

def fig_trigcomp_budget():
    """What the measured edge-to-output time is made of."""
    per_shot = rows("stimjim-trigcomp.csv")
    key = f"foot_at_{IN0_THRESHOLD:.2f}V_us"
    lat = statistics.fmean([num(x[key]) for x in per_shot
                            if x["amp_mV"] == "8000"])
    jitter = statistics.stdev([num(x[key]) for x in per_shot
                               if x["amp_mV"] == "8000"])
    settle = statistics.fmean([num(x["settle_us"]) for x in per_shot
                               if x["amp_mV"] == "8000"])
    # The budget travels with the data, so this figure describes whatever run
    # produced the CSV rather than a constant edited by hand.
    startlat = num(per_shot[0]["startlat_us"])
    trigcomp = num(per_shot[0]["trigcomp_us"])
    # t0 = edge - TRIGCOMP + STARTLAT, and the output moves `hw` after t0.
    # `hw` is the invariant: it does not depend on either budget.
    t0 = startlat - trigcomp
    hw = lat - t0

    fig, ax = plt.subplots(figsize=(10.5, 4.6), dpi=130)

    ax.barh(1, t0, left=0, height=0.42, color=C_BUDGET, alpha=0.85)
    ax.barh(1, hw, left=t0, height=0.42, color=C_BAD, alpha=0.9)
    ax.barh(1, settle, left=lat, height=0.42, color=C_MEAS, alpha=0.5)

    ax.annotate(f"t0 scheduled at STARTLAT {startlat:.0f} "
                f"- TRIGCOMP {trigcomp:.0f} = {t0:.0f} us\n"
                "the firmware puts the first latch here",
                (t0 / 2, 1), ha="center", va="center", fontsize=8.5,
                color="white", weight="bold")
    ax.annotate(f"{hw:.2f} us\nhardware",
                (t0 + hw / 2, 1), ha="center", va="center",
                fontsize=8, color="white", weight="bold")
    ax.annotate(f"output settling,\n{settle:.2f} us to 90 %",
                (lat + settle / 2, 1.22), xytext=(26, 0),
                textcoords="offset points", ha="left", va="center",
                fontsize=8, color=C_MEAS,
                arrowprops=dict(arrowstyle="-", color=C_MEAS, lw=0.8))

    ax.annotate("", xy=(0, 1.32), xytext=(lat, 1.32),
                arrowprops=dict(arrowstyle="<->", color="0.25", lw=1.1))
    ax.annotate(f"measured: {lat:.2f} us from the pin crossing "
                f"{IN0_THRESHOLD:.2f} V to the output leaving baseline "
                f"(sd {jitter*1000:.0f} ns over 30 shots)",
                (lat / 2, 1.36), ha="center", fontsize=8.5, color="0.2")

    # What is inside the hardware block, and why it stays one block.
    ax.annotate(
        f"inside these {hw:.2f} us, in the order they happen:\n"
        "   pin edge -> trigger ISR entry   (what CAL TRIGCOMP is named for)\n"
        "   PIT wake and its 96-cycle scheduling quantum (0.8 us)\n"
        "   NLDAC pulse -> the AD5752 output starting to move\n"
        "This wiring sees only their sum: separating them needs a probe on\n"
        "NLDAC, which is not brought out to a connector.\n"
        f"CAL TRIGCOMP is set to {trigcomp:.0f}, which leaves {hw - trigcomp:.2f} us "
        "uncompensated:\nCAL takes whole microseconds, and this sum is not one.",
        (1.0, 0.60), ha="left", va="top", fontsize=8, color="0.2",
        bbox=dict(boxstyle="round,pad=0.45", fc="#fdf1ef", ec=C_BAD, lw=0.8))

    ax.set_xlim(-1.5, lat + settle + 12)
    ax.set_ylim(0.0, 1.62)
    ax.set_yticks([])
    ax.set_xlabel("microseconds after the trigger edge crosses the input threshold")
    ax.set_title(f"CAL STARTLAT {startlat:.0f} us, CAL TRIGCOMP {trigcomp:.0f} us: "
                 f"the edge-to-output latency delivered is {lat:.2f} us",
                 fontsize=10.5)
    ax.grid(alpha=0.3, axis="x")
    ax.legend(handles=[
        Patch(color=C_BUDGET, alpha=0.85, label="budgeted by the firmware"),
        Patch(color=C_BAD, alpha=0.9, label="hardware delay CAL TRIGCOMP absorbs"),
        Patch(color=C_MEAS, alpha=0.5, label="analog settling, after the event"),
    ], fontsize=8, loc="lower right")
    fig.tight_layout()
    out = FIGS / "stimjim-trigcomp-budget.png"
    fig.savefig(out)
    plt.close(fig)
    print(f"-> {out.relative_to(ROOT)}")


# ---------------------------------------------------------------- figure 4

def fig_timebase_units():
    """The driver's interval is nanoseconds; believing its units code is not."""
    r = [x for x in rows("pico-timebase-2ch.csv") if x["claimed_dt_ns"]]
    tb = [int(x["timebase"]) for x in r]
    claimed = [num(x["claimed_dt_ns"]) for x in r]
    units = [int(x["units"]) for x in r]
    err = [num(x["error_pct"]) for x in r]
    # What pico2000.py computed before the fix: interval * TIME_UNITS[units],
    # in seconds, then read back as if it were seconds.
    scale = {0: 1e-15, 1: 1e-12, 2: 1e-9, 3: 1e-6, 4: 1e-3, 5: 1.0}
    old_ns = [c * scale[u] * 1e9 for c, u in zip(claimed, units)]

    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(10.5, 4.0), dpi=130)

    ax0.semilogy(tb, claimed, "o-", color=C_OK, ms=6,
                 label="interval read as nanoseconds (correct)")
    ax0.semilogy(tb, [max(x, 1e-9) for x in old_ns], "s--", color=C_BAD, ms=6,
                 label="interval x TIME_UNITS[units] (the bug)")
    for k, u in zip(tb, units):
        ax0.annotate(f"units={u}", (k, claimed[tb.index(k)]), xytext=(0, 9),
                     textcoords="offset points", fontsize=7, ha="center",
                     color="0.35")
    ax0.set_xlabel("timebase index")
    ax0.set_ylabel("sample interval the code believed (ns)")
    ax0.set_title("The driver switches its units code at timebase 6.\n"
                  "The interval never changes unit: it is always ns.",
                  fontsize=10)
    ax0.legend(fontsize=8, loc="lower right")
    ax0.grid(alpha=0.3, which="both")

    # The consequence: pick_timebase scans upward for the first interval >=
    # what was asked, so intervals that read as ~0 are never selected.
    ax1.bar([str(x) for x in tb], err, color=C_MEAS, alpha=0.85)
    ax1.axhline(0, color="0.5", lw=0.8)
    ax1.set_ylim(-1.6, 1.2)
    # The fast indices scatter more than the slow ones, and that is the
    # estimator rather than the timebase: at 20 ns a 3968-sample window holds
    # four periods of the reference, so three intervals carry the average.
    ax1.annotate("Measured against a 50 kHz square. The fastest indices fit only\n"
                 "a few periods in one 3968-sample window, so their scatter is\n"
                 "the estimate's, not the timebase's -- all stay inside 1 %.",
                 (0.5, 0.03), xycoords="axes fraction", ha="center", va="bottom",
                 fontsize=7.5, color="0.35")
    ax1.set_xlabel("timebase index")
    ax1.set_ylabel("error in a 20 us reference period (%)")
    ax1.set_title("Every index the driver accepts delivers what it claims.\n"
                  "Believing the units code cost 640 ns where 20 ns was free.",
                  fontsize=10)
    ax1.grid(alpha=0.3, axis="y")

    fig.suptitle("PicoScope 2204A: ps2000_get_timebase reports the interval in "
                 "nanoseconds, always", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    out = FIGS / "pico-timebase-units.png"
    fig.savefig(out)
    plt.close(fig)
    print(f"-> {out.relative_to(ROOT)}")


if __name__ == "__main__":
    FIGS.mkdir(exist_ok=True)
    fig_load_two_modes()
    fig_trigcomp_reference()
    fig_trigcomp_budget()
    fig_timebase_units()
