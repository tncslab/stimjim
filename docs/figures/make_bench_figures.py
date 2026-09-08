"""Figures for the configuration B bench session: the four findings that need one.

Each figure explains a result that a table of numbers states but does not make
obvious. Everything is drawn from measured data written to `tmp/` by
`tests/device/trigcomp.py` and `tests/device/scope_timebase.py`; nothing here
talks to hardware, so re-running those scripts and then this one keeps the
figures and the numbers in step.

  figs/stimjim-load-two-modes.png     why a 1 kOhm resistor read as 4 kOhm, and
                                      why that indicts a readback and not a
                                      contact
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

CAL_STARTLAT = 35         # us, the budget in force during the session
IN0_THRESHOLD = 1.757     # V, measured; the reference instant of the latency


def rows(name):
    with open(TMP / name, newline="") as f:
        return list(csv.DictReader(f))


def num(x):
    """CSV cell to float, with the empty cell meaning 'not measured'."""
    return float(x) if x not in ("", None) else float("nan")


# ---------------------------------------------------------------- figure 1

def fig_load_two_modes():
    """The same 1 kOhm resistor, measured through both output modes."""
    r = rows("stimjim-load-modes.csv")
    v = [x for x in r if x["mode"] == "V"]
    i = [x for x in r if x["mode"] == "I"]

    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(10.5, 4.2), dpi=130,
                                   gridspec_kw={"width_ratios": [1.35, 1]})

    # Left: current against voltage across the load, both modes on one plane.
    # A resistor is a straight line through the origin whichever axis is driven,
    # so two modes that disagree are two different claims about the same line.
    vv = [num(x["V_mV"]) / 1000 for x in v]
    vi = [num(x["I_uA"]) / 1000 for x in v]
    iv = [num(x["scope_V"]) for x in i]
    ii = [num(x["set"]) / 1000 for x in i]
    ax0.plot(vv, vi, "o", color=C_BAD, ms=6,
             label="voltage mode: board's own V and I readback")
    ax0.plot(iv, ii, "s", color=C_OK, ms=6,
             label="current mode: commanded I, scope-measured V")

    lim = max(max(map(abs, vv)), max(map(abs, iv))) * 1.15
    xs = [-lim, lim]
    ax0.plot(xs, [x / 1.0 for x in xs], "-", color=C_OK, lw=1.0, alpha=0.7,
             label="1 kOhm (brown black red gold)")
    ax0.plot(xs, [x / 4.0 for x in xs], "--", color=C_BAD, lw=1.0, alpha=0.7,
             label="4 kOhm, what voltage mode implies")
    ax0.axhline(0, color="0.7", lw=0.6)
    ax0.axvline(0, color="0.7", lw=0.6)
    ax0.set_xlabel("voltage across the load (V)")
    ax0.set_ylabel("current through the load (mA)")
    ax0.set_title("Both modes drive the same resistor.\n"
                  "Only one of them reads the current correctly.", fontsize=10)
    ax0.legend(fontsize=7.5, loc="upper left")
    ax0.grid(alpha=0.3)

    # Right: the resistance each route reports, which is the same data said
    # plainly. The scope-anchored current-mode figure is the true one.
    rv = [abs(num(x["R_board"])) for x in v]
    ri = [abs(num(x["R_scope"])) for x in i]
    ax1.axhline(1000, color=C_OK, lw=1.0, alpha=0.7)
    ax1.annotate("1 kOhm, the part fitted", (0.02, 1000), xycoords=("axes fraction", "data"),
                 fontsize=7.5, va="bottom", color=C_OK)
    for k, val in enumerate(rv):
        ax1.plot(k, val, "o", color=C_BAD, ms=6)
    for k, val in enumerate(ri):
        ax1.plot(k + len(rv) + 1, val, "s", color=C_OK, ms=6)
    ax1.set_xticks([statistics.fmean(range(len(rv))),
                    statistics.fmean(range(len(rv) + 1, len(rv) + 1 + len(ri)))])
    ax1.set_xticklabels([f"voltage mode\nboard V / board I\n"
                         f"{statistics.fmean(rv):.0f} Ohm",
                         f"current mode\nscope V / set I\n"
                         f"{statistics.fmean(ri):.0f} Ohm"], fontsize=8)
    ax1.set_ylim(0, max(rv) * 1.15)
    ax1.set_ylabel("resistance the route reports (Ohm)")
    ax1.set_title(f"{statistics.fmean(rv)/statistics.fmean(ri):.2f}x apart, and "
                  "stable to a few per cent\nin both: a readback, not a contact.",
                  fontsize=10)
    ax1.grid(alpha=0.3, axis="y")

    fig.suptitle("Channel 0's current readback is 4x low in voltage mode "
                 "(Teensy 3.5, fw 0.8.0)", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
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
    ax1.axhline(CAL_STARTLAT, color=C_BUDGET, lw=1.2, ls="--")
    ax1.annotate(f"CAL STARTLAT = {CAL_STARTLAT} us",
                 (0.03, CAL_STARTLAT), xycoords=("axes fraction", "data"),
                 fontsize=8, va="bottom", color=C_BUDGET)
    pick = lat[min(levels, key=lambda x: abs(x - IN0_THRESHOLD))]
    ax1.annotate(f"at the measured threshold:\n{pick:.2f} us,\n"
                 f"{pick - CAL_STARTLAT:.2f} us over budget",
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
    excess = lat - CAL_STARTLAT

    fig, ax = plt.subplots(figsize=(10.5, 4.6), dpi=130)

    # One timeline. The bar is what the firmware budgets, the block after it is
    # what the bench found on top, and the brace names the three things inside
    # that block which this wiring cannot tell apart.
    ax.barh(1, CAL_STARTLAT, left=0, height=0.42, color=C_BUDGET, alpha=0.85)
    ax.barh(1, excess, left=CAL_STARTLAT, height=0.42, color=C_BAD, alpha=0.9)
    ax.barh(1, settle, left=lat, height=0.42, color=C_MEAS, alpha=0.5)

    ax.annotate(f"CAL STARTLAT = {CAL_STARTLAT} us\n"
                "the firmware schedules t0 here",
                (CAL_STARTLAT / 2, 1), ha="center", va="center", fontsize=8.5,
                color="white", weight="bold")
    ax.annotate(f"{excess:.2f} us\nunaccounted",
                (CAL_STARTLAT + excess / 2, 1), ha="center", va="center",
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

    # What is inside the unaccounted block, and why it stays one block.
    ax.annotate(
        "inside these " + f"{excess:.2f} us, in the order they happen:\n"
        "   pin edge -> trigger ISR entry   (what CAL TRIGCOMP is named for)\n"
        "   PIT wake and its 96-cycle scheduling quantum (0.8 us)\n"
        "   NLDAC pulse -> the AD5752 output starting to move\n"
        "This wiring sees only their sum: separating them needs a probe on\n"
        "NLDAC, which is not brought out to a connector.",
        (1.0, 0.60), ha="left", va="top", fontsize=8, color="0.2",
        bbox=dict(boxstyle="round,pad=0.45", fc="#fdf1ef", ec=C_BAD, lw=0.8))

    ax.set_xlim(-1.5, lat + settle + 12)
    ax.set_ylim(0.0, 1.62)
    ax.set_yticks([])
    ax.set_xlabel("microseconds after the trigger edge crosses the input threshold")
    ax.set_title("Setting CAL TRIGCOMP to "
                 f"{round(excess)} us makes the delivered edge-to-output latency "
                 f"the budgeted {CAL_STARTLAT} us", fontsize=10.5)
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
