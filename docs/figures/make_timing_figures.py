"""Timing figures for the stimjimAWG firmware: what each CAL budget pays for.

Two figures, both with the time axis to scale so a reader can see which budget
dominates:

  figs/stimjim-timing-latch.png        the start path and the anatomy of one latch
  figs/stimjim-timing-measurement.png  what a measurement point needs, and whether
                                       it fits the free gap it lives in

The numbers are the Teensy 3.5 register-backend defaults from stimjimAWG/Config.h
plus the bench measurements tabulated in docs/serial-protocol.md section 4. Edit
CAL and MEASURED below when those change; nothing here talks to the firmware.

    python docs/figures/make_timing_figures.py
"""

import pathlib

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch

FIGS = pathlib.Path(__file__).resolve().parents[2] / "figs"

# CAL defaults in microseconds (Teensy 3.5, register backends).
CAL = dict(PRELOAD=4, DACPROG1=3, DACPROG2=5, ADCREAD=3, ADCSWITCH=4,
           GUARD=1, SETTLE=9, STARTLAT=60, TRIGCOMP=0)

# What the BENCH group measures on that board: (min, max) over n = 2000.
MEASURED = {
    "dacProgram":     (1.42, 2.81),    # BENCHDAC
    "dacProgramBoth": (2.75, 4.14),    # BENCHDAC2
    "dacLatch":       (0.44, 1.78),    # BENCHLATCH
    "adcRead":        (2.37, 3.81),    # BENCHADC
    "select+read":    (4.62, 6.06),    # BENCHSW
    "PIT wake":       (0.30, 0.47),    # BENCHPIT,1000,2000
    "latch jitter":   (0.117, 0.158),  # BENCHPIT,1000,2000,4
    "arm bare":       (16.2, 16.6),    # BENCHARM, undriven 0-stage slot
    "arm S+meas":     (23.4, 23.8),    # BENCHARM, measured two-channel S slot
    "arm W+meas":     (37.5, 42.0),    # BENCHARM, sine slot with measurement
}

C_DAC = "#0072B2"    # DAC programming
C_ADC = "#009E73"    # ADC reads
C_WAIT = "#CC79A7"   # settling and guard: time nobody may use
C_ARM = "#E69F00"    # software arming
C_EDGE = "#D55E00"   # the instants themselves
GREY = "0.35"


def band(ax, y, x0, x1, colour, label, h=0.62, fontsize=8.5, textcolour="black",
         hatch=None, inside=True):
    """One labelled interval; a leader line when the bar is too thin to fill."""
    ax.add_patch(plt.Rectangle((x0, y - h / 2), x1 - x0, h, facecolor=colour,
                               edgecolor="black", lw=0.6, alpha=0.85, hatch=hatch,
                               zorder=3))
    if not label:
        return
    if inside:
        ax.text((x0 + x1) / 2, y, label, ha="center", va="center",
                fontsize=fontsize, color=textcolour, zorder=4)
    else:
        ax.annotate(label, xy=((x0 + x1) / 2, y + h / 2), xytext=(0, 14),
                    textcoords="offset points", ha="center", va="bottom",
                    fontsize=fontsize, color=textcolour,
                    arrowprops=dict(arrowstyle="-", lw=0.6, color=GREY), zorder=4)


def instant(ax, x, ymin, ymax, label, colour=C_EDGE, ha="center", top=False):
    """A vertical marker for one instant. The label goes below the line by
    default, which keeps the space above the bars free for the annotations that
    need leader lines; `top` puts it above instead."""
    ax.plot([x, x], [ymin, ymax], color=colour, lw=1.4, zorder=5)
    y, dy, va = (ymax, 4, "bottom") if top else (ymin, -4, "top")
    ax.annotate(label, xy=(x, y), xytext=(0, dy), textcoords="offset points",
                ha=ha, va=va, fontsize=9, color=colour, zorder=6)


def span(ax, y, x0, x1, label, fontsize=8.5, colour=GREY, dy=-13):
    ax.add_patch(FancyArrowPatch((x0, y), (x1, y), arrowstyle="<->",
                                 mutation_scale=8, lw=0.9, color=colour, zorder=5))
    ax.annotate(label, xy=((x0 + x1) / 2, y), xytext=(0, dy),
                textcoords="offset points", ha="center", va="top",
                fontsize=fontsize, color=colour, zorder=6)


def bare(ax):
    ax.set_yticks([])
    for side in ("left", "right", "top"):
        ax.spines[side].set_visible(False)


# ---------------------------------------------------------------------------
# Figure 1: the start path, and one latch
# ---------------------------------------------------------------------------
def fig_latch():
    fig, (ax0, ax1) = plt.subplots(2, 1, figsize=(11.5, 6.8), dpi=150,
                                   gridspec_kw=dict(height_ratios=[1, 1.15]))

    # -- panel A: trigger edge to the first output sample --------------------
    lat = CAL["STARTLAT"]
    arm_typ = MEASURED["arm S+meas"][1]
    arm_max = MEASURED["arm W+meas"][1]
    prog_open = lat - CAL["PRELOAD"] - CAL["DACPROG2"]
    ax0.set_xlim(-7, lat + 32)
    ax0.set_ylim(-2.6, 4.4)

    instant(ax0, 0, -1.5, 1.7, "trigger pin edge", ha="left")
    instant(ax0, lat, -1.5, 1.7, "t0: first sample latched")

    band(ax0, 1.3, 0, 1.8, C_WAIT, "", h=0.7, hatch="//")
    band(ax0, 1.3, 1.8, arm_typ, C_ARM, "Engine::startTrain\n19-24 us (BENCHARM)",
         h=0.7, fontsize=8)
    band(ax0, 1.3, arm_typ, arm_max, C_ARM, "", h=0.7, hatch="\\\\")
    band(ax0, 1.3, arm_max, prog_open, "0.92", "margin", h=0.7, fontsize=8)
    band(ax0, 1.3, prog_open, lat - CAL["DACPROG2"], C_DAC, "PRE", h=0.7,
         fontsize=7.5, textcolour="white")
    band(ax0, 1.3, lat - CAL["DACPROG2"], lat, C_DAC, "PROG2", h=0.7,
         fontsize=7, textcolour="white")

    span(ax0, 0.35, 0, lat, "STARTLAT = 60 us: everything in the row above fits in here")
    ax0.annotate("TRIGCOMP = 0: pin edge to ISR entry,\n"
                 "the one part software cannot see",
                 xy=(0.9, 1.65), xytext=(0, 16), textcoords="offset points",
                 ha="left", va="bottom", fontsize=7.5, color=C_WAIT,
                 arrowprops=dict(arrowstyle="-", lw=0.6, color=GREY))
    ax0.annotate("a sine train with measurement arms in 42 us:\n"
                 "the worst arm measured on this board",
                 xy=(arm_max - 4, 1.65), xytext=(0, 48), textcoords="offset points",
                 ha="center", va="bottom", fontsize=7.5, color=C_ARM,
                 arrowprops=dict(arrowstyle="-|>", lw=0.8, color=C_ARM))
    ax0.annotate("then the slot's DELAY, if any", xy=(lat, 0.35), xytext=(8, 0),
                 textcoords="offset points", ha="left", va="center",
                 fontsize=8.5, color=GREY)
    ax0.set_title("A   Trigger edge to first output sample. A trigger routed to both "
                  "engines arms twice, so it needs twice the orange bar.",
                  fontsize=10, loc="left")
    ax0.set_xlabel("microseconds after the trigger edge", fontsize=9)
    bare(ax0)

    # -- panel B: one latch event -------------------------------------------
    pre, prog, settle = CAL["PRELOAD"], CAL["DACPROG2"], CAL["SETTLE"]
    wake = -(pre + prog)
    prog_end = wake + MEASURED["PIT wake"][1] + MEASURED["dacProgramBoth"][1]
    ax1.set_xlim(wake - 5, settle + 9)
    ax1.set_ylim(-3.0, 4.6)

    instant(ax1, wake, -1.9, 1.75, "PIT fires,\nplayer ISR entered")
    instant(ax1, 0, -1.9, 1.75, "deadline: NLDAC pulses,\nthe output steps")

    band(ax1, 1.35, wake, wake + MEASURED["PIT wake"][1], C_WAIT, "", h=0.66)
    band(ax1, 1.35, wake + MEASURED["PIT wake"][1], prog_end, C_DAC,
         "dacProgramBoth\n4.14 us worst", h=0.66, fontsize=7.5, textcolour="white")
    band(ax1, 1.35, prog_end, 0, "0.92", "spin on CYCCNT", h=0.66, fontsize=8)
    band(ax1, 1.35, 0, settle, C_WAIT, "SETTLE = 9 us", h=0.66, fontsize=8)

    span(ax1, 0.4, wake, -prog, "PRELOAD = 4 us")
    span(ax1, 0.4, -prog, 0, "DACPROG2 = 5 us")
    span(ax1, -0.85, wake, 0,
         "the preload window: woken this early, latching on the deadline")

    ax1.annotate("PIT wake latency,\n0.47 us worst", xy=(wake + 0.2, 1.68),
                 xytext=(-6, 16), textcoords="offset points", ha="center",
                 va="bottom", fontsize=7.5, color=C_WAIT,
                 arrowprops=dict(arrowstyle="-", lw=0.6, color=GREY))
    ax1.annotate("residual latch jitter 42 ns: the spin decides when the output\n"
                 "steps, not the interrupt (BENCHPIT with a preload)",
                 xy=(0, 1.68), xytext=(0, 48), textcoords="offset points",
                 ha="center", va="bottom", fontsize=8, color=C_EDGE,
                 arrowprops=dict(arrowstyle="-|>", lw=0.8, color=C_EDGE))
    ax1.annotate("a reading taken before this describes\nthe output level before the step",
                 xy=(settle + 0.4, 0.4), xytext=(0, 0), textcoords="offset points",
                 ha="left", va="center", fontsize=8, color=C_WAIT)
    ax1.set_title("B   One latch event against its deadline. DACPROG1 = 3 us replaces "
                  "DACPROG2 when the train drives a single channel.",
                  fontsize=10, loc="left")
    ax1.set_xlabel("microseconds relative to the latch deadline", fontsize=9)
    bare(ax1)

    fig.suptitle("stimjimAWG timing budgets (CAL) on a Teensy 3.5 at 120 MHz, "
                 "register backends - bars drawn to scale", fontsize=11.5, y=0.985)
    fig.tight_layout(rect=(0, 0, 1, 0.955))
    out = FIGS / "stimjim-timing-latch.png"
    fig.savefig(out)
    plt.close(fig)
    print("->", out)


# ---------------------------------------------------------------------------
# Figure 2: the measurement window
# ---------------------------------------------------------------------------
def room(n, dual=True):
    """Free gap that a measurement point of n reads needs (protocol section 4)."""
    prog = CAL["DACPROG2"] if dual else CAL["DACPROG1"]
    return (CAL["PRELOAD"] + prog + n * (CAL["ADCREAD"] + CAL["ADCSWITCH"])
            + CAL["GUARD"] + CAL["SETTLE"])


def fig_measurement():
    fig, (ax0, ax1) = plt.subplots(2, 1, figsize=(11.5, 7.0), dpi=150,
                                   gridspec_kw=dict(height_ratios=[1.2, 1]))

    # -- panel A: the window between two latches ----------------------------
    n = 4
    gap = room(n)                       # 47 us: exactly what four reads need
    ax0.set_xlim(-4.5, gap + 6)
    ax0.set_ylim(-3.6, 3.4)

    instant(ax0, 0, -2.6, 2.0, "latch k", ha="left", top=True)
    instant(ax0, gap, -2.6, 2.0, "latch k+1", ha="right", top=True)

    x = 0.0
    band(ax0, 1.15, x, CAL["SETTLE"], C_WAIT, "SETTLE", h=0.62, fontsize=8)
    x = CAL["SETTLE"]
    for _ in range(n):
        band(ax0, 1.15, x, x + CAL["ADCSWITCH"], C_ADC, "sw", h=0.62, fontsize=7,
             textcolour="white")
        band(ax0, 1.15, x + CAL["ADCSWITCH"],
             x + CAL["ADCSWITCH"] + CAL["ADCREAD"], C_ADC, "rd", h=0.62,
             fontsize=7, textcolour="white")
        x += CAL["ADCSWITCH"] + CAL["ADCREAD"]
    band(ax0, 1.15, x, x + CAL["GUARD"], C_WAIT, "", h=0.62)
    x += CAL["GUARD"]
    band(ax0, 1.15, x, x + CAL["PRELOAD"], C_DAC, "PRE", h=0.62, fontsize=7,
         textcolour="white")
    band(ax0, 1.15, x + CAL["PRELOAD"], gap, C_DAC, "DACPROG2", h=0.62,
         fontsize=7.5, textcolour="white")

    reads_end = CAL["SETTLE"] + n * (CAL["ADCREAD"] + CAL["ADCSWITCH"])
    span(ax0, 0.3, CAL["SETTLE"], reads_end,
         "4 reads x (ADCSWITCH 4 + ADCREAD 3) = 28 us")
    span(ax0, -1.35, 0, gap,
         "room(4) = PRELOAD + DACPROG2 + 4 x (ADCREAD + ADCSWITCH) + GUARD "
         "+ SETTLE = 47 us")
    ax0.annotate("GUARD = 1 us", xy=(x - CAL["GUARD"] / 2, 1.5), xytext=(-14, 24),
                 textcoords="offset points", ha="right", va="bottom", fontsize=8,
                 color=C_WAIT, arrowprops=dict(arrowstyle="-", lw=0.6, color=GREY))
    ax0.text(gap / 2, -2.75, "sw = select the ADC input line, rd = the conversion.  "
             "One read per selected line per driven channel.",
             ha="center", va="top", fontsize=8, color=C_ADC)
    ax0.set_title("A   One measurement point of four reads (V and I on both channels), "
                  "sitting between two latches", fontsize=10, loc="left")
    ax0.set_xlabel("microseconds after latch k", fontsize=9)
    bare(ax0)

    # -- panel B: does it fit? ----------------------------------------------
    rows = [("V+I, both channels", 4, True),
            ("one line, both channels", 2, True),
            ("V+I, one channel", 2, False),
            ("one line, one channel", 1, False)]
    for y, (label, nr, dual) in zip(range(len(rows))[::-1], rows):
        prog = CAL["DACPROG2"] if dual else CAL["DACPROG1"]
        x = 0.0
        for width, colour in ((CAL["PRELOAD"] + prog, C_DAC),
                              (nr * (CAL["ADCREAD"] + CAL["ADCSWITCH"]), C_ADC),
                              (CAL["GUARD"] + CAL["SETTLE"], C_WAIT)):
            ax1.add_patch(plt.Rectangle((x, y - 0.3), width, 0.6, facecolor=colour,
                                        edgecolor="black", lw=0.5, alpha=0.85,
                                        zorder=3))
            x += width
        plural = "s" if nr > 1 else ""
        # Fixed column, clear of every gap line, so no label ever sits on one.
        ax1.text(50.0, y, "needs %d us" % room(nr, dual), va="center",
                 ha="left", fontsize=8.5)
        ax1.text(-1.2, y, "%s  (%d read%s)" % (label, nr, plural),
                 va="center", ha="right", fontsize=8.5)

    # Stacked vertically: the three gaps are only 5 us apart on the x axis, so
    # their labels would otherwise sit on top of each other.
    for gapv, style, note, ytext in (
            (20, "-", "gap 20 us: the default L interval and the W sample floor "
                      "- nothing fits it", 5.35),
            (30, "--", "gap 30 us: an L slot with DT 30", 4.75),
            (35, ":", "gap 35 us: an L slot with DT 35", 4.15)):
        ax1.axvline(gapv, color=C_EDGE, lw=1.2, ls=style, zorder=2)
        ax1.annotate(note, xy=(gapv, ytext), xytext=(4, 0),
                     textcoords="offset points", ha="left", va="center",
                     fontsize=7.5, color=C_EDGE)

    ax1.set_xlim(-17, 66)
    ax1.set_ylim(-0.8, 5.9)
    ax1.set_xlabel("microseconds of free gap needed.  Blue: the next latch's "
                   "programming.  Green: the ADC reads.  Pink: SETTLE + GUARD.",
                   fontsize=8.5)
    ax1.set_title("B   A point whose reads do not fit its gap is rotated over "
                  "repetitions (MEAS fit=1) or refused (fit=0)",
                  fontsize=10, loc="left")
    bare(ax1)

    fig.suptitle("stimjimAWG in-train measurement: what a point needs, and what the "
                 "waveform leaves free", fontsize=11.5, y=0.985)
    fig.tight_layout(rect=(0, 0, 1, 0.955))
    out = FIGS / "stimjim-timing-measurement.png"
    fig.savefig(out)
    plt.close(fig)
    print("->", out)


if __name__ == "__main__":
    FIGS.mkdir(exist_ok=True)
    fig_latch()
    fig_measurement()
