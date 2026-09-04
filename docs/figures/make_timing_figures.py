"""Timing figures for the stimjimAWG firmware: what each CAL budget pays for, and
which execution context does the work.

Three figures. The time axis is to scale in all of them, so a reader can see
which budget dominates; what the rows say is *who* is running:

  figs/stimjim-timing-latch.png        the start path and the anatomy of one latch
  figs/stimjim-timing-contexts.png     the same story over a whole train
  figs/stimjim-timing-measurement.png  what a measurement point needs, and whether
                                       it fits the free gap it lives in

Rows are execution contexts, tinted and named on the y axis:

  loop()          background work at no interrupt priority: the preparation of the
                  next arm and the next measurement plan, serial, SD, display
  trigger ISR     priority 80, one per input pin, IN0/IN1 sharing IRQ_PORTC
  player ISR      priority 64, one PIT channel per engine. Every DAC latch in the
                  firmware happens here and nowhere else
  analog output   the AD5752 and the isolated output stage, after the NLDAC pulse

Bar colour keeps its own meaning independent of the row: blue = DAC programming,
green = ADC reads, pink = time nobody may use (settling, guard, the pin-to-ISR
delay), orange = software arming, dark orange = the NLDAC pulse. Explanations sit
in a notes column to the right of the action, inside the row they belong to, so
no label ever crosses a row boundary.

The numbers are the Teensy 3.5 register-backend defaults from stimjimAWG/Config.h
plus the bench measurements tabulated in docs/serial-protocol.md section 4. One
number is an estimate rather than a measurement, drawn hatched and labelled as
such: what is left of the arm after phase 14 moved the rest of it into loop().
Edit CAL, MEASURED and ARM below when those change; nothing here talks to the
firmware.

    python docs/figures/make_timing_figures.py
"""

import pathlib

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Patch

FIGS = pathlib.Path(__file__).resolve().parents[2] / "figs"

# CAL in microseconds (Teensy 3.5, register backends). STARTLAT is the candidate
# the pre-armed path makes reachable, not the build default of 35 -- see the
# panel A note and docs/timing.md section 1.
CAL = dict(PRELOAD=4, DACPROG1=3, DACPROG2=5, ADCREAD=3, ADCSWITCH=4,
           GUARD=1, SETTLE=9, STARTLAT=20, TRIGCOMP=0)
MIN_SCHEDULE = 3          # SJ_MIN_SCHEDULE_US, the slack Cal::validate insists on

# What the BENCH group measures on that board: (min, max) over n = 2000.
MEASURED = {
    "dacProgram":     (1.42, 2.81),    # BENCHDAC
    "dacProgramBoth": (2.75, 4.14),    # BENCHDAC2
    "dacLatch":       (0.44, 1.78),    # BENCHLATCH
    "adcRead":        (2.37, 3.81),    # BENCHADC
    "select+read":    (4.62, 6.06),    # BENCHSW
    "PIT wake":       (0.30, 0.47),    # BENCHPIT,1000,2000
    "latch jitter":   (0.117, 0.158),  # BENCHPIT,1000,2000,4
}

# Engine::startTrain, which since phase 14 has two paths.
ARM = {
    # ESTIMATE from the instruction count, NOT a measurement: two cycles64()
    # reads, t0, four envelope anchors, the plan attach, the buffer swap and one
    # pitProgram. Re-measure with tests/device/bench_arm.py (warmed column).
    "prepared": (2.0, 4.0),
    # BENCHARM phase 13 -- the arm as it was before the preparation moved into
    # loop(), and therefore what the cold path still costs: 9.04 us for a slot
    # that drives nothing, 15.00 for a ten-stage L train, 19.96 when the engine
    # also has to compile its own ten-point measurement plan.
    "cold": (9.0, 20.0),
}

C_DAC = "#0072B2"    # DAC programming
C_ADC = "#009E73"    # ADC reads
C_WAIT = "#CC79A7"   # settling, guard, pin-to-ISR: time nobody may use
C_ARM = "#E69F00"    # software arming
C_EDGE = "#D55E00"   # instants, and the NLDAC pulse
C_BAD = "#B00020"    # a budget that does not fit
GREY = "0.35"

# Row tints, one per execution context. Pale enough to read black text on.
T_LOOP = "#F2F2F2"
T_TRIG = "#FDF0DC"
T_PLAY = "#E8F1F8"
T_OUT = "#FAECF3"

LEGEND = [Patch(facecolor=C_ARM, edgecolor="black", label="software arming"),
          Patch(facecolor="#DADADA", edgecolor="black", label="idle / background work"),
          Patch(facecolor=C_DAC, edgecolor="black", label="DAC programming"),
          Patch(facecolor=C_EDGE, edgecolor="black", label="NLDAC pulse (the output steps)"),
          Patch(facecolor=C_ADC, edgecolor="black", label="ADC reads"),
          Patch(facecolor=C_WAIT, edgecolor="black", label="settling / guard / pin-to-ISR")]


def lanes(ax, rows, x0, x1):
    """Tinted background band and y-axis name for each execution context.

    `rows` is a list of (y, height, tint, name), top to bottom.
    """
    for y, h, tint, _ in rows:
        ax.add_patch(plt.Rectangle((x0, y - h / 2), x1 - x0, h, facecolor=tint,
                                   edgecolor="none", zorder=0))
    ax.set_yticks([y for y, _, _, _ in rows])
    ax.set_yticklabels([n for _, _, _, n in rows], fontsize=8.5)
    ax.tick_params(axis="y", length=0)
    for side in ("left", "right", "top"):
        ax.spines[side].set_visible(False)


def band(ax, y, x0, x1, colour, label, h=0.44, fontsize=8.0, textcolour="black",
         hatch=None, above=False, dy=13):
    """One interval. `above` puts the label over the bar on a leader line, for
    bars too thin to hold text."""
    ax.add_patch(plt.Rectangle((x0, y - h / 2), x1 - x0, h, facecolor=colour,
                               edgecolor="black", lw=0.6, alpha=0.9, hatch=hatch,
                               zorder=3))
    if not label:
        return
    if above:
        ax.annotate(label, xy=((x0 + x1) / 2, y + h / 2), xytext=(0, dy),
                    textcoords="offset points", ha="center", va="bottom",
                    fontsize=fontsize, color=textcolour,
                    arrowprops=dict(arrowstyle="-", lw=0.6, color=GREY), zorder=4)
    else:
        ax.text((x0 + x1) / 2, y, label, ha="center", va="center",
                fontsize=fontsize, color=textcolour, zorder=4)


def note(ax, x, y, text, colour="black", fontsize=7.5):
    """A block of explanation in the notes column, inside its own row."""
    ax.text(x, y, text, ha="left", va="center", fontsize=fontsize, color=colour,
            linespacing=1.35, zorder=6)


def instant(ax, x, ymin, ymax, label, colour=C_EDGE, ha="center", lw=1.4, ls="-"):
    """A vertical marker across the rows, labelled above the topmost one."""
    ax.plot([x, x], [ymin, ymax], color=colour, lw=lw, ls=ls, zorder=5)
    ax.annotate(label, xy=(x, ymax), xytext=(0, 4), textcoords="offset points",
                ha=ha, va="bottom", fontsize=8.5, color=colour, zorder=6)


def span(ax, y, x0, x1, label, fontsize=8.0, colour=GREY, dy=-12, ha="center"):
    """A double-headed measure. A positive dy puts the label above the arrow."""
    ax.add_patch(FancyArrowPatch((x0, y), (x1, y), arrowstyle="<->",
                                 mutation_scale=8, lw=0.9, color=colour, zorder=5))
    ax.annotate(label, xy=((x0 + x1) / 2, y), xytext=(0, dy),
                textcoords="offset points", ha=ha,
                va=("bottom" if dy > 0 else "top"),
                fontsize=fontsize, color=colour, zorder=6)


def nldac(ax, y, x, h=0.44, label=True, minw=0.0):
    """The load-DAC pulse: the only thing in the firmware that moves an output."""
    w = max(MEASURED["dacLatch"][0], minw)
    ax.add_patch(plt.Rectangle((x, y - h / 2), w, h, facecolor=C_EDGE,
                               edgecolor="black", lw=0.5, zorder=5))
    if label:
        ax.annotate("NLDAC\n0.44 us", xy=(x + w, y), xytext=(6, 0),
                    textcoords="offset points", ha="left", va="center",
                    fontsize=7.0, color=C_EDGE, linespacing=1.3, zorder=6)


def latch_tick(ax, y, x, h=0.44):
    """One latch where the panel's scale makes its microseconds invisible: the
    instant itself, not the preload window that led up to it."""
    ax.plot([x, x], [y - h / 2, y + h / 2], color=C_EDGE, lw=1.6, zorder=5,
            solid_capstyle="butt")


def figure_legend(fig):
    fig.legend(handles=LEGEND, loc="lower center", ncol=6, fontsize=8.0,
               frameon=False, handlelength=1.5, handleheight=0.9,
               columnspacing=1.8, bbox_to_anchor=(0.5, 0.0))


# ---------------------------------------------------------------------------
# Figure 1: the start path, and one latch
# ---------------------------------------------------------------------------
def fig_latch():
    fig, (ax0, ax1) = plt.subplots(2, 1, figsize=(12.6, 9.2), dpi=150,
                                   gridspec_kw=dict(height_ratios=[1.28, 1]))

    # -- panel A: trigger edge to the first output sample --------------------
    lat = CAL["STARTLAT"]
    trigcomp = 0.4                      # CAL TRIGCOMP is 0/unmeasured; nominal
    arm_lo, arm_hi = ARM["prepared"]
    cold_lo, cold_hi = ARM["cold"]
    wake = lat - CAL["PRELOAD"] - CAL["DACPROG2"]     # 11: preload window opens
    progstart = lat - CAL["DACPROG2"]                 # 15: the SPI write starts
    x0, x1, nx = -14.5, 62, 31.5

    rows = [(3.75, 0.92, T_LOOP, "loop()\nno priority"),
            (2.35, 1.60, T_TRIG, "trigger ISR\nprio 80"),
            (1.10, 0.86, T_PLAY, "player ISR\nPIT, prio 64"),
            (0.22, 0.74, T_OUT, "analog\noutput")]
    lanes(ax0, rows, x0, x1)
    ax0.set_xlim(x0, x1)
    ax0.set_ylim(-2.15, 4.90)

    # loop(): the preparation, at some earlier and unrelated time
    band(ax0, 3.75, -13.8, -4.0, C_ARM, "prepareArms(), any pass", h=0.46,
         fontsize=7.5)
    ax0.plot([-3.6, -2.4], [3.75, 3.75], color=GREY, lw=0.9, ls=":", zorder=3)
    band(ax0, 3.75, arm_hi, wake, "#DADADA", "idle", h=0.46, fontsize=7.5,
         textcolour=GREY)
    note(ax0, nx, 3.75,
         "Prepared here, into the engine's spare Player: the geometry,\n"
         "the copy-on-arm of the stage triplets, stage 0's DAC codes, the\n"
         "period/duration/preload scalars, the envelope's reciprocals and\n"
         "the measurement plan. 9-20 us, and none of it after the edge.")

    # trigger ISR: two sub-rows, the prepared arm above the cold one
    band(ax0, 2.78, 0, trigcomp, C_WAIT, "", h=0.40)
    band(ax0, 2.78, trigcomp, arm_lo, C_ARM, "", h=0.40)
    band(ax0, 2.78, arm_lo, arm_hi, C_ARM, "", h=0.40, hatch="//")
    note(ax0, nx, 2.83,
         "startTrain, prepared path: t0, envRebase, the plan attach, the\n"
         "park codes, the buffer swap, pitProgram.  2-4 us ESTIMATED from\n"
         "the instruction count - re-measure with bench_arm.py (warmed).",
         colour=C_ARM)
    band(ax0, 1.98, trigcomp, cold_hi, C_ARM, "", h=0.34, hatch="//")
    for xv, txt in ((9.0, "9 bare"), (15.0, "15 ten-stage L"), (20.0, "20 +plan")):
        ax0.plot([xv, xv], [1.81, 2.15], color="black", lw=0.7, zorder=4)
        ax0.annotate(txt, xy=(xv, 1.81), xytext=(0, -3),
                     textcoords="offset points", ha="center", va="top",
                     fontsize=6.5, color=GREY)
    note(ax0, nx, 1.93,
         "Cold path - loop() starved, or the slot edited since its last\n"
         "pass - prepares inside the ISR: 9.0-20.0 us measured (BENCHARM).\n"
         "At STARTLAT = 20 it runs into the preload window, so the first\n"
         "latch is late and the completion says \"set CAL STARTLAT >= 29\".",
         colour=C_BAD)

    # player ISR: the preload window and the latch itself
    band(ax0, 1.10, wake, progstart, C_DAC, "PRELOAD 4", h=0.44, fontsize=7.5,
         textcolour="white")
    band(ax0, 1.10, progstart, lat, C_DAC, "DACPROG2 5", h=0.44, fontsize=7.5,
         textcolour="white")
    nldac(ax0, 1.10, lat, h=0.44)
    note(ax0, nx, 1.10,
         "The first latch happens here, in the timer ISR - never in the\n"
         "trigger ISR, whatever the arm cost. Same code path as every\n"
         "later sample of the train.", colour=C_DAC)

    # analog output
    band(ax0, 0.22, lat, lat + CAL["SETTLE"], C_WAIT, "SETTLE 9", h=0.44,
         fontsize=7.5)
    note(ax0, nx, 0.22,
         "The load sees the onset at t0 and full amplitude 8-9 us later.\n"
         "No scheduling change shortens this.", colour=C_WAIT)

    instant(ax0, 0, -0.20, 4.21, "trigger pin edge", ha="left")
    instant(ax0, wake, -0.20, 4.21, "PIT fires")
    instant(ax0, lat, -0.20, 4.21, "t0: output steps")
    ax0.annotate("TRIGCOMP = 0: the pin edge to\nISR entry, unseen by software",
                 xy=(trigcomp + 0.1, 2.68), xytext=(-13.8, 2.30), ha="left",
                 va="center", fontsize=7, color=C_WAIT, linespacing=1.3,
                 arrowprops=dict(arrowstyle="-|>", lw=0.7, color=C_WAIT,
                                 shrinkA=4, shrinkB=2))

    span(ax0, -0.60, 0, lat, "STARTLAT = 20 us from the edge timestamp the ISR takes "
                             "in its first instruction, then the slot's DELAY if any")
    span(ax0, -1.55, wake, lat,
         "PRELOAD + DACPROG2 = 9 us: the part no scheduling change can remove "
         "(Cal::validate refuses STARTLAT below 9 + 3 = 12)", fontsize=7.5)

    ax0.set_title("A   Trigger edge to first output sample, at the STARTLAT the "
                  "pre-armed path makes reachable. Rows are execution contexts.",
                  fontsize=10, loc="left")
    ax0.set_xlabel("microseconds after the trigger edge", fontsize=9)

    # -- panel B: one latch event -------------------------------------------
    pre, prog2, settle = CAL["PRELOAD"], CAL["DACPROG2"], CAL["SETTLE"]
    w = -(pre + prog2)
    prog_end = w + MEASURED["PIT wake"][1] + MEASURED["dacProgramBoth"][1]
    bx0, bx1, bnx = -13.5, 37, 12.5
    rows = [(2.45, 0.92, T_LOOP, "loop()\nno priority"),
            (1.25, 0.92, T_PLAY, "player ISR\nPIT, prio 64"),
            (0.28, 0.76, T_OUT, "analog\noutput")]
    lanes(ax1, rows, bx0, bx1)
    ax1.set_xlim(bx0, bx1)
    ax1.set_ylim(-2.05, 3.60)

    band(ax1, 2.45, bx0 + 0.5, w, "#DADADA", "loop()", h=0.46, fontsize=7.5,
         textcolour=GREY)
    band(ax1, 2.45, settle + 0.5, bnx - 1.0, "#DADADA", "", h=0.46)
    note(ax1, bnx, 2.45,
         "Between events the CPU is in loop(): serial, SD, the display,\n"
         "and the preparation of the next arm. It is not spinning - the\n"
         "PIT is armed and the core is doing other work.", colour=GREY)

    band(ax1, 1.25, w, w + MEASURED["PIT wake"][1], C_WAIT, "", h=0.46)
    band(ax1, 1.25, w + MEASURED["PIT wake"][1], prog_end, C_DAC,
         "dacProgBoth", h=0.46, fontsize=6.5, textcolour="white")
    band(ax1, 1.25, prog_end, 0, "#DADADA", "spin on CYCCNT", h=0.46, fontsize=7.5)
    nldac(ax1, 1.25, 0, h=0.46)
    note(ax1, bnx, 1.25,
         "One ISR pass: wake early (PIT wake latency 0.47 us worst), write\n"
         "the codes into the DAC's input register (dacProgramBoth, 4.14 us\n"
         "worst), spin to the deadline, pulse NLDAC. The SPI write moves\n"
         "nothing; the pulse does - which is why residual latch jitter is\n"
         "42 ns: the spin decides the instant, not the interrupt.",
         colour=C_DAC)

    band(ax1, 0.28, 0, settle, C_WAIT, "SETTLE = 9 us", h=0.46, fontsize=7.5)
    note(ax1, bnx, 0.28,
         "A reading taken before this describes the output level before\n"
         "the step, which is what CAL SETTLE exists to prevent.",
         colour=C_WAIT)

    instant(ax1, w, -0.15, 2.92, "PIT fires, ISR entered", ha="center")
    instant(ax1, 0, -0.15, 2.92, "deadline")
    span(ax1, -0.55, w, -prog2, "PRELOAD = 4")
    span(ax1, -0.55, -prog2, 0, "DACPROG2 = 5")
    span(ax1, -1.45, w, 0,
         "the preload window: woken this early, latching on the deadline",
         fontsize=7.5)

    ax1.set_title("B   One latch event against its deadline. DACPROG1 = 3 us replaces "
                  "DACPROG2 when the train drives a single channel.",
                  fontsize=10, loc="left")
    ax1.set_xlabel("microseconds relative to the latch deadline", fontsize=9)

    fig.suptitle("stimjimAWG timing budgets (CAL) on a Teensy 3.5 at 120 MHz, register "
                 "backends - bars to scale, rows are who is running", fontsize=11.5,
                 y=0.987)
    figure_legend(fig)
    fig.tight_layout(rect=(0, 0.035, 1, 0.958))
    out = FIGS / "stimjim-timing-latch.png"
    fig.savefig(out)
    plt.close(fig)
    print("->", out)


# ---------------------------------------------------------------------------
# Figure 2: the same story over a whole train
# ---------------------------------------------------------------------------
def fig_contexts():
    """Where a whole train's work sits, over milliseconds rather than
    microseconds: what loop() prepares, what the edge does, what the player does
    per event, and where a stage's arithmetic happens now.

    At this scale a latch's preload window (9 us) and its \nDAC pulse (0.44 us)
    are both thinner than a line, so each latch is drawn as one tick. Its
    anatomy is figure stimjim-timing-latch, panel B."""
    fig, ax = plt.subplots(figsize=(12.6, 5.6), dpi=150)

    lat = CAL["STARTLAT"] / 1000.0      # this panel is in milliseconds
    period = 1.0                        # 1 ms pulse period, three pulses drawn
    stages = [(0.0, 0.20), (0.20, 0.40)]
    x0, x1, nx = -0.62, 6.15, 3.05
    rows = [(3.55, 0.92, T_LOOP, "loop()\nno priority"),
            (2.45, 0.92, T_TRIG, "trigger ISR\nprio 80"),
            (1.32, 0.94, T_PLAY, "player ISR\nPIT, prio 64"),
            (0.32, 0.80, T_OUT, "analog\noutput")]
    lanes(ax, rows, x0, x1)
    ax.set_xlim(x0, x1)
    ax.set_ylim(-0.80, 4.70)

    band(ax, 3.55, -0.58, -0.05, C_ARM, "prepareArms()", h=0.46, fontsize=7.5)
    band(ax, 3.55, lat + 0.10, lat + 0.88, "#DADADA", "prepares the next arm",
         h=0.46, fontsize=7, textcolour=GREY)
    band(ax, 3.55, lat + 2.28, lat + 2.78, "#DADADA",
         "MSUM, the completion line, SD rows, the accumulator clear", h=0.46,
         fontsize=7, textcolour=GREY, above=True, dy=8)
    note(ax, nx, 3.55,
         "Everything that prints or touches the card runs here, after\n"
         "the train - never in an ISR. So does the preparation of the\n"
         "next arm, which may run while this train is still playing.",
         colour=GREY)

    band(ax, 2.45, 0, lat, C_ARM, "", h=0.46)
    note(ax, nx, 2.45,
         "startTrain computes t0 and programs a timer. It emits no\n"
         "signal at all, and at priority 80 - below the players - it\n"
         "can never delay a train that is already playing.", colour=C_ARM)

    for p in range(3):
        base = lat + p * period
        for s0, _ in stages:
            latch_tick(ax, 1.32, base + s0, h=0.50)
        latch_tick(ax, 1.32, base + stages[-1][1], h=0.50)
        band(ax, 0.32, base, base + stages[0][1], C_WAIT, "", h=0.46)
        band(ax, 0.32, base + stages[1][0], base + stages[1][1], C_WAIT, "",
             h=0.46, hatch="//")
        ax.annotate("pulse %d" % (p + 1), xy=(base + 0.20, 0.32 - 0.30),
                    xytext=(0, -3), textcoords="offset points", ha="center",
                    va="top", fontsize=7.5, color=GREY)
    note(ax, nx, 1.32,
         "Every NLDAC pulse in the firmware comes from this row - three\n"
         "ticks per pulse here, one per stage boundary. Each tick is one\n"
         "latch; at this scale its 9 us preload window is thinner than a\n"
         "line. In the idle gap after one, the player also derives the\n"
         "next stage's DAC codes and ramp constants: work the arm used to\n"
         "do before t0.", colour=C_DAC)
    note(ax, nx, 0.32,
         "Two 200 us stages a pulse, then parked on the calibration offset.",
         colour=C_WAIT)

    instant(ax, 0, -0.10, 4.03, "trigger edge", ha="left")
    span(ax, -0.42, lat, lat + period, "period_us = 1000")

    ax.set_xlabel("milliseconds after the trigger edge - the 20 us start latency is the "
                  "leftmost sliver at this scale", fontsize=9)
    ax.set_title("Where a whole train's work sits. The trigger ISR only computes t0 and "
                 "programs a timer; the timer ISR emits every sample.",
                 fontsize=10, loc="left")
    figure_legend(fig)
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    out = FIGS / "stimjim-timing-contexts.png"
    fig.savefig(out)
    plt.close(fig)
    print("->", out)


# ---------------------------------------------------------------------------
# Figure 3: the measurement window
# ---------------------------------------------------------------------------
def room(n, dual=True):
    """Free gap that a measurement point of n reads needs (protocol section 4)."""
    prog = CAL["DACPROG2"] if dual else CAL["DACPROG1"]
    return (CAL["PRELOAD"] + prog + n * (CAL["ADCREAD"] + CAL["ADCSWITCH"])
            + CAL["GUARD"] + CAL["SETTLE"])


def fig_measurement():
    fig, (ax0, ax1) = plt.subplots(2, 1, figsize=(12.6, 7.6), dpi=150,
                                   gridspec_kw=dict(height_ratios=[1.1, 1]))

    # -- panel A: the window between two latches ----------------------------
    n = 4
    gap = room(n)                       # 47 us: exactly what four reads need
    x0, x1, nx = -5.5, 108, 53.0
    rows = [(1.30, 0.90, T_PLAY, "player ISR\nPIT, prio 64"),
            (0.32, 0.78, T_OUT, "analog\noutput")]
    lanes(ax0, rows, x0, x1)
    ax0.set_xlim(x0, x1)
    ax0.set_ylim(-1.75, 2.60)

    band(ax0, 0.32, 0, CAL["SETTLE"], C_WAIT, "SETTLE", h=0.46, fontsize=7.5)
    x = CAL["SETTLE"]
    for _ in range(n):
        band(ax0, 1.30, x, x + CAL["ADCSWITCH"], C_ADC, "sw", h=0.46, fontsize=7,
             textcolour="white")
        band(ax0, 1.30, x + CAL["ADCSWITCH"],
             x + CAL["ADCSWITCH"] + CAL["ADCREAD"], C_ADC, "rd", h=0.46,
             fontsize=7, textcolour="white")
        x += CAL["ADCSWITCH"] + CAL["ADCREAD"]
    band(ax0, 1.30, x, x + CAL["GUARD"], C_WAIT, "", h=0.46)
    x += CAL["GUARD"]
    band(ax0, 1.30, x, x + CAL["PRELOAD"], C_DAC, "PRE", h=0.46, fontsize=7,
         textcolour="white")
    band(ax0, 1.30, x + CAL["PRELOAD"], gap, C_DAC, "PROG2", h=0.46,
         fontsize=7, textcolour="white")
    nldac(ax0, 1.30, 0, h=0.46, label=False)
    nldac(ax0, 1.30, gap, h=0.46, label=False)

    note(ax0, nx, 1.30,
         "All of it one player-ISR pass: the ISR spins to each read's\n"
         "instant rather than returning to the NVIC between them.\n"
         "sw = select the ADC input line, rd = the conversion. One read\n"
         "per selected line per driven channel.", colour=C_ADC)
    note(ax0, nx, 0.32,
         "SETTLE first, because a reading before it describes the level\n"
         "the output had before latch k moved it.", colour=C_WAIT)

    reads_end = CAL["SETTLE"] + n * (CAL["ADCREAD"] + CAL["ADCSWITCH"])
    instant(ax0, 0, -0.10, 1.78, "latch k", ha="left")
    instant(ax0, gap, -0.10, 1.78, "latch k+1", ha="center")
    span(ax0, -0.35, 0, gap,
         "room(4) = PRELOAD + DACPROG2 + 4 x (ADCREAD + ADCSWITCH) + GUARD "
         "+ SETTLE = 47 us")
    span(ax0, -1.00, CAL["SETTLE"], reads_end,
         "4 reads x (ADCSWITCH 4 + ADCREAD 3) = 28 us", fontsize=7.5)
    ax0.annotate("GUARD = 1 us", xy=(x - CAL["GUARD"] / 2, 1.53),
                 xytext=(-16, 20), textcoords="offset points", ha="right",
                 va="bottom", fontsize=7.5, color=C_WAIT,
                 arrowprops=dict(arrowstyle="-", lw=0.6, color=GREY))
    ax0.set_title("A   One measurement point of four reads (V and I on both channels), "
                  "sitting between two latches", fontsize=10, loc="left")
    ax0.set_xlabel("microseconds after latch k", fontsize=9)

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
                                        edgecolor="black", lw=0.5, alpha=0.9,
                                        zorder=3))
            x += width
        plural = "s" if nr > 1 else ""
        ax1.text(50.0, y, "needs %d us" % room(nr, dual), va="center",
                 ha="left", fontsize=8.5)
        ax1.text(-1.2, y, "%s  (%d read%s)" % (label, nr, plural),
                 va="center", ha="right", fontsize=8.5)

    for gapv, style, txt, ytext in (
            (20, "-", "gap 20 us: the default L interval and the W sample floor "
                      "- nothing fits it", 5.35),
            (30, "--", "gap 30 us: an L slot with DT 30", 4.75),
            (35, ":", "gap 35 us: an L slot with DT 35", 4.15)):
        ax1.axvline(gapv, color=C_EDGE, lw=1.2, ls=style, zorder=2)
        ax1.annotate(txt, xy=(gapv, ytext), xytext=(4, 0),
                     textcoords="offset points", ha="left", va="center",
                     fontsize=7.5, color=C_EDGE)

    ax1.set_xlim(-17, 66)
    ax1.set_ylim(-0.8, 5.9)
    ax1.set_yticks([])
    for side in ("left", "right", "top"):
        ax1.spines[side].set_visible(False)
    ax1.set_xlabel("microseconds of free gap needed", fontsize=9)
    ax1.set_title("B   A point whose reads do not fit its gap is rotated over "
                  "repetitions (MEAS fit=1) or refused (fit=0)",
                  fontsize=10, loc="left")

    fig.suptitle("stimjimAWG in-train measurement: what a point needs, and what the "
                 "waveform leaves free", fontsize=11.5, y=0.987)
    figure_legend(fig)
    fig.tight_layout(rect=(0, 0.042, 1, 0.958))
    out = FIGS / "stimjim-timing-measurement.png"
    fig.savefig(out)
    plt.close(fig)
    print("->", out)


if __name__ == "__main__":
    FIGS.mkdir(exist_ok=True)
    fig_latch()
    fig_contexts()
    fig_measurement()
