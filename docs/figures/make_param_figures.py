"""Waveform parametrization figures for stimjimAWG: which number in the command
line sets which part of the waveform.

One figure per train type, each built the same way: the command line across the
top with every field numbered, a view of the whole train (delay, period,
duration, envelope), and a view of one pulse with the per-stage fields marked.
The numbers in the drawing are the numbers in the command line.

  figs/stimjim-param-S.png   piecewise-constant train
  figs/stimjim-param-L.png   piecewise-linear (ramp) train
  figs/stimjim-param-W.png   sine train

The example lines are chosen to draw well (a 10 ms train rather than the 1 s of
the README) and are otherwise ordinary, valid commands.

    python docs/figures/make_param_figures.py
"""

import math
import pathlib

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch

FIGS = pathlib.Path(__file__).resolve().parents[2] / "figs"

C_HDR = "#0072B2"     # header fields: slot, modes, period, duration
C_OPT = "#E69F00"     # optional header fields: delay, dt
C_STG = "#009E73"     # stage / triplet fields
C_ENV = "#CC79A7"     # envelope
C_CH0 = "#0072B2"
C_CH1 = "#D55E00"
GREY = "0.35"

# Monospace advance width as a fraction of the font size. DejaVu Sans Mono, the
# matplotlib default monospace face, is 0.602 em; the figure only needs the
# command line to span its axis, so a value good to a percent is enough.
ADVANCE = 0.602


def callout(ax, x, y, num, colour, fontsize=8.5):
    """The circled field number, used identically in the command line and the
    drawing -- that pairing is the whole point of these figures."""
    ax.text(x, y, str(num), fontsize=fontsize, ha="center", va="center",
            color=colour, zorder=9,
            bbox=dict(boxstyle="circle,pad=0.24", fc="white", ec=colour, lw=1.0))


def command_line(ax, tokens, width_in):
    """Render one command as monospace text on a character grid.

    `tokens` is a list of (text, number or None, colour). Laying the axis out in
    character units is what lets a callout sit exactly over its own field: the
    font size is solved from the axis width so the whole line just fits.
    """
    text = "".join(t for t, _, _ in tokens)
    n = len(text)
    fs = min(14.0, width_in * 72.0 / (ADVANCE * n))
    # One data unit must be exactly one character, or the tokens drift apart
    # from the callouts. When the size cap bites, widen the grid instead of
    # stretching the text: the line then starts at 0 and ends short of `cols`.
    cols = width_in * 72.0 / (ADVANCE * fs)
    ax.set_xlim(0, cols)
    ax.set_ylim(0, 1)
    ax.axis("off")
    pos = 0
    for chunk, num, colour in tokens:
        ax.text(pos, 0.30, chunk, family="monospace", fontsize=fs, ha="left",
                va="center", color=colour, zorder=5)
        if num is not None:
            callout(ax, pos + len(chunk) / 2.0, 0.86, num, colour,
                    fontsize=min(8.5, fs * 0.85))
        pos += len(chunk)


def span(ax, y, x0, x1, label, colour=GREY, fontsize=8.5, dy=-13, num=None):
    ax.add_patch(FancyArrowPatch((x0, y), (x1, y), arrowstyle="<->",
                                 mutation_scale=8, lw=0.9, color=colour, zorder=5))
    ax.annotate(label, xy=((x0 + x1) / 2, y), xytext=(0, dy),
                textcoords="offset points", ha="center", va="top",
                fontsize=fontsize, color=colour, zorder=6)
    if num is not None:
        callout(ax, (x0 + x1) / 2, y, num, colour)


def bare(ax, xlabel):
    ax.set_yticks([])
    for side in ("left", "right", "top"):
        ax.spines[side].set_visible(False)
    ax.set_xlabel(xlabel, fontsize=9)


def channel_axes(ax, colour, name, unit):
    ax.axhline(0, color="0.75", lw=0.8, zorder=1)
    ax.set_ylabel(f"{name}\n{unit}", fontsize=9, color=colour)
    ax.tick_params(axis="y", labelsize=8, colors=colour)
    for side in ("right", "top"):
        ax.spines[side].set_visible(False)


def train_view(ax, delay_ms, period_ms, duration_ms, n_pulses, pulse_ms,
               nums=(4, 5, 6)):
    """Delay, period and duration on one axis, pulses drawn as blocks."""
    t0 = delay_ms
    for k in range(n_pulses):
        x = t0 + k * period_ms
        ax.add_patch(plt.Rectangle((x, 0), pulse_ms, 1.0, facecolor=C_CH0,
                                   edgecolor="black", lw=0.5, alpha=0.85, zorder=3))
    ax.plot([0, 0], [-0.9, 1.5], color=C_CH1, lw=1.4, zorder=5)
    ax.annotate("start request\n(trigger edge, T/U, or the menu)", xy=(0, -0.95),
                xytext=(0, -4), textcoords="offset points", ha="left", va="top",
                fontsize=8.5, color=C_CH1)
    span(ax, 2.15, 0, t0, "delay_us", colour=C_OPT, num=nums[2])
    span(ax, 2.15, t0, t0 + period_ms, "period_us", colour=C_HDR, num=nums[0])
    span(ax, -0.55, t0, t0 + duration_ms, "duration_us, from t0", colour=C_HDR,
         num=nums[1])
    ax.set_ylim(-2.4, 3.1)
    ax.set_xlim(-0.06 * (t0 + duration_ms), 1.10 * (t0 + duration_ms))
    bare(ax, "milliseconds after the start request")


def finish(fig, name):
    out = FIGS / name
    fig.savefig(out)
    plt.close(fig)
    print("->", out)


# ---------------------------------------------------------------------------
# S: piecewise-constant
# ---------------------------------------------------------------------------
def fig_S():
    fig = plt.figure(figsize=(11.5, 7.6), dpi=150)
    gs = fig.add_gridspec(4, 1, height_ratios=[0.55, 1.25, 1.0, 1.0], hspace=0.72,
                          left=0.085, right=0.975, top=0.875, bottom=0.075)

    command_line(fig.add_subplot(gs[0]), [
        ("S0", 1, C_HDR), (",0", 2, C_HDR), (",1", 3, C_HDR),
        (",2000", 4, C_HDR), (",10000", 5, C_HDR), (",5000", 6, C_OPT),
        (";4000,0,150", 7, C_STG), (";-4000,-2000,200", 8, C_STG),
    ], width_in=10.4)

    train_view(fig.add_subplot(gs[1]), delay_ms=5.0, period_ms=2.0,
               duration_ms=10.0, n_pulses=5, pulse_ms=0.35)

    # ---- one pulse, per channel
    stages = [(150, 4000, 0), (200, -4000, -2000)]
    edges = [0]
    for dur, _, _ in stages:
        edges.append(edges[-1] + dur)
    tail = edges[-1] + 120

    for ax_i, colour, name, unit, idx in [
            (2, C_CH0, "channel 0", "mV, mode 0 = voltage", 1),
            (3, C_CH1, "channel 1", "uA, mode 1 = current", 2)]:
        ax = fig.add_subplot(gs[ax_i])
        xs, ys = [0], [0]
        for k, (dur, a0, a1) in enumerate(stages):
            a = a0 if idx == 1 else a1
            xs += [edges[k], edges[k + 1]]
            ys += [a, a]
        xs += [edges[-1], tail]
        ys += [0, 0]
        ax.step(xs, ys, where="post", color=colour, lw=2.0, zorder=4)
        channel_axes(ax, colour, name, unit)
        ax.set_xlim(-25, tail)
        ax.set_ylim(-5600, 5900)
        for k, (dur, a0, a1) in enumerate(stages):
            a = a0 if idx == 1 else a1
            ax.axvline(edges[k + 1], color="0.8", lw=0.8, ls=":", zorder=1)
            ax.annotate(f"{a}", xy=(edges[k + 1] - 6, a), xytext=(0, 8),
                        textcoords="offset points", ha="right", va="bottom",
                        fontsize=8.5, color=colour)
        if ax_i == 2:
            for k in range(len(stages)):
                callout(ax, (edges[k] + edges[k + 1]) / 2, 4900, 7 + k, C_STG)
            ax.set_xticklabels([])
        else:
            for k in range(len(stages)):
                span(ax, -2400, edges[k], edges[k + 1],
                     f"dur_us = {stages[k][0]}", colour=C_STG, fontsize=8)
            ax.annotate("after the last stage the outputs return to the calibration\n"
                        "offset and are grounded until the next pulse starts",
                        xy=(tail * 0.55, 3600), ha="center", va="center",
                        fontsize=8, color=GREY)
            ax.set_xlabel("microseconds after the start of one pulse", fontsize=9)

    fig.suptitle("S - piecewise-constant train\n"
                 "S<idx>,<mode0>,<mode1>,<period_us>,<duration_us>[,<delay_us>]"
                 "; <a0>,<a1>,<dur_us>; ...   (0 to 10 stages)",
                 fontsize=11, y=0.985)
    fig.text(0.085, 0.925,
             "modes: 0 voltage, 1 current, 2 hi-Z, 3 grounded (2 and 3 = this "
             "train does not drive the channel);  90 / 91 = voltage / current "
             "with measurement off",
             fontsize=8.5, color=GREY, ha="left")
    finish(fig, "stimjim-param-S.png")


# ---------------------------------------------------------------------------
# L: piecewise-linear
# ---------------------------------------------------------------------------
def fig_L():
    fig = plt.figure(figsize=(11.5, 7.6), dpi=150)
    gs = fig.add_gridspec(4, 1, height_ratios=[0.55, 1.25, 1.0, 1.0], hspace=0.72,
                          left=0.085, right=0.975, top=0.875, bottom=0.075)

    command_line(fig.add_subplot(gs[0]), [
        ("L0", 1, C_HDR), (",0", 2, C_HDR), (",1", 3, C_HDR),
        (",2000", 4, C_HDR), (",10000", 5, C_HDR), (",5000", 6, C_OPT),
        (",50", 9, C_OPT),
        (";4000,2000,400", 7, C_STG), (";0,0,300", 8, C_STG),
    ], width_in=10.4)

    train_view(fig.add_subplot(gs[1]), delay_ms=5.0, period_ms=2.0,
               duration_ms=10.0, n_pulses=5, pulse_ms=0.7)

    dt = 50
    stages = [(400, 4000, 2000), (300, 0, 0)]
    edges = [0]
    for dur, _, _ in stages:
        edges.append(edges[-1] + dur)
    tail = edges[-1] + 120

    for ax_i, colour, name, unit, idx in [
            (2, C_CH0, "channel 0", "mV, mode 0 = voltage", 1),
            (3, C_CH1, "channel 1", "uA, mode 1 = current", 2)]:
        ax = fig.add_subplot(gs[ax_i])
        # The ramp is latched sample by sample every dt_us, so draw the samples.
        xs, ys = [0.0], [0.0]
        prev = 0.0
        for k, (dur, a0, a1) in enumerate(stages):
            a = float(a0 if idx == 1 else a1)
            n = max(1, int(round(dur / dt)))
            for j in range(1, n + 1):
                xs.append(edges[k] + j * dur / n)
                ys.append(prev + (a - prev) * j / n)
            prev = a
        xs += [edges[-1], tail]
        ys += [0, 0]
        ax.step(xs, ys, where="post", color=colour, lw=1.6, zorder=4)
        ax.plot(xs[:-2], ys[:-2], ls="--", lw=0.9, color="0.55", zorder=3)
        ax.plot(xs[:-2], ys[:-2], "o", ms=2.6, color=colour, zorder=5)
        channel_axes(ax, colour, name, unit)
        ax.set_xlim(-25, tail)
        ax.set_ylim(-2600, 5900)
        for k, (dur, a0, a1) in enumerate(stages):
            a = a0 if idx == 1 else a1
            ax.axvline(edges[k + 1], color="0.8", lw=0.8, ls=":", zorder=1)
            ax.annotate(f"{a}", xy=(edges[k + 1], a), xytext=(4, 8),
                        textcoords="offset points", ha="left", va="bottom",
                        fontsize=8.5, color=colour)
        if ax_i == 2:
            callout(ax, edges[0] + 200, 1500, 7, C_STG)
            callout(ax, edges[1] + 150, 1500, 8, C_STG)
            span(ax, 4900, edges[0], edges[0] + dt, "", colour=C_OPT)
            ax.annotate("dt_us = 50: one latch per sample.\n"
                        "Each stage ramps from where the last one ended.",
                        xy=(edges[0] + dt, 4900), xytext=(10, 6),
                        textcoords="offset points", ha="left", va="bottom",
                        fontsize=8, color=C_OPT)
            callout(ax, edges[0] + dt / 2, 4900, 9, C_OPT)
            ax.set_xticklabels([])
        else:
            for k in range(len(stages)):
                span(ax, -1150, edges[k], edges[k + 1],
                     f"dur_us = {stages[k][0]}", colour=C_STG, fontsize=8)
            ax.annotate("dur_us = 0 is an instant jump, so a jump stage followed\n"
                        "by a same-value stage gives a flat hold",
                        xy=(tail * 0.5, 5300), ha="center", va="center",
                        fontsize=8, color=GREY)
            ax.set_xlabel("microseconds after the start of one pulse", fontsize=9)

    fig.suptitle("L - piecewise-linear (ramp) train\n"
                 "L<idx>,<mode0>,<mode1>,<period_us>,<duration_us>"
                 "[,<delay_us>[,<dt_us>]]; <a0>,<a1>,<dur_us>; ...",
                 fontsize=11, y=0.985)
    fig.text(0.085, 0.935,
             "Same syntax as S; each stage ramps to its amplitudes instead of "
             "jumping. dt_us is the sample interval (default 20 us), and it is "
             "also the free gap\nan in-train measurement point has to fit "
             "(figs/stimjim-timing-measurement.png).",
             fontsize=8.5, color=GREY, ha="left", va="top")
    finish(fig, "stimjim-param-L.png")


# ---------------------------------------------------------------------------
# W: sine
# ---------------------------------------------------------------------------
def fig_W():
    fig = plt.figure(figsize=(11.5, 7.6), dpi=150)
    gs = fig.add_gridspec(3, 1, height_ratios=[0.75, 1.15, 1.35], hspace=0.60,
                          left=0.085, right=0.975, top=0.875, bottom=0.075)

    command_line(fig.add_subplot(gs[0]), [
        ("W0", 1, C_HDR), (",0", 2, C_HDR), (",0", 3, C_HDR),
        (",2000", 4, C_HDR), (",10000", 5, C_HDR), (",5000", 6, C_OPT),
        (";4000,2000,1200", 7, C_STG), (";2000,2000,0", 8, C_STG),
        (";90,0,0", 9, C_STG), (";2000,3000,0", 10, C_ENV),
    ], width_in=10.4)

    # ---- train view with the envelope
    ax = fig.add_subplot(gs[1])
    delay, period, duration, burst = 5.0, 2.0, 10.0, 1.2
    t0 = delay
    for k in range(5):
        x = t0 + k * period
        ax.add_patch(plt.Rectangle((x, 0), burst, 1.0, facecolor=C_CH0,
                                   edgecolor="black", lw=0.5, alpha=0.75, zorder=3))
    # ENV: 0 -> 1 over rampIn from t0, 1 -> 0 ending exactly at duration.
    # Drawn above the burst blocks (scale 1.45) so the two do not sit on
    # each other: it is a scale factor on amplitude, not a waveform.
    rin, rout, escale = 2.0, 3.0, 1.45
    ax.plot([t0, t0 + rin, t0 + duration - rout, t0 + duration],
            [0, escale, escale, 0], color=C_ENV, lw=1.8, ls="--", zorder=6)
    callout(ax, t0 + rin, escale, 10, C_ENV)
    ax.annotate("ENV: the amplitude scale runs 0 to 1 over rampIn_us and back to 0,\n"
                "ending exactly at duration_us. A W line carries it as a 5th triplet.",
                xy=(t0 + duration * 0.68, 2.85), ha="center", va="center",
                fontsize=8, color=C_ENV)
    ax.plot([0, 0], [-0.9, 1.5], color=C_CH1, lw=1.4, zorder=5)
    ax.annotate("start request", xy=(0, -0.95), xytext=(0, -4),
                textcoords="offset points", ha="left", va="top",
                fontsize=8.5, color=C_CH1)
    span(ax, 2.25, 0, t0, "delay_us", colour=C_OPT, num=6)
    span(ax, 2.25, t0, t0 + period, "period_us", colour=C_HDR, num=4)
    span(ax, -0.55, t0, t0 + duration, "duration_us, from t0", colour=C_HDR, num=5)
    ax.set_ylim(-2.8, 3.3)
    ax.set_xlim(-1.0, 17.5)
    bare(ax, "milliseconds after the start request")
    ax.annotate("each block is one burst of burst_us; one burst per period",
                xy=(t0 + duration / 2, -2.15), ha="center", va="center",
                fontsize=8, color=C_HDR)

    # ---- one burst
    ax = fig.add_subplot(gs[2])
    freq, amp, phase_deg, burst_us = 2000.0, 4000.0, 90.0, 1200.0
    fs = 64 * freq                                # Fs = 64 * f_max, clamped
    n = int(burst_us * 1e-6 * fs)
    ts = [i / fs * 1e6 for i in range(n + 1)]
    ys = [amp * math.sin(2 * math.pi * (freq * t * 1e-6) + math.radians(phase_deg))
          for t in ts]
    ax.step(ts + [burst_us, burst_us + 400], ys + [0, 0], where="post",
            color=C_CH0, lw=1.5, zorder=4)
    ax.plot(ts, ys, "o", ms=2.2, color=C_CH0, zorder=5)
    channel_axes(ax, C_CH0, "channel 0", "mV")
    # Empty margins left and right carry the labels that would otherwise sit on
    # the trace: the amplitude arrow on the left, the two notes on the right.
    ax.set_xlim(-330, burst_us + 640)
    ax.set_ylim(-8600, 9200)
    ax.set_xticks([0, 200, 400, 600, 800, 1000, 1200, 1400])

    ax.axhline(amp, color="0.8", lw=0.8, ls=":", zorder=1)
    ax.add_patch(FancyArrowPatch((-150, 0), (-150, amp), arrowstyle="<->",
                                 mutation_scale=8, lw=0.9, color=C_STG, zorder=5))
    ax.annotate("amp0 =\n4000 mV", xy=(-135, amp * 0.5), ha="left", va="center",
                fontsize=8.5, color=C_STG)
    callout(ax, -270, amp * 0.5, 7, C_STG)

    one_period = 1e6 / freq
    span(ax, 7100, 125, 125 + one_period, f"1 / freq0 = {one_period:.0f} us",
         colour=C_STG, fontsize=8.5, num=8)
    span(ax, -5600, 0, burst_us, "burst_us = 1200, the 3rd field of the 1st triplet",
         colour=C_STG, fontsize=8.5, num=7)
    ax.annotate("phase0 = 90 deg:\nthe burst starts at the peak",
                xy=(0, amp), xytext=(30, 30), textcoords="offset points",
                ha="left", va="bottom", fontsize=8.5, color=C_STG,
                arrowprops=dict(arrowstyle="-|>", lw=0.8, color=C_STG))
    callout(ax, 40, amp + 3100, 9, C_STG)
    ax.annotate("the phase restarts every\nburst, so bursts are\nidentical and drift-free",
                xy=(burst_us + 70, 2600), ha="left", va="center",
                fontsize=8, color=GREY)
    ax.annotate("samples on an exact\nCPU-cycle grid, Fs =\nclamp(64 x f_max,\n"
                "1 kHz, 50 kHz)",
                xy=(burst_us + 70, -3400), ha="left", va="center",
                fontsize=8, color=GREY)
    ax.set_xlabel("microseconds after the start of one burst", fontsize=9)

    fig.suptitle("W - sine train\n"
                 "W<idx>,<mode0>,<mode1>,<period_us>,<duration_us>[,<delay_us>];\n"
                 "<amp0>,<amp1>,<burst_us>; <freq0>,<freq1>,0; <phase0>,<phase1>,0"
                 "[; <rampIn_us>,<rampOut_us>,<shape>]",
                 fontsize=10.5, y=0.995)
    fig.text(0.085, 0.895,
             "Frequencies in Hz (decimals accepted), phases in degrees; the 3rd "
             "field of the frequency and phase triplets is reserved and must be 0.",
             fontsize=8.5, color=GREY, ha="left", va="top")
    finish(fig, "stimjim-param-W.png")


if __name__ == "__main__":
    FIGS.mkdir(exist_ok=True)
    fig_S()
    fig_L()
    fig_W()
