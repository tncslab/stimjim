"""Does the microsecond timebase survive its own 32-bit wrap?

`cycles64()` extends a 32-bit cycle counter, and at 120 MHz that counter wraps
every 35.79 s. The extension is carried by the player's own wake-ups rather
than by a periodic interrupt, so the failure mode is a *missed* extension: a
train whose deadlines jump backwards by 35.79 s, which shows up as a
completion that never arrives, or one whose pulse count does not match
`duration / period`.

Nothing here needs an oscilloscope or a load -- the board counts its own
pulses and checks its own deadlines. It needs only time: the default 300 s
crosses the wrap eight times, and one crossing is enough to break a broken
extension.

    python longrun.py COM4                 # 300 s, one pulse a second
    python longrun.py COM4 --seconds 3600  # the hours-long version

Uses slot 86 and drives CH0 into whatever is connected; the amplitude is
small and the duty cycle is 0.2 %.
"""

import argparse
import sys
import time

from sjcon import StimJim

SLOT = 86


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM4")
    ap.add_argument("--seconds", type=float, default=300.0)
    ap.add_argument("--period-us", type=int, default=1000000)
    ap.add_argument("--amp-mv", type=int, default=3000)
    a = ap.parse_args()

    dur_us = int(round(a.seconds * 1e6))
    want = dur_us // a.period_us
    wraps = a.seconds / (2**32 / 120e6)

    with StimJim(a.port) as sj:
        sj.drain(quiet=0.3)
        sj.reset()
        print("stimjim:", sj.cmd("IDN")[0])
        print(f"clock before: {sj.cmd1('CLK?')}")
        # Mode 90: voltage, unmeasured. A measured train would stream summaries
        # for hours and prove nothing extra about the timebase.
        print(sj.cmd1(f"S{SLOT},90,3,{a.period_us},{dur_us};{a.amp_mv},0,2000"))
        print(f"\nplaying {a.seconds:g} s = {want} pulses of {a.amp_mv} mV, "
              f"crossing the 35.79 s cycle wrap {wraps:.1f} times")
        t0 = time.time()
        sj.cmd(f"T{SLOT}", quiet=0.1)

        lines = []
        while time.time() - t0 < a.seconds + 20:
            lines += sj.drain(quiet=0.5, limit=5.0)
            if any(x.startswith("Train #") for x in lines):
                break
        elapsed = time.time() - t0

    done = next((x for x in lines if x.startswith("Train #")), None)
    faults = [x for x in lines
              if x.startswith("WARN engine:") or x.startswith("# engine:")]
    print(f"\nhost saw the completion after {elapsed:.1f} s "
          f"(train asked for {a.seconds:g} s)")
    print(f"completion: {done}")
    for f in faults:
        print(f"           {f}")

    bad = 0
    if done is None:
        print("FAIL: no completion -- the train never finished")
        return 1
    got = int(done.split("Delivered")[1].split()[0])
    # Host timing over USB is good to a few hundred ms, so the window is wide;
    # a missed extension would put the completion 35.79 s out, not 0.5 s.
    for ok, msg in ((got == want, f"pulses {got} vs {want} expected"),
                    (abs(elapsed - a.seconds) < 5.0,
                     f"completion {elapsed - a.seconds:+.1f} s off the "
                     f"requested duration"),
                    (not faults, "no timing fault reported")):
        bad += not ok
        print(f"  {'ok  ' if ok else 'FAIL'}  {msg}")
    print("\nALL OK" if bad == 0 else f"\n{bad} PROBLEM(S)")
    return bad


if __name__ == "__main__":
    sys.exit(main())
