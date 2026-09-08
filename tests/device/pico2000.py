"""Minimal ctypes binding for the legacy PicoScope 2000 driver (ps2000.dll).

The PicoScope 2204A enumerates as USB VID_0CE9 PID_1007 and is served by the
legacy `ps2000` driver, whose API differs from the newer `ps2000a` one: an
`open_unit` that returns the handle directly, 0 for "no device", and negative
returns for errors instead of PICO_STATUS codes.

The DLL ships with the PicoScope application, so nothing needs installing.
The PicoScope application must be CLOSED -- it holds the USB device
exclusively and `open_unit` then returns 0.

Scope.probe() is the one-call diagnostic; see capture.py for the measurements.
"""

import ctypes
import os
import time

DLL_CANDIDATES = [
    r"C:\Program Files\Pico Technology\PicoScope 7 T&M Stable\ps2000.dll",
    r"C:\Program Files\Pico Technology\SDK\lib\ps2000.dll",
]

CHANNEL_A, CHANNEL_B, EXTERNAL, TRIG_NONE = 0, 1, 4, 5
AC, DC = 0, 1
RANGE = {  # volts full scale (+/-) -> driver code
    0.02: 1, 0.05: 2, 0.1: 3, 0.2: 4, 0.5: 5,
    1.0: 6, 2.0: 7, 5.0: 8, 10.0: 9, 20.0: 10,
}
RISING, FALLING = 0, 1
MAX_VALUE = 32767            # legacy driver scales every model to +/-32767

# The `time_units` code get_timebase returns. It is the unit of the timestamp
# array of get_times_and_values, NOT of the sample interval beside it -- the
# interval is always nanoseconds. Nothing here calls get_times_and_values, so
# the table is kept only to name the codes the driver reports.
TIME_UNITS = {0: 1e-15, 1: 1e-12, 2: 1e-9, 3: 1e-6, 4: 1e-3, 5: 1.0}

WAVE_SINE, WAVE_SQUARE, WAVE_TRIANGLE = 0, 1, 2
SWEEP_UP = 0


class PicoError(RuntimeError):
    pass


def _load():
    for path in DLL_CANDIDATES:
        if os.path.exists(path):
            os.add_dll_directory(os.path.dirname(path))
            return ctypes.WinDLL(path)
    raise PicoError("ps2000.dll not found in any known PicoScope install")


class Scope:
    def __init__(self):
        self.lib = _load()
        self.lib.ps2000_open_unit.restype = ctypes.c_int16
        h = self.lib.ps2000_open_unit()
        if h == 0:
            raise PicoError(
                "no PicoScope found - close the PicoScope application first "
                "(it holds the USB device exclusively)")
        if h < 0:
            raise PicoError(f"ps2000_open_unit failed ({h})")
        self.h = ctypes.c_int16(h)
        self.ranges = {}

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def close(self):
        try:
            self.lib.ps2000_stop(self.h)
        finally:
            self.lib.ps2000_close_unit(self.h)

    def _need(self, what, rc):
        if rc == 0:
            raise PicoError(f"{what} failed")
        return rc

    # ----------------------------------------------------------------- info
    def info(self, line):
        buf = ctypes.create_string_buffer(80)
        n = self.lib.ps2000_get_unit_info(self.h, buf, ctypes.c_int16(80),
                                          ctypes.c_int16(line))
        return buf.value.decode() if n > 0 else ""

    def probe(self):
        names = {0: "driver", 1: "usb", 2: "hardware", 3: "variant",
                 4: "serial", 5: "cal date", 6: "error"}
        return {v: self.info(k) for k, v in names.items()}

    # --------------------------------------------------------------- config
    def channel(self, ch, enabled=True, coupling=DC, volts=5.0):
        self._need("set_channel", self.lib.ps2000_set_channel(
            self.h, ctypes.c_int16(ch), ctypes.c_int16(1 if enabled else 0),
            ctypes.c_int16(coupling), ctypes.c_int16(RANGE[volts])))
        if enabled:
            self.ranges[ch] = volts
        else:
            self.ranges.pop(ch, None)

    def trigger(self, ch, level_v, direction=RISING, delay_pct=0, auto_ms=0):
        """Edge trigger. delay_pct is the trigger position within the block,
        as a percentage: 0 = trigger at the first sample, -50 = half the block
        is pre-trigger. auto_ms=0 waits indefinitely."""
        adc = int(round(level_v / self.ranges[ch] * MAX_VALUE))
        self._need("set_trigger", self.lib.ps2000_set_trigger(
            self.h, ctypes.c_int16(ch), ctypes.c_int16(adc),
            ctypes.c_int16(direction), ctypes.c_int16(delay_pct),
            ctypes.c_int16(auto_ms)))

    def no_trigger(self):
        self.lib.ps2000_set_trigger(self.h, ctypes.c_int16(TRIG_NONE),
                                    ctypes.c_int16(0), ctypes.c_int16(RISING),
                                    ctypes.c_int16(0), ctypes.c_int16(0))

    def timebase(self, tb, n):
        """(sample interval in seconds, max samples) for a timebase index,
        or None if the driver rejects that index for this configuration."""
        interval = ctypes.c_int32()
        units = ctypes.c_int16()
        max_samples = ctypes.c_int32()
        rc = self.lib.ps2000_get_timebase(
            self.h, ctypes.c_int16(tb), ctypes.c_int32(n),
            ctypes.byref(interval), ctypes.byref(units), ctypes.c_int16(1),
            ctypes.byref(max_samples))
        if rc == 0:
            return None
        # The interval is in nanoseconds whatever `units` says (see TIME_UNITS).
        return interval.value * 1e-9, max_samples.value

    def pick_timebase(self, want_dt, n):
        """Fastest timebase whose interval is still >= want_dt, so a requested
        window is covered without oversampling past the driver's limit.

        Timebases the current channel configuration cannot reach are refused by
        the driver -- timebase 0 with two channels enabled, for one -- and
        `timebase` returns None for those, so the search skips them. Every
        index the driver does accept delivers the interval it reports, checked
        against the unit's own AWG by tests/device/scope_timebase.py.
        """
        best = None
        for tb in range(0, 24):
            r = self.timebase(tb, n)
            if r is None:
                continue
            dt, mx = r
            best = (tb, dt, mx)
            if dt >= want_dt:
                return best
        return best

    # -------------------------------------------------------------- capture
    def block(self, n, timebase, timeout=20.0, on_armed=None):
        """Capture n samples on both channels. Returns (dt, {ch: [volts]}).

        `on_armed` runs the instant the scope is armed and before the wait for
        the trigger. A one-shot stimulus has to be started from there: started
        earlier it is over before the scope is listening.
        """
        r = self.timebase(timebase, n)
        if r is None:
            raise PicoError(f"timebase {timebase} invalid for {n} samples")
        dt, _ = r
        ready_ms = ctypes.c_int32()
        self._need("run_block", self.lib.ps2000_run_block(
            self.h, ctypes.c_int32(n), ctypes.c_int16(timebase),
            ctypes.c_int16(1), ctypes.byref(ready_ms)))
        if on_armed is not None:
            on_armed()

        t0 = time.time()
        while self.lib.ps2000_ready(self.h) == 0:
            if time.time() - t0 > timeout:
                self.lib.ps2000_stop(self.h)
                raise PicoError("capture timed out waiting for the trigger")
            time.sleep(0.002)

        a = (ctypes.c_int16 * n)()
        b = (ctypes.c_int16 * n)()
        overflow = ctypes.c_int16()
        got = self.lib.ps2000_get_values(
            self.h, ctypes.byref(a), ctypes.byref(b), None, None,
            ctypes.byref(overflow), ctypes.c_int32(n))
        self.lib.ps2000_stop(self.h)
        if got <= 0:
            raise PicoError("get_values returned no samples")

        out = {}
        for ch, buf in ((CHANNEL_A, a), (CHANNEL_B, b)):
            if ch in self.ranges:
                s = self.ranges[ch] / MAX_VALUE
                out[ch] = [buf[i] * s for i in range(got)]
        return dt, out, bool(overflow.value)

    # ----------------------------------------------- built-in signal gen
    def square(self, freq_hz, pk_to_pk_v, offset_v=None):
        """AWG square wave -- the trigger source wired to StimJim IN0.

        The trigger input wants a unipolar edge, so the wave sits on half its
        own amplitude unless an explicit offset is given.
        """
        pk = int(round(pk_to_pk_v * 1e6))                       # microvolts
        off = int(round((pk_to_pk_v / 2 if offset_v is None else offset_v) * 1e6))
        self._need("set_sig_gen_built_in", self.lib.ps2000_set_sig_gen_built_in(
            self.h, ctypes.c_int32(off), ctypes.c_uint32(pk),
            ctypes.c_int32(WAVE_SQUARE),
            ctypes.c_float(freq_hz), ctypes.c_float(freq_hz),
            ctypes.c_float(0), ctypes.c_float(0),
            ctypes.c_int32(SWEEP_UP), ctypes.c_uint32(0)))

    def siggen_off(self):
        self.lib.ps2000_set_sig_gen_built_in(
            self.h, ctypes.c_int32(0), ctypes.c_uint32(0), ctypes.c_int32(WAVE_SQUARE),
            ctypes.c_float(1000), ctypes.c_float(1000), ctypes.c_float(0),
            ctypes.c_float(0), ctypes.c_int32(SWEEP_UP), ctypes.c_uint32(0))


if __name__ == "__main__":
    with Scope() as s:
        for k, v in s.probe().items():
            print(f"{k:9s}: {v}")
        s.channel(CHANNEL_A, volts=5.0)
        s.channel(CHANNEL_B, volts=5.0)
        for want in (1e-7, 1e-6, 1e-5, 1e-4):
            print(f"want dt={want*1e6:8.2f} us ->", s.pick_timebase(want, 2000))
