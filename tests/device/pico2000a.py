"""Minimal ctypes binding for the PicoScope 2000a driver (PicoScope 2204A).

Only what block-mode capture needs. The driver DLL ships with the PicoScope 7
application, so nothing has to be installed: no picosdk package, no C SDK.
Every call returns a PICO_STATUS and is checked, so a wrong constant shows up
immediately as a named failure rather than as silently wrong data.

The PicoScope application must be closed -- it holds the USB device exclusively.
"""

import ctypes
import os
import time

DLL_CANDIDATES = [
    r"C:\Program Files\Pico Technology\PicoScope 7 T&M Stable\ps2000a.dll",
    r"C:\Program Files\Pico Technology\SDK\lib\ps2000a.dll",
]

# --- enums (ps2000aApi.h) --------------------------------------------------
CHANNEL_A, CHANNEL_B, CHANNEL_EXT = 0, 1, 4
AC, DC = 0, 1
RANGE = {  # volts (full scale, +/-) -> driver code
    0.02: 1, 0.05: 2, 0.1: 3, 0.2: 4, 0.5: 5,
    1.0: 6, 2.0: 7, 5.0: 8, 10.0: 9, 20.0: 10,
}
RISING, FALLING = 2, 3
RATIO_MODE_NONE = 0

# Signal generator (used for the trigger source into StimJim IN0)
SIGGEN_NONE, SIGGEN_SCOPE, SIGGEN_AUX = 0, 1, 2
WAVE_SQUARE = 1
SWEEP_UP = 0
SIGGEN_TRIG_RISING = 2
SIGGEN_TRIG_SOURCE_NONE = 0


class PicoError(RuntimeError):
    pass


def _load():
    for path in DLL_CANDIDATES:
        if os.path.exists(path):
            # The DLL's own directory must be searchable for its dependencies.
            os.add_dll_directory(os.path.dirname(path))
            return ctypes.WinDLL(path)
    raise PicoError("ps2000a.dll not found in any known PicoScope install")


class Scope:
    def __init__(self):
        self.lib = _load()
        self.h = ctypes.c_int16()
        self._check("OpenUnit", self.lib.ps2000aOpenUnit(ctypes.byref(self.h), None))
        mv = ctypes.c_int16()
        self._check("MaximumValue", self.lib.ps2000aMaximumValue(self.h, ctypes.byref(mv)))
        self.max_adc = mv.value
        self.ranges = {}          # channel -> volts full scale, for scaling

    # ------------------------------------------------------------- plumbing
    def _check(self, what, status):
        if status != 0:
            raise PicoError(f"{what} failed, PICO_STATUS=0x{status & 0xFFFFFFFF:08X}")

    def info(self, line=3):
        buf = ctypes.create_string_buffer(64)
        need = ctypes.c_int16()
        self._check("GetUnitInfo", self.lib.ps2000aGetUnitInfo(
            self.h, buf, 64, ctypes.byref(need), ctypes.c_uint32(line)))
        return buf.value.decode()

    def close(self):
        try:
            self.lib.ps2000aStop(self.h)
        finally:
            self.lib.ps2000aCloseUnit(self.h)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # -------------------------------------------------------------- config
    def channel(self, ch, enabled=True, coupling=DC, volts=5.0, offset=0.0):
        code = RANGE[volts]
        self._check("SetChannel", self.lib.ps2000aSetChannel(
            self.h, ctypes.c_int32(ch), ctypes.c_int16(1 if enabled else 0),
            ctypes.c_int32(coupling), ctypes.c_int32(code), ctypes.c_float(offset)))
        if enabled:
            self.ranges[ch] = volts
        else:
            self.ranges.pop(ch, None)

    def trigger(self, ch, level_v, direction=RISING, delay=0, auto_ms=0):
        """Simple edge trigger. auto_ms=0 waits indefinitely for the edge."""
        adc = int(round(level_v / self.ranges[ch] * self.max_adc))
        self._check("SetSimpleTrigger", self.lib.ps2000aSetSimpleTrigger(
            self.h, ctypes.c_int16(1), ctypes.c_int32(ch), ctypes.c_int16(adc),
            ctypes.c_int32(direction), ctypes.c_uint32(delay), ctypes.c_int16(auto_ms)))

    def no_trigger(self):
        self._check("SetSimpleTrigger", self.lib.ps2000aSetSimpleTrigger(
            self.h, ctypes.c_int16(0), ctypes.c_int32(0), ctypes.c_int16(0),
            ctypes.c_int32(RISING), ctypes.c_uint32(0), ctypes.c_int16(0)))

    def timebase(self, tb, n):
        """Resolve a timebase index to its real sample interval, in seconds."""
        interval_ns = ctypes.c_float()
        max_samples = ctypes.c_int32()
        self._check("GetTimebase2", self.lib.ps2000aGetTimebase2(
            self.h, ctypes.c_uint32(tb), ctypes.c_int32(n),
            ctypes.byref(interval_ns), ctypes.c_int16(0),
            ctypes.byref(max_samples), ctypes.c_uint32(0)))
        return interval_ns.value * 1e-9, max_samples.value

    def pick_timebase(self, want_dt, n):
        """Smallest timebase index whose interval is >= want_dt seconds.

        The 2000a mapping is model-dependent, so it is probed rather than
        computed: ask the driver, walk up until the interval is long enough.
        """
        best = None
        for tb in range(0, 2 ** 16):
            try:
                dt, _ = self.timebase(tb, n)
            except PicoError:
                continue
            best = (tb, dt)
            if dt >= want_dt:
                return tb, dt
            if tb > 2000:      # far past anything useful; stop probing
                break
        return best

    # ------------------------------------------------------------- capture
    def block(self, channels, n, timebase, pre_frac=0.0, timeout=20.0):
        """Capture `n` samples on `channels`; returns (dt, {ch: volts list})."""
        dt, _ = self.timebase(timebase, n)
        pre = int(n * pre_frac)
        post = n - pre

        bufs = {}
        for ch in channels:
            b = (ctypes.c_int16 * n)()
            bufs[ch] = b
            self._check("SetDataBuffer", self.lib.ps2000aSetDataBuffer(
                self.h, ctypes.c_int32(ch), ctypes.byref(b), ctypes.c_int32(n),
                ctypes.c_uint32(0), ctypes.c_int32(RATIO_MODE_NONE)))

        ready_ms = ctypes.c_int32()
        self._check("RunBlock", self.lib.ps2000aRunBlock(
            self.h, ctypes.c_int32(pre), ctypes.c_int32(post),
            ctypes.c_uint32(timebase), ctypes.c_int16(0), ctypes.byref(ready_ms),
            ctypes.c_uint32(0), None, None))

        ready = ctypes.c_int16(0)
        t0 = time.time()
        while not ready.value:
            self._check("IsReady", self.lib.ps2000aIsReady(self.h, ctypes.byref(ready)))
            if time.time() - t0 > timeout:
                self.lib.ps2000aStop(self.h)
                raise PicoError("capture timed out waiting for the trigger")
            time.sleep(0.002)

        got = ctypes.c_uint32(n)
        overflow = ctypes.c_int16()
        self._check("GetValues", self.lib.ps2000aGetValues(
            self.h, ctypes.c_uint32(0), ctypes.byref(got), ctypes.c_uint32(1),
            ctypes.c_int32(RATIO_MODE_NONE), ctypes.c_uint32(0),
            ctypes.byref(overflow)))
        self.lib.ps2000aStop(self.h)

        m = got.value
        out = {}
        for ch, b in bufs.items():
            scale = self.ranges[ch] / self.max_adc
            out[ch] = [b[i] * scale for i in range(m)]
        return dt, out, bool(overflow.value)

    # ------------------------------------------------- built-in signal gen
    def square(self, freq_hz, pk_to_pk_v, offset_v=None):
        """Drive the AWG output with a square wave (the StimJim IN0 trigger).

        The trigger input wants a unipolar 0->2 V edge, so the wave is offset
        by half its amplitude unless told otherwise.
        """
        pk = int(round(pk_to_pk_v * 1e6))              # microvolts
        off = int(round((pk_to_pk_v / 2 if offset_v is None else offset_v) * 1e6))
        self._check("SetSigGenBuiltIn", self.lib.ps2000aSetSigGenBuiltIn(
            self.h, ctypes.c_int32(off), ctypes.c_uint32(pk),
            ctypes.c_int16(WAVE_SQUARE),
            ctypes.c_float(freq_hz), ctypes.c_float(freq_hz),
            ctypes.c_float(0), ctypes.c_float(0), ctypes.c_int32(SWEEP_UP),
            ctypes.c_int32(0),                          # PS2000A_ES_OFF
            ctypes.c_uint32(0), ctypes.c_uint32(0),     # shots, sweeps
            ctypes.c_int32(SIGGEN_TRIG_RISING),
            ctypes.c_int32(SIGGEN_TRIG_SOURCE_NONE),    # free-running
            ctypes.c_int16(0)))


if __name__ == "__main__":
    with Scope() as s:
        for i, name in ((0, "driver"), (3, "model"), (4, "serial"), (6, "cal date")):
            try:
                print(f"{name:9s}: {s.info(i)}")
            except PicoError as e:
                print(f"{name:9s}: {e}")
        print("max ADC :", s.max_adc)
        s.channel(CHANNEL_A, volts=5.0)
        for want in (1e-6, 1e-5, 1e-4):
            print(f"want dt={want*1e6:6.1f} us ->", s.pick_timebase(want, 1000))
