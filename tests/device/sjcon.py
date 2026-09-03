"""Serial console helper for the stimjimAWG firmware.

Opens the Teensy USB CDC port, sends command lines and collects the replies.
The protocol has no single reply terminator -- some commands answer with one
line, some with a block ending in ``OK`` -- so replies are collected until the
port stays quiet for ``quiet`` seconds. That is reliable over USB CDC and keeps
the helper free of per-command knowledge.

Usage as a library::

    from sjcon import StimJim
    with StimJim("COM4") as sj:
        print(sj.cmd("IDN"))

Usage from the shell::

    python sjcon.py COM4 IDN "S0,0,0,10000,500000,2000;5000,0,1000" S0?
"""

import sys
import time

import serial


class StimJim:
    def __init__(self, port="COM4", timeout=0.05):
        # Baud is irrelevant for USB CDC but pyserial insists on a number.
        self.ser = serial.Serial(port, 115200, timeout=timeout)
        self.log = []

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def close(self):
        self.ser.close()

    def drain(self, quiet=0.30, limit=8.0):
        """Read until the port has been silent for `quiet` seconds."""
        out, last, start = [], time.time(), time.time()
        buf = b""
        while time.time() - last < quiet and time.time() - start < limit:
            chunk = self.ser.read(4096)
            if chunk:
                buf += chunk
                last = time.time()
        for line in buf.split(b"\n"):
            line = line.rstrip(b"\r").decode("utf-8", "replace")
            if line:
                out.append(line)
        return out

    def cmd(self, line, quiet=0.30, limit=8.0):
        self.ser.reset_input_buffer()
        self.ser.write((line + "\n").encode())
        self.ser.flush()
        reply = self.drain(quiet, limit)
        self.log.append((line, reply))
        return reply

    # Lines the firmware emits on its own schedule from loop(), which can land
    # in the middle of any reply: end-of-train summaries and deferred warnings.
    ASYNC = ("Train #", "Note: no measurement", "WARN trigger:", "MSUM,", "MDATA,")

    def cmd1(self, line, **kw):
        """Send a command expected to answer with exactly one record line.

        Asynchronous notices go to `self.async_lines` and `#` comments to
        `self.comments` rather than confusing the reply, so neither a train
        finishing mid-command nor an explanatory note is an error. Skipping `#`
        is the machine-parse rule the protocol states in §1.
        """
        r = self.cmd(line, **kw)
        keep = [x for x in r if not x.startswith(self.ASYNC) and not x.startswith("#")]
        self.async_lines = [x for x in r if x.startswith(self.ASYNC)]
        self.comments = [x for x in r if x.startswith("#")]
        assert len(keep) == 1, f"{line!r} -> expected 1 record line, got {r}"
        return keep[0]

    def reset(self):
        """Stop both engines and swallow whatever they print on the way out."""
        self.cmd("T-1", quiet=0.2)
        self.cmd("U-1", quiet=0.2)
        self.drain(quiet=0.6)


def main(argv):
    port = argv[1] if len(argv) > 1 else "COM4"
    with StimJim(port) as sj:
        banner = sj.drain(quiet=0.4)     # anything the board was still printing
        for line in banner:
            print("<<", line)
        for c in argv[2:]:
            print(">>", c)
            for line in sj.cmd(c):
                print("<<", line)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
