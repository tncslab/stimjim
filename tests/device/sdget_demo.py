"""Fetch the newest SD log from a stimjimAWG over USB serial and split it into sessions.

    python sdget_demo.py COM4
    python sdget_demo.py /dev/ttyACM0 --save logs/ --write chunk
    python sdget_demo.py /dev/ttyACM0 --set-clock

Which file is the newest: the open log if there is one (`LOG?`), otherwise the
highest-numbered `LOGnnnn.CSV` on the card (`SDLIST`). The firmware names a new log
after the *lowest free* index, so once a file has been deleted the next log fills
the gap and the highest index is no longer the newest; the script says so when it
sees a gap.

Sessions: a session is one open-to-close of a log file, and every session starts
with the header line `# stimjimAWG log — fw=…`. An automatically named file holds
exactly one: the box opens a new `LOGnnnn.CSV` at every boot and on every bare
`LOG1`. Only `LOG1,<name>` naming a file that already exists appends a second
session to it, because the firmware opens logs in append mode. The script splits on
the header either way, so both cases come out as a list.

The transfer is `SDGET` in chunks, each framed by a byte count and checked by its
CRC-32 (protocol §4 "The `SD` group"). The firmware serves a whole chunk inside one
`loop()` pass, so no other line can land inside a reply, but the waveform players
keep running from their interrupts meanwhile: a trigger edge still starts a train,
and whatever that train prints (completion, `MSUM`, `MDATA`, ring overflow) waits
and arrives before the next reply. The script collects every such line and warns,
because it means (a) the file grew after the snapshot was taken and (b) a long
chunk kept `loop()` from draining the `MDATA` ring, which can drop records.

Clock: `set_clock` gives the board this host's UTC time before a measurement, so
the log's `# clock:` anchors read `src=host` rather than a compile time. It is
meant to be called by the Pi that runs the session, but uses only `time.time_ns()`
and the serial port, so any host works; on Linux it also warns when `timedatectl`
says the host clock itself is not NTP-synchronized.
"""

import argparse
import os
import re
import subprocess
import sys
import time
import warnings
import zlib

from sjcon import StimJim

SESSION_HEADER = "# stimjimAWG log"     # first line SdLog's writeHeader() puts down
LOG_NAME = re.compile(r"^LOG(\d{4})\.CSV$", re.IGNORECASE)
CHUNK = 64 * 1024                       # bytes per SDGET; each chunk carries its own CRC
RETRIES = 3


class RetrievalWarning(UserWarning):
    pass


def warn(msg):
    warnings.warn(msg, RetrievalWarning, stacklevel=2)


def read_line(ser, deadline):
    """One line without its CR/LF, or raise TimeoutError."""
    buf = b""
    while not buf.endswith(b"\n"):
        if time.time() > deadline:
            raise TimeoutError(f"no line terminator, got {buf!r}")
        buf += ser.read_until(b"\n")
    return buf.rstrip(b"\r\n").decode("utf-8", "replace")


def read_exact(ser, n, deadline):
    buf = bytearray()
    while len(buf) < n:
        if time.time() > deadline:
            raise TimeoutError(f"payload stopped at {len(buf)} of {n} bytes")
        buf += ser.read(n - len(buf))
    return bytes(buf)


def sdget(ser, name, offset, length, stray):
    """One framed SDGET chunk -> (payload, total file size).

    Lines that are not part of the reply go to `stray`, which the caller owns so
    that they survive a chunk that fails and is retried.

    `length` 0 asks for the rest of the file; an `offset` past the end returns an
    empty payload with the current size, which is how the size is probed.
    """
    ser.write(f"SDGET,{name},{offset},{length}\n".encode())
    ser.flush()
    # Generous: USB CDC moves several hundred kB/s, the card read is not slower.
    deadline = time.time() + 5.0 + (length or CHUNK) / 50e3
    while True:                                     # lines ahead of the header are not ours
        line = read_line(ser, deadline)
        if line.startswith("SDGET,"):
            break
        if line.startswith("ERR"):
            raise RuntimeError(line)
        stray.append(line)
    # rsplit: the name is the only field that could itself contain a comma
    _, sent_off, sent_len, total = line.rsplit(",", 3)
    sent_off, sent_len, total = int(sent_off), int(sent_len), int(total)
    assert sent_off == min(offset, total), line
    payload = read_exact(ser, sent_len, deadline)
    assert read_line(ser, deadline) == "", "missing newline after payload"
    crc = None
    while True:                                     # trailer: '# crc32=…', maybe a short-read note, 'OK'
        line = read_line(ser, deadline)
        if line == "OK":
            break
        if line.startswith("# crc32="):
            crc = int(line[8:], 16)
        elif line.startswith("# SDGET: short read"):
            raise IOError(line)
        else:
            stray.append(line)
    if crc != zlib.crc32(payload):                  # zlib's CRC-32 is CRC-32/ISO-HDLC
        raise IOError(f"CRC mismatch at offset {offset}")
    return payload, total


def clk_exchange(ser, line):
    """Send one CLK command -> (reply fields, host ns at write, host ns at reply).

    Times the exchange itself rather than going through `StimJim.cmd`, whose
    silence wait would add 0.3 s to every round trip. Lines ahead of the record
    (a train finishing) are dropped; the one `#` line after it is consumed.
    """
    deadline = time.time() + 2.0
    t_write = time.time_ns()
    ser.write((line + "\n").encode())
    ser.flush()
    while True:
        reply = read_line(ser, deadline)
        if reply.startswith("CLK,"):
            break
        if reply.startswith("ERR"):
            raise RuntimeError(f"{line} -> {reply}")
    t_reply = time.time_ns()
    assert read_line(ser, deadline).startswith("#"), "missing '# clock:' line"
    _, secs, ms, us, src = reply.split(",")
    return (int(secs), int(ms), int(us), src), t_write, t_reply


def host_clock_synced():
    """True/False when the OS can say whether its clock is NTP-disciplined, else None.

    A Raspberry Pi has no RTC: until NTP answers, it runs on fake-hwclock, i.e.
    the time it was last shut down, and handing that to the board is worse than
    leaving it at `build`, because the source would then read `host`.
    """
    try:
        r = subprocess.run(["timedatectl", "show", "-p", "NTPSynchronized", "--value"],
                           capture_output=True, text=True, timeout=2.0)
    except (OSError, subprocess.TimeoutExpired):
        return None                                 # not systemd (Windows, macOS, ...)
    return {"yes": True, "no": False}.get(r.stdout.strip())


def set_clock(sj, probes=5):
    """Set the board's RTC to this host's UTC time and check it -> dict.

    The firmware sets the clock when it parses the line, about half a round trip
    after the write, so the value sent is the host time at the write plus half of
    the shortest of `probes` `CLK?` round trips. The read-back offset is
    board time minus the midpoint of the exchange, taken from the fastest of
    `probes` reads; its uncertainty is half that read's round trip plus the RTC's
    millisecond display step. `time.time_ns()` is UTC on every OS, so this works
    the same from a Pi, a Linux PC or Windows.

    Returns offset_ms, rtt_ms, src, and the (unix_ms, us_since_boot) anchor pair.
    """
    ser = sj.ser
    if host_clock_synced() is False:
        warn("host clock is not NTP-synchronized (timedatectl); the board will get "
             "whatever this host believes, labelled src=host")
    stray = sj.drain(quiet=0.1)                     # nothing may sit ahead of the timed replies
    if stray:
        warn(f"board was printing before CLK: {stray[:3]}")

    rtt = min(b - a for _, a, b in (clk_exchange(ser, "CLK?") for _ in range(probes)))
    target = time.time_ns() + rtt // 2              # when the board will parse what we send
    secs, rem = divmod(target, 1_000_000_000)
    (_, _, _, src), _, _ = clk_exchange(ser, f"CLK,{secs},{rem // 1_000_000}")
    if src != "host":
        raise RuntimeError(f"CLK accepted but src={src}, expected host")

    best = None
    for _ in range(probes):
        (secs, ms, us, src), a, b = clk_exchange(ser, "CLK?")
        if best is None or b - a < best[1]:
            best = (secs * 1000 + ms - (a + b) / 2e6, b - a, secs * 1000 + ms, us)
    offset_ms, rtt_ns, unix_ms, us = best
    return {"offset_ms": offset_ms, "rtt_ms": rtt_ns / 1e6, "src": src,
            "anchor": (unix_ms, us)}


def latest_log(sj):
    """Name of the open log, else of the highest-numbered LOGnnnn.CSV, and whether it is open."""
    status = sj.cmd1("LOG?")                        # LOG,<open>,<name>,<bytes>
    if status.startswith("ERR"):
        raise RuntimeError(status)
    _, is_open, name, _ = status.split(",", 3)
    if is_open == "1":
        return name, True
    listing = sj.cmd("SDLIST")
    if not listing or listing[-1] != "OK":
        raise RuntimeError(f"SDLIST failed: {listing}")
    numbers = {}
    for line in listing:
        if line.startswith("SDLIST,"):
            fname = line[len("SDLIST,"):].rsplit(",", 1)[0]
            m = LOG_NAME.match(fname)
            if m:
                numbers[int(m.group(1))] = fname
    if not numbers:
        raise FileNotFoundError("no LOGnnnn.CSV on the card")
    top = max(numbers)
    if len(numbers) != top + 1:
        warn(f"LOG indices have gaps below {top}: the firmware reuses the lowest free "
             f"index, so {numbers[top]} may not be the newest file")
    return numbers[top], False


def fetch(sj, name, on_chunk=None):
    """The whole file as bytes, as it stood when the first chunk was served.

    `on_chunk(bytes)` is called with each piece once its CRC has passed, in file order.
    """
    ser = sj.ser
    before = sj.drain(quiet=0.3)                    # what the board was printing before we asked
    if before:
        warn(f"board was printing before retrieval (train running?): {before[:3]}")
    data, stray, target, last_total = bytearray(), [], None, None
    while target is None or len(data) < target:
        for attempt in range(RETRIES):
            try:
                chunk, last_total = sdget(ser, name, len(data), CHUNK, stray)
                break
            except (IOError, TimeoutError, AssertionError) as e:
                warn(f"chunk at {len(data)} failed ({e}), retry {attempt + 1}/{RETRIES}")
                # Whatever is left of the broken reply is payload junk; lines already
                # read are in `stray`.
                sj.drain(quiet=0.5)
        else:
            raise IOError(f"giving up on {name} at offset {len(data)}")
        if target is None:
            target = last_total                     # the first header fixes the snapshot
        piece = chunk[:target - len(data)]          # later chunks may run past it if the file grew
        data += piece
        if on_chunk is not None:
            on_chunk(piece)
        if not chunk:
            break
    # Output queued behind the last chunk arrives now; then probe the size once more.
    stray += sj.drain(quiet=0.5)
    _, size_after = sdget(ser, name, 1 << 40, 0, stray)
    stray += sj.drain(quiet=0.2)

    if stray:
        overflow = [x for x in stray if "ring overflowed" in x]
        warn(f"{len(stray)} line(s) arrived during retrieval, so trains ran meanwhile; "
             f"first: {stray[:3]}" + (f"; RECORDS WERE DROPPED: {overflow}" if overflow else ""))
    if size_after != target:
        warn(f"{name} grew from {target} to {size_after} bytes during retrieval; "
             f"the returned content ends at byte {target}")
    return bytes(data), stray


class SafeWriter:
    """Write to `<path>.part`, then rename it to `<path>` once the transfer is complete.

    A reader of `<path>` never sees a half-written file. A failed transfer leaves
    `.part` behind with everything that passed its CRC.

    The `fsync` calls are commented out. They would make the file survive a power cut
    (Linux holds written data in memory for up to ~30 s before it reaches the card),
    but each one blocks until the card has taken the data and, on ext4, can force a
    journal commit that stalls every other process writing to the same card. On a Pi
    that also coordinates other hardware from that card, that is a pause at an
    unknown moment in someone else's timing. The stimjim itself is unaffected either
    way: the calls fall between chunks, when the board sends nothing. Uncomment them
    when losing the last half-minute of a download to a power cut matters more.
    """

    def __init__(self, path):
        self.path, self.part = path, path + ".part"
        self.f = open(self.part, "wb")

    def write(self, data):
        self.f.write(data)
        self.f.flush()                              # to the OS; the card gets it at writeback
        # os.fsync(self.f.fileno())

    def commit(self):
        self.f.close()
        os.replace(self.part, self.path)            # atomic on POSIX and on Windows (same volume)
        # if os.name == "posix":                    # Windows cannot open a directory for fsync
        #     d = os.open(os.path.dirname(os.path.abspath(self.path)), os.O_RDONLY)
        #     os.fsync(d)                           # makes the rename itself durable
        #     os.close(d)

    def abort(self):
        self.f.close()
        warn(f"transfer failed: partial data kept in {self.part}")


def split_sessions(text):
    """Split a log's text at every session header; each piece starts with its header."""
    starts = [m.start() for m in re.finditer(f"^{re.escape(SESSION_HEADER)}", text, re.MULTILINE)]
    if not starts or starts[0] != 0:
        warn("file does not start with a session header")
        starts = [0] + starts
    return [text[a:b] for a, b in zip(starts, starts[1:] + [len(text)])]


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("port", nargs="?", default="COM4", help="COM4, /dev/ttyACM0, ...")
    ap.add_argument("--save", metavar="PATH",
                    help="write the file here; an existing directory gets the log's own name. "
                         "An existing file is replaced, so polling the active log keeps the "
                         "newest snapshot")
    ap.add_argument("--write", choices=("end", "chunk"), default="end",
                    help="end: write once the whole file has arrived; chunk: write each "
                         "chunk as soon as its CRC passes, so a failed transfer keeps "
                         "what arrived (default: end)")
    ap.add_argument("--set-clock", action="store_true",
                    help="set the board's clock to this host's UTC time first (set_clock); "
                         "a Pi does this before a measurement so the log's '# clock:' "
                         "anchors read src=host")
    args = ap.parse_args(argv)

    with StimJim(args.port) as sj:
        if args.set_clock:
            c = set_clock(sj)
            print(f"clock set: offset {c['offset_ms']:+.1f} ms, round trip "
                  f"{c['rtt_ms']:.2f} ms, src={c['src']}, anchor unix_ms={c['anchor'][0]} "
                  f"us_since_boot={c['anchor'][1]}")
        name, is_open = latest_log(sj)
        path = None
        if args.save:
            path = os.path.join(args.save, name) if os.path.isdir(args.save) else args.save
        writer = SafeWriter(path) if path and args.write == "chunk" else None
        try:
            raw, stray = fetch(sj, name, on_chunk=writer.write if writer else None)
        except BaseException:
            if writer:
                writer.abort()
            raise
    if path:
        if writer is None:
            writer = SafeWriter(path)
            writer.write(raw)
        writer.commit()
        print(f"saved {len(raw)} bytes to {path} ({args.write} mode)")

    text = raw.decode("utf-8")                      # the header carries a UTF-8 em dash
    sessions = split_sessions(text)
    print(f"{name}: {len(raw)} bytes, {'open (active)' if is_open else 'closed'}, "
          f"{len(sessions)} session(s)")
    for i, s in enumerate(sessions):
        lines = s.splitlines()
        rows = sum(1 for x in lines if x and not x.startswith("#"))
        trains = sum(1 for x in lines if x.startswith("# done:") or x.startswith("# stop:"))
        clock = next((x for x in lines if x.startswith("# clock:")), "# clock: none")
        print(f"  session {i}: {len(lines)} lines, {rows} data rows, {trains} train(s), {clock}")
    if not is_open and not text.endswith("\n"):
        # A closed file ending mid-line was never closed: power went while it was open,
        # and whatever was written after the last flush (<= 1 s or 64 rows) is gone.
        warn(f"{name} ends mid-line: its session was cut off by a power loss")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
