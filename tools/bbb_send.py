#!/usr/bin/env python3
"""Copy a text file (G-code) onto the BeagleBone over the USB console.

Needs nothing but the mini-USB cable the console already uses: the file
is typed into the board's shell as a here-document, one line at a time
(each line waits for the shell's prompt, so nothing is lost), then checked with
md5sum. Close picocom first: only one program can hold the port.

    python3 tools/bbb_send.py drawing.gcode            # -> /root/drawing.gcode
    python3 tools/bbb_send.py drawing.gcode /root/a.gcode --port /dev/ttyACM1

Python standard library only (Linux/macOS).
"""

import argparse
import hashlib
import os
import re
import select
import sys
import termios
import time

EOF_MARK = "SCARA_EOF_7f3a"


class Port:
    def __init__(self, path):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
        t = termios.tcgetattr(self.fd)
        t[0] = 0                                   # iflag: raw
        t[1] = 0                                   # oflag
        t[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        t[3] = 0                                   # lflag: no echo, raw
        t[4] = t[5] = termios.B115200
        t[6][termios.VMIN], t[6][termios.VTIME] = 0, 0
        termios.tcsetattr(self.fd, termios.TCSANOW, t)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        self.buf = ""

    def send(self, s):
        os.write(self.fd, s.encode())

    def wait(self, pattern, timeout):
        """Read until the regex matches; return the match, drop what's before."""
        end = time.monotonic() + timeout
        while True:
            m = re.search(pattern, self.buf)
            if m:
                self.buf = self.buf[m.end():]
                return m
            left = end - time.monotonic()
            if left <= 0:
                return None
            if select.select([self.fd], [], [], left)[0]:
                self.buf += os.read(self.fd, 4096).decode(errors="replace")

    def drain(self, quiet=0.3):
        while select.select([self.fd], [], [], quiet)[0]:
            self.buf += os.read(self.fd, 4096).decode(errors="replace")
        out, self.buf = self.buf, ""
        return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("file")
    ap.add_argument("dest", nargs="?",
                    help="path on the board (default /root/<file name>)")
    ap.add_argument("--port", default="/dev/ttyACM0")
    a = ap.parse_args()
    dest = a.dest or "/root/" + os.path.basename(a.file)
    if not re.fullmatch(r"[\w./-]+", dest):
        sys.exit("error: keep the board path to letters, digits, _ . / -")

    with open(a.file, encoding="ascii", errors="strict") as fh:
        lines = [l.rstrip("\r\n") for l in fh]
    if any(l.strip() == EOF_MARK for l in lines):
        sys.exit("error: the file contains the end marker " + EOF_MARK)
    if any(len(l) > 1000 for l in lines):
        sys.exit("error: a line is longer than 1000 characters")
    md5 = hashlib.md5(("\n".join(lines) + "\n").encode()).hexdigest()

    try:
        p = Port(a.port)
    except OSError as e:
        sys.exit(f"error: {a.port}: {e.strerror} (board plugged in? "
                 "picocom closed? user in the dialout group?)")

    # Get to a shell prompt: log in if the board asks, else Ctrl-C any
    # half-typed line.
    p.send("\x03\n")
    out = p.drain(0.5)
    if "login:" in out:
        p.send("root\n")
        p.drain(1.0)
    p.send("echo READY_$((40+2))\n")
    if not p.wait(r"READY_42", 5):
        sys.exit(f"error: no shell answering on {a.port}")
    p.drain()

    # Each line is sent once the shell shows its "> " prompt for the next
    # one, so the board's input buffer never overflows; md5sum below
    # catches anything mangled on the way.
    p.send(f"cat > {dest} <<'{EOF_MARK}'\n")
    for n, line in enumerate(lines, 1):
        if not p.wait(r"\n> ", 5):
            sys.exit(f"error: no prompt before line {n}; the file on the "
                     "board is incomplete")
        p.send(line + "\n")
        if n % 50 == 0 or n == len(lines):
            print(f"\r{n}/{len(lines)} lines", end="", file=sys.stderr)
    print(file=sys.stderr)
    p.wait(r"\n> ", 5)
    p.send(EOF_MARK + "\n")
    p.drain()

    p.send(f"md5sum {dest}\n")
    m = p.wait(r"([0-9a-f]{32})\s+" + re.escape(dest), 10)
    if not m or m.group(1) != md5:
        sys.exit(f"error: {dest} on the board does not match (md5 "
                 f"{m.group(1) if m else '?'} instead of {md5})")
    print(f"ok: {a.file} -> {dest} ({len(lines)} lines, md5 checked)")


if __name__ == "__main__":
    main()
