#!/usr/bin/env python3
"""Fake Mega running ArduinoSlave.ino on a pseudo-terminal, for testing the scripts without the rig.
Prints the pty path; pass it with --port. Counts steps from the timer value over real time."""
import os
import pty
import struct
import sys
import time
import tty

master, slave = pty.openpty()
tty.setraw(slave)
print(os.ttyname(slave), flush=True)
buf = bytearray()
pos = 0.0
rate = 0.0
enabled = True
last = time.monotonic()
log = open(sys.argv[1], "w") if len(sys.argv) > 1 else None
while True:
    data = os.read(master, 64)
    buf.extend(data)
    while len(buf) >= 4:
        n = struct.unpack("<i", bytes(buf[:4]))[0]
        del buf[:4]
        now = time.monotonic()
        pos += rate * (now - last)
        last = now
        if n == -1:
            enabled, rate = False, 0.0
        elif enabled and n != 0:
            f = 125000.0 / (abs(n) + 1)
            rate = -f if n > 0 else f     # timer > 0 -> direction -1
        p = int(round(pos))
        if log:
            log.write(f"{now:.4f},{n},{p}\n")
        os.write(master, struct.pack(">h", ((p + 32768) & 0xFFFF) - 32768))
