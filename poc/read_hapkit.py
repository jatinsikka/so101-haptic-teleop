#!/usr/bin/env python3
"""Read the Hapkit's "P <xh>" stream and report the measured rate. Milestone 2.

    .venv/bin/python poc/read_hapkit.py --port /dev/cu.usbmodemXXXX [--duration 60]

Parses every line, but prints only at 10 Hz: printing 200 lines/s could slow this loop and distort
the rate being measured. It never sends anything to the Arduino.

USB can deliver lines in bursts, so the worst gap partly reflects USB rather than the firmware. The
average rate is the number to trust. A lost line shows up only as a lower rate.

Exit status: 0 finished, 2 port problem or no valid data.
"""

import argparse
import re
import sys
import time

import serial
from serial.tools import list_ports

BAUD = 115200
TARGET_HZ = 200.0
PRINT_PERIOD_S = 0.1
RATE_WINDOW_S = 1.0
STARTUP_TIMEOUT_S = 5.0  # the Uno bootloader takes a couple of seconds after a reset
POLL_SLEEP_S = 0.001
MAX_LINE_BYTES = 64  # far longer than any valid line

BOOT_LINE = b"# hapkit_gripper"
SAMPLE_LINE = re.compile(rb"P (-?\d+(?:\.\d+)?)")


class HapkitReader:
    """Turns raw serial bytes into position samples without ever blocking.

    Reads can split a line anywhere, so bytes wait in a buffer until their newline arrives. Before
    the first valid line, junk counts as startup noise rather than as an error, because the port
    can open partway through a line or during a reset.
    """

    def __init__(self, ser):
        self.ser = ser
        self.buffer = bytearray()
        self.synced = False
        self.boot_lines = 0
        self.startup_discards = 0
        self.bad_lines = 0

    def poll(self):
        """Return every sample (xh in cm) that has fully arrived since the last call."""
        waiting = self.ser.in_waiting
        if waiting:
            self.buffer += self.ser.read(waiting)

        samples = []
        while True:
            newline = self.buffer.find(b"\n")
            if newline < 0:
                break
            line = bytes(self.buffer[:newline]).rstrip(b"\r")
            del self.buffer[: newline + 1]
            sample = self._parse(line)
            if sample is not None:
                samples.append(sample)

        if len(self.buffer) > MAX_LINE_BYTES:
            self._count_junk()
            self.buffer.clear()
        return samples

    def _parse(self, line):
        if line.startswith(BOOT_LINE):
            self.boot_lines += 1
            self.synced = True
            return None
        match = SAMPLE_LINE.fullmatch(line)
        if match is None:
            self._count_junk()
            return None
        self.synced = True
        return float(match.group(1))

    def _count_junk(self):
        if self.synced:
            self.bad_lines += 1
        else:
            self.startup_discards += 1


class StreamStats:
    """Rate, gaps and range. Takes timestamps so tests can run on a fake clock."""

    def __init__(self):
        self.count = 0
        self.first_time = None
        self.last_time = None
        self.worst_gap = 0.0
        self.minimum = None
        self.maximum = None

    def record(self, now, xh):
        if self.last_time is None:
            self.first_time = now
        else:
            self.worst_gap = max(self.worst_gap, now - self.last_time)
        self.last_time = now
        self.count += 1
        self.minimum = xh if self.minimum is None else min(self.minimum, xh)
        self.maximum = xh if self.maximum is None else max(self.maximum, xh)

    def average_rate(self):
        if self.count < 2 or self.last_time == self.first_time:
            return 0.0
        return (self.count - 1) / (self.last_time - self.first_time)


def wait_for_first_sample(reader, clock=time.perf_counter, sleep=time.sleep,
                          timeout=STARTUP_TIMEOUT_S):
    """Poll until data arrives. Returns (samples, seconds waited); samples is [] on timeout."""
    start = clock()
    while clock() - start < timeout:
        samples = reader.poll()
        if samples:
            return samples, clock() - start
        sleep(POLL_SLEEP_S)
    return [], clock() - start


def stream(reader, first_samples, duration, out=print, clock=time.perf_counter, sleep=time.sleep):
    """Run until Ctrl+C or duration. Returns the StreamStats."""
    start = clock()
    stats = StreamStats()
    for xh in first_samples:
        stats.record(start, xh)

    latest = first_samples[-1]
    window_start = start
    window_count = 0
    window_rate = 0.0
    next_print = start

    try:
        while duration is None or clock() - start < duration:
            samples = reader.poll()
            now = clock()
            for xh in samples:
                stats.record(now, xh)
                latest = xh
            window_count += len(samples)

            if now - window_start >= RATE_WINDOW_S:
                window_rate = window_count / (now - window_start)
                window_start = now
                window_count = 0

            if now >= next_print:
                next_print += PRINT_PERIOD_S
                out(f"t={now - start:6.1f}s  xh={latest:+7.2f} cm  rate={window_rate:6.1f} Hz  "
                    f"bad={reader.bad_lines}")
            sleep(POLL_SLEEP_S)
    except KeyboardInterrupt:
        pass
    return stats


def summary_lines(stats, reader, startup_s):
    lines = [
        f"--- summary: {stats.count} samples, average {stats.average_rate():.1f} Hz "
        f"(target {TARGET_HZ:.0f})",
        f"worst gap: {stats.worst_gap * 1000:.1f} ms | bad lines: {reader.bad_lines} | "
        f"startup discards: {reader.startup_discards}",
        f"xh range: {stats.minimum:+.2f} to {stats.maximum:+.2f} cm "
        f"(span {stats.maximum - stats.minimum:.2f} cm)",
        f"first sample after {startup_s:.2f} s | boot lines seen: {reader.boot_lines}",
    ]
    if reader.boot_lines == 0:
        lines.append("note: no boot line, so opening the port did not reset the Uno during this "
                     "run and the firmware version is unconfirmed")
    elif reader.boot_lines > 1:
        lines.append("WARNING: more than one boot line, so the Uno reset during the run and xh "
                     "re-zeroed")
    return lines


def main(argv=None):
    parser = argparse.ArgumentParser(description="Read the Hapkit position stream.")
    parser.add_argument("--port", required=True, help="Arduino port, e.g. /dev/cu.usbmodemXXXX")
    parser.add_argument("--duration", type=float, default=None,
                        help="seconds to stream (default: until Ctrl+C)")
    args = parser.parse_args(argv)

    try:
        ser = serial.Serial(args.port, BAUD, timeout=0)
    except serial.SerialException as exc:
        print(f"ERROR: cannot open {args.port}: {exc}")
        print("Available ports:", ", ".join(sorted(p.device for p in list_ports.comports())))
        return 2

    try:
        reader = HapkitReader(ser)
        print(f"waiting up to {STARTUP_TIMEOUT_S:.0f} s for data (opening the port may reset the "
              "Uno)")
        first_samples, startup_s = wait_for_first_sample(reader)
        if not first_samples:
            print(f"ERROR: no valid P lines in {STARTUP_TIMEOUT_S:.0f} s "
                  f"({reader.startup_discards} other lines). Is hapkit_gripper uploaded?")
            return 2
        stats = stream(reader, first_samples, args.duration)
        for line in summary_lines(stats, reader, startup_s):
            print(line)
        return 0
    finally:
        ser.close()


if __name__ == "__main__":
    sys.exit(main())
