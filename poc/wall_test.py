#!/usr/bin/env python3
"""Check the Hapkit wall on its own, with the gripper out of the loop. Milestone 4.

    .venv/bin/python poc/wall_test.py --port /dev/cu.usbmodemXXXX --k 30

Part 1 holds a wall just right of the handle, which should push the handle to the right. If the
handle moves left instead, the force is backwards, and the test stops at once.
Part 2 sends one W and then goes silent. The firmware watchdog should remove the push within 250 ms.

Exit status: 0 finished, 2 port or data problem, 3 force pushed the wrong way.
"""

import argparse
import statistics
import sys
import time

import serial
from serial.tools import list_ports

from contact_wall import MAX_WALL_K, WALL_SEND_PERIOD_S
from read_hapkit import BAUD, HapkitReader, wait_for_first_sample

EXIT_OK = 0
EXIT_ERROR = 2
EXIT_WRONG_DIRECTION = 3

WALL_OFFSET_CM = 1.5
HOLD_S = 3.0
WATCHDOG_WATCH_S = 1.0
WRONG_WAY_CM = 0.5
SETTLE_WINDOW_S = 0.25
PRINT_PERIOD_S = 0.2


def resting_position(reader, clock=time.perf_counter, sleep=time.sleep):
    """Return the handle's position after a pause at a prompt, or None if data is missing."""
    if not reader.discard_pending(clock, sleep):
        return None
    samples = []
    start = clock()
    while clock() - start < SETTLE_WINDOW_S:
        samples += reader.poll()
        sleep(0.001)
    return statistics.median(samples) if samples else None


def hold_wall(ser, reader, x0, k, out, clock=time.perf_counter, sleep=time.sleep,
              duration=HOLD_S):
    """Keep a wall at x0 + WALL_OFFSET_CM. Returns (furthest left, furthest right) of the handle.

    Stops early, returning at once, if the handle moves left past WRONG_WAY_CM.
    """
    x_c = x0 + WALL_OFFSET_CM
    start = clock()
    next_send = start
    next_print = start
    latest = lowest = highest = x0
    try:
        while clock() - start < duration:
            now = clock()
            if now >= next_send:
                next_send = now + WALL_SEND_PERIOD_S
                ser.write(f"W {x_c:.2f} {k:.1f}\n".encode())
            samples = reader.poll()
            if samples:
                latest = samples[-1]
                lowest = min(lowest, min(samples))
                highest = max(highest, max(samples))
            if now >= next_print:
                next_print = now + PRINT_PERIOD_S
                out(f"  t={now - start:4.1f}s  xh={latest:+6.2f} cm  moved {latest - x0:+5.2f} cm")
            if lowest < x0 - WRONG_WAY_CM:
                break
            sleep(0.001)
    finally:
        ser.write(b"R\n")
    return lowest, highest


def watch_watchdog(ser, reader, k, out, clock=time.perf_counter, sleep=time.sleep,
                   duration=WATCHDOG_WATCH_S):
    """Send a single W and then nothing, so that only the firmware's watchdog can remove it."""
    x0 = resting_position(reader, clock, sleep)
    if x0 is None:
        return None
    x_c = x0 + WALL_OFFSET_CM
    ser.write(f"W {x_c:.2f} {k:.1f}\n".encode())
    start = clock()
    next_print = start
    latest = x0
    try:
        while clock() - start < duration:
            samples = reader.poll()
            if samples:
                latest = samples[-1]
            now = clock()
            if now >= next_print:
                next_print = now + 0.1
                out(f"  t={now - start:4.2f}s  xh={latest:+6.2f} cm")
            sleep(0.001)
    finally:
        ser.write(b"R\n")
    return latest - x0


def run_test(ser, k, hold_s=HOLD_S, out=print, wait_for_enter=input, watchdog_only=False):
    reader = HapkitReader(ser)
    first_samples, _waited = wait_for_first_sample(reader)
    if not first_samples:
        out("ERROR: no P lines. Is hapkit_gripper uploaded?")
        return EXIT_ERROR
    if reader.firmware and b"m4" not in reader.firmware:
        out(f"ERROR: this needs the m4 firmware, but the Uno reports {reader.firmware.decode()!r}.")
        return EXIT_ERROR

    if watchdog_only:
        return run_watchdog_part(ser, reader, k, out, wait_for_enter)

    wait_for_enter(f"PART 1. Center the handle and rest a finger lightly on it. k = {k}. Press Enter: ")
    x0 = resting_position(reader)
    if x0 is None:
        out("ERROR: lost the handle data.")
        return EXIT_ERROR
    out(f"wall at {x0 + WALL_OFFSET_CM:+.2f} cm. It should push the handle RIGHT.")
    lowest, highest = hold_wall(ser, reader, x0, k, out, duration=hold_s)

    if lowest < x0 - WRONG_WAY_CM:
        out(f"STOP: the handle moved LEFT by {x0 - lowest:.2f} cm, and the wall is off. If your finger "
            "did not move it, the force is backwards. Do not raise k.")
        return EXIT_WRONG_DIRECTION
    out(f"handle moved right by up to {highest - x0:.2f} cm, wall off.")
    if highest - x0 < 0.1:
        out("It did not move. That may be friction at low k: did you feel a push to the right?")

    return run_watchdog_part(ser, reader, k, out, wait_for_enter)


def run_watchdog_part(ser, reader, k, out, wait_for_enter):
    wait_for_enter("WATCHDOG CHECK. Rest a finger lightly on the handle. Press Enter: ")
    moved = watch_watchdog(ser, reader, k, out)
    if moved is None:
        out("ERROR: lost the handle data.")
        return EXIT_ERROR
    out("One W sent, then silence. You should have felt a push lasting only about 0.25 s.")
    if reader.isr_max_us is not None:
        out(f"longest ISR time in the last report: {reader.isr_max_us} us (period is 1000 us)")
    out(f"bad lines: {reader.bad_lines}")
    return EXIT_OK


def wall_stiffness(text):
    value = float(text)
    if not 0 < value <= MAX_WALL_K:
        raise argparse.ArgumentTypeError(f"must be above 0 and at most {MAX_WALL_K:.0f}")
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description="Check the Hapkit wall on its own.")
    parser.add_argument("--port", required=True, help="Arduino port")
    parser.add_argument("--k", type=wall_stiffness, default=30.0,
                        help="wall stiffness in PWM per cm (default 30)")
    parser.add_argument("--hold-s", type=float, default=HOLD_S,
                        help=f"seconds to hold the wall in part 1 (default {HOLD_S:.0f})")
    parser.add_argument("--watchdog-only", action="store_true",
                        help="skip the held wall and run only the watchdog check")
    args = parser.parse_args(argv)

    try:
        ser = serial.Serial(args.port, BAUD, timeout=0, write_timeout=0.05)
    except serial.SerialException as exc:
        print(f"ERROR: cannot open {args.port}: {exc}")
        print("Available ports:", ", ".join(sorted(p.device for p in list_ports.comports())))
        return EXIT_ERROR

    try:
        return run_test(ser, args.k, args.hold_s, watchdog_only=args.watchdog_only)
    except KeyboardInterrupt:
        print("stopped")
        return EXIT_OK
    finally:
        try:
            ser.write(b"R\n")
        except serial.SerialException:
            pass
        ser.close()


if __name__ == "__main__":
    sys.exit(main())
