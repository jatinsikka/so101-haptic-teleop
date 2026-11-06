#!/usr/bin/env python3
"""Read-only probe of the SO-101 follower gripper (STS3215, ID 6, 1 Mbps). Milestone 1.

    .venv/bin/python poc/probe_gripper.py --port /dev/cu.usbmodemXXXX [--duration 10]

What it does:
  1. Pings ID 6 and stops unless the model is 777 (STS3215). Other Feetech series lay out their
     registers differently, so these addresses are only meaningful after this check.
  2. Prints the torque and protection registers once.
  3. If torque is on, reports it and exits with status 3. It has no way to turn torque off.
  4. Otherwise streams position, load and temperature at 50 Hz until Ctrl+C or --duration, then
     prints a summary. The position range is what milestone 3 needs, so move the gripper by hand
     through its full travel during the run.

It cannot write: all traffic goes through ReadOnlyPortHandler (see servo_safety.py).

Exit status: 0 finished, 2 bus or model problem, 3 torque is on.

Register sources:
  Addresses, areas, sign bits: LeRobot v0.6.1, src/lerobot/motors/feetech/tables.py
    (STS_SMS_SERIES_CONTROL_TABLE and STS_SMS_SERIES_ENCODINGS_TABLE).
  Also listed by Feetech's FTServo_Arduino src/SMS_STS.h: 3, 5, 9, 11, 31, 33, 40, 48, 55, 56, 58,
    60, 62, 63. The rest appear only in the LeRobot table.
  Voltage in 0.1 V: Feetech STS3215 product sheet.
  Status bit names: third-party copy of the memory table in the python-st3215 README. Not confirmed
    by Feetech, so the raw value is always printed beside them.
"""

import argparse
import sys
import time

from scservo_sdk import COMM_SUCCESS, PacketHandler
from serial import SerialException
from serial.tools import list_ports

from servo_safety import ReadOnlyPortHandler

SERVO_ID = 6
BAUD = 1_000_000
EXPECTED_MODEL = 777  # LeRobot MODEL_NUMBER_TABLE["sts3215"]

EXIT_OK = 0
EXIT_BUS_ERROR = 2
EXIT_TORQUE_ON = 3

RATE_HZ = 50.0
SLOW_CHECK_PERIOD_S = 1.0
MAX_CONSECUTIVE_MISSES = 50  # give up after this many failed reads in a row
MAX_SLOW_CHECK_FAILURES = 3  # never keep streaming while torque state is unknown
STARTUP_READ_ATTEMPTS = 2

TORQUE_ENABLE = 40
STATUS = 65

# Contiguous block read once per tick. Registers 56-63 have no gaps between them.
TELEMETRY_ADDR = 56
TELEMETRY_LENGTH = 8

# (address, size in bytes, name, sign bit or None) -- printed once at startup.
STARTUP_REGISTERS = (
    (0, 1, "Firmware_Major_Version", None),
    (1, 1, "Firmware_Minor_Version", None),
    (5, 1, "ID", None),
    (7, 1, "Return_Delay_Time", None),
    (9, 2, "Min_Position_Limit", None),
    (11, 2, "Max_Position_Limit", None),
    (13, 1, "Max_Temperature_Limit", None),
    (14, 1, "Max_Voltage_Limit", None),
    (15, 1, "Min_Voltage_Limit", None),
    (16, 2, "Max_Torque_Limit", None),
    (19, 1, "Unloading_Condition", None),
    (20, 1, "LED_Alarm_Condition", None),
    (21, 1, "P_Coefficient", None),
    (22, 1, "D_Coefficient", None),
    (28, 2, "Protection_Current", None),
    (31, 2, "Homing_Offset", 11),
    (33, 1, "Operating_Mode", None),
    (34, 1, "Protective_Torque", None),
    (35, 1, "Protection_Time", None),
    (36, 1, "Overload_Torque", None),
    (38, 1, "Over_Current_Protection_Time", None),
    (40, 1, "Torque_Enable", None),
    (48, 2, "Torque_Limit", None),
    (55, 1, "Lock", None),
    (65, 1, "Status", None),
)

STATUS_BITS = ("voltage", "sensor", "temperature", "current", "angle", "overload")


def sign_magnitude(raw, sign_bit):
    """Decode an STS sign-magnitude value: bit `sign_bit` is the sign, lower bits the size."""
    magnitude = raw & ((1 << sign_bit) - 1)
    return -magnitude if raw & (1 << sign_bit) else magnitude


def little_endian(data):
    """Combine register bytes. STS servos store the low byte first (SDK protocol_end = 0)."""
    return sum(byte << (8 * i) for i, byte in enumerate(data))


def describe_status(value):
    names = [name for bit, name in enumerate(STATUS_BITS) if value & (1 << bit)]
    return ", ".join(names) if names else "ok"


def decode_telemetry(data):
    """Decode the 8-byte block starting at register 56."""
    return {
        "position": sign_magnitude(little_endian(data[0:2]), 15),
        "speed": sign_magnitude(little_endian(data[2:4]), 15),
        "load": sign_magnitude(little_endian(data[4:6]), 10),
        "volts": data[6] / 10.0,
        "temperature": data[7],
    }


def read_bytes(bus, port, addr, length):
    """Return the bytes read, or None if the reply was missing or the wrong size.

    A failed read clears the input buffer. The SDK does not, so a late reply would otherwise be
    taken as the answer to the next request, which could misreport another register.
    """
    data, comm, _error = bus.readTxRx(port, SERVO_ID, addr, length)
    if comm != COMM_SUCCESS or len(data) != length:
        port.ser.reset_input_buffer()
        return None
    return data


def read_startup_register(bus, port, addr, size):
    for _ in range(STARTUP_READ_ATTEMPTS):
        data = read_bytes(bus, port, addr, size)
        if data is not None:
            return data
    return None


class Stats:
    """Run totals for the end-of-run summary. Takes timestamps so it never reads the clock."""

    def __init__(self, start):
        self.start = start
        self.last_tick = None
        self.worst_gap = 0.0
        self.reads = 0
        self.misses = 0
        self.resyncs = 0
        self.position = None
        self.load = None
        self.max_temperature = None

    def tick(self, now):
        if self.last_tick is not None:
            self.worst_gap = max(self.worst_gap, now - self.last_tick)
        self.last_tick = now

    def record(self, sample):
        self.reads += 1
        position, load = sample["position"], sample["load"]
        self.position = (
            (position, position)
            if self.position is None
            else (min(self.position[0], position), max(self.position[1], position))
        )
        self.load = (
            (load, load)
            if self.load is None
            else (min(self.load[0], load), max(self.load[1], load))
        )
        if self.max_temperature is None or sample["temperature"] > self.max_temperature:
            self.max_temperature = sample["temperature"]

    def summary(self, now):
        elapsed = max(now - self.start, 1e-9)
        lines = [
            f"--- summary: {elapsed:.2f} s, {self.reads} reads, {self.reads / elapsed:.1f} Hz "
            f"(target {RATE_HZ:.0f})",
            f"worst tick gap: {self.worst_gap * 1000:.1f} ms | misses: {self.misses} | "
            f"schedule resyncs: {self.resyncs}",
        ]
        if self.position is None:
            lines.append("no telemetry received")
        else:
            lines.append(
                f"position range: {self.position[0]} to {self.position[1]} ticks | "
                f"load range: {self.load[0]:+d} to {self.load[1]:+d} raw | "
                f"max temperature: {self.max_temperature} C"
            )
        return "\n".join(lines)


def run_startup_checks(bus, port, out):
    """Check the model, print the registers and check torque. Returns an exit status."""
    model, comm, _error = bus.ping(port, SERVO_ID)
    if comm != COMM_SUCCESS:
        out(f"ERROR: no reply from servo ID {SERVO_ID} ({bus.getTxRxResult(comm)}). "
            "Check the bus wiring and that the follower's supply is on.")
        return EXIT_BUS_ERROR
    if model != EXPECTED_MODEL:
        out(f"ERROR: ID {SERVO_ID} reports model {model}, expected {EXPECTED_MODEL} (STS3215). "
            "Stopping: these register addresses may not apply to that model.")
        return EXIT_BUS_ERROR
    out(f"servo ID {SERVO_ID}: model {model} (STS3215)")

    out("addr  name                           value")
    torque = None
    for addr, size, name, sign_bit in STARTUP_REGISTERS:
        data = read_startup_register(bus, port, addr, size)
        if data is None:
            out(f"ERROR: could not read register {addr} ({name}).")
            return EXIT_BUS_ERROR
        raw = little_endian(data)
        value = raw if sign_bit is None else sign_magnitude(raw, sign_bit)
        note = f"  ({describe_status(raw)})" if addr == STATUS else ""
        out(f"{addr:>4}  {name:<30} {value}{note}")
        if addr == TORQUE_ENABLE:
            torque = raw

    if torque != 0:
        out(f"TORQUE IS ON (register {TORQUE_ENABLE} = {torque}). The servo is holding position, "
            "so do not back-drive the gripper by hand.")
        out("Probe stopped. It will not turn torque off; that needs your approval as a separate "
            "step.")
        return EXIT_TORQUE_ON
    return EXIT_OK


def stream(bus, port, duration, out, slow_period=SLOW_CHECK_PERIOD_S):
    """Print telemetry at RATE_HZ, with torque and status checks every slow_period seconds."""
    period = 1.0 / RATE_HZ
    start = time.perf_counter()
    stats = Stats(start)
    next_tick = start
    next_slow_check = start + slow_period
    consecutive_misses = 0
    slow_failures = 0
    last_status = 0
    result = EXIT_OK

    out("time_s   position  load  temp_C  speed  volts   (load is raw drive duty, not force)")
    try:
        while duration is None or time.perf_counter() - start < duration:
            now = time.perf_counter()
            stats.tick(now)

            data = read_bytes(bus, port, TELEMETRY_ADDR, TELEMETRY_LENGTH)
            if data is None:
                stats.misses += 1
                consecutive_misses += 1
                if consecutive_misses >= MAX_CONSECUTIVE_MISSES:
                    out(f"ERROR: {consecutive_misses} failed reads in a row. Stopping.")
                    result = EXIT_BUS_ERROR
                    break
            else:
                consecutive_misses = 0
                sample = decode_telemetry(data)
                stats.record(sample)
                out(f"{now - start:6.2f}  {sample['position']:>8}  {sample['load']:>+4}  "
                    f"{sample['temperature']:>6}  {sample['speed']:>+5}  {sample['volts']:>5.1f}")

            if now >= next_slow_check:
                next_slow_check += slow_period
                torque = read_bytes(bus, port, TORQUE_ENABLE, 1)
                status = read_bytes(bus, port, STATUS, 1)
                if torque is None or status is None:
                    stats.misses += 1
                    slow_failures += 1
                    if slow_failures >= MAX_SLOW_CHECK_FAILURES:
                        out(f"ERROR: torque state unknown after {slow_failures} failed checks. "
                            "Stopping.")
                        result = EXIT_BUS_ERROR
                        break
                else:
                    slow_failures = 0
                    if torque[0] != 0:
                        out(f"TORQUE TURNED ON during the run (register {TORQUE_ENABLE} = "
                            f"{torque[0]}). Another program may be using the bus. Stopping.")
                        result = EXIT_TORQUE_ON
                        break
                    if status[0] != last_status:
                        out(f"status changed: {status[0]} ({describe_status(status[0])})")
                        last_status = status[0]

            # Absolute deadlines: a late tick does not delay the ones after it. After a stall of
            # more than one period, restart the schedule instead of firing a burst of reads.
            next_tick += period
            remaining = next_tick - time.perf_counter()
            if remaining > 0:
                time.sleep(remaining)
            elif remaining < -period:
                next_tick = time.perf_counter()
                stats.resyncs += 1
    except KeyboardInterrupt:
        pass

    out(stats.summary(time.perf_counter()))
    return result


def probe(bus, port, duration, out=print, slow_period=SLOW_CHECK_PERIOD_S):
    result = run_startup_checks(bus, port, out)
    if result != EXIT_OK:
        return result
    return stream(bus, port, duration, out, slow_period)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Read-only probe of the SO-101 gripper servo.")
    parser.add_argument("--port", required=True, help="servo bus port, e.g. /dev/cu.usbmodemXXXX")
    parser.add_argument("--duration", type=float, default=None,
                        help="seconds to stream (default: until Ctrl+C)")
    args = parser.parse_args(argv)

    port = ReadOnlyPortHandler(args.port, allowed_id=SERVO_ID)
    try:
        opened = port.setBaudRate(BAUD)  # opens the port; openPort() would just open it twice
    except SerialException as exc:
        print(f"ERROR: cannot open {args.port}: {exc}")
        opened = False
    if not opened:
        print("Available ports:", ", ".join(sorted(p.device for p in list_ports.comports())))
        return EXIT_BUS_ERROR

    try:
        return probe(PacketHandler(0), port, args.duration)
    finally:
        port.closePort()


if __name__ == "__main__":
    sys.exit(main())
