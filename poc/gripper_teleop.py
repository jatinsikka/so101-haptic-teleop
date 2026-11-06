#!/usr/bin/env python3
"""Drive the SO-101 gripper from the Hapkit handle. Milestone 3: one-way position control.

    .venv/bin/python poc/gripper_teleop.py --gripper-port /dev/cu.wchusbserial... \
        --hapkit-port /dev/cu.usbmodem... --dry-run

Handle left = gripper closed, handle right = gripper open. You zero the handle at the left stop.

--dry-run writes nothing: it stays on the read-only port, and you move the handle and gripper by
hand to confirm the mapping. Without it, the gripper closes at startup because the handle is at the
closed end, then follows the handle. Ctrl+C turns torque off and restores the torque limit.

Exit status: 0 finished, 2 bus problem, 3 torque already on at start, 4 stopped by a safety check.

Register sources: LeRobot v0.6.1 tables.py lists all of these. Feetech's SMS_STS.h also lists 9, 11,
40, 42, 48 and 56-63; 16 and 65 appear only in the LeRobot table.
"""

import argparse
import csv
import statistics
import sys
import time

import serial
from scservo_sdk import COMM_SUCCESS, PacketHandler
from serial.tools import list_ports

from probe_gripper import (EXPECTED_MODEL, SERVO_ID, TELEMETRY_ADDR, TELEMETRY_LENGTH,
                           decode_telemetry, describe_status, little_endian, read_bytes)
from read_hapkit import BAUD as HAPKIT_BAUD
from read_hapkit import HapkitReader, wait_for_first_sample
from servo_safety import (GOAL_POSITION_ADDR, MAX_GOAL_TICK, MAX_TORQUE_LIMIT, MIN_GOAL_TICK,
                          TORQUE_ENABLE_ADDR, TORQUE_LIMIT_ADDR, GripperPortHandler,
                          ReadOnlyPortHandler, WriteBlockedError, check_torque_enable_value)

GRIPPER_BAUD = 1_000_000

EXIT_OK = 0
EXIT_BUS_ERROR = 2
EXIT_TORQUE_ON = 3
EXIT_SAFETY_STOP = 4

# Mapping. Measured: handle stops 15.57 cm apart. Gripper low ticks = closed, from the direction
# check. Both goal ends sit 20 ticks inside the servo limits, to avoid pressing into end of travel.
HANDLE_SPAN_CM = 15.57
CLOSED_TICK = MIN_GOAL_TICK + 20
OPEN_TICK = MAX_GOAL_TICK - 20

# Control
RATE_HZ = 50.0
DEFAULT_TORQUE_LIMIT = 200   # below Overload_Torque = 25%, so a grasp should not trigger it
DEFAULT_RATE_LIMIT = 600     # ticks/s
MAX_RATE_LIMIT = 1500        # ticks/s; a hand reached about 1200
MAX_LEAD_TICKS = 200         # a blocked gripper's goal cannot run far ahead of its position

# Safety
HANDLE_STALE_S = 0.25
HANDLE_RANGE_MARGIN_CM = 1.0
MAX_CONSECUTIVE_BUS_FAILURES = 25  # 0.5 s at 50 Hz
MAX_TEMPERATURE_C = 55
OVERTEMP_READINGS = 5  # single readings can glitch, so require several in a row
WRITE_ATTEMPTS = 3

# Zeroing
ZERO_WINDOW_S = 0.25
ZERO_MIN_SAMPLES = 20
ZERO_MAX_SPREAD_CM = 0.15  # about 2 encoder counts

PRINT_PERIOD_S = 0.1
STATUS_PERIOD_S = 1.0

MIN_POSITION_LIMIT_ADDR = 9
MAX_POSITION_LIMIT_ADDR = 11
MAX_TORQUE_LIMIT_ADDR = 16
STATUS_ADDR = 65


class ServoError(RuntimeError):
    """The servo did not respond as needed, or its configuration is not what this code expects."""


def clamp(value, low, high):
    return max(low, min(high, value))


def handle_to_ticks(xh, xh_closed):
    """Map handle position to a gripper target. xh_closed is the handle's left stop."""
    opening = clamp((xh - xh_closed) / HANDLE_SPAN_CM, 0.0, 1.0)
    return round(CLOSED_TICK + opening * (OPEN_TICK - CLOSED_TICK))


def next_goal(target, previous_goal, position, max_step):
    """Move toward target by at most max_step, staying near the position and in servo limits."""
    goal = previous_goal + clamp(target - previous_goal, -max_step, max_step)
    goal = clamp(goal, position - MAX_LEAD_TICKS, position + MAX_LEAD_TICKS)
    return clamp(goal, MIN_GOAL_TICK, MAX_GOAL_TICK)


def hold_reason(xh, xh_closed, sample_age):
    """Why the gripper should hold still rather than follow the handle, or None."""
    if sample_age > HANDLE_STALE_S:
        return "no handle data"
    if not (xh_closed - HANDLE_RANGE_MARGIN_CM <= xh
            <= xh_closed + HANDLE_SPAN_CM + HANDLE_RANGE_MARGIN_CM):
        return "handle out of range"
    return None


def zero_handle(reader, clock=time.perf_counter, sleep=time.sleep):
    """Return the handle's resting xh, or raise ServoError if it is moving or data is missing."""
    if not reader.discard_pending(clock, sleep):
        raise ServoError("the handle stream did not catch up after the prompt")
    samples = []
    start = clock()
    while clock() - start < ZERO_WINDOW_S:
        samples += reader.poll()
        sleep(0.001)
    if len(samples) < ZERO_MIN_SAMPLES:
        raise ServoError(f"zeroing got only {len(samples)} handle samples")
    if max(samples) - min(samples) > ZERO_MAX_SPREAD_CM:
        raise ServoError("handle moved during zeroing; hold it still at the left stop")
    return statistics.median(samples)


class Gripper:
    """Register access for one servo. In dry-run mode, every write is skipped."""

    def __init__(self, bus, port, dry_run):
        self.bus = bus
        self.port = port
        self.dry_run = dry_run

    def read(self, addr, size):
        data = read_bytes(self.bus, self.port, addr, size)
        return None if data is None else little_endian(data)

    def read_required(self, addr, size):
        for _ in range(2):
            value = self.read(addr, size)
            if value is not None:
                return value
        raise ServoError(f"could not read register {addr}")

    def telemetry(self):
        data = read_bytes(self.bus, self.port, TELEMETRY_ADDR, TELEMETRY_LENGTH)
        return None if data is None else decode_telemetry(data)

    def write(self, addr, size, value):
        """Send one write. Returns True if acknowledged (always True in dry-run)."""
        if self.dry_run:
            return True
        send = self.bus.write1ByteTxRx if size == 1 else self.bus.write2ByteTxRx
        comm, _error = send(self.port, SERVO_ID, addr, value)
        if comm != COMM_SUCCESS:
            self.port.ser.reset_input_buffer()
            return False
        return True

    def write_required(self, addr, size, value):
        for _ in range(WRITE_ATTEMPTS):
            if self.write(addr, size, value):
                return
        raise ServoError(f"write to register {addr} failed {WRITE_ATTEMPTS} times")

    def set_torque(self, value):
        self.write_required(TORQUE_ENABLE_ADDR, 1, check_torque_enable_value(value))


def check_servo(gripper, torque_limit, out):
    """Confirm the servo matches this code's assumptions. Returns the torque limit to restore."""
    model, comm, _error = gripper.bus.ping(gripper.port, SERVO_ID)
    if comm != COMM_SUCCESS:
        raise ServoError(f"no reply from servo ID {SERVO_ID}; is the follower's supply on?")
    if model != EXPECTED_MODEL:
        raise ServoError(f"ID {SERVO_ID} is model {model}, expected {EXPECTED_MODEL}")

    limits = (gripper.read_required(MIN_POSITION_LIMIT_ADDR, 2),
              gripper.read_required(MAX_POSITION_LIMIT_ADDR, 2))
    if limits != (MIN_GOAL_TICK, MAX_GOAL_TICK):
        raise ServoError(f"position limits are {limits}, expected "
                         f"{(MIN_GOAL_TICK, MAX_GOAL_TICK)}; the mapping would be wrong")

    max_torque = gripper.read_required(MAX_TORQUE_LIMIT_ADDR, 2)
    original_limit = gripper.read_required(TORQUE_LIMIT_ADDR, 2)
    if torque_limit > max_torque or original_limit > MAX_TORQUE_LIMIT:
        raise ServoError(f"torque limits unexpected: requested {torque_limit}, register 16 = "
                         f"{max_torque}, register 48 = {original_limit}")

    out(f"servo ID {SERVO_ID}: model {model}, limits {limits}, torque limit {original_limit}")
    return original_limit


def engage(gripper, torque_limit):
    """Goal = here first, so turning torque on does not make the gripper jump. Returns the goal."""
    sample = gripper.telemetry()
    if sample is None:
        raise ServoError("could not read the position before enabling torque")
    goal = clamp(sample["position"], MIN_GOAL_TICK, MAX_GOAL_TICK)
    gripper.write_required(GOAL_POSITION_ADDR, 2, goal)
    gripper.write_required(TORQUE_LIMIT_ADDR, 2, torque_limit)
    gripper.set_torque(1)
    return goal


def disengage(gripper, original_limit, out):
    """Torque off, then restore the limit. Tries both even if the first fails."""
    try:
        gripper.set_torque(0)
        out("torque off")
    except Exception as exc:
        out(f"WARNING: could not confirm torque off ({exc}). Power-cycle the follower.")
    try:
        gripper.write_required(TORQUE_LIMIT_ADDR, 2, original_limit)
    except Exception as exc:
        out(f"WARNING: could not restore the torque limit ({exc}). A power cycle resets it.")


def control_loop(gripper, reader, xh_closed, goal, rate_limit, duration, out, log=None,
                 clock=time.perf_counter, sleep=time.sleep):
    """Run the 50 Hz loop until duration, Ctrl+C or a safety stop. Returns an exit status."""
    period = 1.0 / RATE_HZ
    max_step = max(1, round(rate_limit * period))
    start = clock()
    next_tick = start
    next_print = start
    next_status = start + STATUS_PERIOD_S

    boot_lines = reader.boot_lines
    latest_xh = xh_closed
    last_sample_time = start
    bus_failures = 0
    hot_readings = 0
    last_hold = None
    last_status = 0
    ticks = misses = 0
    worst_gap = 0.0
    previous_tick_time = None
    load_range = None
    temperatures = []
    result = EXIT_OK

    out("t_s     xh_cm   target    goal  position  load  temp_C")
    try:
        while duration is None or clock() - start < duration:
            now = clock()
            if previous_tick_time is not None:
                worst_gap = max(worst_gap, now - previous_tick_time)
            previous_tick_time = now
            ticks += 1

            samples = reader.poll()
            if samples:
                latest_xh = samples[-1]
                last_sample_time = now
            if reader.boot_lines != boot_lines:
                out("STOP: the Hapkit reset during the run, so xh re-zeroed.")
                result = EXIT_SAFETY_STOP
                break

            sample = gripper.telemetry()
            written = False
            if sample is not None:
                position = sample["position"]
                hot_readings = hot_readings + 1 if sample["temperature"] > MAX_TEMPERATURE_C else 0
                if hot_readings >= OVERTEMP_READINGS:
                    out(f"STOP: temperature above {MAX_TEMPERATURE_C} C for {hot_readings} "
                        "readings in a row.")
                    result = EXIT_SAFETY_STOP
                    break

                target = handle_to_ticks(latest_xh, xh_closed)
                hold = hold_reason(latest_xh, xh_closed, now - last_sample_time)
                if hold != last_hold:
                    out(f"HOLD: {hold}" if hold else "resumed following the handle")
                    last_hold = hold
                if hold:
                    goal = clamp(position, MIN_GOAL_TICK, MAX_GOAL_TICK)
                else:
                    goal = next_goal(target, goal, position, max_step)
                written = gripper.write(GOAL_POSITION_ADDR, 2, goal)

                load = sample["load"]
                load_range = ((load, load) if load_range is None
                              else (min(load_range[0], load), max(load_range[1], load)))
                temperatures.append(sample["temperature"])
                if log:
                    log.writerow([f"{now - start:.3f}", latest_xh, target, goal, position, load,
                                  sample["temperature"], sample["speed"], hold or ""])
                if now >= next_print:
                    next_print += PRINT_PERIOD_S
                    out(f"{now - start:5.1f}  {latest_xh:+6.2f}  {target:>6}  {goal:>6}  "
                        f"{position:>8}  {load:>+4}  {sample['temperature']:>6}")

            if sample is None or not written:
                misses += 1
                bus_failures += 1
                if bus_failures >= MAX_CONSECUTIVE_BUS_FAILURES:
                    out(f"STOP: {bus_failures} failed bus ticks in a row.")
                    result = EXIT_BUS_ERROR
                    break
            else:
                bus_failures = 0

            if now >= next_status:
                next_status += STATUS_PERIOD_S
                status = gripper.read(STATUS_ADDR, 1)
                if status is not None and status != last_status:
                    out(f"servo status changed: {status} ({describe_status(status)})")
                    last_status = status

            next_tick += period
            remaining = next_tick - clock()
            if remaining > 0:
                sleep(remaining)
            elif remaining < -period:
                next_tick = clock()
    except KeyboardInterrupt:
        pass

    elapsed = max(clock() - start, 1e-9)
    out(f"--- summary: {elapsed:.1f} s, {ticks / elapsed:.1f} Hz (target {RATE_HZ:.0f}), worst gap "
        f"{worst_gap * 1000:.1f} ms, {misses} missed ticks")
    if temperatures:
        out(f"load range: {load_range[0]:+d} to {load_range[1]:+d} raw | "
            f"median temperature: {statistics.median(temperatures):.0f} C")
    return result


def positive_rate_limit(text):
    value = int(text)
    if not 1 <= value <= MAX_RATE_LIMIT:
        raise argparse.ArgumentTypeError(f"must be 1-{MAX_RATE_LIMIT} ticks/s")
    return value


def torque_limit_value(text):
    value = int(text)
    if not 1 <= value <= MAX_TORQUE_LIMIT:
        raise argparse.ArgumentTypeError(f"must be 1-{MAX_TORQUE_LIMIT}")
    return value


def run(args, log, out=print, wait_for_enter=input):
    port_class = ReadOnlyPortHandler if args.dry_run else GripperPortHandler
    port = port_class(args.gripper_port, allowed_id=SERVO_ID)
    hapkit = None
    try:
        try:
            if not port.setBaudRate(GRIPPER_BAUD):
                raise serial.SerialException("could not set the baud rate")
            hapkit = serial.Serial(args.hapkit_port, HAPKIT_BAUD, timeout=0)
        except serial.SerialException as exc:
            out(f"ERROR: cannot open port: {exc}")
            out("Available: " + ", ".join(sorted(p.device for p in list_ports.comports())))
            return EXIT_BUS_ERROR

        gripper = Gripper(PacketHandler(0), port, args.dry_run)
        try:
            original_limit = check_servo(gripper, args.torque_limit, out)
            if gripper.read_required(TORQUE_ENABLE_ADDR, 1) != 0:
                out("Torque is already on. Stopping without changing anything. Power-cycle the "
                    "follower or approve a torque-off write.")
                return EXIT_TORQUE_ON

            reader = HapkitReader(hapkit)
            first_samples, _waited = wait_for_first_sample(reader)
            if not first_samples:
                raise ServoError("no P lines from the Hapkit; is hapkit_gripper uploaded?")

            wait_for_enter("Hold the handle still at the LEFT stop (closed), then press Enter: ")
            xh_closed = zero_handle(reader)
            out(f"zeroed: left stop at xh = {xh_closed:+.2f} cm")

            if args.dry_run:
                out("DRY RUN: nothing will be written. Move the handle and gripper by hand.")
                sample = gripper.telemetry()
                if sample is None:
                    raise ServoError("could not read the gripper position")
                goal = clamp(sample["position"], MIN_GOAL_TICK, MAX_GOAL_TICK)
                return control_loop(gripper, reader, xh_closed, goal, args.rate_limit,
                                    args.duration, out, log)

            out("Gripper will close now, then follow the handle. Keep fingers clear. Ctrl+C stops.")
            try:
                goal = engage(gripper, args.torque_limit)
                return control_loop(gripper, reader, xh_closed, goal, args.rate_limit,
                                    args.duration, out, log)
            finally:
                disengage(gripper, original_limit, out)
        except (ServoError, WriteBlockedError) as exc:
            out(f"ERROR: {exc}")
            return EXIT_BUS_ERROR
    finally:
        if hapkit is not None:
            hapkit.close()
        if port.is_open:
            port.closePort()


def main(argv=None):
    parser = argparse.ArgumentParser(description="Drive the gripper from the Hapkit handle.")
    parser.add_argument("--gripper-port", required=True)
    parser.add_argument("--hapkit-port", required=True)
    parser.add_argument("--dry-run", action="store_true", help="never write to the servo")
    parser.add_argument("--torque-limit", type=torque_limit_value, default=DEFAULT_TORQUE_LIMIT,
                        help=f"register 48 value, 1-{MAX_TORQUE_LIMIT} (default "
                             f"{DEFAULT_TORQUE_LIMIT})")
    parser.add_argument("--rate-limit", type=positive_rate_limit, default=DEFAULT_RATE_LIMIT,
                        help=f"ticks/s, up to {MAX_RATE_LIMIT} (default {DEFAULT_RATE_LIMIT})")
    parser.add_argument("--duration", type=float, default=None,
                        help="seconds to run (default: until Ctrl+C)")
    parser.add_argument("--log", default=None, help="write every tick to this CSV file")
    args = parser.parse_args(argv)

    if args.log is None:
        return run(args, None)
    with open(args.log, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t_s", "xh_cm", "target", "goal", "position", "load", "temperature",
                         "speed", "hold"])
        return run(args, writer)


if __name__ == "__main__":
    sys.exit(main())
