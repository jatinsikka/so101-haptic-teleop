#!/usr/bin/env python3
"""Drive the SO-101 gripper from the Hapkit handle. Milestone 3: one-way position control.

    .venv/bin/python poc/gripper_teleop.py --gripper-port /dev/cu.wchusbserial... \
        --hapkit-port /dev/cu.usbmodem... --dry-run

You zero the handle at the left stop. The first 20% of handle travel does nothing: the gripper stays
closed. Past that threshold it opens, reaching fully open at the right stop.

--dry-run writes nothing: it stays on the read-only port, and you move the handle and gripper by
hand to confirm the mapping. Without it, the gripper closes at startup because the handle is at the
closed end, then follows the handle. Ctrl+C turns torque off and restores the torque limit.

Exit status: 0 finished, 2 bus problem, 3 torque already on at start, 4 stopped by a safety check.

Register sources: LeRobot v0.6.1 tables.py lists all of these. Feetech's SMS_STS.h also lists 9, 11,
40, 42, 48 and 56-63; 16 and 65 appear only in the LeRobot table.
"""

import argparse
import csv
import signal
import statistics
import sys
import termios
import time
import tty

import serial
from scservo_sdk import COMM_SUCCESS, PacketHandler
from serial.tools import list_ports

from contact_wall import (MAX_WALL_K, SURFACE_AMP_PER_CM_S, SURFACE_BASE_AMP, SURFACE_DECAY_S,
                          SURFACE_FREQ_HZ, SURFACE_MAX_AMP, WALL_LANDING_SPEED_CM_S, WALL_SPRING, WALL_SURFACE, ContactWall)
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
# check, where the closed gripper read 2038-2040. The open end sits 20 ticks inside the servo limit.
HANDLE_SPAN_CM = 15.57
CLOSE_THRESHOLD_FRACTION = 0.20  # share of handle travel, from the left stop, that stays closed
CLOSE_THRESHOLD_CM = CLOSE_THRESHOLD_FRACTION * HANDLE_SPAN_CM
CLOSED_TICK = 2040
OPEN_TICK = MAX_GOAL_TICK - 20
TICKS_PER_HANDLE_CM = (OPEN_TICK - CLOSED_TICK) / (HANDLE_SPAN_CM - CLOSE_THRESHOLD_CM)

# Control
RATE_HZ = 100.0
DEFAULT_TORQUE_LIMIT = 500   # sets speed; grip force is bounded by the stall lead below
DEFAULT_RATE_LIMIT = 3000    # ticks/s
MAX_RATE_LIMIT = 3000        # ticks/s

# Goal lead allowed ahead of the position. Fast motion needs a large lead, but a stalled gripper
# pushes with force set by its lead: in logged grasps, load was about 4 per tick of error, so 50
# ticks should hold near 200, below Overload_Torque = 25%.
STALL_LEAD_TICKS = 50
LEAD_PER_SPEED = 0.3         # extra ticks of lead per tick/s of measured speed
MAX_LEAD_TICKS = 300

# The servo trails its goal by about 70 ms, so aim ahead of the handle by part of that. The handle
# moves in whole encoder counts, so its velocity needs filtering to avoid jitter at low speed.
DEFAULT_FEEDFORWARD_MS = 70
FEEDFORWARD_FILTER_S = 0.04
FEEDFORWARD_MIN_SPEED = 300  # ticks/s; below this, one encoder count would only make the goal wiggle

# Wall stiffness in PWM per cm during contact. The firmware caps this at K_MAX_PWM_PER_CM.
DEFAULT_WALL_K = 200.0

# Safety
HANDLE_STALE_S = 0.25
HANDLE_RANGE_MARGIN_CM = 1.0
MAX_CONSECUTIVE_BUS_FAILURES = 50  # 0.5 s at 100 Hz
MAX_TEMPERATURE_C = 55
OVERTEMP_READINGS = 10  # 0.1 s at 100 Hz; single readings can glitch, so require several in a row
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
    """Map handle position to a gripper target. xh_closed is the handle's left stop.

    Closed from the left stop up to the threshold, then linear up to open at the right stop.
    """
    travel = xh - xh_closed - CLOSE_THRESHOLD_CM
    opening = clamp(travel / (HANDLE_SPAN_CM - CLOSE_THRESHOLD_CM), 0.0, 1.0)
    return round(CLOSED_TICK + opening * (OPEN_TICK - CLOSED_TICK))


def allowed_lead(speed):
    """How far the goal may lead the position: small when stalled, larger when moving fast."""
    return min(MAX_LEAD_TICKS, round(STALL_LEAD_TICKS + LEAD_PER_SPEED * abs(speed)))


def ticks_to_handle(ticks, xh_closed):
    """The handle position that maps to these gripper ticks; the inverse of handle_to_ticks."""
    opening = clamp((ticks - CLOSED_TICK) / (OPEN_TICK - CLOSED_TICK), 0.0, 1.0)
    return xh_closed + CLOSE_THRESHOLD_CM + opening * (HANDLE_SPAN_CM - CLOSE_THRESHOLD_CM)


def next_goal(target, previous_goal, position, speed, max_step):
    """Move toward target by at most max_step, staying near the position and in servo limits."""
    goal = previous_goal + clamp(target - previous_goal, -max_step, max_step)
    lead = allowed_lead(speed)
    goal = clamp(goal, position - lead, position + lead)
    return clamp(goal, MIN_GOAL_TICK, MAX_GOAL_TICK)


class TargetPredictor:
    """Aims ahead of a moving target by its filtered velocity, to offset the servo's lag.

    It works on the mapped target, so velocity is zero inside the closed zone.
    """

    def __init__(self, ahead_s, period_s):
        self.ahead_s = ahead_s
        self.period_s = period_s
        self.alpha = period_s / (FEEDFORWARD_FILTER_S + period_s)
        self.reset()

    def reset(self):
        self.previous = None
        self.velocity = 0.0

    def update(self, target):
        if self.previous is not None:
            raw = (target - self.previous) / self.period_s
            self.velocity += self.alpha * (raw - self.velocity)
        self.previous = target
        if abs(self.velocity) < FEEDFORWARD_MIN_SPEED:
            return target
        return clamp(round(target + self.velocity * self.ahead_s), CLOSED_TICK, OPEN_TICK)


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
    """Torque off, then restore the limit. Each write runs even if anything before it failed."""
    messages = []
    try:
        gripper.set_torque(0)
        messages.append("torque off")
    except Exception as exc:
        messages.append(f"WARNING: could not confirm torque off ({exc}). Power-cycle the follower.")
    try:
        gripper.write_required(TORQUE_LIMIT_ADDR, 2, original_limit)
    except Exception as exc:
        messages.append(f"WARNING: could not restore the torque limit ({exc}). A power cycle resets "
                        "it.")
    # Report last: if stdout has gone away, such as a closed pipe, the writes above still happened.
    try:
        for message in messages:
            out(message)
    except OSError:
        pass


def control_loop(gripper, reader, xh_closed, goal, rate_limit, duration, out, log=None,
                 clock=time.perf_counter, sleep=time.sleep, feedforward_s=0.0, wall=None):
    """Run the control loop until duration, Ctrl+C or a safety stop. Returns an exit status."""
    period = 1.0 / RATE_HZ
    max_step = max(1, round(rate_limit * period))
    predictor = TargetPredictor(feedforward_s, period)
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
                if wall:
                    if hold:
                        if wall.in_contact:
                            wall.stop()
                            out("wall off: holding")
                    else:
                        event = wall.update(now, latest_xh, position, goal, sample["speed"])
                        if event == "contact":
                            impact = (f", impact amplitude {wall.impact_amp:.0f}"
                                      if wall.wall_type == WALL_SURFACE else "")
                            out(f"CONTACT at {position} ticks: wall at {wall.x_c:+.2f} cm, handle at "
                                f"{latest_xh:+.2f} cm{impact}")
                        elif event == "released":
                            out("released: wall off")
                if hold:
                    predictor.reset()
                    goal = clamp(position, MIN_GOAL_TICK, MAX_GOAL_TICK)
                else:
                    aim = predictor.update(target)
                    goal = next_goal(aim, goal, position, sample["speed"], max_step)
                written = gripper.write(GOAL_POSITION_ADDR, 2, goal)

                load = sample["load"]
                load_range = ((load, load) if load_range is None
                              else (min(load_range[0], load), max(load_range[1], load)))
                temperatures.append(sample["temperature"])
                if log:
                    log.writerow([f"{now - start:.3f}", latest_xh, target, goal, position, load,
                                  sample["temperature"], sample["speed"], hold or "",
                                  f"{wall.x_c:.2f}" if wall and wall.x_c is not None else "",
                                  f"{wall.commanded_k:.1f}" if wall else "",
                                  f"{wall.impact_amp:.0f}" if wall and wall.impact_amp else ""])
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


def wall_stiffness_value(text):
    value = float(text)
    if not 0 <= value <= MAX_WALL_K:
        raise argparse.ArgumentTypeError(f"must be 0-{MAX_WALL_K:.0f}")
    return value


def torque_limit_value(text):
    value = int(text)
    if not 1 <= value <= MAX_TORQUE_LIMIT:
        raise argparse.ArgumentTypeError(f"must be 1-{MAX_TORQUE_LIMIT}")
    return value


def read_key():
    """Read one key press without needing Enter. Ctrl+C still works."""
    if not sys.stdin.isatty():
        return sys.stdin.readline().strip()[:1]
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)


def choose_wall_type(out, read=read_key):
    out("Wall type: press 1 for spring wall, 2 for decaying sinusoid hard surface")
    while True:
        key = read()
        if key == "1":
            out("  -> 1: spring wall")
            return WALL_SPRING
        if key == "2":
            out("  -> 2: decaying sinusoid hard surface")
            return WALL_SURFACE


def run(args, log, out=print, wait_for_enter=input):
    port_class = ReadOnlyPortHandler if args.dry_run else GripperPortHandler
    port = port_class(args.gripper_port, allowed_id=SERVO_ID)
    hapkit = None
    try:
        try:
            if not port.setBaudRate(GRIPPER_BAUD):
                raise serial.SerialException("could not set the baud rate")
            hapkit = serial.Serial(args.hapkit_port, HAPKIT_BAUD, timeout=0, write_timeout=0.01)
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

            wall_type = args.wall_type
            if wall_type is None and args.wall_k > 0 and not args.dry_run:
                wall_type = choose_wall_type(out)

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
                                    args.duration, out, log,
                                    feedforward_s=args.feedforward_ms / 1000.0)

            wall = None
            if args.wall_k > 0:
                if reader.firmware and b"m4.1" not in reader.firmware:
                    raise ServoError(f"walls need the m4.1 firmware, but the Uno reports "
                                     f"{reader.firmware.decode()!r}; use --wall-k 0 or upload it")
                wall = ContactWall(hapkit, lambda ticks: ticks_to_handle(ticks, xh_closed),
                                   args.wall_k, wall_type, TICKS_PER_HANDLE_CM,
                                   args.surface_base, args.surface_gain,
                                   args.wall_landing_speed, args.surface_max)
                if wall_type == WALL_SURFACE:
                    wall.configure_surface(args.surface_freq, args.surface_decay)

            out("Gripper will close now, then follow the handle. Keep fingers clear. Ctrl+C stops.")
            try:
                goal = engage(gripper, args.torque_limit)
                return control_loop(gripper, reader, xh_closed, goal, args.rate_limit,
                                    args.duration, out, log,
                                    feedforward_s=args.feedforward_ms / 1000.0, wall=wall)
            finally:
                if wall:
                    wall.stop()
                disengage(gripper, original_limit, out)
                if reader.isr_max_us is not None:
                    out(f"longest ISR time in the last report: {reader.isr_max_us} us "
                        "(period is 1000 us)")
        except (ServoError, WriteBlockedError) as exc:
            out(f"ERROR: {exc}")
            return EXIT_BUS_ERROR
    finally:
        if hapkit is not None:
            hapkit.close()
        if port.is_open:
            port.closePort()


def raise_keyboard_interrupt(_signum, _frame):
    raise KeyboardInterrupt


def install_signal_handlers():
    """Keep the exit path that turns torque off.

    The servo holds its last goal with torque on until told otherwise. Ctrl+Z would pause this
    program in that state, so it is ignored. Closing the window or a plain kill exits like Ctrl+C.
    """
    signal.signal(signal.SIGTSTP, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, raise_keyboard_interrupt)
    signal.signal(signal.SIGHUP, raise_keyboard_interrupt)


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
    parser.add_argument("--feedforward-ms", type=float, default=DEFAULT_FEEDFORWARD_MS,
                        help=f"aim this far ahead of the handle, 0 to disable (default "
                             f"{DEFAULT_FEEDFORWARD_MS})")
    parser.add_argument("--wall-k", type=wall_stiffness_value, default=DEFAULT_WALL_K,
                        help=f"handle wall stiffness in PWM/cm during contact, 0 for no wall "
                             f"(default {DEFAULT_WALL_K:.0f})")
    parser.add_argument("--wall-landing-speed", type=float, default=WALL_LANDING_SPEED_CM_S,
                        help=f"cm/s the wall moves to the contact point, 0 for at once (default "
                             f"{WALL_LANDING_SPEED_CM_S:.0f})")
    parser.add_argument("--wall-type", choices=(WALL_SPRING, WALL_SURFACE), default=None,
                        help="skip the 1 or 2 prompt at startup")
    parser.add_argument("--surface-freq", type=float, default=SURFACE_FREQ_HZ,
                        help=f"hard surface frequency in Hz (default {SURFACE_FREQ_HZ:.0f})")
    parser.add_argument("--surface-decay", type=float, default=SURFACE_DECAY_S,
                        help=f"hard surface time to decay to 1%%, in s (default {SURFACE_DECAY_S})")
    parser.add_argument("--surface-base", type=float, default=SURFACE_BASE_AMP,
                        help=f"hard surface PWM amplitude at zero speed (default {SURFACE_BASE_AMP:.0f})")
    parser.add_argument("--surface-gain", type=float, default=SURFACE_AMP_PER_CM_S,
                        help=f"extra PWM amplitude per cm/s of impact (default {SURFACE_AMP_PER_CM_S:.0f})")
    parser.add_argument("--surface-max", type=float, default=SURFACE_MAX_AMP,
                        help=f"hard surface PWM amplitude cap, at most 400 (default "
                             f"{SURFACE_MAX_AMP:.0f})")
    parser.add_argument("--duration", type=float, default=None,
                        help="seconds to run (default: until Ctrl+C)")
    parser.add_argument("--log", default=None, help="write every tick to this CSV file")
    args = parser.parse_args(argv)
    install_signal_handlers()

    try:
        if args.log is None:
            return run(args, None)
        with open(args.log, "w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["t_s", "xh_cm", "target", "goal", "position", "load", "temperature",
                             "speed", "hold", "wall_x_cm", "wall_k", "impact_amp"])
            return run(args, writer)
    except KeyboardInterrupt:
        print("stopped")  # Ctrl+C at the prompt; run() has already cleaned up
        return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
