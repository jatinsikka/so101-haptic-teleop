"""Hardware-free tests for the gripper loop. Uses the real SDK against a fake servo.

    .venv/bin/python -m unittest discover -s poc -v
"""

import signal
import unittest

from scservo_sdk import PacketHandler

import gripper_teleop as gt
from read_hapkit import HapkitReader
from servo_safety import (GripperPortHandler, INST_READ, INST_WRITE, ReadOnlyPortHandler,
                          WriteBlockedError)
from test_probe_gripper import FakeServo

SERVO_ID = 6


class WritableFakeServo(FakeServo):
    """A fake servo that also accepts writes. Position follows the goal unless blocked."""

    def __init__(self, position=2500):
        super().__init__()
        self.memory[9:11] = (2035).to_bytes(2, "little")
        self.memory[11:13] = (3463).to_bytes(2, "little")
        self.memory[16:18] = (500).to_bytes(2, "little")
        self.memory[48:50] = (500).to_bytes(2, "little")
        self.memory[56:58] = position.to_bytes(2, "little")
        self.memory[63] = 34
        self.writes = []
        self.follows_goal = True

    def write(self, packet):
        packet = list(packet)
        if packet[4] != INST_WRITE:
            return super().write(packet)
        self.packets.append(packet)
        addr, data = packet[5], packet[6:-1]
        value = sum(byte << (8 * i) for i, byte in enumerate(data))
        self.writes.append((addr, value))
        self.memory[addr:addr + len(data)] = bytes(data)
        if addr == 42 and self.follows_goal:
            self.memory[56:58] = bytes(data)
        if not self.silent:
            self._reply([])
        return len(packet)

    def position(self):
        return int.from_bytes(self.memory[56:58], "little")


class FakeHapkit:
    """A serial stand-in that streams the current handle position whenever it is polled."""

    def __init__(self, xh=0.0, clock=None):
        self.xh = xh
        self.clock = clock
        self.next_line = clock() if clock else None
        self.pending = bytearray()
        self.silent = False

    @property
    def in_waiting(self):
        if not self.silent and (self.clock is None or self.clock() >= self.next_line):
            self.pending += f"P {self.xh:.2f}\r\n".encode()
            if self.clock:
                self.next_line = self.clock() + 0.005
        return len(self.pending)

    def read(self, size):
        data = bytes(self.pending[:size])
        del self.pending[:size]
        return data

    def reset_input_buffer(self):
        raise AssertionError("resetting a full buffer stalled the real stream; drain it instead")


class FakeClock:
    def __init__(self):
        self.now = 100.0
        self.on_sleep = None

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds
        if self.on_sleep:
            self.on_sleep(self.now)


def make_gripper(servo, port_class=GripperPortHandler, dry_run=False):
    port = port_class("fake", allowed_id=SERVO_ID)
    port.ser = servo
    port.is_open = True
    port.tx_time_per_byte = 0.01
    return gt.Gripper(PacketHandler(0), port, dry_run)


class GripperGuardTests(unittest.TestCase):
    def setUp(self):
        self.servo = WritableFakeServo()
        self.gripper = make_gripper(self.servo)
        self.bus, self.port = self.gripper.bus, self.gripper.port

    def blocked(self, send):
        with self.assertRaises(WriteBlockedError):
            send()
        self.assertEqual(self.servo.packets, [], "a blocked packet reached the wire")

    def test_allows_the_three_registers(self):
        self.bus.write1ByteTxRx(self.port, SERVO_ID, 40, 1)
        self.bus.write2ByteTxRx(self.port, SERVO_ID, 42, 2500)
        self.bus.write2ByteTxRx(self.port, SERVO_ID, 48, 200)
        self.assertEqual(self.servo.writes, [(40, 1), (42, 2500), (48, 200)])

    def test_blocks_every_eeprom_register_and_lock(self):
        for addr in list(range(0, 40)) + [55]:
            with self.subTest(addr=addr):
                self.blocked(lambda: self.bus.write1ByteTxRx(self.port, SERVO_ID, addr, 0))

    def test_blocks_calibration_value_on_register_40(self):
        self.blocked(lambda: self.bus.write1ByteTxRx(self.port, SERVO_ID, 40, 128))

    def test_blocks_goals_outside_servo_limits(self):
        self.blocked(lambda: self.bus.write2ByteTxRx(self.port, SERVO_ID, 42, 2034))
        self.blocked(lambda: self.bus.write2ByteTxRx(self.port, SERVO_ID, 42, 3464))

    def test_blocks_torque_limit_above_cap(self):
        self.blocked(lambda: self.bus.write2ByteTxRx(self.port, SERVO_ID, 48, 501))

    def test_blocks_wrong_write_sizes(self):
        self.blocked(lambda: self.bus.write2ByteTxRx(self.port, SERVO_ID, 40, 0))
        self.blocked(lambda: self.bus.write1ByteTxRx(self.port, SERVO_ID, 42, 0))
        self.blocked(lambda: self.bus.write4ByteTxRx(self.port, SERVO_ID, 39, 0))

    def test_blocks_other_servos_and_broadcast(self):
        self.blocked(lambda: self.bus.write1ByteTxRx(self.port, 1, 40, 0))
        self.blocked(lambda: self.port.writePort([0xFF, 0xFF, 0xFE, 4, INST_WRITE, 40, 0, 0]))

    def test_blocks_other_write_instructions(self):
        self.blocked(lambda: self.bus.action(self.port, SERVO_ID))
        self.blocked(lambda: self.port.writePort([0xFF, 0xFF, SERVO_ID, 5, 4, 42, 0, 8, 0]))
        self.blocked(lambda: self.port.writePort([0xFF, 0xFF, SERVO_ID, 5, 0x83, 42, 0, 8, 0]))

    def test_still_allows_reads(self):
        self.assertIsNotNone(self.gripper.read(56, 2))

    def test_torque_off_still_works_after_a_blocked_write(self):
        self.blocked(lambda: self.bus.write1ByteTxRx(self.port, SERVO_ID, 21, 32))
        output = []
        gt.disengage(self.gripper, 500, output.append)
        self.assertEqual(self.servo.writes, [(40, 0), (48, 500)])
        self.assertEqual(output, ["torque off"])


class MappingTests(unittest.TestCase):
    def test_left_stop_is_closed_and_right_stop_is_open(self):
        self.assertEqual(gt.handle_to_ticks(-7.34, -7.34), gt.CLOSED_TICK)
        self.assertEqual(gt.handle_to_ticks(8.23, -7.34), gt.OPEN_TICK)

    def test_low_ticks_are_closed(self):
        self.assertLess(gt.CLOSED_TICK, gt.OPEN_TICK)

    def test_threshold_is_20_percent_of_the_span(self):
        self.assertAlmostEqual(gt.CLOSE_THRESHOLD_CM, 0.2 * 15.57)

    def test_stays_closed_from_left_stop_to_threshold(self):
        for fraction in (0.0, 0.05, 0.1, 0.19, 0.2):
            with self.subTest(fraction=fraction):
                xh = -7.34 + fraction * gt.HANDLE_SPAN_CM
                self.assertEqual(gt.handle_to_ticks(xh, -7.34), gt.CLOSED_TICK)

    def test_opens_just_past_threshold(self):
        xh = -7.34 + gt.CLOSE_THRESHOLD_CM + 0.1
        self.assertGreater(gt.handle_to_ticks(xh, -7.34), gt.CLOSED_TICK)

    def test_midpoint_of_operating_range(self):
        middle = gt.handle_to_ticks(-7.34 + 0.6 * gt.HANDLE_SPAN_CM, -7.34)
        self.assertEqual(middle, round((gt.CLOSED_TICK + gt.OPEN_TICK) / 2))

    def test_clamps_past_both_stops(self):
        self.assertEqual(gt.handle_to_ticks(-20.0, -7.34), gt.CLOSED_TICK)
        self.assertEqual(gt.handle_to_ticks(20.0, -7.34), gt.OPEN_TICK)

    def test_goal_ends_are_inside_servo_limits(self):
        self.assertGreater(gt.CLOSED_TICK, gt.MIN_GOAL_TICK)
        self.assertLess(gt.OPEN_TICK, gt.MAX_GOAL_TICK)


class SignalTests(unittest.TestCase):
    def setUp(self):
        self.saved = {sig: signal.getsignal(sig)
                      for sig in (signal.SIGTSTP, signal.SIGTERM, signal.SIGHUP)}
        self.addCleanup(lambda: [signal.signal(sig, old) for sig, old in self.saved.items()])
        gt.install_signal_handlers()

    def test_ctrl_z_is_ignored(self):
        self.assertEqual(signal.getsignal(signal.SIGTSTP), signal.SIG_IGN)

    def test_kill_and_closed_window_take_the_clean_exit(self):
        for sig in (signal.SIGTERM, signal.SIGHUP):
            with self.subTest(sig=sig), self.assertRaises(KeyboardInterrupt):
                signal.raise_signal(sig)


class NextGoalTests(unittest.TestCase):
    def test_rate_limited_both_ways(self):
        self.assertEqual(gt.next_goal(3000, 2500, 2500, 0, 12), 2512)
        self.assertEqual(gt.next_goal(2000, 2500, 2500, 0, 12), 2488)

    def test_reaches_a_close_target_exactly(self):
        self.assertEqual(gt.next_goal(2505, 2500, 2500, 0, 12), 2505)

    def test_stalled_gripper_gets_only_the_small_lead(self):
        self.assertEqual(gt.next_goal(3000, 2800, 2500, 0, 12), 2500 + gt.STALL_LEAD_TICKS)
        self.assertEqual(gt.next_goal(2000, 2200, 2500, 0, 12), 2500 - gt.STALL_LEAD_TICKS)

    def test_lead_grows_with_speed_in_either_direction(self):
        self.assertEqual(gt.allowed_lead(0), 50)
        self.assertEqual(gt.allowed_lead(500), 200)
        self.assertEqual(gt.allowed_lead(-500), 200)

    def test_lead_is_capped(self):
        self.assertEqual(gt.allowed_lead(5000), gt.MAX_LEAD_TICKS)

    def test_stays_in_servo_limits(self):
        self.assertEqual(gt.next_goal(3463, 3463, 3472, 0, 12), 3463)


class TargetPredictorTests(unittest.TestCase):
    def predictor(self, ahead_s=0.05):
        return gt.TargetPredictor(ahead_s, 0.01)

    def test_first_update_and_rest_add_nothing(self):
        predictor = self.predictor()
        for _ in range(5):
            self.assertEqual(predictor.update(2500), 2500)

    def test_steady_motion_aims_ahead_by_velocity_times_time(self):
        predictor = self.predictor()
        target = 2100
        for _ in range(100):
            target += 10  # 1000 ticks/s
            aim = predictor.update(target)
        self.assertEqual(aim, target + 50)

    def test_aims_ahead_when_closing_too(self):
        predictor = self.predictor()
        target = 3400
        for _ in range(100):
            target -= 10
            aim = predictor.update(target)
        self.assertEqual(aim, target - 50)

    def test_aim_stays_within_gripper_range(self):
        predictor = self.predictor()
        for target in range(3300, gt.OPEN_TICK + 1, 20):
            self.assertLessEqual(predictor.update(target), gt.OPEN_TICK)
        predictor = self.predictor()
        for target in range(2300, gt.CLOSED_TICK - 1, -20):
            self.assertGreaterEqual(predictor.update(target), gt.CLOSED_TICK)

    def test_single_encoder_count_causes_only_small_jitter(self):
        predictor = self.predictor()
        predictor.update(2500)
        self.assertLessEqual(abs(predictor.update(2508) - 2508), 10)

    def test_zero_ahead_disables_it(self):
        predictor = self.predictor(0.0)
        predictor.update(2500)
        self.assertEqual(predictor.update(2600), 2600)

    def test_reset_clears_velocity(self):
        predictor = self.predictor()
        predictor.update(2500)
        predictor.update(2600)
        predictor.reset()
        self.assertEqual(predictor.update(2700), 2700)


class HoldTests(unittest.TestCase):
    def test_follows_in_range_with_fresh_data(self):
        self.assertIsNone(gt.hold_reason(0.0, -7.34, 0.01))

    def test_holds_on_stale_data(self):
        self.assertEqual(gt.hold_reason(0.0, -7.34, 0.26), "no handle data")

    def test_holds_out_of_range_either_side(self):
        self.assertEqual(gt.hold_reason(-8.4, -7.34, 0.0), "handle out of range")
        self.assertEqual(gt.hold_reason(9.3, -7.34, 0.0), "handle out of range")

    def test_margin_allows_small_overshoot(self):
        self.assertIsNone(gt.hold_reason(-7.5, -7.34, 0.0))


class ZeroTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()

    def test_uses_resting_position(self):
        reader = HapkitReader(FakeHapkit(-7.34, self.clock))
        self.assertEqual(gt.zero_handle(reader, self.clock, self.clock.sleep), -7.34)

    def test_old_backlog_is_not_used_for_the_zero(self):
        hapkit = FakeHapkit(-7.34, self.clock)
        hapkit.pending += b"P 5.00\r\n" * 3000
        self.assertEqual(gt.zero_handle(HapkitReader(hapkit), self.clock, self.clock.sleep), -7.34)

    def test_fails_if_the_stream_never_catches_up(self):
        hapkit = FakeHapkit(-7.34, self.clock)

        def flood(_now):
            hapkit.pending += b"P 5.00\r\n" * 10
        self.clock.on_sleep = flood
        with self.assertRaisesRegex(gt.ServoError, "catch up"):
            gt.zero_handle(HapkitReader(hapkit), self.clock, self.clock.sleep)

    def test_rejects_moving_handle(self):
        hapkit = FakeHapkit(-7.0, self.clock)

        def move(_now):
            hapkit.xh += 0.01
        self.clock.on_sleep = move
        with self.assertRaisesRegex(gt.ServoError, "moved"):
            gt.zero_handle(HapkitReader(hapkit), self.clock, self.clock.sleep)

    def test_rejects_missing_data(self):
        hapkit = FakeHapkit(0.0, self.clock)
        hapkit.silent = True
        with self.assertRaisesRegex(gt.ServoError, "samples"):
            gt.zero_handle(HapkitReader(hapkit), self.clock, self.clock.sleep)


class CheckServoTests(unittest.TestCase):
    def setUp(self):
        self.servo = WritableFakeServo()
        self.gripper = make_gripper(self.servo)

    def check(self, torque_limit=200):
        return gt.check_servo(self.gripper, torque_limit, lambda _line: None)

    def test_returns_limit_to_restore(self):
        self.assertEqual(self.check(), 500)

    def test_rejects_changed_position_limits(self):
        self.servo.memory[9:11] = (100).to_bytes(2, "little")
        with self.assertRaisesRegex(gt.ServoError, "position limits"):
            self.check()

    def test_rejects_limit_above_register_16(self):
        self.servo.memory[16:18] = (100).to_bytes(2, "little")
        with self.assertRaisesRegex(gt.ServoError, "torque limits"):
            self.check()

    def test_rejects_wrong_model(self):
        self.servo.memory[3:5] = (1).to_bytes(2, "little")
        with self.assertRaisesRegex(gt.ServoError, "model"):
            self.check()

    def test_checks_write_nothing(self):
        self.check()
        self.assertEqual(self.servo.writes, [])


class EngageTests(unittest.TestCase):
    def setUp(self):
        self.servo = WritableFakeServo(position=2500)
        self.gripper = make_gripper(self.servo)
        self.output = []

    def test_goal_is_set_before_torque_turns_on(self):
        self.assertEqual(gt.engage(self.gripper, 200), 2500)
        self.assertEqual(self.servo.writes, [(42, 2500), (48, 200), (40, 1)])

    def test_clamps_position_past_the_limit(self):
        self.servo.memory[56:58] = (3472).to_bytes(2, "little")
        self.assertEqual(gt.engage(self.gripper, 200), 3463)

    def test_disengage_turns_torque_off_then_restores_limit(self):
        gt.disengage(self.gripper, 500, self.output.append)
        self.assertEqual(self.servo.writes, [(40, 0), (48, 500)])

    def test_disengage_warns_when_servo_stops_answering(self):
        self.servo.silent = True
        gt.disengage(self.gripper, 500, self.output.append)
        self.assertIn("Power-cycle", "\n".join(self.output))
        self.assertIn("restore the torque limit", "\n".join(self.output))


class ControlLoopTests(unittest.TestCase):
    def setUp(self):
        self.servo = WritableFakeServo(position=2500)
        self.hapkit = FakeHapkit(-7.34)
        self.reader = HapkitReader(self.hapkit)
        self.clock = FakeClock()
        self.output = []

    def run_loop(self, duration=1.0, rate_limit=600, dry_run=False, goal=2500):
        port_class = ReadOnlyPortHandler if dry_run else GripperPortHandler
        gripper = make_gripper(self.servo, port_class, dry_run)
        return gt.control_loop(gripper, self.reader, -7.34, goal, rate_limit, duration,
                               self.output.append, None, self.clock, self.clock.sleep)

    def text(self):
        return "\n".join(self.output)

    def goals(self):
        return [value for addr, value in self.servo.writes if addr == 42]

    def test_starts_at_closed_and_moves_at_the_rate_limit(self):
        self.assertEqual(self.run_loop(duration=0.2), gt.EXIT_OK)
        goals = self.goals()
        self.assertEqual(goals[0], 2494)
        self.assertTrue(all(abs(b - a) <= 6 for a, b in zip([2500] + goals, goals)))

    def test_reaches_closed_with_handle_at_left_stop(self):
        self.run_loop(duration=2.0)
        self.assertEqual(self.servo.position(), gt.CLOSED_TICK)

    def test_follows_handle_to_open(self):
        self.hapkit.xh = 8.23
        self.run_loop(duration=3.0)
        self.assertEqual(self.servo.position(), gt.OPEN_TICK)

    def test_dry_run_writes_nothing(self):
        self.hapkit.xh = 8.23
        self.assertEqual(self.run_loop(duration=0.2, dry_run=True), gt.EXIT_OK)
        self.assertEqual(self.servo.writes, [])
        self.assertTrue(all(p[4] != INST_WRITE for p in self.servo.packets))

    def test_stale_handle_holds_at_current_position(self):
        self.hapkit.xh = 8.23

        def lose_handle(now):
            self.hapkit.silent = now > 100.2
        self.clock.on_sleep = lose_handle
        self.servo.follows_goal = False
        self.assertEqual(self.run_loop(duration=1.0), gt.EXIT_OK)
        self.assertIn("HOLD: no handle data", self.text())
        self.assertEqual(self.goals()[-1], self.servo.position())

    def test_out_of_range_handle_holds_then_resumes(self):
        self.hapkit.xh = 12.0

        def bring_back(now):
            if now > 100.3:
                self.hapkit.xh = 0.0
        self.clock.on_sleep = bring_back
        self.run_loop(duration=0.6)
        self.assertIn("HOLD: handle out of range", self.text())
        self.assertIn("resumed following", self.text())

    def test_hapkit_reset_stops_the_loop(self):
        def reset(now):
            if now > 100.2:
                self.hapkit.pending += b"# hapkit_gripper m2\r\n"
        self.clock.on_sleep = reset
        self.assertEqual(self.run_loop(), gt.EXIT_SAFETY_STOP)
        self.assertIn("Hapkit reset", self.text())

    def test_single_temperature_glitch_does_not_stop(self):
        ticks = []

        def glitch(_now):
            ticks.append(1)
            self.servo.memory[63] = 90 if len(ticks) == 5 else 34
        self.clock.on_sleep = glitch
        self.assertEqual(self.run_loop(), gt.EXIT_OK)

    def test_sustained_overtemperature_stops(self):
        self.servo.memory[63] = 60
        self.assertEqual(self.run_loop(), gt.EXIT_SAFETY_STOP)
        self.assertIn("temperature above", self.text())

    def test_servo_silence_stops_the_loop(self):
        self.clock.on_sleep = lambda now: setattr(self.servo, "silent", now > 100.1)
        self.assertEqual(self.run_loop(duration=10.0), gt.EXIT_BUS_ERROR)
        self.assertIn("failed bus ticks", self.text())

    def test_status_change_is_reported(self):
        self.clock.on_sleep = lambda now: self.servo.memory.__setitem__(65, 32 if now > 100.5 else 0)
        self.run_loop(duration=1.5)
        self.assertIn("servo status changed: 32 (overload)", self.text())

    def test_ctrl_c_ends_the_loop_cleanly(self):
        def interrupt(now):
            if now > 100.1:
                raise KeyboardInterrupt
        self.clock.on_sleep = interrupt
        self.assertEqual(self.run_loop(duration=None), gt.EXIT_OK)
        self.assertIn("--- summary", self.text())

    def test_reads_use_only_id_6(self):
        self.run_loop(duration=0.1)
        for packet in self.servo.packets:
            self.assertEqual(packet[2], SERVO_ID)
            self.assertIn(packet[4], (1, INST_READ, INST_WRITE))


if __name__ == "__main__":
    unittest.main()
