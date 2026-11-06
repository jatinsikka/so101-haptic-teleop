"""Hardware-free tests for contact detection, the wall link and the wall test script.

    .venv/bin/python -m unittest discover -s poc -v
"""

import unittest
from unittest import mock

import serial

import contact_wall as cw
import gripper_teleop as gt
import wall_test as wt
from read_hapkit import HapkitReader


class FakeSerial:
    """Records what is written, and can serve lines of handle data."""

    def __init__(self, clock=None):
        self.written = []
        self.pending = bytearray()
        self.fail_writes = False
        self.xh = 0.0
        self.clock = clock
        self.next_line = clock() if clock else None

    def write(self, data):
        if self.fail_writes:
            raise serial.SerialTimeoutException("full")
        self.written.append(data.decode())
        return len(data)

    @property
    def in_waiting(self):
        if self.clock is None or self.clock() >= self.next_line:
            self.pending += f"P {self.xh:.2f}\r\n".encode()
            if self.clock:
                self.next_line = self.clock() + 0.005  # the real stream's 200 lines/s
        return len(self.pending)

    def read(self, size):
        data = bytes(self.pending[:size])
        del self.pending[:size]
        return data


# Gripper state during a logged grasp: stopped while asked to close.
GRASP = {"position": 2125, "goal": 2075, "speed": 0}
FREE = {"position": 2500, "goal": 2495, "speed": 0}


class ContactWallTests(unittest.TestCase):
    def setUp(self):
        self.ser = FakeSerial()
        self.wall = cw.ContactWall(self.ser, lambda ticks: (ticks - 2040) * 0.01, 100.0)

    def run_for(self, start, duration, xh=1.0, state=GRASP):
        events = []
        now = start
        while now < start + duration:
            event = self.wall.update(now, xh, **state)
            if event:
                events.append(event)
            now += 0.01
        return events

    def enter_contact(self):
        self.assertEqual(self.run_for(0.0, 0.2), ["contact"])

    def test_no_contact_in_free_motion(self):
        self.assertEqual(self.run_for(0.0, 1.0, state=FREE), [])
        self.assertEqual(self.ser.written, [])

    def test_brief_stall_at_a_direction_change_is_not_contact(self):
        self.assertEqual(self.run_for(0.0, 0.03), [])
        self.wall.update(0.03, 1.0, **FREE)
        self.assertEqual(self.run_for(0.04, 0.03), [])
        self.assertEqual(self.ser.written, [])

    def test_sustained_stall_becomes_contact_after_debounce(self):
        self.assertEqual(self.run_for(0.0, 0.03), [])
        self.assertEqual(self.wall.update(0.05, 1.0, **GRASP), "contact")

    def test_load_bouncing_after_impact_does_not_delay_contact(self):
        # A logged impact: load fell to 96 and rose to 332 within 40 ms. Load is no longer an input.
        self.assertEqual(self.run_for(0.0, 0.06), ["contact"])

    def test_each_condition_is_required(self):
        for change in ({"speed": 400}, {"speed": -400}, {"goal": GRASP["position"] - 5},
                       {"goal": GRASP["position"] + 50}):
            with self.subTest(change=change):
                wall = cw.ContactWall(self.ser, lambda t: 0.0, 100.0)
                now = 0.0
                while now < 1.0:
                    self.assertIsNone(wall.update(now, 1.0, **{**GRASP, **change}))
                    now += 0.01

    def test_wall_is_placed_where_the_gripper_stopped(self):
        self.enter_contact()
        self.assertAlmostEqual(self.wall.x_c, (2125 - 2040) * 0.01)

    def wall_commands(self):
        return [line.split() for line in self.ser.written if line.startswith("W")]

    def test_full_stiffness_from_the_first_command(self):
        self.enter_contact()
        self.assertEqual(self.wall_commands()[0][2], "100.0")

    def test_wall_starts_at_the_handle_when_the_handle_is_past_contact(self):
        self.assertEqual(self.run_for(0.0, 0.05, xh=-1.0), ["contact"])
        self.assertEqual(self.wall_commands()[0][1], "-1.00")

    def test_wall_moves_at_the_landing_speed_and_stops_at_the_contact_point(self):
        self.run_for(0.0, 0.05, xh=-1.0)
        contact_time = self.wall.contact_time
        self.wall.update(contact_time + 0.1, -1.0, **GRASP)
        self.assertAlmostEqual(float(self.wall_commands()[-1][1]),
                               -1.0 + cw.WALL_LANDING_SPEED_CM_S * 0.1, delta=0.01)
        self.wall.next_send = 0.0
        self.wall.update(contact_time + 5.0, -1.0, **GRASP)
        self.assertAlmostEqual(float(self.wall_commands()[-1][1]), self.wall.x_c, places=2)

    def test_larger_overshoot_takes_longer_to_land(self):
        def landing_time(xh):
            wall = cw.ContactWall(self.ser, lambda t: 2.0, 100.0)
            return (2.0 - xh) / wall.landing_speed
        self.assertAlmostEqual(landing_time(1.0), 1.0 / cw.WALL_LANDING_SPEED_CM_S)
        self.assertAlmostEqual(landing_time(-1.0), 3.0 / cw.WALL_LANDING_SPEED_CM_S)

    def test_handle_short_of_contact_starts_at_the_contact_point(self):
        self.assertEqual(self.run_for(0.0, 0.05, xh=self.wall.to_handle(2125) + 0.2), ["contact"])
        self.assertAlmostEqual(float(self.wall_commands()[0][1]), self.wall.x_c, places=2)

    def test_zero_landing_goes_straight_to_the_contact_point(self):
        wall = cw.ContactWall(self.ser, lambda t: 2.0, 100.0, landing_speed=0.0)
        now = 0.0
        while now < 0.06:
            wall.update(now, -1.0, **GRASP)
            now += 0.01
        self.assertEqual(self.wall_commands()[0][1], "2.00")

    def test_fast_grasp_is_detected_sooner_than_a_slow_one(self):
        fast = {"position": 2125, "goal": 2025, "speed": -500}
        self.assertEqual(self.wall.update(0.0, 1.0, **fast), None)
        self.assertEqual(self.wall.update(0.01, 1.0, **fast), None)
        self.assertEqual(self.wall.update(0.02, 1.0, **fast), "contact")

    def test_slow_grasp_still_needs_the_longer_hold(self):
        for now in (0.0, 0.01, 0.02, 0.03):
            self.assertIsNone(self.wall.update(now, 1.0, **GRASP))
        self.assertEqual(self.wall.update(0.04, 1.0, **GRASP), "contact")

    def test_fast_path_rejects_a_gripper_still_moving_quickly(self):
        moving = {"position": 2125, "goal": 2025, "speed": -700}
        for now in (0.0, 0.01, 0.02, 0.03, 0.04, 0.05):
            self.assertIsNone(self.wall.update(now, 1.0, **moving))

    def test_keeps_sending_faster_than_the_watchdog(self):
        self.enter_contact()
        sent_before = sum(line.startswith("W") for line in self.ser.written)
        self.run_for(1.0, 1.0)
        self.assertGreaterEqual(sum(line.startswith("W") for line in self.ser.written) - sent_before,
                                40)

    def test_low_load_at_the_wall_does_not_release(self):
        self.enter_contact()
        at_wall = {"position": 2125, "goal": 2125, "speed": 0}
        self.assertEqual(self.run_for(1.0, 2.0, xh=self.wall.x_c, state=at_wall), [])
        self.assertTrue(self.wall.in_contact)

    def test_handle_moving_right_releases(self):
        self.enter_contact()
        xh = self.wall.x_c + cw.RELEASE_HANDLE_MARGIN_CM + 0.01
        self.assertEqual(self.wall.update(1.0, xh, **GRASP), "released")
        self.assertFalse(self.wall.in_contact)
        self.assertEqual(self.ser.written[-1], "R\n")

    def test_small_handle_motion_inside_the_margin_does_not_release(self):
        self.enter_contact()
        xh = self.wall.x_c + cw.RELEASE_HANDLE_MARGIN_CM - 0.01
        self.assertIsNone(self.wall.update(1.0, xh, **GRASP))

    def test_gripper_closing_further_releases(self):
        self.enter_contact()
        moved = {**GRASP, "position": GRASP["position"] - cw.RELEASE_CLOSING_TICKS - 1}
        self.assertEqual(self.wall.update(1.0, 1.0, **moved), "released")

    def test_contact_can_return_after_release(self):
        self.enter_contact()
        self.wall.update(1.0, 50.0, **GRASP)
        self.assertEqual(self.run_for(2.0, 0.2), ["contact"])

    def test_stop_always_sends_r(self):
        self.wall.stop()
        self.assertEqual(self.ser.written, ["R\n"])

    def test_stiffness_is_capped(self):
        wall = cw.ContactWall(self.ser, lambda t: 0.0, 900.0)
        self.assertEqual(wall.k, cw.MAX_WALL_K)

    def test_full_serial_buffer_does_not_raise(self):
        self.ser.fail_writes = True
        self.wall.stop()


class SurfaceTests(unittest.TestCase):
    def setUp(self):
        self.ser = FakeSerial()
        self.wall = cw.ContactWall(self.ser, lambda t: 1.0, 200.0, cw.WALL_SURFACE, ticks_per_cm=100.0)

    def grasp_after_closing_at(self, speed):
        self.wall.update(0.0, 1.0, position=2500, goal=2400, speed=speed)
        now = 0.01
        while now < 0.1:
            self.wall.update(now, 1.0, **GRASP)
            now += 0.01

    def impulses(self):
        return [line for line in self.ser.written if line.startswith("I")]

    def test_amplitude_has_floor_slope_and_cap(self):
        self.assertEqual(cw.surface_amplitude(0.0), cw.SURFACE_BASE_AMP)
        self.assertEqual(cw.surface_amplitude(5.0), cw.SURFACE_BASE_AMP + 5 * cw.SURFACE_AMP_PER_CM_S)
        self.assertEqual(cw.surface_amplitude(1000.0), cw.SURFACE_MAX_AMP)
        self.assertEqual(cw.surface_amplitude(-5.0), cw.SURFACE_BASE_AMP)

    def test_one_impulse_per_contact_sent_after_the_wall(self):
        self.grasp_after_closing_at(-1000)
        for now in (0.2, 0.3, 0.4):
            self.wall.update(now, 1.0, **GRASP)
        self.assertEqual(len(self.impulses()), 1)
        lines = self.ser.written
        self.assertLess(next(i for i, l in enumerate(lines) if l.startswith("W")),
                        next(i for i, l in enumerate(lines) if l.startswith("I")))

    def test_amplitude_follows_closing_speed_before_contact(self):
        self.grasp_after_closing_at(-1000)  # 10 cm/s at 100 ticks per cm
        self.assertEqual(self.impulses()[0], f"I {cw.surface_amplitude(10.0):.0f}\n")

    def test_opening_speed_does_not_count_as_impact(self):
        self.grasp_after_closing_at(1000)
        self.assertEqual(self.impulses()[0], f"I {cw.SURFACE_BASE_AMP:.0f}\n")

    def test_new_contact_gets_a_new_impulse(self):
        self.grasp_after_closing_at(-1000)
        self.wall.update(0.2, 50.0, **GRASP)
        now = 0.3
        while now < 0.4:
            self.wall.update(now, 1.0, **GRASP)
            now += 0.01
        self.assertEqual(len(self.impulses()), 2)

    def test_spring_type_sends_no_impulse(self):
        wall = cw.ContactWall(self.ser, lambda t: 1.0, 200.0, cw.WALL_SPRING)
        now = 0.0
        while now < 0.2:
            wall.update(now, 2.0, **GRASP)
            now += 0.01
        self.assertEqual(self.impulses(), [])

    def test_configure_sets_transient_shape(self):
        self.wall.configure_surface(25.0, 0.12)
        self.assertEqual(self.ser.written, ["C 25.0 0.120\n"])


class WallTypePromptTests(unittest.TestCase):
    def choose(self, *keys):
        output = []
        presses = iter(keys)
        return gt.choose_wall_type(output.append, lambda: next(presses)), output

    def test_1_is_spring(self):
        self.assertEqual(self.choose("1")[0], cw.WALL_SPRING)

    def test_2_is_surface(self):
        self.assertEqual(self.choose("2")[0], cw.WALL_SURFACE)

    def test_other_keys_are_ignored(self):
        self.assertEqual(self.choose("x", "", "3", "2")[0], cw.WALL_SURFACE)


class HandleInverseTests(unittest.TestCase):
    def test_inverts_the_mapping_across_the_operating_range(self):
        for ticks in (2040, 2300, 2741, 3200, gt.OPEN_TICK):
            with self.subTest(ticks=ticks):
                xh = gt.ticks_to_handle(ticks, -7.34)
                self.assertAlmostEqual(gt.handle_to_ticks(xh, -7.34), ticks, delta=1)

    def test_positions_past_closed_map_to_the_threshold(self):
        self.assertAlmostEqual(gt.ticks_to_handle(2000, 0.0), gt.CLOSE_THRESHOLD_CM)

    def test_positions_past_open_map_to_the_right_stop(self):
        self.assertAlmostEqual(gt.ticks_to_handle(3470, 0.0), gt.HANDLE_SPAN_CM)


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class WallTestScriptTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.ser = FakeSerial(self.clock)
        self.reader = HapkitReader(self.ser)
        self.output = []

    def test_holds_wall_right_of_the_handle_and_turns_it_off(self):
        self.ser.xh = 1.0
        wt.hold_wall(self.ser, self.reader, 1.0, 30.0, self.output.append, self.clock,
                     self.clock.sleep, duration=0.1)
        self.assertEqual(self.ser.written[0], "W 2.50 30.0\n")
        self.assertEqual(self.ser.written[-1], "R\n")

    def test_stops_at_once_if_the_handle_moves_left(self):
        self.ser.xh = 0.0
        lowest, _highest = wt.hold_wall(self.ser, self.reader, 1.0, 30.0, self.output.append,
                                        self.clock, self.clock.sleep, duration=3.0)
        self.assertLess(lowest, 1.0 - wt.WRONG_WAY_CM)
        self.assertLess(self.clock.now, 0.1)
        self.assertEqual(self.ser.written[-1], "R\n")

    def test_watchdog_only_mode_skips_the_held_wall(self):
        self.ser.xh = 1.0
        prompts = []
        with mock.patch.object(wt, "wait_for_first_sample", return_value=([1.0], 0.0)), \
                mock.patch.object(wt, "watch_watchdog", return_value=0.0) as watchdog, \
                mock.patch.object(wt, "hold_wall") as hold:
            result = wt.run_test(self.ser, 250.0, out=self.output.append,
                                 wait_for_enter=prompts.append, watchdog_only=True)
        self.assertEqual(result, wt.EXIT_OK)
        hold.assert_not_called()
        watchdog.assert_called_once()
        self.assertEqual(len(prompts), 1)

    def test_watchdog_check_sends_exactly_one_w(self):
        wt.watch_watchdog(self.ser, self.reader, 30.0, self.output.append, self.clock,
                          self.clock.sleep, duration=0.2)
        self.assertEqual(sum(line.startswith("W") for line in self.ser.written), 1)
        self.assertEqual(self.ser.written[-1], "R\n")


if __name__ == "__main__":
    unittest.main()
