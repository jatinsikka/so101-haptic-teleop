"""Hardware-free tests for the probe and safety rules. Uses the real SDK against a fake servo.

    .venv/bin/python -m unittest discover -s poc -v
"""

import unittest
from unittest import mock

from scservo_sdk import PacketHandler

import probe_gripper
from probe_gripper import (EXIT_BUS_ERROR, EXIT_OK, EXIT_TORQUE_ON, SERVO_ID, TELEMETRY_ADDR,
                           decode_telemetry, little_endian, probe, sign_magnitude)
from servo_safety import (INST_PING, INST_READ, ReadOnlyPortHandler, WriteBlockedError,
                          check_torque_enable_value)


class FakeServo:
    """Stands in for pyserial's Serial object and answers PING and READ like a servo."""

    def __init__(self, servo_id=SERVO_ID, model=777):
        self.servo_id = servo_id
        self.memory = bytearray(256)
        self.memory[3:5] = model.to_bytes(2, "little")
        self.rx = bytearray()
        self.packets = []
        self.silent = False
        self.on_read = None

    def write(self, packet):
        packet = list(packet)
        self.packets.append(packet)
        if self.silent or packet[2] != self.servo_id:
            return len(packet)
        if packet[4] == INST_PING:
            self._reply([])
        elif packet[4] == INST_READ:
            addr, length = packet[5], packet[6]
            if self.on_read:
                self.on_read(addr)
            self._reply(list(self.memory[addr:addr + length]))
        return len(packet)

    def _reply(self, params):
        body = [self.servo_id, len(params) + 2, 0] + params
        self.rx += bytes([0xFF, 0xFF] + body + [~sum(body) & 0xFF])

    def read(self, length):
        data = bytes(self.rx[:length])
        del self.rx[:length]
        return data

    def flush(self):
        pass

    def reset_input_buffer(self):
        self.rx.clear()

    def close(self):
        pass

    def read_addresses(self):
        return [p[5] for p in self.packets if p[4] == INST_READ]


def make_bus(servo):
    port = ReadOnlyPortHandler("fake", allowed_id=SERVO_ID)
    port.ser = servo
    port.is_open = True
    port.tx_time_per_byte = 0.01
    return PacketHandler(0), port


class ReadOnlyGuardTests(unittest.TestCase):
    def setUp(self):
        self.servo = FakeServo()
        self.bus, self.port = make_bus(self.servo)

    def assert_blocked(self, send):
        with self.assertRaises(WriteBlockedError):
            send()
        self.assertEqual(self.servo.packets, [], "a blocked packet reached the wire")

    def test_blocks_ram_write(self):
        self.assert_blocked(lambda: self.bus.write1ByteTxRx(self.port, SERVO_ID, 40, 0))

    def test_blocks_eeprom_write(self):
        self.assert_blocked(lambda: self.bus.write1ByteTxRx(self.port, SERVO_ID, 21, 32))

    def test_blocks_two_byte_write(self):
        self.assert_blocked(lambda: self.bus.write2ByteTxRx(self.port, SERVO_ID, 48, 500))

    def test_blocks_unacknowledged_write(self):
        self.assert_blocked(lambda: self.bus.write1ByteTxOnly(self.port, SERVO_ID, 55, 0))

    def test_blocks_action(self):
        self.assert_blocked(lambda: self.bus.action(self.port, SERVO_ID))

    def test_blocks_sync_write_instruction(self):
        packet = [0xFF, 0xFF, SERVO_ID, 4, 0x83, 42, 2, 0]
        self.assert_blocked(lambda: self.port.writePort(packet))

    def test_blocks_read_to_other_servo(self):
        self.assert_blocked(lambda: self.bus.read1ByteTxRx(self.port, 1, 56))

    def test_blocks_broadcast(self):
        packet = [0xFF, 0xFF, 0xFE, 2, INST_PING, 0]
        self.assert_blocked(lambda: self.port.writePort(packet))

    def test_blocks_malformed_packets(self):
        self.assert_blocked(lambda: self.port.writePort([0xFF, 0xFF, SERVO_ID]))
        self.assert_blocked(lambda: self.port.writePort([0, 0, SERVO_ID, 2, INST_READ, 0]))

    def test_allows_read(self):
        self.servo.memory[63] = 31
        value, _comm, _error = self.bus.read1ByteTxRx(self.port, SERVO_ID, 63)
        self.assertEqual(value, 31)


class TorqueEnableRuleTests(unittest.TestCase):
    def test_accepts_only_zero_and_one(self):
        self.assertEqual(check_torque_enable_value(0), 0)
        self.assertEqual(check_torque_enable_value(1), 1)

    def test_rejects_everything_else(self):
        for value in (128, 2, -1, 255, True, False, 1.0, "1", None):
            with self.subTest(value=value), self.assertRaises(WriteBlockedError):
                check_torque_enable_value(value)


class DecodeTests(unittest.TestCase):
    def test_sign_magnitude(self):
        self.assertEqual(sign_magnitude(500, 10), 500)
        self.assertEqual(sign_magnitude(1024 + 500, 10), -500)
        self.assertEqual(sign_magnitude(0x8000 + 20, 15), -20)
        self.assertEqual(sign_magnitude(2048 + 7, 11), -7)

    def test_little_endian(self):
        self.assertEqual(little_endian([0x00, 0x08]), 2048)

    def test_telemetry_fields(self):
        data = [0x00, 0x08, 20, 0x80, 0xF4, 0x05, 121, 33]
        self.assertEqual(decode_telemetry(data), {
            "position": 2048,
            "speed": -20,
            "load": -500,
            "volts": 12.1,
            "temperature": 33,
        })


class ProbeFlowTests(unittest.TestCase):
    def setUp(self):
        self.servo = FakeServo()
        self.servo.memory[56:58] = (2048).to_bytes(2, "little")
        self.servo.memory[63] = 30
        self.output = []
        self.bus, self.port = make_bus(self.servo)

    def run_probe(self, duration=0.1, **kwargs):
        return probe(self.bus, self.port, duration, out=self.output.append, **kwargs)

    def text(self):
        return "\n".join(self.output)

    def assert_read_only(self):
        for packet in self.servo.packets:
            self.assertIn(packet[4], (INST_PING, INST_READ))
            self.assertEqual(packet[2], SERVO_ID)

    def test_streams_when_torque_is_off(self):
        self.assertEqual(self.run_probe(), EXIT_OK)
        self.assertIn(TELEMETRY_ADDR, self.servo.read_addresses())
        self.assertIn("position range: 2048 to 2048", self.text())
        self.assert_read_only()

    def test_prints_every_startup_register(self):
        self.run_probe(duration=0)
        for _addr, _size, name, _sign in probe_gripper.STARTUP_REGISTERS:
            self.assertIn(name, self.text())

    def test_torque_on_exits_before_streaming(self):
        self.servo.memory[40] = 1
        self.assertEqual(self.run_probe(), EXIT_TORQUE_ON)
        self.assertIn("TORQUE IS ON", self.text())
        self.assertNotIn(TELEMETRY_ADDR, self.servo.read_addresses())
        self.assertEqual(self.servo.memory[40], 1)
        self.assert_read_only()

    def test_any_nonzero_torque_value_counts_as_on(self):
        self.servo.memory[40] = 128
        self.assertEqual(self.run_probe(), EXIT_TORQUE_ON)

    def test_wrong_model_stops_before_register_dump(self):
        self.servo.memory[3:5] = (1234).to_bytes(2, "little")
        self.assertEqual(self.run_probe(), EXIT_BUS_ERROR)
        self.assertIn("model 1234", self.text())
        self.assertNotIn(40, self.servo.read_addresses())

    def test_no_reply_is_a_bus_error(self):
        self.servo.silent = True
        self.assertEqual(self.run_probe(), EXIT_BUS_ERROR)
        self.assertIn("no reply", self.text())

    def test_torque_turning_on_mid_run_stops_the_stream(self):
        def turn_torque_on_after_first_sample(addr):
            if addr == TELEMETRY_ADDR:
                self.servo.memory[40] = 1
        self.servo.on_read = turn_torque_on_after_first_sample
        self.assertEqual(self.run_probe(duration=1.0, slow_period=0.02), EXIT_TORQUE_ON)
        self.assertIn("TORQUE TURNED ON", self.text())
        self.assert_read_only()

    def test_status_change_is_reported(self):
        def raise_overload(addr):
            if addr == TELEMETRY_ADDR:
                self.servo.memory[65] = 1 << 5
        self.servo.on_read = raise_overload
        self.assertEqual(self.run_probe(duration=0.1, slow_period=0.02), EXIT_OK)
        self.assertIn("status changed: 32 (overload)", self.text())

    def test_repeated_misses_stop_the_stream(self):
        def go_silent(addr):
            if addr == TELEMETRY_ADDR:
                self.servo.silent = True
        self.servo.on_read = go_silent
        with mock.patch.object(probe_gripper, "MAX_CONSECUTIVE_MISSES", 2):
            result = self.run_probe(duration=5.0)
        self.assertEqual(result, EXIT_BUS_ERROR)
        self.assertIn("failed reads in a row", self.text())

    def test_failed_read_clears_late_replies(self):
        self.servo.silent = True
        with mock.patch.object(self.servo, "reset_input_buffer") as reset:
            self.assertIsNone(probe_gripper.read_bytes(self.bus, self.port, 40, 1))
        reset.assert_called_once()

    def test_short_reply_counts_as_a_miss(self):
        real_write = self.servo.write

        def reply_with_one_byte(packet):
            real_write(list(packet[:5]) + [40, 1, 0])
            return len(packet)

        with mock.patch.object(self.servo, "write", reply_with_one_byte):
            self.assertIsNone(probe_gripper.read_bytes(self.bus, self.port, 56, 8))


if __name__ == "__main__":
    unittest.main()
