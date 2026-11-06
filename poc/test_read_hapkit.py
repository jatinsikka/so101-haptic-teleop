"""Hardware-free tests for the Hapkit reader.

    .venv/bin/python -m unittest discover -s poc -v
"""

import unittest

from read_hapkit import (MAX_LINE_BYTES, HapkitReader, StreamStats, stream, summary_lines,
                         wait_for_first_sample)


class FakeSerial:
    """Serves queued chunks of bytes, one chunk per poll, like separate serial reads."""

    def __init__(self, *chunks):
        self.chunks = list(chunks)
        self.written = []

    @property
    def in_waiting(self):
        return len(self.chunks[0]) if self.chunks else 0

    def read(self, _size):
        return self.chunks.pop(0)

    def write(self, data):
        self.written.append(data)


class FakeClock:
    """Time that moves only when the code under test sleeps."""

    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def read_all(*chunks):
    reader = HapkitReader(FakeSerial(*chunks))
    samples = []
    for _ in chunks:
        samples += reader.poll()
    return reader, samples


class ParsingTests(unittest.TestCase):
    def test_parses_positive_negative_and_zero(self):
        _, samples = read_all(b"P 1.25\r\nP -6.53\r\nP 0.00\r\n")
        self.assertEqual(samples, [1.25, -6.53, 0.0])

    def test_line_split_across_reads(self):
        _, samples = read_all(b"P -6", b".5", b"3\r", b"\nP 1.00\r\n")
        self.assertEqual(samples, [-6.53, 1.0])

    def test_incomplete_line_waits_for_its_newline(self):
        reader = HapkitReader(FakeSerial(b"P 1.2"))
        self.assertEqual(reader.poll(), [])
        self.assertEqual(reader.bad_lines, 0)

    def test_accepts_line_without_carriage_return(self):
        _, samples = read_all(b"P 2.50\n")
        self.assertEqual(samples, [2.5])

    def test_rejects_malformed_lines_after_sync(self):
        bad = (b"P\r\n", b"P \r\n", b"P abc\r\n", b"P 1.2.3\r\n", b"X 1.00\r\n", b"P 1.00 2\r\n",
               b"P nan\r\n", b"P ovf\r\n", b"\r\n", b"\xff\xfe\r\n")
        for line in bad:
            with self.subTest(line=line):
                reader, samples = read_all(b"P 0.00\r\n", line)
                self.assertEqual(samples, [0.0])
                self.assertEqual(reader.bad_lines, 1)

    def test_overlong_line_is_dropped_and_recovers(self):
        reader, samples = read_all(b"P 0.00\r\n", b"x" * (MAX_LINE_BYTES + 1), b"\r\nP 1.00\r\n")
        self.assertEqual(samples, [0.0, 1.0])
        self.assertEqual(reader.bad_lines, 2)  # the dropped bytes, then the empty remainder


class StartupTests(unittest.TestCase):
    def test_noise_before_sync_is_not_an_error(self):
        reader, samples = read_all(b"-6.53\r\n\xf0\x00\r\n# hapkit_gripper m2\r\nP 0.00\r\n")
        self.assertEqual(samples, [0.0])
        self.assertEqual(reader.startup_discards, 2)
        self.assertEqual(reader.bad_lines, 0)
        self.assertEqual(reader.boot_lines, 1)

    def test_records_firmware_version(self):
        reader, _ = read_all(b"# hapkit_gripper m4\r\nP 0.00\r\n")
        self.assertEqual(reader.firmware, b"# hapkit_gripper m4")

    def test_status_lines_are_not_errors(self):
        reader, samples = read_all(b"P 0.00\r\n# isr_max_us 88\r\n# something else\r\nP 1.00\r\n")
        self.assertEqual(samples, [0.0, 1.0])
        self.assertEqual(reader.isr_max_us, 88)
        self.assertEqual(reader.bad_lines, 0)
        self.assertEqual(reader.boot_lines, 0)

    def test_old_firmware_output_never_syncs(self):
        reader, samples = read_all(b"|xh: 1.00 |vh: |0.00 |PWM: 0.00 |xm: 0.50 |vm: 0.00\r\n")
        self.assertEqual(samples, [])
        self.assertFalse(reader.synced)

    def test_wait_returns_first_samples(self):
        clock = FakeClock()
        reader = HapkitReader(FakeSerial(b"# hapkit_gripper m2\r\n", b"P 1.00\r\n"))
        samples, waited = wait_for_first_sample(reader, clock, clock.sleep)
        self.assertEqual(samples, [1.0])
        self.assertLess(waited, 1.0)

    def test_wait_times_out_without_data(self):
        clock = FakeClock()
        samples, waited = wait_for_first_sample(HapkitReader(FakeSerial()), clock, clock.sleep,
                                                timeout=0.5)
        self.assertEqual(samples, [])
        self.assertGreaterEqual(waited, 0.5)


class StatsTests(unittest.TestCase):
    def test_rate_gap_and_range(self):
        stats = StreamStats()
        for i, xh in enumerate((1.0, -2.0, 3.0)):
            stats.record(10.0 + i * 0.005, xh)
        stats.record(10.03, 0.0)
        self.assertAlmostEqual(stats.average_rate(), 100.0)
        self.assertAlmostEqual(stats.worst_gap, 0.02)
        self.assertEqual((stats.minimum, stats.maximum), (-2.0, 3.0))

    def test_rate_needs_two_samples(self):
        stats = StreamStats()
        self.assertEqual(stats.average_rate(), 0.0)
        stats.record(1.0, 0.0)
        self.assertEqual(stats.average_rate(), 0.0)


class StreamTests(unittest.TestCase):
    def run_stream(self, *chunks, duration=0.05):
        clock = FakeClock()
        reader = HapkitReader(FakeSerial(*chunks))
        output = []
        stats = stream(reader, [0.5], duration, output.append, clock, clock.sleep)
        return reader, stats, output

    def test_records_samples_and_prints(self):
        _, stats, output = self.run_stream(b"P 1.00\r\n", b"P 2.00\r\nP 3.00\r\n")
        self.assertEqual(stats.count, 4)
        self.assertEqual(stats.maximum, 3.0)
        self.assertIn("xh=  +1.00", output[0])  # the first poll runs before the first print

    def test_reader_never_sends_anything(self):
        reader, _, _ = self.run_stream(b"P 1.00\r\n")
        self.assertEqual(reader.ser.written, [])

    def test_summary_flags_missing_and_repeated_boot_lines(self):
        stats = StreamStats()
        stats.record(0.0, 0.0)
        stats.record(0.005, 0.0)
        reader = HapkitReader(FakeSerial())
        self.assertIn("no boot line", "\n".join(summary_lines(stats, reader, 1.0)))
        reader.boot_lines = 1
        self.assertNotIn("boot line,", "\n".join(summary_lines(stats, reader, 1.0)))
        reader.boot_lines = 2
        self.assertIn("reset during the run", "\n".join(summary_lines(stats, reader, 1.0)))


if __name__ == "__main__":
    unittest.main()
