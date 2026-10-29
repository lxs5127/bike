"""Checks for telemetry parsing, 3D attitude math, and live log consumption."""
import math
from pathlib import Path
import tempfile
import unittest

from view_3d import IDENTITY, Stream, multiply, parse_sample, rotation, transform, transpose

LINE = ('I (875) JY901P: acc[g] X=0.349 Y=-0.938 Z=0.041 | '
        'gyro[deg/s] X=0.061 Y=0.000 Z=-0.061 | '
        'angle[deg] Roll=-87.484 Pitch=-20.286 Yaw=-88.770\n')


class AttitudeTests(unittest.TestCase):
    def assertVector(self, actual, expected):
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=10)

    def test_parse_real_board_line_and_ansi(self):
        sample = parse_sample('\x1b[0;32m' + LINE + '\x1b[0m')
        self.assertEqual(sample.angles, (-87.484, -20.286, -88.770))
        self.assertEqual(sample.acceleration, (.349, -.938, .041))
        self.assertEqual(sample.gyro, (.061, 0., -.061))

    def test_boot_errors_partial_and_nonfinite_are_ignored(self):
        for line in ['Init failed', LINE[:50], LINE.replace('0.349', '1e999')]:
            self.assertIsNone(parse_sample(line))

    def test_right_handed_axis_rotations(self):
        self.assertVector(transform(rotation((90, 0, 0)), (0, 1, 0)), (0, 0, 1))
        self.assertVector(transform(rotation((0, 90, 0)), (1, 0, 0)), (0, 0, -1))
        self.assertVector(transform(rotation((0, 0, 90)), (1, 0, 0)), (0, 1, 0))

    def test_general_rotation_preserves_length_and_reference_is_identity(self):
        for angles in [(-87.484, -20.286, -88.770), (45, 88, -170), (180, -90, 179)]:
            matrix = rotation(angles)
            vector = transform(matrix, (1.2, -.8, .3))
            self.assertAlmostEqual(sum(x*x for x in vector), 2.17)
            for row, expected in zip(multiply(transpose(matrix), matrix), IDENTITY):
                self.assertVector(row, expected)

    def test_yaw_wrap_does_not_change_physical_orientation(self):
        for a, b in zip(rotation((17, -28, -180)), rotation((17, -28, 180))):
            self.assertVector(a, b)

    def test_live_stream_waits_for_complete_lines_and_returns_latest(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'live.log'
            path.write_text(LINE)  # Old samples must not appear as fresh data.
            stream = Stream(log_path=path)
            try:
                self.assertEqual(stream.poll(), (None, 0))
                with path.open('a') as log:
                    log.write(LINE[:50])
                self.assertEqual(stream.poll(), (None, 0))
                with path.open('a') as log:
                    log.write(LINE[50:] + LINE.replace('Roll=-87.484', 'Roll=23.5'))
                sample, count = stream.poll()
                self.assertEqual(count, 2)
                self.assertEqual(sample.angles[0], 23.5)
                self.assertEqual(stream.poll(), (None, 0))
                path.write_text(LINE.replace('Roll=-87.484', 'Roll=8.0'))
                sample, count = stream.poll()
                self.assertEqual(count, 1)
                self.assertEqual(sample.angles[0], 8.)
            finally:
                stream.close()


if __name__ == '__main__':
    unittest.main()
