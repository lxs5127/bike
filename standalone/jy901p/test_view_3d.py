"""Checks for telemetry parsing, 3D attitude math, and live log consumption."""
from collections import deque
from dataclasses import replace
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

from view_3d import (IDENTITY, Stream, Viewer, load_reference, matrix_angles, multiply,
                     parse_sample, rotation, save_reference, select_serial_port, transform, transpose)

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

    def test_euler_roundtrip_including_pitch_singularities_and_yaw_wrap(self):
        for angles in [(-87.484, -20.286, -88.770), (35, 89.99, 170),
                       (30, 90, 40), (30, -90, 40), (179, -45, -179), (12, 0, 181)]:
            original = rotation(angles)
            restored = rotation(matrix_angles(original))
            for actual, expected in zip(restored, original):
                self.assertVector(actual, expected)

    def test_relative_angles_respect_tilted_mount_and_wrap(self):
        for base in [(35, -25, 179), (-87.484, -20.286, -88.770)]:
            reference = rotation(base)
            delta = (4, -3, 6)
            current = multiply(reference, rotation(delta))
            relative = multiply(transpose(reference), current)
            self.assertVector(matrix_angles(relative), delta)

    def test_reference_button_zeros_numeric_labels_and_trace_then_restores_raw(self):
        class Label:
            def config(self, **values):
                self.text = values['text']

        viewer = Viewer.__new__(Viewer)
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        viewer.reference_file = Path(directory.name) / 'reference.json'
        viewer.saved_reference_angles = None
        viewer.sample = parse_sample(LINE)
        original = viewer.sample.angles
        viewer.reference, viewer.relative = IDENTITY, False
        viewer.received_at = time.monotonic()
        viewer.angle_labels = [Label(), Label(), Label()]
        viewer.attitude_title, viewer.mode = Label(), Label()
        viewer.history = deque([(99, 88, 77)])
        viewer.draw = lambda: None

        viewer.set_reference()
        self.assertTrue(viewer.relative)
        self.assertEqual(viewer.display_angles(), (0., 0., 0.))
        self.assertEqual([float(label.text.rstrip('°')) for label in viewer.angle_labels], [0, 0, 0])
        self.assertEqual(list(viewer.history), [(0., 0., 0.)])
        self.assertEqual(viewer.attitude_title.text, '相对参考姿态')
        self.assertEqual(viewer.sample.angles, original)
        self.assertEqual(load_reference(viewer.reference_file), original)

        # Future readings must use the same relative rotation as the 3D model.
        relative_motion = (5, -2, 3)
        moved = matrix_angles(multiply(rotation(original), rotation(relative_motion)))
        viewer.sample = replace(viewer.sample, angles=moved)
        viewer.refresh_angles()
        self.assertVector(viewer.display_angles(), relative_motion)
        self.assertEqual([float(label.text.rstrip('°')) for label in viewer.angle_labels], [5, -2, 3])

        viewer.clear_reference()
        self.assertFalse(viewer.relative)
        self.assertEqual(viewer.display_angles(), moved)
        self.assertEqual(viewer.attitude_title.text, '传感器原始姿态')
        self.assertEqual(list(viewer.history), [moved])

        # Returning to absolute mode must retain the saved balance pose.
        self.assertEqual(load_reference(viewer.reference_file), original)
        viewer.restore_reference()
        self.assertTrue(viewer.relative)
        self.assertVector(viewer.display_angles(), relative_motion)

        # A restarted viewer uses the old zero, not its current leaning pose.
        restarted = Viewer.__new__(Viewer)
        restarted.reference_file = viewer.reference_file
        restarted.saved_reference_angles = None
        restarted.sample = viewer.sample
        restarted.reference, restarted.relative = IDENTITY, False
        restarted.mode, restarted.attitude_title = Label(), Label()
        restarted.angle_labels = [Label(), Label(), Label()]
        restarted.history = deque()
        restarted.draw = lambda: None
        restarted.restore_reference()
        self.assertVector(restarted.display_angles(), relative_motion)
        self.assertEqual(restarted.saved_reference_angles, original)

    def test_reference_file_validation_and_missing_file(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'reference.json'
            self.assertIsNone(load_reference(path))
            for data in [{'version': 2, 'reference_angles_deg': [0, 0, 0]},
                         {'version': 1, 'reference_angles_deg': [0, 0]},
                         {'version': 1, 'reference_angles_deg': [0, float('nan'), 0]},
                         {'version': 1, 'reference_angles_deg': [True, 0, 0]},
                         {'version': 1, 'reference_angles_deg': [10**1000, 0, 0]},
                         {'version': 1, 'reference_angles_deg': [181, 0, 0]}, []]:
                path.write_text(json.dumps(data))
                with self.assertRaises(ValueError):
                    load_reference(path)
            path.write_text('{broken json')
            with self.assertRaises(ValueError):
                load_reference(path)

    def test_failed_atomic_save_preserves_previous_zero(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'reference.json'
            original = (10.7, 6.2, 179.)
            save_reference(path, original)
            with patch.object(Path, 'replace', side_effect=OSError('disk write failed')):
                with self.assertRaises(OSError):
                    save_reference(path, (20., 30., 40.))
            self.assertEqual(load_reference(path), original)
            self.assertEqual(list(Path(folder).iterdir()), [path])

    def test_usb_port_reenumeration_and_explicit_selection(self):
        self.assertEqual(select_serial_port('auto', ['/dev/ttyS0', '/dev/ttyUSB1']), '/dev/ttyUSB1')
        self.assertEqual(select_serial_port('/dev/ttyUSB0', ['/dev/ttyUSB1']), '/dev/ttyUSB1')
        self.assertEqual(select_serial_port('auto', ['/dev/ttyACM0']), '/dev/ttyACM0')
        self.assertEqual(select_serial_port('/dev/ttyUSB0', ['/dev/ttyUSB0','/dev/ttyUSB1']), '/dev/ttyUSB0')
        self.assertEqual(select_serial_port('/dev/serial/by-id/my-esp32', []), '/dev/serial/by-id/my-esp32')
        with self.assertRaises(FileNotFoundError):
            select_serial_port('auto', ['/dev/ttyS0'])
        with self.assertRaises(OSError):
            select_serial_port('auto', ['/dev/ttyUSB0','/dev/ttyUSB1'])

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
