"""Wi-Fi receive/reconnect and command tests; no serial or physical motors."""
from collections import deque
import json
import time
import unittest
from unittest.mock import patch
from wifi_client import WifiClient, valid_sample
from view_3d import Viewer, WifiStream, build_motor_command


class FakeSocket:
    def __init__(self):
        self.incoming, self.sent = deque(), []
        self.closed = False

    def setblocking(self, value):
        assert value is False

    def connect(self, address):
        self.address = address

    def send(self, packet):
        self.sent.append(packet)
        return len(packet)

    def recv(self, size):
        if not self.incoming:
            raise BlockingIOError
        return self.incoming.popleft()

    def close(self):
        self.closed = True


class WifiClientTests(unittest.TestCase):
    def setUp(self):
        self.sock = FakeSocket()
        with patch('wifi_client.socket.socket', return_value=self.sock):
            self.client = WifiClient()
        self.addCleanup(self.client.close)

    def inject(self, **data):
        self.sock.incoming.append(json.dumps(data).encode())

    def welcome(self, token=123):
        self.client.poll()
        self.inject(type='welcome', token=f'{token:016x}', time_us=1000000,
                    client=f'{self.client.client:016x}', sequence=self.client.sequence)
        self.client.poll()

    def sample(self, stamp=1010000, token=123, **changes):
        data = dict(type='sample', token=f'{token:016x}', time_us=stamp,
                    sample_us=stamp-10000, imu_valid=True, mode=-1,
                    acc_g=[.1, -.2, 1.], gyro_deg_s=[1., 2., 3.], angles_deg=[4., 5., 6.])
        data.update(changes)
        self.inject(**data)

    def test_handshake_and_exact_wire_commands_without_serial(self):
        self.welcome()
        self.assertEqual(self.client.token, 123)
        self.assertEqual(self.sock.address, ('192.168.4.1', 3334))
        self.assertEqual(self.sock.sent[0].decode().split()[2:], ['0000000000000000', '1', '0', 'hello'])
        self.client.send_command('status')
        self.assertEqual(self.sock.sent[-1].decode().split()[2:], ['000000000000007b', '2', '1000000', 'status'])

    def test_reordered_samples_return_newest_and_count_only_fresh(self):
        self.welcome()
        self.sample(1030000)
        self.sample(1020000)
        self.sample(1040000)
        latest, count = self.client.poll()
        self.assertEqual(count, 2)
        self.assertEqual(latest['time_us'], 1040000)

    def test_bad_datagrams_stale_imu_and_foreign_tokens_are_ignored(self):
        self.welcome()
        for packet in [b'{bad', b'[]', b'null', b'{}', b'\xff']:
            self.sock.incoming.append(packet)
        self.sample(token=456)
        self.sample(imu_valid=False)
        self.sample(1020000, acc_g=[True, 0, 0])
        self.sample(1030000, gyro_deg_s=[float('nan'), 0, 0])
        self.sample(1040000, sample_us=1000000)
        self.sample(1050000, angles_deg=[1, 2])
        self.sample(1060000, torque_nm='bad')
        self.sample(1070000, mode=[])
        self.assertEqual(self.client.poll(), (None, 0))

    def test_reconnect_changes_token_without_replaying_commands(self):
        self.welcome()
        self.sample(9000000)
        self.client.poll()
        self.client.next_hello = 0
        self.welcome(token=456)
        self.sample(1010000, token=456)
        latest, count = self.client.poll()
        self.assertEqual(count, 1)
        self.assertEqual(latest['time_us'], 1010000)
        self.assertTrue(all(packet.endswith(b'hello') for packet in self.sock.sent))

    def test_close_and_disconnection_do_not_send_stop_or_reset(self):
        self.welcome()
        self.client.last_received = time.monotonic()-1
        before = len(self.sock.sent)
        with self.assertRaises(OSError):
            self.client.send_command('balance')
        self.client.close()
        self.assertTrue(self.sock.closed)
        self.assertEqual(len(self.sock.sent), before)
        self.assertFalse(any(b'stop' in packet or b'reconnect' in packet for packet in self.sock.sent))

    def test_viewer_uses_full_three_axis_wifi_sample(self):
        self.welcome()
        self.sample()
        stream = WifiStream.__new__(WifiStream)
        stream.client = self.client
        sample, count = stream.poll()
        self.assertEqual(count, 1)
        self.assertEqual(sample.angles, (4., 5., 6.))
        self.assertEqual(sample.gyro, (1., 2., 3.))
        self.assertEqual(sample.acceleration, (.1, -.2, 1.))

    def make_motor_viewer(self):
        class Widget:
            def __init__(self, text=''):
                self.values = {'text': text}

            def get(self):
                return self.values['text']

            def config(self, **values):
                self.values.update(values)

            def cget(self, key):
                return self.values[key]

        viewer = Viewer.__new__(Viewer)
        viewer.stream = WifiStream.__new__(WifiStream)
        viewer.stream.client = self.client
        viewer.torque_entry = Widget('-0.25')
        viewer.speed_entry = Widget('1')
        viewer.motor_hint = Widget()
        viewer.motor_limits_button = Widget()
        viewer.motor_buttons = [Widget(), Widget()]
        return viewer

    def test_motor_buttons_send_separate_axis_commands_in_odrive_units(self):
        self.welcome()
        self.sample(mode=0, torque_limit=.5, flywheel_limit=4, wheel_limit=3)
        self.client.poll()
        viewer = self.make_motor_viewer()
        before = len(self.sock.sent)
        self.assertTrue(viewer.send_motor_target('torque'))
        self.assertEqual(len(self.sock.sent), before+1)
        self.assertTrue(self.sock.sent[-1].endswith(b' torque0 -0.25'))
        self.assertTrue(viewer.send_motor_target('speed'))
        self.assertEqual(len(self.sock.sent), before+2)
        self.assertTrue(self.sock.sent[-1].endswith(b' speed1 1'))
        viewer.speed_entry.values['text'] = '-1'
        self.assertTrue(viewer.send_motor_target('speed'))
        self.assertTrue(self.sock.sent[-1].endswith(b' speed1 -1'))

    def test_motor_buttons_do_not_send_for_imu_only_balancing_or_lost_link(self):
        self.welcome()
        viewer = self.make_motor_viewer()
        for mode in (-1, 1, None):
            with self.subTest(mode=mode):
                self.client.telemetry = {'mode': mode}
                before = len(self.sock.sent)
                self.assertFalse(viewer.send_motor_target('torque'))
                self.assertFalse(viewer.send_motor_target('speed'))
                self.assertEqual(len(self.sock.sent), before)
                self.assertTrue(all(button.cget('state') == 'disabled' for button in viewer.motor_buttons))
        self.client.telemetry = {'mode': 0}
        self.client.last_received = time.monotonic()-1
        before = len(self.sock.sent)
        self.assertFalse(viewer.send_motor_target('torque'))
        self.assertEqual(len(self.sock.sent), before)

    def test_only_running_motor_target_can_be_updated(self):
        self.welcome()
        self.sample(mode=3, torque_limit=.5, flywheel_limit=4, wheel_limit=3)
        self.client.poll()
        viewer = self.make_motor_viewer()
        before = len(self.sock.sent)
        self.assertFalse(viewer.send_motor_target('torque'))
        self.assertEqual(len(self.sock.sent), before)
        self.assertTrue(viewer.send_motor_target('speed'))
        self.assertTrue(self.sock.sent[-1].endswith(b' speed1 1'))
        self.sample(1020000, mode=2, torque_limit=.5, flywheel_limit=4, wheel_limit=3)
        self.client.poll()
        self.assertFalse(viewer.send_motor_target('speed'))
        self.assertTrue(viewer.send_motor_target('torque'))
        self.assertTrue(self.sock.sent[-1].endswith(b' torque0 -0.25'))

    def test_motor_input_rejects_invalid_value_without_send(self):
        self.welcome()
        self.client.telemetry = {'mode': 0, 'torque_limit': .5, 'flywheel_limit': 4, 'wheel_limit': 3}
        viewer = self.make_motor_viewer()
        for target in ('nan', 'inf', '1e999', 'oops'):
            with self.subTest(target=target):
                viewer.torque_entry.values['text'] = target
                before = len(self.sock.sent)
                self.assertFalse(viewer.send_motor_target('torque'))
                self.assertEqual(len(self.sock.sent), before)

    def test_motor_controls_wait_for_limit_telemetry_after_configuration(self):
        self.welcome()
        self.sample(mode=0, torque_limit=0, flywheel_limit=0, wheel_limit=0)
        self.client.poll()
        viewer = self.make_motor_viewer()
        before = len(self.sock.sent)
        self.assertFalse(viewer.send_motor_target('speed'))
        self.assertEqual(len(self.sock.sent), before)
        self.assertEqual(viewer.motor_limits_button.cget('state'), 'normal')
        viewer.send_motor_limits(.5, 4, 3)
        self.assertTrue(self.sock.sent[-1].endswith(b' limits 0.5 4 3'))
        self.assertFalse(viewer.refresh_motor_controls())
        self.sample(1020000, mode=0, torque_limit=.5, flywheel_limit=4, wheel_limit=3)
        self.client.poll()
        self.assertTrue(viewer.refresh_motor_controls())
        self.assertIn('4 圈/s', viewer.motor_hint.cget('text'))
        viewer.speed_entry.values['text'] = '3.1'
        before = len(self.sock.sent)
        self.assertFalse(viewer.send_motor_target('speed'))
        self.assertEqual(len(self.sock.sent), before)
        self.assertIn('3 圈/s', self.client.last_event)

    def test_motor_command_accepts_zero_targets(self):
        self.assertEqual(build_motor_command('torque', '0'), 'torque0 0')
        self.assertEqual(build_motor_command('speed', '0'), 'speed1 0')
        with self.assertRaises(ValueError):
            build_motor_command('unknown', '1')


if __name__ == '__main__':
    unittest.main()
