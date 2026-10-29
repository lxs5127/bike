"""C/Python protocol, safety guard, policy parity and UDP integration."""
import ctypes as ct
from dataclasses import replace
import os
from pathlib import Path
import socket
import select
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'host'))
# Before NumPy is imported, keep this tiny actor on one BLAS thread.
for name in ('OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'OMP_NUM_THREADS'):
    os.environ[name] = '1'
from controller import Controller, load_policy
from protocol import (State, Parser, encode_state, encode_command, decode_state,
                      STATE_TYPE, COMMAND_TYPE, COMMAND, VALID, DRIVE_READY, MOCK,
                      MOTOR_DISABLED, HELLO_TYPE, encode_hello, decode_datagram)
from udp_link import UDPLink
from test_policy_runtime import State as CState, Controller as CController, Output as COutput


def sample(sequence=1, timestamp=100000, **changes):
    state = State(sequence, 1234, timestamp, 1000, 2000, .03, -.2, 40., 0,
                  VALID | DRIVE_READY | MOCK | MOTOR_DISABLED, 0, 0, 0)
    return replace(state, **changes)


SHIM = r'''
#include "bike_protocol.h"
#include "bike_link_guard.h"
#include "bike_udp_peer.h"
static bike_link_guard_t guard;
static bike_udp_peer_t peer;
static bike_command_parser_t parser;
static bike_wire_command_t last;
static uint32_t last_sequence;
void reset(void) {
    bike_link_guard_init(&guard, 1234);
    parser = (bike_command_parser_t){0};
    peer = (bike_udp_peer_t){0};
}
void sent(uint32_t seq, uint64_t timestamp) { bike_link_guard_sent(&guard, seq, timestamp); }
int accept(uint32_t seq, uint32_t boot, uint64_t stamp, float torque, uint32_t flags, uint64_t now) {
    bike_wire_command_t c = {boot, stamp, torque, flags};
    return bike_link_guard_accept(&guard, seq, &c, now);
}
int alive(uint64_t now) { return bike_link_guard_alive(&guard, now); }
int udp_hello(uint32_t address, uint16_t port, uint64_t now) {
    return bike_udp_peer_hello(&peer, address, port, now);
}
int udp_command(uint32_t address, uint16_t port, uint64_t now) {
    return bike_udp_peer_command(&peer, address, port, now);
}
int udp_alive(uint64_t now) { return bike_udp_peer_alive(&peer, now); }
int feed(const unsigned char *bytes, unsigned size) {
    int count = 0;
    for (unsigned i = 0; i < size; ++i)
        count += bike_parse_command_byte(&parser, bytes[i], &last_sequence, &last);
    return count;
}
float parsed_torque(void) { return last.torque_nm; }
uint32_t parsed_sequence(void) { return last_sequence; }
unsigned encode(unsigned char *bytes) {
    bike_wire_state_t s = {
        .boot_id = 1234, .timestamp_us = 100000,
        .imu_age_us = 1000, .encoder_age_us = 2000,
        .roll_rad = .03f, .roll_rate_rad_s = -.2f, .flywheel_rad_s = 40.f,
        .flags = 15,
    };
    return bike_encode_state(bytes, 1, &s);
}
'''


class PCLinkTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        shim = Path(cls.tmp.name) / 'shim.c'
        shim.write_text(SHIM)
        library = Path(cls.tmp.name) / 'link.so'
        main = ROOT / 'main'
        command = ['gcc', '-shared', '-fPIC', '-O2', '-Wall', '-Wextra', '-Werror']
        for subdir in ['', 'link', 'control', 'policy']:
            command += ['-I' + str(main / subdir)]
        for name in ['link/bike_protocol.c', 'link/bike_link_guard.c', 'link/bike_udp_peer.c', 'control/bike_safety.c',
                     'control/bike_controller.c', 'policy/rl_policy.c', 'policy/policy_weights.c']:
            command.append(str(main / name))
        subprocess.run(command + [str(shim), '-lm', '-o', str(library)], check=True)
        cls.lib = ct.CDLL(str(library))
        cls.lib.sent.argtypes = [ct.c_uint32, ct.c_uint64]
        cls.lib.accept.argtypes = [ct.c_uint32, ct.c_uint32, ct.c_uint64, ct.c_float,
                                  ct.c_uint32, ct.c_uint64]
        cls.lib.alive.argtypes = [ct.c_uint64]
        cls.lib.feed.argtypes = [ct.c_char_p, ct.c_uint]
        cls.lib.parsed_torque.restype = ct.c_float
        cls.lib.parsed_sequence.restype = ct.c_uint32
        cls.lib.encode.argtypes = [ct.c_void_p]
        cls.lib.bike_validate_packet.argtypes = [ct.c_char_p, ct.c_size_t, ct.c_uint8]
        cls.lib.bike_validate_packet.restype = ct.c_bool
        cls.lib.udp_hello.argtypes = cls.lib.udp_command.argtypes = [ct.c_uint32, ct.c_uint16, ct.c_uint64]
        cls.lib.udp_alive.argtypes = [ct.c_uint64]
        cls.lib.bike_controller_reset.argtypes = [ct.POINTER(CController)]
        cls.lib.bike_controller_step.argtypes = [ct.POINTER(CController), ct.POINTER(CState),
                                               ct.c_float, ct.POINTER(COutput)]
        cls.lib.bike_controller_step.restype = ct.c_bool
        cls.lib.bike_limit_torque.argtypes = [ct.c_float, ct.c_float]
        cls.lib.bike_limit_torque.restype = ct.c_float
        cls.actor = load_policy()

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def setUp(self):
        self.lib.reset()

    def test_wire_compatibility_and_fragmentation(self):
        encoded = ct.create_string_buffer(68)
        self.assertEqual(self.lib.encode(encoded), 68)
        self.assertEqual(encoded.raw, encode_state(sample()))
        parser = Parser()
        received = []
        for byte in b'boot log\r\n' + encoded.raw:
            received += parser.feed(bytes([byte]))
        self.assertEqual(len(received), 1)
        kind, sequence, payload = received[0]
        self.assertEqual(kind, STATE_TYPE)
        self.assertAlmostEqual(decode_state(sequence, payload).flywheel_rad_s, 40)
        command = encode_command(sample(), .7)
        self.assertEqual(len(command), 36)
        self.assertEqual(sum(self.lib.feed(bytes([b]), 1) for b in command), 1)
        self.assertAlmostEqual(self.lib.parsed_torque(), .7, places=6)
        self.assertEqual(self.lib.parsed_sequence(), 1)

    def test_crc_noise_and_dropped_bytes_resynchronize(self):
        command = encode_command(sample(), -.8)
        corrupted = bytearray(command)
        corrupted[24] ^= 0x08
        data = b'noiseBKR1\xff\x02\x14\x00' + corrupted + command[:-1] + command
        self.assertEqual(self.lib.feed(data, len(data)), 1)
        self.assertAlmostEqual(self.lib.parsed_torque(), -.8, places=6)
        state = encode_state(sample())
        bad = bytearray(state)
        bad[32] ^= 0x02
        parser = Parser()
        packets = parser.feed(bytes(bad) + state[:-3] + state)
        self.assertEqual(len(packets), 1)
        self.assertGreaterEqual(parser.bad_packets, 2)

    def test_guard_rejects_stale_replayed_and_nonfinite_commands(self):
        self.lib.sent(1, 100000)
        self.assertEqual(self.lib.accept(1, 999, 100000, .5, 0, 101000), 0)
        self.assertEqual(self.lib.accept(2, 1234, 100000, .5, 0, 101000), 0)
        self.assertEqual(self.lib.accept(1, 1234, 99999, .5, 0, 101000), 0)
        for torque in (float('nan'), float('inf'), 5., -5.):
            self.assertEqual(self.lib.accept(1, 1234, 100000, torque, 0, 101000), 0)
        self.assertEqual(self.lib.accept(1, 1234, 100000, .5, 2, 101000), 0)
        self.assertEqual(self.lib.accept(1, 1234, 100000, .5, 0, 99999), 0)
        self.assertEqual(self.lib.accept(1, 1234, 100000, .5, 0, 125001), 0)
        self.assertEqual(self.lib.accept(1, 1234, 100000, .5, 0, 105000), 1)
        self.assertEqual(self.lib.accept(1, 1234, 100000, .5, 0, 106000), 0)
        self.assertEqual(self.lib.alive(149999), 1)
        self.assertEqual(self.lib.alive(150000), 0)
        self.lib.sent(2, 116667)
        self.assertEqual(self.lib.accept(2, 1234, 116667, 0., 1, 120000), 1)
        self.assertEqual(self.lib.alive(120000), 0)

    def test_guard_sequence_wrap_and_independent_limits(self):
        self.lib.sent(0xffffffff, 100000)
        self.assertEqual(self.lib.accept(0xffffffff, 1234, 100000, .5, 0, 101000), 1)
        self.lib.sent(0, 116667)
        self.assertEqual(self.lib.accept(0, 1234, 116667, .5, 0, 117000), 1)
        self.assertAlmostEqual(self.lib.bike_limit_torque(10, 0), 4.9, places=6)
        self.assertEqual(self.lib.bike_limit_torque(1, 250), 0)
        self.assertEqual(self.lib.bike_limit_torque(-1, -250), 0)
        self.assertEqual(self.lib.bike_limit_torque(-1, 300), -1)

    def test_host_controller_matches_previous_c_controller(self):
        controller, context = Controller(self.actor), CController()
        self.lib.bike_controller_reset(ct.byref(context))
        import numpy as np
        errors = []
        for i in range(600):
            state = sample(i + 1, 100000 + i * 16667,
                           roll_rad=float(np.float32(.04 * np.sin(i / 20))),
                           roll_rate_rad_s=float(np.float32(.12 * np.cos(i / 20))),
                           flywheel_rad_s=float(np.float32(280 * np.sin(i / 30))))
            action, torque = controller.step(state)
            cstate = CState(state.roll_rad, state.roll_rate_rad_s, state.flywheel_rad_s,
                            state.timestamp_us, state.timestamp_us, True, True)
            output = COutput()
            self.assertTrue(self.lib.bike_controller_step(ct.byref(context), ct.byref(cstate),
                                                        .016667, ct.byref(output)))
            errors.append(abs(action - output.action))
            self.assertAlmostEqual(torque, 4.9 * action)
        self.assertLess(max(errors), 2e-5)

    def test_host_memory_reset_on_boot_gap_and_fault(self):
        controller = Controller(self.actor)
        controller.step(sample())
        controller.step(sample(2, 116667))
        self.assertGreater(controller.rotation, 0)
        controller.step(sample(1, 100000, boot_id=5678))
        self.assertEqual(controller.rotation, 0)
        with self.assertRaises(ValueError):
            controller.step(sample(3, 133334, boot_id=5678))
        self.assertIsNone(controller.previous_timestamp)
        with self.assertRaises(ValueError):
            controller.step(sample(fault=4))
        with self.assertRaises(ValueError):
            controller.step(sample(imu_age_us=20001))
        with self.assertRaises(ValueError):
            controller.step(sample(roll_rad=float('nan')))

    def test_udp_datagram_validation(self):
        for frame, kind in [(encode_hello(), HELLO_TYPE), (encode_state(sample()), STATE_TYPE),
                            (encode_command(sample(), .5), COMMAND_TYPE)]:
            self.assertTrue(self.lib.bike_validate_packet(frame, len(frame), kind))
            decode_datagram(frame, kind)
            for bad in (frame[:-1], frame + b'x', frame + frame,
                        frame[:-1] + bytes([frame[-1] ^ 1])):
                self.assertFalse(self.lib.bike_validate_packet(bad, len(bad), kind))
                with self.assertRaises(ValueError):
                    decode_datagram(bad, kind)

    def test_udp_peer_registration_lease_and_other_client(self):
        self.assertEqual(self.lib.udp_command(1, 2222, 100000), 0)
        self.assertEqual(self.lib.udp_hello(1, 2222, 100000), 1)
        self.assertEqual(self.lib.udp_alive(100000), 1)
        self.assertEqual(self.lib.udp_hello(1, 3333, 110000), 0)
        self.assertEqual(self.lib.udp_command(2, 2222, 110000), 0)
        self.assertEqual(self.lib.udp_command(1, 3333, 110000), 0)
        self.assertEqual(self.lib.udp_command(1, 2222, 110000), 1)
        self.assertEqual(self.lib.udp_alive(1109999), 1)
        self.assertEqual(self.lib.udp_alive(1110000), 0)
        self.assertEqual(self.lib.udp_command(1, 2222, 1110000), 0)
        self.assertEqual(self.lib.udp_hello(2, 3333, 1110000), 1)

    def test_udp_burst_reordering_crc_and_boot_replay(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server.bind(('127.0.0.1', 0))
        server.settimeout(1)
        link = UDPLink('127.0.0.1', server.getsockname()[1])
        try:
            self.assertIsNone(link.read_state(0))
            hello, address = server.recvfrom(2048)
            decode_datagram(hello, HELLO_TYPE)
            server.sendto(encode_state(sample(3, 133334)), address)
            server.sendto(encode_state(sample(1, 100000)), address)
            server.sendto(encode_state(sample(2, 116667)), address)
            server.sendto(encode_state(sample(3, 133334)), address)
            bad = bytearray(encode_state(sample(4, 150001)))
            bad[-1] ^= 1
            server.sendto(bad, address)
            # Sending/receiving on loopback is asynchronous: collect all bursts.
            latest = None
            deadline = time.monotonic() + .5
            while time.monotonic() < deadline:
                state = link.read_state(.01)
                if state is not None:
                    latest = state
                if link.bad_packets == 1 and link.old_packets >= 3:
                    break
            self.assertEqual(latest.sequence, 3)
            self.assertEqual(link.old_packets, 3)
            self.assertEqual(link.bad_packets, 1)
            server.sendto(encode_state(sample(1, 1000, boot_id=999)), address)
            self.assertEqual(link.read_state(.1).boot_id, 999)
            server.sendto(encode_state(sample(4, 150001)), address)
            self.assertIsNone(link.read_state(.1))
            self.assertEqual(link.boot_id, 999)
        finally:
            link.close()
            server.close()

    def test_sequential_burst_reports_coalescing_without_network_gaps(self):
        transport = Mock()
        transport.send.side_effect = lambda data: len(data)
        with patch('udp_link.socket.socket', return_value=transport), \
                patch('udp_link.select.select', return_value=([transport], [], [])):
            link = UDPLink('127.0.0.1', 3333)
            try:
                controller = Controller(self.actor)
                transport.recv.side_effect = [encode_state(sample(1, 100000)), BlockingIOError()]
                controller.step(link.read_state())
                transport.recv.side_effect = [encode_state(sample(2, 116667)),
                                              encode_state(sample(3, 133334)),
                                              encode_state(sample(4, 150001)), BlockingIOError()]
                state = link.read_state()
                self.assertEqual(state.sequence, 4)
                self.assertEqual(link.gaps, 0)  # all sequence numbers reached UDP reader
                self.assertEqual(link.coalesced, 2)  # only newest reached controller
                with self.assertRaisesRegex(ValueError, 'seq_step=3 dt=50001us'):
                    controller.step(state)
                # Do not queue/replay delayed actions to hide the timing gap.
                self.assertIsNone(controller.previous_timestamp)
            finally:
                link.close()

    def test_runner_over_udp(self):
        # Real laptop runner/network code; simulated ESP32 over loopback.
        server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server.bind(('127.0.0.1', 0))
        server.settimeout(2)
        process = subprocess.Popen([sys.executable, str(ROOT / 'host/run_policy.py'),
                                    '--esp-ip', '127.0.0.1', '--udp-port',
                                    str(server.getsockname()[1])], stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True)
        try:
            readable = select.select([process.stdout], [], [], 10)[0]
            self.assertTrue(readable, 'Runner failed to start')
            self.assertIn('Wi-Fi UDP', process.stdout.readline())
            hello, address = server.recvfrom(2048)
            decode_datagram(hello, HELLO_TYPE)
            started = time.monotonic()
            for i in range(12):
                state = sample(i + 1, 100000 + i * 16667)
                server.sendto(encode_state(state), address)
                while True:
                    reply, sender = server.recvfrom(2048)
                    self.assertEqual(sender, address)
                    if reply[5] == HELLO_TYPE:
                        continue
                    sequence, payload = decode_datagram(reply, COMMAND_TYPE)
                    break
                self.assertEqual(sequence, i + 1)
                boot, stamp, torque, flags = COMMAND.unpack(payload)
                self.assertEqual((boot, stamp, flags), (1234, state.timestamp_us, 0))
                self.assertTrue(-4.9 <= torque <= 4.9)
                target = started + (i + 1) / 60
                time.sleep(max(0, target - time.monotonic()))
            process.send_signal(signal.SIGINT)
            output, _ = process.communicate(timeout=5)
            self.assertEqual(process.returncode, 0, output)
            self.assertIn('Stopped laptop inference', output)
            while True:
                reply, _ = server.recvfrom(2048)
                if reply[5] == COMMAND_TYPE:
                    _, payload = decode_datagram(reply, COMMAND_TYPE)
                    break
            self.assertEqual(COMMAND.unpack(payload)[-1], 1)
        finally:
            if process.poll() is None:
                process.kill()
                process.communicate()
            server.close()


if __name__ == '__main__':
    unittest.main(verbosity=2)
