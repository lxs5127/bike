"""Check the board-test runner's pass/fail criteria with deterministic telemetry."""
import math
from dataclasses import replace
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "host"))
from controller import Controller, load_policy
from protocol import (State, VALID, DRIVE_READY, MOCK, MOTOR_DISABLED, COMMAND_VALID,
                      LINK_FAULT, COMMAND_TYPE, COMMAND, STOP, decode_datagram)
from test_wifi import run_test


class Telemetry:
    """Fake transport/ESP telemetry; actual ESP task/guard tested separately."""
    def __init__(self, fixed=False, ignore_expiry=False, drop_recovery=False, no_states=False):
        self.now = 0
        self.sequence = 0
        self.source = None
        self.ack = 0
        self.valid = False
        self.fixed, self.ignore_expiry = fixed, ignore_expiry
        self.drop_recovery, self.no_states = drop_recovery, no_states
        self.closed = False
        self.commands = []
        self.bad_packets = self.old_packets = self.gaps = 0
        self.coalesced = 0

    def read_state(self):
        self.now += 16667
        self.sequence += 1
        if self.no_states:
            return None
        age = self.now + 100000 - self.source if self.source is not None else 0xffffffff
        alive = self.valid and (age < 50000 or self.ignore_expiry)
        angle = self.now * 1e-6 * math.pi / 2
        roll = .0034906585 if self.fixed else math.radians(2) * math.sin(angle)
        rate = 0 if self.fixed else math.radians(2) * math.pi / 2 * math.cos(angle)
        return State(self.sequence, 1234, self.now + 100000, 1000, 1000, roll, rate, 0,
                     0 if alive else LINK_FAULT,
                     VALID | DRIVE_READY | MOCK | MOTOR_DISABLED | (COMMAND_VALID if alive else 0),
                     self.ack, age, 3000)

    def write(self, data):
        sequence, payload = decode_datagram(data, COMMAND_TYPE)
        boot, timestamp, torque, flags = COMMAND.unpack(payload)
        self.commands.append((sequence, timestamp, torque, flags))
        if not self.drop_recovery or self.now < 5300000:
            self.source, self.ack, self.valid = timestamp, sequence, not bool(flags & STOP)

    def close(self):
        self.closed = True


class WiFiExperimentTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.policy = load_policy()

    def run_experiment(self, stream):
        return run_test(stream, Controller(self.policy), clock=lambda: stream.now * 1e-6,
                        emit=lambda _: None)

    def assert_stopped(self, stream):
        self.assertTrue(stream.closed)
        self.assertEqual(stream.commands[-1][-1], STOP)
        self.assertEqual(stream.commands[-1][-2], 0)

    def test_dynamic_pause_recovery_passes_and_exit_sends_stop(self):
        stream = Telemetry()
        report = self.run_experiment(stream)
        self.assertEqual(report["result"], "PASS")
        self.assertGreater(report["expired_command_states"], 0)
        self.assertGreaterEqual(report["first_observed_expired_command_age_us"], 50000)
        self.assertTrue(report["pause_baseline_acknowledged_run"])
        self.assertLess(report["pause_baseline_command_age_us"], 25000)
        self.assert_stopped(stream)

    def test_already_stopped_at_pause_boundary_cannot_pass(self):
        class Stopped(Telemetry):
            def read_state(self):
                state = super().read_state()
                if self.now >= 4900000:
                    return replace(state, fault=LINK_FAULT, flags=state.flags & ~COMMAND_VALID)
                return state
        stream = Stopped()
        with self.assertRaisesRegex(ValueError, "No fresh acknowledged RUN baseline"):
            self.run_experiment(stream)
        self.assert_stopped(stream)

    def test_early_stop_during_pause_is_not_misreported_as_expiry(self):
        class EarlyStop(Telemetry):
            def read_state(self):
                state = super().read_state()
                if self.now > 5000000 and state.command_age_us == 33334:
                    return replace(state, fault=LINK_FAULT, flags=state.flags & ~COMMAND_VALID)
                return state
        stream = EarlyStop()
        with self.assertRaisesRegex(ValueError, "stopped before its deadline"):
            self.run_experiment(stream)
        self.assert_stopped(stream)

    def test_command_identity_change_during_pause_is_detected(self):
        class ChangedACK(Telemetry):
            def read_state(self):
                state = super().read_state()
                if self.now > 5000000 and state.command_age_us >= 50000:
                    return replace(state, acknowledged_sequence=state.acknowledged_sequence+1)
                return state
        stream = ChangedACK()
        with self.assertRaisesRegex(ValueError, "Acknowledged command changed"):
            self.run_experiment(stream)
        self.assert_stopped(stream)

    def test_fixed_input_is_not_a_dynamic_test_pass(self):
        stream = Telemetry(fixed=True)
        with self.assertRaisesRegex(ValueError, "Dynamic roll/action"):
            self.run_experiment(stream)
        self.assert_stopped(stream)

    def test_expiry_failure_is_detected(self):
        stream = Telemetry(ignore_expiry=True)
        with self.assertRaisesRegex(ValueError, "Expired command still marked valid"):
            self.run_experiment(stream)
        self.assert_stopped(stream)

    def test_missing_recovery_is_detected(self):
        stream = Telemetry(drop_recovery=True)
        with self.assertRaisesRegex(ValueError, "Not enough healthy"):
            self.run_experiment(stream)
        self.assert_stopped(stream)

    def test_missing_state_times_out(self):
        stream = Telemetry(no_states=True)
        with self.assertRaisesRegex(ValueError, "State stream missing"):
            self.run_experiment(stream)
        self.assertTrue(stream.closed)
        self.assertEqual(stream.commands, [])


if __name__ == "__main__":
    unittest.main()
