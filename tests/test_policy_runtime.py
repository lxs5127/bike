"""Host checks: real C inference against the existing NumPy deployment actor.

Run with the existing legged_gym Python. No flashing or physical motor access.
"""
import ctypes as ct
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
TRAINING = Path("/home/lxs/work_place/balance_bike")


class State(ct.Structure):
    _fields_ = [("roll", ct.c_float), ("rate", ct.c_float), ("speed", ct.c_float),
                ("imu_time", ct.c_int64), ("encoder_time", ct.c_int64),
                ("valid", ct.c_bool), ("drive_healthy", ct.c_bool)]


class Memory(ct.Structure):
    _fields_ = [("rotation", ct.c_float), ("previous_speed", ct.c_float),
                ("initialized", ct.c_bool)]


class Controller(ct.Structure):
    _fields_ = [("policy", Memory), ("previous_action", ct.c_float)]


class Output(ct.Structure):
    _fields_ = [("action", ct.c_float), ("torque", ct.c_float), ("rotation", ct.c_float)]


class RuntimeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        library = Path(cls.temp.name) / "policy.so"
        main = ROOT / "main"
        subprocess.run(["gcc", "-shared", "-fPIC", "-O2", "-Wall", "-Wextra", "-Werror",
            "-I" + str(main), "-I" + str(main / "policy"), "-I" + str(main / "control"),
            str(main / "policy/rl_policy.c"), str(main / "policy/policy_weights.c"),
            str(main / "control/bike_controller.c"), str(main / "control/bike_safety.c"),
            "-lm", "-o", str(library)], check=True)
        cls.lib = ct.CDLL(str(library))
        cls.lib.rl_policy_action_from_observation.argtypes = [ct.POINTER(ct.c_float), ct.c_float]
        cls.lib.rl_policy_action_from_observation.restype = ct.c_float
        cls.lib.rl_policy_step.argtypes = [ct.POINTER(Memory), ct.POINTER(State), ct.c_float,
                                         ct.POINTER(ct.c_float)]
        cls.lib.rl_policy_step.restype = ct.c_bool
        cls.lib.rl_policy_reset.argtypes = [ct.POINTER(Memory)]
        cls.lib.bike_controller_step.argtypes = [ct.POINTER(Controller), ct.POINTER(State),
                                               ct.c_float, ct.POINTER(Output)]
        cls.lib.bike_controller_step.restype = ct.c_bool
        cls.lib.bike_controller_reset.argtypes = [ct.POINTER(Controller)]
        cls.lib.bike_limit_action.argtypes = [ct.c_float, ct.c_float]
        cls.lib.bike_limit_action.restype = ct.c_float
        cls.lib.bike_validate_state.argtypes = [ct.POINTER(State), ct.c_int64]
        cls.lib.bike_validate_state.restype = ct.c_int
        spec = importlib.util.spec_from_file_location("reference_policy", TRAINING / "sim2sim/policy.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        manifest = json.loads((main / "policy/policy_manifest.json").read_text())
        cls.reference = module.NumpyPolicy(manifest["source_npz"])
        cls.report = {"source_npz": manifest["source_npz"], "npz_sha256": manifest["npz_sha256"],
                      "board_timing_measured": False, "motor_output_tested": False}

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def action(self, observation, rotation=0):
        array = np.asarray(observation, dtype=np.float32)
        return self.lib.rl_policy_action_from_observation(
            array.ctypes.data_as(ct.POINTER(ct.c_float)), np.float32(rotation))

    def test_actor_matches_numpy(self):
        rng = np.random.RandomState(42)
        observations = rng.normal(size=(512, 4)).astype(np.float32)
        rotations = rng.uniform(-15000, 15000, 512).astype(np.float32)
        errors = [abs(self.action(o, r) - self.reference.action(o, r))
                  for o, r in zip(observations, rotations)]
        self.report["max_actor_action_error"] = max(errors)
        self.assertLess(max(errors), 2e-5)

    def test_mirror_symmetry_and_zero(self):
        self.assertEqual(self.action([0, 1, 0, 0]), 0)
        obs = np.array([.07, .99, .12, .3], dtype=np.float32)
        mirror = -obs.copy()
        mirror[1] = obs[1]
        self.assertAlmostEqual(self.action(obs, 400), -self.action(mirror, -400), places=7)

    def test_memory_and_controller_trajectory(self):
        context = Controller()
        self.lib.bike_controller_reset(ct.byref(context))
        dt = np.float32(1 / 60)
        rotation = np.float32(0)
        previous_speed = np.float32(0)
        errors = []
        for step in range(600):
            roll = np.float32(.04 * np.sin(step / 20))
            rate = np.float32(.12 * np.cos(step / 20))
            speed = np.float32(50 * np.sin(step / 30))
            if step:
                rotation = np.float32(rotation + np.float32(.5) *
                                     np.float32(previous_speed + speed) * dt)
            previous_speed = speed
            obs = [np.sin(roll), np.cos(roll), np.clip(rate, -5, 5),
                   np.clip(speed / 420, -1.5, 1.5)]
            state = State(roll, rate, speed, 100000, 100000, True, True)
            output = Output()
            self.assertTrue(self.lib.bike_controller_step(ct.byref(context), ct.byref(state),
                                                        dt, ct.byref(output)))
            errors.append(abs(output.action - self.reference.action(obs, rotation)))
            self.assertAlmostEqual(output.rotation, rotation, delta=2e-4)
            self.assertAlmostEqual(output.torque, 4.9 * output.action, delta=2e-6)
        self.report["max_trajectory_action_error"] = max(errors)
        self.assertLess(max(errors), 2e-5)
        self.lib.bike_controller_reset(ct.byref(context))
        self.assertEqual(context.policy.rotation, 0)
        self.assertFalse(context.policy.initialized)

    def test_speed_guard_matches_simulation_formula(self):
        span = max(.2 * 250, 2 * 4.9 / 60 / .00739)
        for speed in [-300, -250, -230, -200, 0, 200, 230, 250, 300]:
            for action in [-1, -.5, 0, .5, 1]:
                expected = action * np.clip((250 - speed * np.sign(action)) / span, 0, 1)
                brake = np.clip((abs(speed) - 250) / span, 0, 1)
                if speed > 250: expected = min(expected, -brake)
                if speed < -250: expected = max(expected, brake)
                self.assertAlmostEqual(self.lib.bike_limit_action(action, speed), expected, places=6)

    def test_invalid_sensor_values_are_rejected(self):
        memory, action = Memory(), ct.c_float()
        for field in ["roll", "rate", "speed"]:
            state = State(0, 0, 0, 100000, 100000, True, True)
            setattr(state, field, float("nan"))
            self.assertFalse(self.lib.rl_policy_step(ct.byref(memory), ct.byref(state),
                                                    1 / 60, ct.byref(action)))
            self.assertEqual(self.lib.bike_validate_state(ct.byref(state), 100000), 1)
        state = State(0, 0, 0, 100000, 100000, True, True)
        self.assertFalse(self.lib.rl_policy_step(ct.byref(memory), ct.byref(state),
                                                0, ct.byref(action)))

    def test_feedback_age_fall_and_drive_faults(self):
        state = State(0, 0, 0, 100000, 100000, True, True)
        self.assertEqual(self.lib.bike_validate_state(ct.byref(state), 110000), 0)
        self.assertEqual(self.lib.bike_validate_state(ct.byref(state), 120001), 2)
        state.encoder_time = 130000
        self.assertEqual(self.lib.bike_validate_state(ct.byref(state), 110000), 2)
        state.encoder_time = 100000
        state.roll = np.deg2rad(11)
        self.assertEqual(self.lib.bike_validate_state(ct.byref(state), 110000), 16)
        state.roll, state.drive_healthy = 0, False
        self.assertEqual(self.lib.bike_validate_state(ct.byref(state), 110000), 32)


if __name__ == "__main__":
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(RuntimeTests)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    if result.wasSuccessful():
        RuntimeTests.report["tests_passed"] = result.testsRun
        (ROOT / "docs/validation.json").write_text(json.dumps(RuntimeTests.report, indent=2) + "\n")
    raise SystemExit(0 if result.wasSuccessful() else 1)
