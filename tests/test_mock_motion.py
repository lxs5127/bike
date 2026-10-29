"""Compile the actual mock IMU port; check waveform/gyro consistency and actor response."""
import ctypes as ct
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "host"))
from controller import Controller, load_policy
from protocol import State, VALID, DRIVE_READY, MOCK, MOTOR_DISABLED


class IMUSample(ct.Structure):
    _fields_ = [("roll", ct.c_float), ("rate", ct.c_float),
                ("timestamp", ct.c_int64), ("valid", ct.c_bool)]


class MockMotionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        folder = Path(cls.temp.name)
        (folder / "esp_err.h").write_text(
            "typedef int esp_err_t;\n#define ESP_OK 0\n"
            "#define ESP_ERR_INVALID_ARG 0x102\n#define ESP_ERR_NOT_SUPPORTED 0x106\n")
        (folder / "esp_timer.h").write_text("#include <stdint.h>\nint64_t esp_timer_get_time(void);\n")
        stub = folder / "clock.c"
        stub.write_text("#include <stdint.h>\nstatic int64_t now;\n"
                        "void set_time(int64_t time) { now=time; }\n"
                        "int64_t esp_timer_get_time(void) { return now; }\n")
        cls.ports = {}
        for mode in (0, 1):
            (folder / "sdkconfig.h").write_text(
                f"#define CONFIG_BIKE_MOCK_HARDWARE 1\n#define CONFIG_BIKE_MOCK_MOTION {mode}\n")
            library = folder / f"imu-{mode}.so"
            subprocess.run(["gcc", "-shared", "-fPIC", "-O2", "-Wall", "-Wextra", "-Werror",
                            "-I"+str(folder), "-I"+str(ROOT / "main"),
                            str(ROOT / "main/ports/imu_port.c"), str(stub), "-lm", "-o", str(library)],
                           check=True)
            port = ct.CDLL(str(library))
            port.set_time.argtypes = [ct.c_int64]
            port.imu_port_read.argtypes = [ct.POINTER(IMUSample)]
            cls.ports[mode] = port
        cls.policy = load_policy()

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def sample(self, port, timestamp):
        port.set_time(timestamp)
        sample = IMUSample()
        self.assertEqual(port.imu_port_read(ct.byref(sample)), 0)
        self.assertTrue(sample.valid)
        self.assertEqual(sample.timestamp, timestamp)
        return sample

    def test_waveform_extrema_and_long_runtime_precision(self):
        port = self.ports[1]
        port.set_time(100000)
        self.assertEqual(port.imu_port_init(), 0)
        for seconds, expected_roll in [(0, 0), (1, 2), (2, 0), (3, -2), (4, 0),
                                       (86400 * 20 + 1, 2)]:
            sample = self.sample(port, 100000 + seconds * 1000000)
            self.assertAlmostEqual(math.degrees(sample.roll), expected_roll, places=5)
        self.assertEqual(port.imu_port_read(None), 0x102)

    def test_gyro_is_derivative_of_roll(self):
        port = self.ports[1]
        port.set_time(100000)
        port.imu_port_init()
        for elapsed in range(5000, 4000000, 100000):
            now = 100000 + elapsed
            before = self.sample(port, now - 1000).roll
            current = self.sample(port, now)
            after = self.sample(port, now + 1000).roll
            self.assertAlmostEqual(current.rate, (after - before)/.002, delta=1e-5)

    def test_fixed_mode_remains_available(self):
        port = self.ports[0]
        port.imu_port_init()
        for stamp in (100000, 9999999):
            sample = self.sample(port, stamp)
            self.assertAlmostEqual(math.degrees(sample.roll), .2, places=6)
            self.assertEqual(sample.rate, 0)

    def test_real_laptop_actor_changes_with_actual_mock_port(self):
        port = self.ports[1]
        port.set_time(100000)
        port.imu_port_init()
        controller = Controller(self.policy)
        actions = []
        for i in range(300):
            sample = self.sample(port, 100000 + i * 16667)
            state = State(i+1, 1, sample.timestamp+1, 1, 1, sample.roll, sample.rate, 0,
                          0, VALID | DRIVE_READY | MOCK | MOTOR_DISABLED, i, 1, 1000)
            action, torque = controller.step(state)
            self.assertTrue(math.isfinite(action))
            self.assertLessEqual(abs(torque), 4.9)
            actions.append(action)
        self.assertLess(min(actions), -.01)
        self.assertGreater(max(actions), .01)


if __name__ == "__main__":
    unittest.main()
