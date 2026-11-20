"""Run the actual local controller against representative poses and failure cases."""
from pathlib import Path
import subprocess
import tempfile
import unittest

SOURCE = Path(__file__).resolve().parents[1] / 'standalone/odrive_uart/main'
HARNESS = r'''
#include <assert.h>
#include <float.h>
#include <math.h>
#include <string.h>
#include "balance_loop.h"

int main(int argc, char **argv) {
    assert(argc == 2);
    balance_parameters_t p = {10.f, 1.f, .5f, 20.f, 1};
    balance_input_t s = {.roll_rad=.02f, .roll_rate_rad_s=.1f,
        .flywheel_turns_s=1.f, .imu_us=990000, .encoder_us=950000,
        .imu_valid=true, .encoder_valid=true};
    float torque = 123.f, zero = 0.f;
    bool calibrated = true;
    balance_result_t expected = LOCAL_BALANCE_OK;
    if (!strcmp(argv[1], "direction")) {
        assert(balance_compute(&p,&s,true,0,1000000,&torque) == LOCAL_BALANCE_OK);
        assert(fabsf(torque-.3f) < 1e-6f);
        p.torque_direction = -1;
        assert(balance_compute(&p,&s,true,0,1000000,&torque) == LOCAL_BALANCE_OK);
        assert(fabsf(torque+.3f) < 1e-6f);
        s.roll_rad = -.02f; s.roll_rate_rad_s = -.1f;
        assert(balance_compute(&p,&s,true,0,1000000,&torque) == LOCAL_BALANCE_OK);
        assert(fabsf(torque-.3f) < 1e-6f);
        return 0;
    } else if (!strcmp(argv[1], "clamp")) {
        s.roll_rad = .1f;
        assert(balance_compute(&p,&s,true,0,1000000,&torque) == LOCAL_BALANCE_OK);
        assert(torque == .5f);
        s.roll_rad = -.1f; s.roll_rate_rad_s = -.1f;
        assert(balance_compute(&p,&s,true,0,1000000,&torque) == LOCAL_BALANCE_OK);
        assert(torque == -.5f);
        return 0;
    } else if (!strcmp(argv[1], "wrap")) {
        zero = 3.13f; s.roll_rad = -3.13f; s.roll_rate_rad_s = 0;
        assert(balance_compute(&p,&s,true,zero,1000000,&torque) == LOCAL_BALANCE_OK);
        assert(torque > .2f && torque < .3f);
        s.roll_rad = zero;
        assert(balance_compute(&p,&s,true,zero,1000000,&torque) == LOCAL_BALANCE_OK);
        assert(torque == 0.f);
        return 0;
    } else if (!strcmp(argv[1], "zero-missing")) {
        calibrated = false; expected = LOCAL_BALANCE_NOT_CALIBRATED;
    } else if (!strcmp(argv[1], "zero-nan")) {
        zero = NAN; expected = LOCAL_BALANCE_NOT_CALIBRATED;
    } else if (!strcmp(argv[1], "imu-old")) {
        s.imu_us = 969999; expected = LOCAL_BALANCE_STALE;
    } else if (!strcmp(argv[1], "encoder-old")) {
        s.encoder_us = 899999; expected = LOCAL_BALANCE_STALE;
    } else if (!strcmp(argv[1], "future")) {
        s.imu_us = 1000001; expected = LOCAL_BALANCE_STALE;
    } else if (!strcmp(argv[1], "invalid")) {
        s.imu_valid = false; expected = LOCAL_BALANCE_BAD_SENSOR;
    } else if (!strcmp(argv[1], "sensor-nan")) {
        s.roll_rate_rad_s = NAN; expected = LOCAL_BALANCE_BAD_SENSOR;
    } else if (!strcmp(argv[1], "fall")) {
        s.roll_rad = .2f; expected = LOCAL_BALANCE_FALL;
    } else if (!strcmp(argv[1], "overspeed")) {
        s.flywheel_turns_s = -20.f; expected = LOCAL_BALANCE_OVERSPEED;
    } else if (!strcmp(argv[1], "gains-unset")) {
        p.kp_nm_rad = 0; expected = LOCAL_BALANCE_BAD_PARAMETERS;
    } else if (!strcmp(argv[1], "limit-inf")) {
        p.torque_limit_nm = INFINITY; expected = LOCAL_BALANCE_BAD_PARAMETERS;
    } else if (!strcmp(argv[1], "direction-unset")) {
        p.torque_direction = 0; expected = LOCAL_BALANCE_BAD_PARAMETERS;
    } else if (!strcmp(argv[1], "overflow")) {
        p.kd_nm_rad_s = FLT_MAX; s.roll_rate_rad_s = FLT_MAX;
        expected = LOCAL_BALANCE_BAD_PARAMETERS;
    } else assert(!"Unknown case");
    assert(balance_compute(&p,&s,calibrated,zero,1000000,&torque) == expected);
    assert(torque == 0.f);  /* A fault must never reuse the last nonzero output. */
    return 0;
}
'''


class LocalBalanceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        folder = Path(cls.temp.name)
        harness = folder / 'harness.c'
        harness.write_text(HARNESS)
        cls.executable = folder / 'balance'
        subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                        '-I'+str(SOURCE), str(harness), str(SOURCE/'balance_loop.c'),
                        '-lm', '-o', str(cls.executable)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def run_case(self, name):
        subprocess.run([str(self.executable), name], check=True)

    def test_pose_rate_direction_and_torque_limit(self):
        for case in ['direction', 'clamp', 'wrap']:
            with self.subTest(case=case):
                self.run_case(case)

    def test_all_faults_zero_output(self):
        for case in ['zero-missing', 'zero-nan', 'imu-old', 'encoder-old', 'future',
                     'invalid', 'sensor-nan', 'fall', 'overspeed', 'gains-unset',
                     'limit-inf', 'direction-unset', 'overflow']:
            with self.subTest(case=case):
                self.run_case(case)


if __name__ == '__main__':
    unittest.main()
