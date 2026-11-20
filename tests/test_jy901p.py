"""Compile the real JY901P port against an emulated ESP-IDF I2C bus."""
import math
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

HEADERS = {
    "sdkconfig.h": """
#define CONFIG_BIKE_IMU_JY901P 1
#define CONFIG_BIKE_MOCK_HARDWARE 1
#define CONFIG_BIKE_IMU_SDA_GPIO 21
#define CONFIG_BIKE_IMU_SCL_GPIO 22
#define CONFIG_BIKE_IMU_I2C_ADDRESS 0x50
#define CONFIG_BIKE_IMU_I2C_HZ 100000
#define CONFIG_BIKE_IMU_BANDWIDTH 2
""",
    "esp_err.h": """#pragma once
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_NOT_FOUND 0x105
#define ESP_ERR_TIMEOUT 0x107
#define ESP_ERR_INVALID_RESPONSE 0x108
const char *esp_err_to_name(esp_err_t);
""",
    "esp_timer.h": "#include <stdint.h>\nint64_t esp_timer_get_time(void);\n",
    "esp_log.h": """#pragma once
static inline void fake_log(const char *tag, const char *fmt, ...) {}
#define ESP_LOGE(...) fake_log(__VA_ARGS__)
#define ESP_LOGI(...) fake_log(__VA_ARGS__)
""",
    "freertos/FreeRTOS.h": """#pragma once
typedef unsigned TickType_t;
#define pdMS_TO_TICKS(ms) ((ms) / 10)
""",
    "freertos/task.h": '#include "FreeRTOS.h"\nvoid vTaskDelay(TickType_t);\n',
    "driver/gpio.h": "#define GPIO_IS_VALID_OUTPUT_GPIO(pin) ((pin) >= 0 && (pin) <= 33)\n",
    "driver/i2c_master.h": """#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
typedef void *i2c_master_bus_handle_t;
typedef void *i2c_master_dev_handle_t;
#define I2C_NUM_0 0
#define I2C_CLK_SRC_DEFAULT 0
#define I2C_ADDR_BIT_LEN_7 0
typedef struct {
    int i2c_port, sda_io_num, scl_io_num, clk_source, glitch_ignore_cnt;
    struct { bool enable_internal_pullup; } flags;
} i2c_master_bus_config_t;
typedef struct {
    int dev_addr_length;
    uint16_t device_address;
    uint32_t scl_speed_hz;
} i2c_device_config_t;
esp_err_t i2c_new_master_bus(const i2c_master_bus_config_t *, i2c_master_bus_handle_t *);
esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t, const i2c_device_config_t *, i2c_master_dev_handle_t *);
esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t);
esp_err_t i2c_del_master_bus(i2c_master_bus_handle_t);
esp_err_t i2c_master_probe(i2c_master_bus_handle_t, uint16_t, int);
esp_err_t i2c_master_transmit(i2c_master_dev_handle_t, const uint8_t *, size_t, int);
esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t, const uint8_t *, size_t, uint8_t *, size_t, int);
""",
}

HARNESS = r'''
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "imu_port.c"

static uint16_t registers[128];
static int fail_stage, writes, unlocked, buses, devices, read_error;
static int64_t clock_us = 1000000, transfer_us = 1000;
int64_t esp_timer_get_time(void) { return clock_us; }
void vTaskDelay(TickType_t ticks) { clock_us += ticks * 10000; }
const char *esp_err_to_name(esp_err_t err) { return "emulated error"; }
esp_err_t i2c_new_master_bus(const i2c_master_bus_config_t *c, i2c_master_bus_handle_t *out) {
    assert(c->i2c_port == 0 && c->sda_io_num == 21 && c->scl_io_num == 22);
    if (fail_stage == 1) return ESP_FAIL;
    ++buses; *out = &buses; return ESP_OK;
}
esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t bus, const i2c_device_config_t *c,
                                   i2c_master_dev_handle_t *out) {
    assert(bus && c->device_address == 0x50 && c->scl_speed_hz == 100000);
    if (fail_stage == 2) return ESP_FAIL;
    ++devices; *out = &devices; return ESP_OK;
}
esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t device) {
    assert(device && devices == 1); --devices; return ESP_OK;
}
esp_err_t i2c_del_master_bus(i2c_master_bus_handle_t bus) {
    assert(bus && buses == 1 && devices == 0); --buses; return ESP_OK;
}
esp_err_t i2c_master_probe(i2c_master_bus_handle_t bus, uint16_t address, int timeout) {
    assert(bus && address == 0x50 && timeout >= 10 && timeout <= 20);
    return fail_stage == 3 ? ESP_ERR_NOT_FOUND : ESP_OK;
}
esp_err_t i2c_master_transmit(i2c_master_dev_handle_t device, const uint8_t *data,
                              size_t size, int timeout) {
    assert(device && size == 3 && timeout >= 10 && timeout <= 20);
    if (fail_stage == 4) return ESP_FAIL;
    ++writes;
    if (data[0] == 0x69) {
        assert(data[1] == 0x88 && data[2] == 0xb5);
        unlocked = 1;
    } else {
        assert(unlocked);
        assert((data[0] == 0x03 && data[1] == 0x0b) ||
               (data[0] == 0x1f && data[1] == 2));
        assert(data[2] == 0);
        registers[data[0]] = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
        unlocked = 0;
    }
    return ESP_OK;
}
esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t device,
        const uint8_t *reg, size_t write_size, uint8_t *data, size_t size, int timeout) {
    assert(device && write_size == 1 && timeout >= 10 && timeout <= 20);
    assert((*reg == 0x34 && size == 24) ||
           ((*reg == 0x03 || *reg == 0x1f) && size == 2));
    clock_us += transfer_us;
    if (read_error) return read_error;
    if (fail_stage == 5) return ESP_ERR_TIMEOUT;
    for (size_t i = 0; i < size / 2; ++i) {
        uint16_t value = registers[*reg + i];
        data[i * 2] = (uint8_t)value;
        data[i * 2 + 1] = (uint8_t)(value >> 8);
    }
    if (fail_stage == 6) data[0] ^= 1;
    return ESP_OK;
}
int main(int argc, char **argv) {
    assert(argc >= 2);
    imu_sample_t sample = { .valid = true, .timestamp_us = 1 };
    if (!strcmp(argv[1], "before-init")) {
        assert(imu_port_read(NULL) == ESP_ERR_INVALID_ARG);
        assert(imu_port_read(&sample) == ESP_ERR_INVALID_STATE);
        assert(!sample.valid && sample.timestamp_us == 0);
        return 0;
    }
    if (!strcmp(argv[1], "init-failure")) {
        fail_stage = atoi(argv[2]);
        assert(imu_port_init() != ESP_OK);
        assert(!s_ready && !s_bus && !s_device && !buses && !devices);
        fail_stage = 0;
    }
    assert(imu_port_init() == ESP_OK);
    int previous_writes = writes;
    assert(imu_port_init() == ESP_OK && writes == previous_writes);
    assert(registers[0x03] == 0x0b && registers[0x1f] == 2);
    if (!strcmp(argv[1], "sample")) {
        registers[0x3d] = (uint16_t)atoi(argv[2]);
        registers[0x37] = (uint16_t)atoi(argv[3]);
        /* Other axes must not leak into the Roll / GX mapping. */
        registers[0x38] = 12345; registers[0x3e] = 23456;
        int64_t started = clock_us;
        assert(imu_port_read(&sample) == ESP_OK && sample.valid);
        assert(sample.timestamp_us == started - 5000);
        printf("%.9g %.9g\n", (double)sample.roll_rad, (double)sample.roll_rate_rad_s);
    } else if (!strcmp(argv[1], "full-sample")) {
        registers[0x34] = 2048; registers[0x35] = (uint16_t)-4096; registers[0x36] = 8192;
        registers[0x37] = 16384; registers[0x38] = (uint16_t)-16384; registers[0x39] = 0;
        registers[0x3d] = 16384; registers[0x3e] = (uint16_t)-8192; registers[0x3f] = 4096;
        assert(imu_port_read(&sample) == ESP_OK && sample.valid);
        assert(sample.acceleration_g[0] == 1.f && sample.acceleration_g[1] == -2.f && sample.acceleration_g[2] == 4.f);
        assert(sample.gyro_deg_s[0] == 1000.f && sample.gyro_deg_s[1] == -1000.f && sample.gyro_deg_s[2] == 0.f);
        assert(sample.angles_deg[0] == 90.f && sample.angles_deg[1] == -45.f && sample.angles_deg[2] == 22.5f);
    } else if (!strcmp(argv[1], "read-failure")) {
        read_error = atoi(argv[2]);
        assert(imu_port_read(&sample) == read_error);
        assert(!sample.valid && sample.timestamp_us == 0 && sample.roll_rad == 0);
    } else if (!strcmp(argv[1], "slow-read")) {
        transfer_us = 20001;
        assert(imu_port_read(&sample) == ESP_ERR_TIMEOUT);
        assert(!sample.valid && sample.timestamp_us == 0);
    }
    release_i2c();
    assert(!buses && !devices);
    return 0;
}
'''


class JY901PTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        folder = Path(cls.temp.name)
        for name, source in HEADERS.items():
            target = folder / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(source)
        harness = folder / "harness.c"
        harness.write_text(HARNESS)
        cls.executable = folder / "jy901p"
        subprocess.run(["gcc", "-O2", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
                        "-I" + str(folder), "-I" + str(ROOT / "main"),
                        "-I" + str(ROOT / "main/ports"), str(harness), "-lm",
                        "-o", str(cls.executable)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def run_case(self, *args):
        return subprocess.check_output([str(self.executable), *map(str, args)], text=True)

    def test_little_endian_signed_roll_and_gyro_in_radians(self):
        for roll, gyro in [(0, 0), (16384, -16384), (-32768, 32767), (0x1234, -1234)]:
            with self.subTest(roll=roll, gyro=gyro):
                actual_roll, actual_rate = map(float, self.run_case("sample", roll, gyro).split())
                self.assertAlmostEqual(actual_roll, roll / 32768 * math.pi, delta=3e-7)
                self.assertAlmostEqual(actual_rate, gyro / 32768 * math.radians(2000), delta=4e-6)

    def test_invalid_input_and_read_before_init(self):
        self.run_case("before-init")

    def test_full_three_axis_telemetry_units(self):
        self.run_case("full-sample")

    def test_failed_init_cleans_resources_and_allows_retry(self):
        for stage in range(1, 7):
            with self.subTest(stage=stage):
                self.run_case("init-failure", stage)

    def test_nack_and_timeout_clear_sample_validity_and_timestamp(self):
        for error in (0x105, 0x107):
            with self.subTest(error=error):
                self.run_case("read-failure", error)

    def test_overlong_successful_transfer_is_rejected(self):
        self.run_case("slow-read")


if __name__ == "__main__":
    unittest.main()
