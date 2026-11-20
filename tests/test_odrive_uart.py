"""Compile the actual ODrive UART component against an emulated ESP32 UART."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
COMPONENT = ROOT / 'components/odrive_uart'

HEADERS = {
    'esp_err.h': '''#pragma once
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NO_MEM 0x101
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_TIMEOUT 0x107
#define ESP_ERR_INVALID_RESPONSE 0x108
#define ESP_ERR_INVALID_SIZE 0x109
''',
    'esp_timer.h': '#include <stdint.h>\nint64_t esp_timer_get_time(void);\n',
    'freertos/FreeRTOS.h': '''#pragma once
typedef unsigned TickType_t;
#define portTICK_PERIOD_MS 10U
#define pdTRUE 1
''',
    'freertos/semphr.h': '''#pragma once
#include "FreeRTOS.h"
typedef void *SemaphoreHandle_t;
SemaphoreHandle_t xSemaphoreCreateMutex(void);
int xSemaphoreTake(SemaphoreHandle_t, TickType_t);
int xSemaphoreGive(SemaphoreHandle_t);
void vSemaphoreDelete(SemaphoreHandle_t);
''',
    'driver/gpio.h': '''
#define GPIO_IS_VALID_OUTPUT_GPIO(pin) ((pin) >= 0 && (pin) <= 33)
#define GPIO_IS_VALID_GPIO(pin) ((pin) >= 0 && (pin) <= 39)
''',
    'driver/uart.h': '''#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#define UART_DATA_8_BITS 8
#define UART_PARITY_DISABLE 0
#define UART_STOP_BITS_1 1
#define UART_HW_FLOWCTRL_DISABLE 0
#define UART_SCLK_APB 1
#define UART_PIN_NO_CHANGE -1
typedef struct { int baud_rate, data_bits, parity, stop_bits, flow_ctrl, source_clk; } uart_config_t;
bool uart_is_driver_installed(int);
esp_err_t uart_driver_install(int, int, int, int, void *, int);
esp_err_t uart_driver_delete(int);
esp_err_t uart_param_config(int, const uart_config_t *);
esp_err_t uart_set_pin(int, int, int, int, int);
esp_err_t uart_flush_input(int);
esp_err_t uart_wait_tx_done(int, TickType_t);
int uart_tx_chars(int, const char *, uint32_t);
int uart_read_bytes(int, void *, uint32_t, TickType_t);
''',
}

HARNESS = r'''
#include <assert.h>
#include <string.h>
#include "odrive_uart.c"

static bool installed, mutex_live, mutex_taken, block_mutex, short_write;
static int fail_stage, wait_calls, wait_fail_at;
static int64_t clock_us = 1000000;
static char transmitted[16384];
static size_t tx_size;
static const char *feedback_reply = "-1.25 2.5\r\n";
static const char *velocity_reply = "-2.5\r\n";
static const char *error_reply = "0\n", *state_reply = "8\n", *rx = "";
int64_t esp_timer_get_time(void) { return clock_us; }
SemaphoreHandle_t xSemaphoreCreateMutex(void) {
    if (fail_stage == 1) return NULL;
    assert(!mutex_live); mutex_live = true; return &mutex_live;
}
int xSemaphoreTake(SemaphoreHandle_t handle, TickType_t ticks) {
    assert(handle && ticks > 0 && ticks <= 2 && !mutex_taken);
    if (block_mutex) return 0;
    mutex_taken = true; return pdTRUE;
}
int xSemaphoreGive(SemaphoreHandle_t handle) {
    assert(handle && mutex_taken); mutex_taken = false; return pdTRUE;
}
void vSemaphoreDelete(SemaphoreHandle_t handle) {
    assert(handle && mutex_live && !mutex_taken); mutex_live = false;
}
bool uart_is_driver_installed(int port) { assert(port == 2); return installed; }
esp_err_t uart_driver_install(int port, int rx_size, int tx, int count, void *queue, int flags) {
    assert(port == 2 && rx_size >= 256 && tx == 0 && count == 0 && !queue && !flags);
    if (fail_stage == 2) return ESP_FAIL;
    installed = true; return ESP_OK;
}
esp_err_t uart_driver_delete(int port) {
    assert(port == 2 && installed); installed = false; return ESP_OK;
}
esp_err_t uart_param_config(int port, const uart_config_t *c) {
    assert(port == 2 && c->baud_rate == 115200 && c->data_bits == 8 &&
           c->parity == 0 && c->stop_bits == 1 && c->flow_ctrl == 0);
    return fail_stage == 3 ? ESP_FAIL : ESP_OK;
}
esp_err_t uart_set_pin(int port, int tx, int rx_pin, int rts, int cts) {
    assert(port == 2 && tx == 25 && rx_pin == 26 && rts == -1 && cts == -1);
    return fail_stage == 4 ? ESP_FAIL : ESP_OK;
}
esp_err_t uart_flush_input(int port) {
    assert(port == 2); rx = ""; return ESP_OK;
}
esp_err_t uart_wait_tx_done(int port, TickType_t ticks) {
    assert(port == 2 && ticks > 0 && ticks <= 2);
    ++wait_calls;
    return wait_fail_at == wait_calls ? ESP_ERR_TIMEOUT : ESP_OK;
}
int uart_tx_chars(int port, const char *bytes, uint32_t count) {
    assert(port == 2 && installed && count < 96);
    assert(tx_size + count < sizeof(transmitted));
    if (short_write) return count - 1;
    memcpy(transmitted + tx_size, bytes, count);
    tx_size += count; transmitted[tx_size] = 0;
    if (!strncmp(bytes, "f ", 2)) rx = feedback_reply;
    if (strstr(bytes, ".encoder.vel_estimate\n")) rx = velocity_reply;
    if (strstr(bytes, ".error\n")) rx = error_reply;
    if (strstr(bytes, ".current_state\n")) rx = state_reply;
    return (int)count;
}
int uart_read_bytes(int port, void *out, uint32_t count, TickType_t ticks) {
    assert(port == 2 && count == 1 && ticks > 0 && ticks <= 2);
    if (!*rx) { clock_us += ticks * 10000; return 0; }
    *(unsigned char *)out = (unsigned char)*rx++;
    clock_us += 100;
    return 1;
}
static void clear_output(void) { tx_size = 0; transmitted[0] = 0; }
int main(int argc, char **argv) {
    assert(argc >= 2);
    if (!strcmp(argv[1], "init-failure")) {
        fail_stage = atoi(argv[2]);
        assert(odrive_uart_init() != ESP_OK && !installed && !mutex_live);
        fail_stage = 0;
    }
    if (!strcmp(argv[1], "before-init")) {
        assert(odrive_uart_set_motor0_torque(0) == ESP_ERR_INVALID_STATE);
        installed = true;
        assert(odrive_uart_init() == ESP_ERR_INVALID_STATE && !mutex_live);
        installed = false;
    }
    assert(odrive_uart_init() == ESP_OK);
    assert(!strstr(transmitted, "requested_state 8"));
    if (strcmp(argv[1], "init-failure")) {
        assert(!strcmp(transmitted,
          "w axis0.requested_state 1\nc 0 0\nw axis1.requested_state 1\nv 1 0 0\n"));
    }
    size_t initialized_size = tx_size;
    assert(odrive_uart_init() == ESP_OK && tx_size == initialized_size);
    clear_output();
    if (!strcmp(argv[1], "controls")) {
        assert(odrive_uart_set_motor0_torque(-.25f) == ESP_OK);
        assert(odrive_uart_set_motor1_velocity(10.f) == ESP_OK);
        assert(odrive_uart_set_motor1_rpm(-120.f) == ESP_OK);
        assert(odrive_uart_set_motor1_rad_s(TWO_PI) == ESP_OK);
        assert(!strcmp(transmitted, "c 0 -0.25\nv 1 10 0\nv 1 -2 0\nv 1 1 0\n"));
        clear_output();
        assert(odrive_uart_enable(0) == ESP_OK);
        assert(odrive_uart_enable(1) == ESP_OK);
        assert(!strcmp(transmitted,
          "w axis0.requested_state 1\nc 0 0\n"
          "w axis0.controller.config.input_mode 1\nw axis0.controller.config.control_mode 1\n"
          "w axis0.requested_state 8\nw axis1.requested_state 1\nv 1 0 0\n"
          "w axis1.controller.config.input_mode 1\nw axis1.controller.config.control_mode 2\n"
          "w axis1.requested_state 8\n"));
        assert(odrive_uart_feed_watchdog(1) == ESP_OK);
        assert(odrive_uart_clear_errors() == ESP_OK);
        assert(strstr(transmitted,"u 1\nsc\n"));
    } else if (!strcmp(argv[1], "invalid-inputs")) {
        assert(odrive_uart_set_motor0_torque(NAN) == ESP_ERR_INVALID_ARG);
        assert(odrive_uart_set_motor1_velocity(INFINITY) == ESP_ERR_INVALID_ARG);
        assert(odrive_uart_set_motor1_rpm(NAN) == ESP_ERR_INVALID_ARG);
        assert(odrive_uart_enable(2) == ESP_ERR_INVALID_ARG);
        assert(odrive_uart_stop(2) == ESP_ERR_INVALID_ARG);
        assert(odrive_uart_feed_watchdog(2) == ESP_ERR_INVALID_ARG);
        assert(odrive_uart_read_feedback(0, NULL) == ESP_ERR_INVALID_ARG);
        assert(odrive_uart_read_axis_state(0, NULL) == ESP_ERR_INVALID_ARG);
        float speed = 123; int64_t time = 123;
        assert(odrive_uart_read_velocity(0, NULL, &time) == ESP_ERR_INVALID_ARG);
        assert(odrive_uart_read_velocity(0, &speed, NULL) == ESP_ERR_INVALID_ARG);
        assert(odrive_uart_read_velocity(2, &speed, &time) == ESP_ERR_INVALID_ARG);
        assert(speed == 0 && time == 0);
        assert(tx_size == 0);
    } else if (!strcmp(argv[1], "feedback")) {
        odrive_uart_feedback_t sample = {0};
        int64_t start = clock_us;
        assert(odrive_uart_read_feedback(1, &sample) == ESP_OK && sample.valid);
        assert(sample.position_turns == -1.25f && sample.velocity_turns_s == 2.5f);
        assert(sample.received_at_us > start && !strcmp(transmitted, "f 1\n"));
        odrive_uart_axis_state_t state;
        error_reply = "0x10\r\n";
        assert(odrive_uart_read_axis_state(0, &state) == ESP_OK);
        assert(state.valid && state.axis_error == 16 && state.current_state == 8);
    } else if (!strcmp(argv[1], "velocity")) {
        float speed; int64_t timestamp;
        int64_t start = clock_us;
        assert(odrive_uart_read_velocity(0, &speed, &timestamp) == ESP_OK);
        assert(speed == -2.5f && timestamp > start);
        assert(!strcmp(transmitted, "r axis0.encoder.vel_estimate\n"));
        clear_output();
        assert(odrive_uart_read_velocity(1, &speed, &timestamp) == ESP_OK);
        assert(!strcmp(transmitted, "r axis1.encoder.vel_estimate\n"));
    } else if (!strcmp(argv[1], "bad-velocity")) {
        velocity_reply = argv[2];
        float speed = 123; int64_t timestamp = 123;
        assert(odrive_uart_read_velocity(0, &speed, &timestamp) != ESP_OK);
        assert(speed == 0 && timestamp == 0);
        velocity_reply = "0\n";
        size_t before = tx_size;
        assert(odrive_uart_read_velocity(0, &speed, &timestamp) == ESP_ERR_INVALID_STATE);
        assert(tx_size == before);
        assert(odrive_uart_stop_all() == ESP_OK);
    } else if (!strcmp(argv[1], "bad-feedback")) {
        feedback_reply = argv[2];
        odrive_uart_feedback_t sample = {.valid=true, .received_at_us=123};
        assert(odrive_uart_read_feedback(0, &sample) != ESP_OK);
        assert(!sample.valid && sample.received_at_us == 0);
        feedback_reply = "0 0\n";
        size_t before = tx_size;
        assert(odrive_uart_read_feedback(0, &sample) == ESP_ERR_INVALID_STATE);
        assert(tx_size == before);
        assert(odrive_uart_stop_all() == ESP_OK);  /* RX fault never suppresses stop. */
    } else if (!strcmp(argv[1], "bad-state")) {
        error_reply = argv[2];
        odrive_uart_axis_state_t state = {.valid=true, .axis_error=123};
        assert(odrive_uart_read_axis_state(0, &state) == ESP_ERR_INVALID_RESPONSE);
        assert(!state.valid && state.axis_error == 0);
    } else if (!strcmp(argv[1], "stop-failure")) {
        wait_fail_at = wait_calls + 1;
        assert(odrive_uart_stop_all() == ESP_ERR_TIMEOUT);
        assert(strstr(transmitted, "w axis1.requested_state 1\nv 1 0 0\n"));
    } else if (!strcmp(argv[1], "mutex-timeout")) {
        block_mutex = true;
        assert(odrive_uart_set_motor0_torque(1) == ESP_ERR_TIMEOUT);
        assert(!tx_size); block_mutex = false;
    } else if (!strcmp(argv[1], "short-write")) {
        short_write = true;
        assert(odrive_uart_set_motor0_torque(1) == ESP_FAIL);
        short_write = false;
        assert(odrive_uart_set_motor1_velocity(1) == ESP_ERR_INVALID_STATE);
        assert(odrive_uart_deinit() == ESP_ERR_INVALID_STATE);
        assert(!installed && !mutex_live);
        return 0;
    }
    assert(odrive_uart_deinit() == ESP_OK && !installed && !mutex_live);
    return 0;
}
'''


class ODriveUARTTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        folder = Path(cls.temp.name)
        for name, source in HEADERS.items():
            target = folder / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(source)
        harness = folder / 'harness.c'
        harness.write_text(HARNESS)
        cls.executable = folder / 'odrive-uart'
        subprocess.run(['gcc', '-O2', '-Wall', '-Wextra', '-Werror',
                        '-I'+str(folder), '-I'+str(COMPONENT), '-I'+str(COMPONENT/'include'),
                        str(harness), '-lm', '-o', str(cls.executable)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def run_case(self, *args):
        subprocess.run([str(self.executable), *map(str, args)], check=True)

    def test_init_and_resource_cleanup(self):
        self.run_case('before-init')
        for stage in range(1, 5):
            self.run_case('init-failure', stage)

    def test_exact_axis_modes_commands_units_and_enable_order(self):
        self.run_case('controls')

    def test_invalid_inputs_never_send(self):
        self.run_case('invalid-inputs')

    def test_feedback_and_axis_state_response_parsing(self):
        self.run_case('feedback')

    def test_velocity_uses_manual_read_command_for_each_axis(self):
        self.run_case('velocity')

    def test_bad_velocity_is_invalid_and_latches_query_fault(self):
        for reply in ['', '1 2\n', 'nan\n', 'inf\n', '1e999\n',
                      'invalid property\n', '0\x01\n', 'A'*100+'\n']:
            with self.subTest(reply=reply):
                self.run_case('bad-velocity', reply)

    def test_missing_corrupt_nonfinite_and_overlong_responses_fail_closed(self):
        for reply in ['', '1\n', '1 2 extra\n', 'nan 1\n', '0 inf\n', '1e999 1\n',
                      '0 0\x01\n', 'A'*100+'\n']:
            with self.subTest(reply=reply):
                self.run_case('bad-feedback', reply)

    def test_axis_error_must_be_uint32(self):
        for reply in ['-1\n', '4294967296\n', 'invalid property\n', '0 junk\n']:
            self.run_case('bad-state', reply)

    def test_stop_attempts_both_axes_after_first_timeout(self):
        self.run_case('stop-failure')

    def test_mutex_wait_is_bounded(self):
        self.run_case('mutex-timeout')

    def test_partial_transmit_does_not_retry_or_append_motion_commands(self):
        self.run_case('short-write')


if __name__ == '__main__':
    unittest.main()
