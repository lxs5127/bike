"""Exercise the real control task with a sensor update during queue peek."""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "host"))
from protocol import decode_datagram


HEADERS = {
    "sdkconfig.h": "#define CONFIG_BIKE_MOCK_HARDWARE 1\n#define CONFIG_FREERTOS_UNICORE 0\n",
    "esp_err.h": """#pragma once
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NO_MEM 0x101
#define ESP_ERR_NOT_FOUND 0x105
""",
    "esp_random.h": "#include <stdint.h>\nuint32_t esp_random(void);\n",
    "esp_log.h": """#pragma once
static inline void fake_log(const char *tag, const char *format, ...) {}
#define ESP_LOGE(...) fake_log(__VA_ARGS__)
#define ESP_LOGI(...) fake_log(__VA_ARGS__)
#define ESP_LOGW(...) fake_log(__VA_ARGS__)
""",
    "esp_timer.h": """#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
typedef void *esp_timer_handle_t;
typedef void (*esp_timer_cb_t)(void *);
typedef struct {
    esp_timer_cb_t callback;
    const char *name;
    int dispatch_method;
    bool skip_unhandled_events;
} esp_timer_create_args_t;
#define ESP_TIMER_TASK 0
int64_t esp_timer_get_time(void);
esp_err_t esp_timer_create(const esp_timer_create_args_t *, esp_timer_handle_t *);
esp_err_t esp_timer_start_periodic(esp_timer_handle_t, uint64_t);
""",
    "freertos/FreeRTOS.h": """#pragma once
#include <stdint.h>
typedef int BaseType_t;
typedef unsigned TickType_t;
typedef void *TaskHandle_t;
typedef void *QueueHandle_t;
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define portMAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(ms) ((ms) / 10)
#define eSetBits 1
""",
    "freertos/task.h": """#pragma once
#include "FreeRTOS.h"
void xTaskNotifyGive(TaskHandle_t);
BaseType_t xTaskNotify(TaskHandle_t, uint32_t, int);
uint32_t ulTaskNotifyTake(BaseType_t, TickType_t);
BaseType_t xTaskNotifyWait(uint32_t, uint32_t, uint32_t *, TickType_t);
void vTaskDelay(TickType_t);
BaseType_t xTaskCreatePinnedToCore(void (*)(void *), const char *, unsigned,
                                 void *, unsigned, TaskHandle_t *, BaseType_t);
""",
    "freertos/queue.h": """#pragma once
#include "FreeRTOS.h"
QueueHandle_t xQueueCreate(unsigned, unsigned);
BaseType_t xQueueOverwrite(QueueHandle_t, const void *);
BaseType_t xQueuePeek(QueueHandle_t, void *, TickType_t);
BaseType_t xQueueReceive(QueueHandle_t, void *, TickType_t);
""",
}

HARNESS = r'''
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include "bike_app.c"

static jmp_buf finished;
static int cycle;
static int64_t clock_us = 100000;
int64_t esp_timer_get_time(void) { return clock_us; }
BaseType_t xTaskNotifyWait(uint32_t clear_entry, uint32_t clear_exit,
                          uint32_t *events, TickType_t timeout) {
    if (cycle == 3) longjmp(finished, 1);
    ++cycle;
    clock_us += BIKE_CONTROL_PERIOD_US;
    *events = EVENT_SEND_STATE;
    return pdTRUE;
}
BaseType_t xQueuePeek(QueueHandle_t queue, void *output, TickType_t timeout) {
    /* Fresh sampling preempts control after it wakes. A genuine expired
       sample in cycle 2 must still stop, and remain latched in cycle 3. */
    int64_t sample_us = cycle == 2 ? clock_us - 20001 : clock_us + 5;
    clock_us += 10;
    *(bike_state_t *)output = (bike_state_t){
        .roll_rad = .0034906585f,
        .imu_timestamp_us = sample_us, .encoder_timestamp_us = sample_us,
        .valid = true, .drive_healthy = true,
    };
    return pdTRUE;
}
BaseType_t xQueueReceive(QueueHandle_t queue, void *output, TickType_t timeout) {
    return pdFALSE;
}
bool pc_link_port_ready(void) { return true; }
esp_err_t pc_link_port_send(const uint8_t *bytes, size_t size) {
    assert(size == BIKE_PACKET_MAX_SIZE);
    assert(fwrite(bytes, 1, size, stdout) == size);
    return ESP_OK;
}
esp_err_t odrive_port_stop(void) { return ESP_OK; }
esp_err_t odrive_port_write_command(const odrive_command_t *command) {
    assert(!command->enable && command->flywheel_torque_nm == 0);
    return ESP_OK;
}
int main(void) {
    s_boot_id = 1234;
    if (!setjmp(finished)) control_task(NULL);
    return 0;
}
'''


class _ControlHarness(unittest.TestCase):
    harness = HARNESS
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        folder = Path(cls.temp.name)
        for name, source in HEADERS.items():
            target = folder / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(source)
        harness = folder / "harness.c"
        harness.write_text(cls.harness)
        executable = folder / "control-task"
        main = ROOT / "main"
        command = ["gcc", "-O2", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
                   "-ffunction-sections", "-fdata-sections", "-Wl,--gc-sections"]
        for path in [folder, main, main / "control", main / "link", main / "ports"]:
            command += ["-I" + str(path)]
        command += [str(harness), str(main / "control/bike_safety.c"),
                    str(main / "link/bike_protocol.c"), str(main / "link/bike_link_guard.c"),
                    "-lm", "-o", str(executable)]
        subprocess.run(command, check=True)
        frames = subprocess.check_output([str(executable)])
        cls.states = [decode_datagram(frames[i:i + 68]) for i in range(0, len(frames), 68)]

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()


class AppSnapshotTests(_ControlHarness):
    def test_fresh_preempting_sample_is_not_misclassified_as_future(self):
        self.assertEqual(len(self.states), 3)
        self.assertEqual(self.states[0].fault, 0x40)  # waiting for PC only
        self.assertEqual(self.states[0].imu_age_us, 5)
        self.assertEqual(self.states[0].encoder_age_us, 5)

    def test_genuinely_stale_sample_still_latches_stop(self):
        self.assertEqual(self.states[1].fault, 0x42)
        self.assertGreater(self.states[1].imu_age_us, 20000)
        self.assertEqual(self.states[2].fault, 0x42)
        self.assertEqual(self.states[2].imu_age_us, 5)


class LinkTimeoutTests(_ControlHarness):
    # Exercise the actual task/guard when the UDP peer lease remains live but
    # COMMAND packets disappear. New commands must restore only LINK status.
    harness = HARNESS.replace("cycle == 3", "cycle == 9").replace(
        "*events = EVENT_SEND_STATE;",
        "*events = EVENT_SEND_STATE | ((cycle == 2 || cycle == 3 || cycle == 8) ? EVENT_COMMAND : 0);")
    harness = harness.replace("cycle == 2 ? clock_us - 20001 : clock_us + 5", "clock_us + 5")
    harness = harness.replace("return pdFALSE;", r'''
    *(command_message_t *)output = (command_message_t){
        .sequence = cycle - 1,
        .command = {
            .boot_id = 1234, .state_timestamp_us = clock_us - BIKE_CONTROL_PERIOD_US - 10,
            .torque_nm = 1.0f, .flags = 0,
        },
    };
    return pdTRUE;
''')
    harness = harness.replace("esp_err_t odrive_port_stop(void) { return ESP_OK; }", r'''
esp_err_t odrive_port_stop(void) {
    assert(cycle == 1 || (cycle >= 5 && cycle <= 7));
    return ESP_OK;
}''')
    harness = harness.replace("assert(!command->enable && command->flywheel_torque_nm == 0);", r'''
    assert(cycle == 2 || cycle == 3 || cycle == 4 || cycle == 8 || cycle == 9);
    assert(!command->enable && command->flywheel_torque_nm == 0);
''')

    def test_live_peer_without_commands_times_out_and_recovers_disabled(self):
        self.assertEqual([s.fault for s in self.states], [64, 0, 0, 0, 64, 64, 64, 0, 0])
        for state in self.states[4:7]:
            self.assertGreaterEqual(state.command_age_us, 50000)
            self.assertFalse(state.flags & 16)  # COMMAND_VALID cleared
            self.assertEqual(state.acknowledged_sequence, 2)
        self.assertTrue(self.states[7].flags & 16)
        self.assertEqual(self.states[7].acknowledged_sequence, 7)
        self.assertTrue(all(s.flags & 8 for s in self.states))  # MOTOR_DISABLED always


if __name__ == "__main__":
    unittest.main()
