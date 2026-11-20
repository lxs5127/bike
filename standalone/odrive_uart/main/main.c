/* ESP32 本地 UART 直立调试：0号反作用飞轮力矩，1号驱动轮速度。
 * 使用用户串口手册的 c / v / r / w 指令，不执行版本或电机校准检测。
 * 单一任务拥有 IMU、UART、NVS；Wi-Fi 通过单槽队列发布最新请求；断线不改变本地控制。
 */
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "wifi_link.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "imu_port.h"
#include "odrive_uart.h"
#include "balance_loop.h"
#include "sdkconfig.h"

#if !CONFIG_BIKE_IMU_JY901P
#error "UART motor control requires a real JY901P; synthetic IMU is not permitted."
#endif

#define PERIOD_MS 10U
#define RAD_TO_DEG 57.29577951308232f
static const char *TAG = "uart_balance";
typedef enum { CMD_ZERO, CMD_LIMITS, CMD_GAINS, CMD_BALANCE, CMD_TORQUE,
               CMD_SPEED, CMD_STATUS, CMD_STREAM, CMD_RECONNECT } command_kind_t;
typedef struct { command_kind_t kind; float a, b, c; int direction; unsigned ms; } command_t;
typedef enum { STOPPED, BALANCING, MANUAL_TORQUE, MANUAL_SPEED } app_mode_t;
static QueueHandle_t s_commands;
static atomic_bool s_stop_requested;
static balance_parameters_t s_parameters;
static float s_wheel_limit, s_zero, s_last_torque, s_wheel_target;
static bool s_zero_valid, s_stream = true, s_manual_target_pending;
static app_mode_t s_mode = STOPPED;
static balance_input_t s_input;
static imu_sample_t s_imu;
static int64_t s_motion_until, s_next_feedback;

static float angle_error(void)
{
    return atan2f(sinf(s_input.roll_rad-s_zero), cosf(s_input.roll_rad-s_zero));
}

static void halt(const char *reason)
{
    s_mode = STOPPED;
    s_last_torque = 0.f;
    s_wheel_target = 0.f;
    s_motion_until = 0;
    s_manual_target_pending = false;
    s_input.encoder_valid = false;
    esp_err_t result = odrive_uart_stop_all();
    wifi_link_event("STOP reason=%s stop_tx=%s\n", reason, esp_err_to_name(result));
}

static void load_zero(void)
{
    nvs_handle_t handle;
    int32_t value;
    s_zero_valid = false;
    if (nvs_open("bike_zero", NVS_READONLY, &handle) != ESP_OK) return;
    if (nvs_get_i32(handle, "roll_urad", &value) == ESP_OK &&
        value >= -3141593 && value <= 3141593) {
        s_zero = (float)value / 1000000.f;
        s_zero_valid = true;
    }
    nvs_close(handle);
}

static void calibrate_zero(void)
{
    if (s_mode != STOPPED) { wifi_link_event("%s","Send stop before zero."); return; }
    wifi_link_event("%s","Hold the bicycle at its confirmed upright reference; sampling for 2 seconds...");
    double sum_sin = 0., sum_cos = 0.;
    float first = 0.f;
    TickType_t wake = xTaskGetTickCount();
    for (unsigned i = 0; i < 200; ++i) {
        imu_sample_t sample;
        esp_err_t result = imu_port_read(&sample);
        if (atomic_load(&s_stop_requested) || result != ESP_OK || !sample.valid ||
            !isfinite(sample.roll_rad) || !isfinite(sample.roll_rate_rad_s) ||
            fabsf(sample.roll_rate_rad_s) > 2.f/RAD_TO_DEG) {
            wifi_link_event("%s","Zero not saved: sample invalid, moving, or stop requested."); return;
        }
        if (!i) first = sample.roll_rad;
        float spread = atan2f(sinf(sample.roll_rad-first), cosf(sample.roll_rad-first));
        if (fabsf(spread) > .5f/RAD_TO_DEG) { wifi_link_event("%s","Zero not saved: bicycle moved."); return; }
        sum_sin += sin(sample.roll_rad); sum_cos += cos(sample.roll_rad);
        xTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_MS));
    }
    float zero = (float)atan2(sum_sin, sum_cos);
    nvs_handle_t handle;
    esp_err_t result = nvs_open("bike_zero", NVS_READWRITE, &handle);
    if (result == ESP_OK) {
        result = nvs_set_i32(handle, "roll_urad", (int32_t)lroundf(zero*1000000.f));
        if (result == ESP_OK) result = nvs_commit(handle);
        nvs_close(handle);
    }
    if (result == ESP_OK) {
        s_zero = zero; s_zero_valid = true;
        wifi_link_event("ZERO saved to ESP32 NVS: raw_roll=%.4fdeg\n", (double)zero*RAD_TO_DEG);
    } else wifi_link_event("ZERO save failed: %s\n", esp_err_to_name(result));
}

static bool float_argument(const char *text, float *value)
{
    char *end;
    errno = 0; *value = strtof(text, &end);
    return end != text && !*end && errno != ERANGE && isfinite(*value);
}

static int command_dispatch(int argc, char **argv)
{
    command_t command = {0};
    const char *name = argv[0];
    if (!strcmp(name, "stop")) {
        atomic_store(&s_stop_requested, true); wifi_link_event("%s","Stop requested."); return 0;
    }
    if (!strcmp(name, "zero") && argc == 1) command.kind = CMD_ZERO;
    else if (!strcmp(name, "status") && argc == 1) command.kind = CMD_STATUS;
    else if (!strcmp(name, "reconnect") && argc == 1) command.kind = CMD_RECONNECT;
    else if (!strcmp(name, "balance") && argc == 1) command.kind = CMD_BALANCE;
    else if (!strcmp(name, "stream") && argc == 2 &&
             (!strcmp(argv[1], "on") || !strcmp(argv[1], "off"))) {
        command.kind = CMD_STREAM; command.direction = !strcmp(argv[1], "on");
    } else if (!strcmp(name, "limits") && argc == 4 &&
               float_argument(argv[1], &command.a) && float_argument(argv[2], &command.b) &&
               float_argument(argv[3], &command.c) && command.a > 0 && command.b > 0 && command.c > 0) {
        command.kind = CMD_LIMITS;
    } else if (!strcmp(name, "gains") && argc == 4 &&
               float_argument(argv[1], &command.a) && float_argument(argv[2], &command.b) &&
               (!strcmp(argv[3], "1") || !strcmp(argv[3], "-1")) && command.a > 0 && command.b >= 0) {
        command.kind = CMD_GAINS; command.direction = atoi(argv[3]);
    } else if ((!strcmp(name, "torque0") || !strcmp(name, "speed1")) &&
               (argc == 2 || argc == 3) && float_argument(argv[1], &command.a)) {
        command.kind = !strcmp(name, "torque0") ? CMD_TORQUE : CMD_SPEED;
        /* A duration is optional for compatibility with older clients. */
        if (argc == 3) {
            float duration;
            if (!float_argument(argv[2], &duration) ||
                duration < 100 || duration > 2000 || floorf(duration) != duration) {
                wifi_link_event("Optional duration must be an integer from 100 to 2000 ms.");
                return 1;
            }
            command.ms = (unsigned)duration;
        }
    } else {
        wifi_link_event("%s","Invalid command/arguments; see the Wi-Fi command table in README."); return 1;
    }
    xQueueOverwrite(s_commands, &command);
    wifi_link_event("%s","Request queued.");
    return 0;
}

static esp_err_t update_flywheel_encoder(void)
{
    float speed;
    int64_t timestamp;
    esp_err_t result = odrive_uart_read_velocity(ODRIVE_MOTOR0, &speed, &timestamp);
    s_input.encoder_valid = result == ESP_OK;
    if (result == ESP_OK) { s_input.flywheel_turns_s = speed; s_input.encoder_us = timestamp; }
    return result;
}

static void print_status(void)
{
    wifi_telemetry_t status = {
        .imu=s_imu, .mode=(int)s_mode, .zero_valid=s_zero_valid,
        .encoder_valid=s_input.encoder_valid,
        .tilt_deg=s_zero_valid ? angle_error()*RAD_TO_DEG : 0.f,
        .speed_turns_s=s_input.flywheel_turns_s, .torque_nm=s_last_torque,
        .wheel_target=s_wheel_target,
        .kp=s_parameters.kp_nm_rad, .kd=s_parameters.kd_nm_rad_s,
        .direction=s_parameters.torque_direction,
        .torque_limit=s_parameters.torque_limit_nm,
        .flywheel_limit=s_parameters.flywheel_speed_limit_turns_s,
        .wheel_limit=s_wheel_limit,
    };
    wifi_link_publish(&status);
}

static void halt_encoder_error(esp_err_t result)
{
    char reason[96];
    snprintf(reason, sizeof(reason), "ODrive axis0 encoder UART read failed: %s",
             esp_err_to_name(result));
    halt(reason);
}

static void apply_motion_command(const command_t *command)
{
    unsigned motor = command->kind == CMD_SPEED ? 1U : 0U;
    app_mode_t next_mode = command->kind == CMD_BALANCE ? BALANCING :
                          (motor == 0 ? MANUAL_TORQUE : MANUAL_SPEED);
    bool starting = s_mode == STOPPED;
    if (!starting && (s_mode != next_mode || next_mode == BALANCING)) {
        wifi_link_event("Send stop before switching control modes.");
        return;
    }
    if (s_parameters.torque_limit_nm <= 0 || s_parameters.flywheel_speed_limit_turns_s <= 0 || s_wheel_limit <= 0) {
        wifi_link_event("%s","Set real motor limits first: limits <Nm> <flywheel_turns/s> <wheel_turns/s>"); return;
    }
    if (command->kind == CMD_TORQUE && fabsf(command->a) > s_parameters.torque_limit_nm) {
        wifi_link_event("%s","Torque exceeds configured limit."); return;
    }
    if (command->kind == CMD_SPEED && fabsf(command->a) > s_wheel_limit) {
        wifi_link_event("%s","Wheel speed exceeds configured limit."); return;
    }
    if (starting) {
        s_input.encoder_valid = false;
        if (next_mode == BALANCING) {
            esp_err_t result = update_flywheel_encoder();
            if (result != ESP_OK) { halt_encoder_error(result); return; }
            float candidate;
            balance_result_t checked = balance_compute(&s_parameters, &s_input, s_zero_valid,
                                                       s_zero, esp_timer_get_time(), &candidate);
            if (checked != LOCAL_BALANCE_OK) { wifi_link_event("Balance not started: state=%d\n", (int)checked); return; }
            s_next_feedback = esp_timer_get_time() + 50000;
        }
        esp_err_t result = odrive_uart_enable(motor);
        if (result != ESP_OK) { halt("enable transmission failed"); return; }
    }
    s_mode = next_mode;
    s_motion_until = command->ms ? esp_timer_get_time() + (int64_t)command->ms*1000 : 0;
    s_wheel_target = s_mode == MANUAL_SPEED ? command->a : 0.f;
    s_last_torque = s_mode == MANUAL_TORQUE ? command->a : 0.f;
    if (s_mode == BALANCING) {
        wifi_link_event("Balance mode started.");
    } else {
        s_manual_target_pending = true;
    }
}

static void handle_command(const command_t *command)
{
    if (command->kind == CMD_STATUS) print_status();
    else if (command->kind == CMD_STREAM) s_stream = command->direction != 0;
    else if (command->kind == CMD_ZERO) {
        calibrate_zero();
        /* Cancel motion requests entered while sampling, including failed sampling. */
        xQueueReset(s_commands);
    }
    else if (command->kind == CMD_LIMITS || command->kind == CMD_GAINS) {
        if (s_mode != STOPPED) { wifi_link_event("%s","Send stop before changing parameters."); return; }
        if (command->kind == CMD_LIMITS) {
            s_parameters.torque_limit_nm = command->a;
            s_parameters.flywheel_speed_limit_turns_s = command->b;
            s_wheel_limit = command->c;
        } else {
            s_parameters.kp_nm_rad = command->a; s_parameters.kd_nm_rad_s = command->b;
            s_parameters.torque_direction = command->direction;
        }
        print_status();
    } else if (command->kind == CMD_RECONNECT) {
        if (s_mode != STOPPED) { wifi_link_event("%s","Send stop before reconnect."); return; }
        esp_err_t closed = odrive_uart_deinit();
        esp_err_t opened = odrive_uart_init();
        wifi_link_event("UART reconnect: previous_stop=%s open=%s\n", esp_err_to_name(closed), esp_err_to_name(opened));
    } else apply_motion_command(command);
}

static void update_imu(void)
{
    imu_sample_t imu = {0};
    esp_err_t result = imu_port_read(&imu);
    s_imu = imu;
    s_input.imu_valid = result == ESP_OK && imu.valid &&
                        isfinite(imu.roll_rad) && isfinite(imu.roll_rate_rad_s);
    if (s_input.imu_valid) {
        s_input.roll_rad = imu.roll_rad; s_input.roll_rate_rad_s = imu.roll_rate_rad_s;
        s_input.imu_us = imu.timestamp_us;
    }
}

static void control_task(void *arg)
{
    (void)arg;
    TickType_t wake = xTaskGetTickCount();
    while (1) {
        int64_t started = esp_timer_get_time();
        bool stopped = atomic_exchange(&s_stop_requested, false);
        if (stopped) {
            xQueueReset(s_commands); halt("operator");
        }
        update_imu();
        command_t command;
        if (!stopped && xQueueReceive(s_commands, &command, 0) == pdTRUE) {
            handle_command(&command);
            /* Configuration/zero/enable may block; begin a fresh control cycle afterwards. */
            started = esp_timer_get_time();
            update_imu();
            wake = xTaskGetTickCount();
        }
        if (atomic_exchange(&s_stop_requested, false)) {
            xQueueReset(s_commands); halt("operator");
        }
        if (s_mode != STOPPED) {
            int64_t now = esp_timer_get_time();
            if (!s_input.imu_valid || now-s_input.imu_us > 30000 || s_input.imu_us > now) halt("IMU invalid/stale");
            /* Manual targets use writes only. Balancing is the sole feedback consumer. */
            if (s_mode == BALANCING && now >= s_next_feedback) {
                esp_err_t result = update_flywheel_encoder();
                if (result != ESP_OK) halt_encoder_error(result);
                s_next_feedback = esp_timer_get_time() + 50000;
            }
            now = esp_timer_get_time();
            if (s_mode == BALANCING) {
                balance_result_t checked = balance_compute(&s_parameters, &s_input, s_zero_valid, s_zero, now, &s_last_torque);
                if (checked != LOCAL_BALANCE_OK) { wifi_link_event("Balance fault=%d\n", (int)checked); halt("balance guard"); }
            } else if (s_mode != STOPPED && s_motion_until > 0 && now >= s_motion_until) {
                halt("pulse duration complete");
            }
            if (s_mode != STOPPED && esp_timer_get_time()-started > 20000) halt("control overrun");
            if (atomic_exchange(&s_stop_requested, false)) {
                xQueueReset(s_commands); halt("operator");
            }
            if (s_mode != STOPPED) {
                esp_err_t result = s_mode == MANUAL_SPEED ? odrive_uart_set_motor1_velocity(s_wheel_target) :
                                                          odrive_uart_set_motor0_torque(s_last_torque);
                if (result != ESP_OK) {
                    halt("command UART transmission failed");
                } else if (s_manual_target_pending) {
                    s_manual_target_pending = false;
                    bool speed_mode = s_mode == MANUAL_SPEED;
                    wifi_link_event("UART TX complete: %s %.6g %s",
                                    speed_mode ? "axis1 speed" : "axis0 torque",
                                    (double)(speed_mode ? s_wheel_target : s_last_torque),
                                    speed_mode ? "turn/s" : "Nm");
                }
            }
        }
        if (s_stream) print_status();  /* Latest snapshot only; Wi-Fi task sends at 100 Hz. */
        if (xTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_MS)) != pdTRUE) wake = xTaskGetTickCount();
    }
}

/* Called by the Wi-Fi task. Execution remains in the local control task. */
static bool network_command(const char *text, char *reply, size_t capacity)
{
    char buffer[128], *argv[6], *save = NULL;
    if (strlen(text) >= sizeof(buffer)) return false;
    strcpy(buffer,text);
    int argc = 0;
    for (char *word = strtok_r(buffer," ",&save); word; word = strtok_r(NULL," ",&save)) {
        if (argc == 6) { snprintf(reply,capacity,"Too many arguments."); return false; }
        argv[argc++] = word;
    }
    if (!argc) { snprintf(reply,capacity,"Empty command."); return false; }
    int result = command_dispatch(argc,argv);
    snprintf(reply,capacity,"%s",result ? "Invalid command/arguments." : "Request queued; watch events and state for the result.");
    return result == 0;
}

void app_main(void)
{
    ESP_LOGI(TAG, "ODrive UART TX=%d RX=%d baud=%d; PC data/control use Wi-Fi UDP",
             ODRIVE_UART_TX_GPIO, ODRIVE_UART_RX_GPIO, ODRIVE_UART_BAUDRATE);
    esp_err_t result = odrive_uart_init();
    if (result != ESP_OK) { ESP_LOGE(TAG,"UART init: %s",esp_err_to_name(result)); return; }
    result = imu_port_init();
    if (result != ESP_OK) { halt("IMU init failed"); return; }
    result = nvs_flash_init();
    if (result != ESP_OK) { halt("NVS init failed"); return; }
    load_zero();
    s_commands = xQueueCreate(1,sizeof(command_t));
    if (!s_commands) { halt("command allocation failed"); return; }
    result = wifi_link_init(network_command);
    if (result != ESP_OK) { halt("Wi-Fi init failed"); ESP_LOGE(TAG,"Wi-Fi init: %s",esp_err_to_name(result)); return; }
    if (xTaskCreate(control_task,"balance",8192,NULL,10,NULL) != pdPASS) {
        halt("task allocation failed"); return;
    }
    ESP_LOGI(TAG,"STOPPED. Configure zero/limits/gains through Wi-Fi; disconnection does not stop local control.");
}
