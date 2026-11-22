/*
 * ESP32 本地控制入口，同时供根工程和独立 ODrive 工程使用。
 * 电机 0 通过反作用飞轮提供横滚力矩，电机 1 提供驱动轮转速。
 * Wi-Fi 任务只解析指令并提交请求。控制任务独占 IMU、UART 和 NVS，
 * 以 10 ms 为目标周期执行本地控制。已生效的目标由控制任务保存。
 */
#include "uart_balance_app.h"

#include <errno.h>
#include <math.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "balance_loop.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "imu_port.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "odrive_uart.h"
#include "sdkconfig.h"
#include "wifi_link.h"

#if !CONFIG_BIKE_IMU_JY901P
#error "UART motor control requires a real JY901P; synthetic IMU is not permitted."
#endif

#define CONTROL_PERIOD_MS 10U
#define FLYWHEEL_FEEDBACK_PERIOD_US 50000LL
#define CONTROL_OVERRUN_US 20000LL
#define ZERO_SAMPLE_COUNT 200U
#define ZERO_MAX_RATE_DEG_S 2.f
#define ZERO_MAX_SPREAD_DEG 0.5f
#define RAD_TO_DEG 57.29577951308232f
#define MICRORAD_PER_RAD 1000000.f
#define MAX_ABS_ZERO_MICRORAD 3141593  /* π rad，四舍五入到整数微弧度 */
#define ZERO_NVS_NAMESPACE "bike_zero"
#define ZERO_NVS_KEY "roll_urad"

static const char *TAG = "uart_balance";

typedef enum {
    CMD_ZERO,
    CMD_LIMITS,
    CMD_GAINS,
    CMD_BALANCE,
    CMD_TORQUE,
    CMD_SPEED,
    CMD_STATUS,
    CMD_STREAM,
    CMD_RECONNECT,
} command_kind_t;

typedef struct {
    command_kind_t kind;
    union {
        struct {
            float torque_nm;
            float flywheel_turns_s;
            float drive_wheel_turns_s;
        } limits;
        struct {
            float kp_nm_rad;
            float kd_nm_rad_s;
            int torque_direction;
        } gains;
        struct {
            float target;         /* CMD_TORQUE 用 N·m，CMD_SPEED 用圈/s */
            unsigned duration_ms; /* 0 表示保持目标，非零用于兼容旧客户端 */
        } motion;
        bool stream_enabled;
    } data;
} command_t;

/* 模式编号会经 Wi-Fi 发送，查看器用它决定可操作的按钮。 */
typedef enum {
    STOPPED = 0,
    BALANCING = 1,
    MANUAL_TORQUE = 2,
    MANUAL_SPEED = 3,
} app_mode_t;

/* 普通请求只保留最新一条。停机用单独的原子标志，避免被新请求覆盖。 */
static QueueHandle_t s_commands;
static atomic_bool s_stop_requested;

static balance_parameters_t s_parameters;
static float s_wheel_limit_turns_s;
static float s_zero_rad;
static float s_torque_target_nm;
static float s_wheel_target_turns_s;
static bool s_zero_valid;
static bool s_stream = true;
static bool s_manual_target_pending; /* 目标首次由 UART 发完后给出回执 */
static app_mode_t s_mode = STOPPED;
static balance_input_t s_input;
static imu_sample_t s_imu;
static int64_t s_motion_until_us;
static int64_t s_next_feedback_us;

/* 角差按一整圈折回到 ±π，避免横滚角在 ±180° 处跳变。 */
static float wrapped_angle_error_rad(float angle_rad, float reference_rad)
{
    const float difference_rad = angle_rad - reference_rad;
    return atan2f(sinf(difference_rad), cosf(difference_rad));
}

static void halt(const char *reason)
{
    /* 先清除本地输出，再向 ODrive 两轴请求 IDLE 并写零目标。 */
    s_mode = STOPPED;
    s_torque_target_nm = 0.f;
    s_wheel_target_turns_s = 0.f;
    s_motion_until_us = 0;
    s_manual_target_pending = false;
    s_input.encoder_valid = false;
    esp_err_t result = odrive_uart_stop_all();
    wifi_link_event("STOP reason=%s stop_tx=%s\n", reason, esp_err_to_name(result));
}

static bool handle_stop_request(void)
{
    if (!atomic_exchange(&s_stop_requested, false)) {
        return false;
    }
    xQueueReset(s_commands);
    halt("operator");
    return true;
}

static void load_zero(void)
{
    nvs_handle_t handle;
    int32_t zero_microrad;
    s_zero_valid = false;
    if (nvs_open(ZERO_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return;
    }
    if (nvs_get_i32(handle, ZERO_NVS_KEY, &zero_microrad) == ESP_OK &&
        zero_microrad >= -MAX_ABS_ZERO_MICRORAD && zero_microrad <= MAX_ABS_ZERO_MICRORAD) {
        s_zero_rad = (float)zero_microrad / MICRORAD_PER_RAD;
        s_zero_valid = true;
    }
    nvs_close(handle);
}

static void calibrate_zero(void)
{
    if (s_mode != STOPPED) {
        wifi_link_event("Send stop before zero.");
        return;
    }
    wifi_link_event("Hold the bicycle at its confirmed upright reference; sampling for 2 seconds...");
    double sum_sin = 0.;
    double sum_cos = 0.;
    float first_roll_rad = 0.f;
    TickType_t wake = xTaskGetTickCount();
    for (unsigned i = 0; i < ZERO_SAMPLE_COUNT; ++i) {
        imu_sample_t sample;
        esp_err_t result = imu_port_read(&sample);
        /* 角速度和相对首帧的偏差都需足够小，避免把倾倒过程写成平衡参考。 */
        if (atomic_load(&s_stop_requested) || result != ESP_OK || !sample.valid ||
            !isfinite(sample.roll_rad) || !isfinite(sample.roll_rate_rad_s) ||
            fabsf(sample.roll_rate_rad_s) > ZERO_MAX_RATE_DEG_S / RAD_TO_DEG) {
            wifi_link_event("Zero not saved: sample invalid, moving, or stop requested.");
            return;
        }
        if (i == 0) {
            first_roll_rad = sample.roll_rad;
        }
        const float spread_rad = wrapped_angle_error_rad(sample.roll_rad, first_roll_rad);
        if (fabsf(spread_rad) > ZERO_MAX_SPREAD_DEG / RAD_TO_DEG) {
            wifi_link_event("Zero not saved: bicycle moved.");
            return;
        }
        /* 圆周平均。179.9° 与 -179.9° 的平均应落在 ±180° 附近。 */
        sum_sin += sin(sample.roll_rad);
        sum_cos += cos(sample.roll_rad);
        xTaskDelayUntil(&wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
    const float zero_rad = (float)atan2(sum_sin, sum_cos);
    nvs_handle_t handle;
    esp_err_t result = nvs_open(ZERO_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (result == ESP_OK) {
        result = nvs_set_i32(handle, ZERO_NVS_KEY, (int32_t)lroundf(zero_rad * MICRORAD_PER_RAD));
        if (result == ESP_OK) {
            result = nvs_commit(handle);
        }
        nvs_close(handle);
    }
    /* 写入成功才更新运行时零点，采样或保存失败时保留原值。 */
    if (result == ESP_OK) {
        s_zero_rad = zero_rad;
        s_zero_valid = true;
        wifi_link_event("ZERO saved to ESP32 NVS: raw_roll=%.4fdeg\n", (double)zero_rad * RAD_TO_DEG);
    } else {
        wifi_link_event("ZERO save failed: %s\n", esp_err_to_name(result));
    }
}

static bool float_argument(const char *text, float *value)
{
    char *end;
    errno = 0;
    *value = strtof(text, &end);
    return end != text && !*end && errno != ERANGE && isfinite(*value);
}

/* Wi-Fi 线程只检查格式和提交请求。限幅、模式和传感器检查在控制任务内执行。 */
static bool enqueue_command(int argc, char **argv)
{
    command_t command = {0};
    const char *name = argv[0];
    if (!strcmp(name, "stop")) {
        atomic_store(&s_stop_requested, true);
        wifi_link_event("Stop requested.");
        return true;
    }

    if (!strcmp(name, "zero") && argc == 1) {
        command.kind = CMD_ZERO;
    } else if (!strcmp(name, "status") && argc == 1) {
        command.kind = CMD_STATUS;
    } else if (!strcmp(name, "reconnect") && argc == 1) {
        command.kind = CMD_RECONNECT;
    } else if (!strcmp(name, "balance") && argc == 1) {
        command.kind = CMD_BALANCE;
    } else if (!strcmp(name, "stream") && argc == 2 &&
               (!strcmp(argv[1], "on") || !strcmp(argv[1], "off"))) {
        command.kind = CMD_STREAM;
        command.data.stream_enabled = !strcmp(argv[1], "on");
    } else if (!strcmp(name, "limits") && argc == 4 &&
               float_argument(argv[1], &command.data.limits.torque_nm) &&
               float_argument(argv[2], &command.data.limits.flywheel_turns_s) &&
               float_argument(argv[3], &command.data.limits.drive_wheel_turns_s) &&
               command.data.limits.torque_nm > 0.f &&
               command.data.limits.flywheel_turns_s > 0.f &&
               command.data.limits.drive_wheel_turns_s > 0.f) {
        command.kind = CMD_LIMITS;
    } else if (!strcmp(name, "gains") && argc == 4 &&
               float_argument(argv[1], &command.data.gains.kp_nm_rad) &&
               float_argument(argv[2], &command.data.gains.kd_nm_rad_s) &&
               (!strcmp(argv[3], "1") || !strcmp(argv[3], "-1")) &&
               command.data.gains.kp_nm_rad > 0.f && command.data.gains.kd_nm_rad_s >= 0.f) {
        command.kind = CMD_GAINS;
        command.data.gains.torque_direction = atoi(argv[3]);
    } else if ((!strcmp(name, "torque0") || !strcmp(name, "speed1")) &&
               (argc == 2 || argc == 3) && float_argument(argv[1], &command.data.motion.target)) {
        command.kind = !strcmp(name, "torque0") ? CMD_TORQUE : CMD_SPEED;
        command.data.motion.duration_ms = 0;
        /* 只有旧客户端显式附带时长时，固件才安排到时停机。 */
        if (argc == 3) {
            float duration_ms;
            if (!float_argument(argv[2], &duration_ms) ||
                duration_ms < 100 || duration_ms > 2000 || floorf(duration_ms) != duration_ms) {
                wifi_link_event("Optional duration must be an integer from 100 to 2000 ms.");
                return false;
            }
            command.data.motion.duration_ms = (unsigned)duration_ms;
        }
    } else {
        wifi_link_event("Invalid command/arguments; see the Wi-Fi command table in README.");
        return false;
    }

    xQueueOverwrite(s_commands, &command);
    wifi_link_event("Request queued.");
    return true;
}

static esp_err_t update_flywheel_encoder(void)
{
    float speed_turns_s;
    int64_t timestamp_us;
    esp_err_t result = odrive_uart_read_velocity(ODRIVE_MOTOR0, &speed_turns_s, &timestamp_us);
    s_input.encoder_valid = result == ESP_OK;
    if (result == ESP_OK) {
        s_input.flywheel_turns_s = speed_turns_s;
        s_input.encoder_us = timestamp_us;
    }
    return result;
}

static void publish_status(void)
{
    /* Wi-Fi 获得一份快照。网络收发不占用控制任务的 UART 或 IMU。 */
    const float tilt_deg = s_zero_valid
        ? wrapped_angle_error_rad(s_input.roll_rad, s_zero_rad) * RAD_TO_DEG
        : 0.f;
    wifi_telemetry_t status = {
        .imu = s_imu,
        .mode = (int)s_mode,
        .zero_valid = s_zero_valid,
        .encoder_valid = s_input.encoder_valid,
        .tilt_deg = tilt_deg,
        .speed_turns_s = s_input.flywheel_turns_s,
        .torque_nm = s_torque_target_nm,
        .wheel_target = s_wheel_target_turns_s,
        .kp = s_parameters.kp_nm_rad,
        .kd = s_parameters.kd_nm_rad_s,
        .direction = s_parameters.torque_direction,
        .torque_limit = s_parameters.torque_limit_nm,
        .flywheel_limit = s_parameters.flywheel_speed_limit_turns_s,
        .wheel_limit = s_wheel_limit_turns_s,
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
    const unsigned motor = command->kind == CMD_SPEED ? ODRIVE_MOTOR1 : ODRIVE_MOTOR0;
    const app_mode_t next_mode = command->kind == CMD_BALANCE ? BALANCING :
                                (motor == ODRIVE_MOTOR0 ? MANUAL_TORQUE : MANUAL_SPEED);
    const bool starting = s_mode == STOPPED;
    /* 同一手动模式可原地更新目标，切换电机或退出平衡前先停机。 */
    if (!starting && (s_mode != next_mode || next_mode == BALANCING)) {
        wifi_link_event("Send stop before switching control modes.");
        return;
    }
    if (s_parameters.torque_limit_nm <= 0.f || s_parameters.flywheel_speed_limit_turns_s <= 0.f ||
        s_wheel_limit_turns_s <= 0.f) {
        wifi_link_event("Set real motor limits first: limits <Nm> <flywheel_turns/s> <wheel_turns/s>");
        return;
    }
    if (command->kind == CMD_TORQUE && fabsf(command->data.motion.target) > s_parameters.torque_limit_nm) {
        wifi_link_event("Torque exceeds configured limit.");
        return;
    }
    if (command->kind == CMD_SPEED && fabsf(command->data.motion.target) > s_wheel_limit_turns_s) {
        wifi_link_event("Wheel speed exceeds configured limit.");
        return;
    }

    if (starting) {
        s_input.encoder_valid = false;
        if (next_mode == BALANCING) {
            /* 平衡先用新鲜的飞轮反馈和 IMU 样本检查能否启动。
             * 候选力矩只用于校验，实际值在控制周期内重新计算并发送。 */
            esp_err_t result = update_flywheel_encoder();
            if (result != ESP_OK) {
                halt_encoder_error(result);
                return;
            }
            float candidate_nm;
            balance_result_t checked = balance_compute(&s_parameters, &s_input, s_zero_valid,
                                                       s_zero_rad, esp_timer_get_time(), &candidate_nm);
            if (checked != LOCAL_BALANCE_OK) {
                wifi_link_event("Balance not started: state=%d\n", (int)checked);
                return;
            }
            s_next_feedback_us = esp_timer_get_time() + FLYWHEEL_FEEDBACK_PERIOD_US;
        }
        /* 手动模式直接写使能指令。两种模式都在 UART 发送成功后切换本地状态。 */
        esp_err_t result = odrive_uart_enable(motor);
        if (result != ESP_OK) {
            halt("enable transmission failed");
            return;
        }
    }

    s_mode = next_mode;
    const unsigned duration_ms = next_mode == BALANCING ? 0 : command->data.motion.duration_ms;
    s_motion_until_us = duration_ms ? esp_timer_get_time() + (int64_t)duration_ms * 1000 : 0;
    s_wheel_target_turns_s = s_mode == MANUAL_SPEED ? command->data.motion.target : 0.f;
    s_torque_target_nm = s_mode == MANUAL_TORQUE ? command->data.motion.target : 0.f;
    if (s_mode == BALANCING) {
        wifi_link_event("Balance mode started.");
    } else {
        s_manual_target_pending = true;
    }
}

static void handle_command(const command_t *command)
{
    switch (command->kind) {
    case CMD_STATUS:
        publish_status();
        break;
    case CMD_STREAM:
        s_stream = command->data.stream_enabled;
        break;
    case CMD_ZERO:
        calibrate_zero();
        /* 标定期间可能收到新的运行请求。结束后清队列，避免标定一结束就启动。 */
        xQueueReset(s_commands);
        break;
    case CMD_LIMITS:
    case CMD_GAINS:
        if (s_mode != STOPPED) {
            wifi_link_event("Send stop before changing parameters.");
            return;
        }
        if (command->kind == CMD_LIMITS) {
            s_parameters.torque_limit_nm = command->data.limits.torque_nm;
            s_parameters.flywheel_speed_limit_turns_s = command->data.limits.flywheel_turns_s;
            s_wheel_limit_turns_s = command->data.limits.drive_wheel_turns_s;
        } else {
            s_parameters.kp_nm_rad = command->data.gains.kp_nm_rad;
            s_parameters.kd_nm_rad_s = command->data.gains.kd_nm_rad_s;
            s_parameters.torque_direction = command->data.gains.torque_direction;
        }
        publish_status();
        break;
    case CMD_RECONNECT: {
        if (s_mode != STOPPED) {
            wifi_link_event("Send stop before reconnect.");
            return;
        }
        esp_err_t closed = odrive_uart_deinit();
        esp_err_t opened = odrive_uart_init();
        wifi_link_event("UART reconnect: previous_stop=%s open=%s\n",
                        esp_err_to_name(closed), esp_err_to_name(opened));
        break;
    }
    case CMD_BALANCE:
    case CMD_TORQUE:
    case CMD_SPEED:
        apply_motion_command(command);
        break;
    }
}

static void update_imu(void)
{
    imu_sample_t imu = {0};
    esp_err_t result = imu_port_read(&imu);
    s_imu = imu;
    s_input.imu_valid = result == ESP_OK && imu.valid &&
                        isfinite(imu.roll_rad) && isfinite(imu.roll_rate_rad_s);
    if (s_input.imu_valid) {
        s_input.roll_rad = imu.roll_rad;
        s_input.roll_rate_rad_s = imu.roll_rate_rad_s;
        s_input.imu_us = imu.timestamp_us;
    }
}

static void send_motor_output(void)
{
    const bool speed_mode = s_mode == MANUAL_SPEED;
    esp_err_t result = speed_mode
        ? odrive_uart_set_motor1_velocity(s_wheel_target_turns_s)
        : odrive_uart_set_motor0_torque(s_torque_target_nm);

    if (result != ESP_OK) {
        halt("command UART transmission failed");
    } else if (s_manual_target_pending) {
        /* 只通知一次新目标已从 ESP32 发出。ODrive 的响应不参与手动设置。 */
        s_manual_target_pending = false;
        wifi_link_event("UART TX complete: %s %.6g %s",
                        speed_mode ? "axis1 speed" : "axis0 torque",
                        (double)(speed_mode ? s_wheel_target_turns_s : s_torque_target_nm),
                        speed_mode ? "turn/s" : "Nm");
    }
}

static void run_motor_cycle(int64_t started_us)
{
    int64_t now_us = esp_timer_get_time();
    if (!s_input.imu_valid || now_us - s_input.imu_us > BALANCE_IMU_MAX_AGE_US ||
        s_input.imu_us > now_us) {
        halt("IMU invalid/stale");
    }

    /* 飞轮反馈约每 50 ms 查询一次，只供平衡模式检查速度和数据有效性。 */
    if (s_mode == BALANCING && now_us >= s_next_feedback_us) {
        esp_err_t result = update_flywheel_encoder();
        if (result != ESP_OK) {
            halt_encoder_error(result);
        }
        s_next_feedback_us = esp_timer_get_time() + FLYWHEEL_FEEDBACK_PERIOD_US;
    }

    now_us = esp_timer_get_time();
    if (s_mode == BALANCING) {
        balance_result_t checked = balance_compute(&s_parameters, &s_input, s_zero_valid,
                                                   s_zero_rad, now_us, &s_torque_target_nm);
        if (checked != LOCAL_BALANCE_OK) {
            wifi_link_event("Balance fault=%d\n", (int)checked);
            halt("balance guard");
        }
    } else if (s_mode != STOPPED && s_motion_until_us > 0 && now_us >= s_motion_until_us) {
        halt("pulse duration complete");
    }

    /* UART 查询期间可能收到停机指令。发任何运行目标前再检查一次。 */
    if (s_mode != STOPPED && esp_timer_get_time() - started_us > CONTROL_OVERRUN_US) {
        halt("control overrun");
    }
    handle_stop_request();
    if (s_mode != STOPPED) {
        send_motor_output();
    }
}

static void control_task(void *arg)
{
    (void)arg;
    TickType_t wake = xTaskGetTickCount();
    while (1) {
        int64_t started_us = esp_timer_get_time();
        const bool stopped = handle_stop_request();
        update_imu();

        command_t command;
        if (!stopped && xQueueReceive(s_commands, &command, 0) == pdTRUE) {
            handle_command(&command);
            /* 标定、重连和使能耗时较长。处理完请求后重采 IMU 并重新计时。 */
            started_us = esp_timer_get_time();
            update_imu();
            wake = xTaskGetTickCount();
        }

        handle_stop_request();
        if (s_mode != STOPPED) {
            run_motor_cycle(started_us);
        }
        if (s_stream) {
            publish_status();
        }
        /* 以固定周期唤醒。若已错过周期，从当前时刻重新排程，避免连续补跑。 */
        if (xTaskDelayUntil(&wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS)) != pdTRUE) {
            wake = xTaskGetTickCount();
        }
    }
}

/* 给 Wi-Fi 任务的回调。此处的成功表示已接受请求，执行结果由控制任务上报。 */
static bool network_command(const char *text, char *reply, size_t capacity)
{
    char buffer[128];
    char *argv[6];
    char *save = NULL;
    if (strlen(text) >= sizeof(buffer)) {
        return false;
    }
    strcpy(buffer, text);

    int argc = 0;
    for (char *word = strtok_r(buffer, " ", &save); word; word = strtok_r(NULL, " ", &save)) {
        if (argc == (int)(sizeof(argv) / sizeof(argv[0]))) {
            snprintf(reply, capacity, "Too many arguments.");
            return false;
        }
        argv[argc++] = word;
    }
    if (argc == 0) {
        snprintf(reply, capacity, "Empty command.");
        return false;
    }
    const bool accepted = enqueue_command(argc, argv);
    snprintf(reply, capacity, "%s", accepted
             ? "Request queued; watch events and state for the result."
             : "Invalid command/arguments.");
    return accepted;
}

void uart_balance_start(void)
{
    ESP_LOGI(TAG, "ODrive UART TX=%d RX=%d baud=%d; PC data/control use Wi-Fi UDP",
             ODRIVE_UART_TX_GPIO, ODRIVE_UART_RX_GPIO, ODRIVE_UART_BAUDRATE);
    /* 先向两轴发送 IDLE，再准备传感器和队列。等应用就绪后才对外接收指令。 */
    esp_err_t result = odrive_uart_init();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "UART init: %s", esp_err_to_name(result));
        return;
    }
    result = imu_port_init();
    if (result != ESP_OK) {
        halt("IMU init failed");
        return;
    }
    result = nvs_flash_init();
    if (result != ESP_OK) {
        halt("NVS init failed");
        return;
    }

    load_zero();
    s_commands = xQueueCreate(1, sizeof(command_t));
    if (!s_commands) {
        halt("command allocation failed");
        return;
    }
    result = wifi_link_init(network_command);
    if (result != ESP_OK) {
        halt("Wi-Fi init failed");
        ESP_LOGE(TAG, "Wi-Fi init: %s", esp_err_to_name(result));
        return;
    }
    if (xTaskCreate(control_task, "balance", 8192, NULL, 10, NULL) != pdPASS) {
        halt("task allocation failed");
        return;
    }
    ESP_LOGI(TAG, "STOPPED. Configure zero/limits/gains through Wi-Fi; disconnection does not stop local control.");
}
