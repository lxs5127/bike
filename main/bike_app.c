/* 应用层调度：周期采样 → 接收上位机力矩命令 → 安全检查 → 上报状态。
 * 当前原型只验证通信与控制链路，电机输出由配置固定禁用。 */
#include "bike_app.h"

#include <limits.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "app_config.h"
#include "bike_safety.h"
#include "bike_protocol.h"
#include "bike_link_guard.h"
#include "imu_port.h"
#include "odrive_port.h"
#include "pc_link_port.h"

/* 控制任务使用通知位接收事件；两个事件可在同一轮同时处理。 */
#define EVENT_SEND_STATE 1U  /* 周期定时器触发状态上报。 */
#define EVENT_COMMAND 2U     /* 接收任务已发布新的完整命令。 */

/* 两个队列均只有一个槽位，覆盖写入以保留最新状态和命令。 */
static QueueHandle_t s_state_queue, s_command_queue;
static TaskHandle_t s_sensor_task, s_control_task;
/* 每次启动随机生成，用于拒绝上一次启动遗留的命令。 */
static uint32_t s_boot_id;

typedef struct {
    uint32_t sequence;  /* 命令对应的状态帧序号，由链路保护模块校验。 */
    bike_wire_command_t command;
} command_message_t;

/* 定时器回调运行在 ESP_TIMER_TASK 任务中，而非中断中。
 * 回调只通知业务任务，避免在定时器任务中执行采样或通信。 */
static void wake_sensor(void *arg) { xTaskNotifyGive(s_sensor_task); }
static void wake_control(void *arg)
{
    xTaskNotify(s_control_task, EVENT_SEND_STATE, eSetBits);
}

/* 创建并启动周期定时器，周期单位为微秒；创建失败时直接返回错误。 */
static esp_err_t make_timer(const char *name, esp_timer_cb_t callback,
                            uint32_t period_us, esp_timer_handle_t *timer)
{
    const esp_timer_create_args_t args = {
        .callback = callback, .name = name,
        .dispatch_method = ESP_TIMER_TASK, .skip_unhandled_events = false,
    };
    esp_err_t result = esp_timer_create(&args, timer);
    return result == ESP_OK ? esp_timer_start_periodic(*timer, period_us) : result;
}

/* 合并 IMU 与驱动器反馈；采样有效性和驱动器健康状态分别记录。 */
static void read_state(bike_state_t *state)
{
    imu_sample_t imu = {0};
    odrive_feedback_t encoder = {0};
    esp_err_t imu_result = imu_port_read(&imu);
    esp_err_t drive_result = odrive_port_read_feedback(&encoder);
    *state = (bike_state_t) {
        .roll_rad = imu.roll_rad, .roll_rate_rad_s = imu.roll_rate_rad_s,
        .flywheel_rad_s = encoder.flywheel_rad_s,
        .imu_timestamp_us = imu.timestamp_us,
        .encoder_timestamp_us = encoder.timestamp_us,
        .valid = imu_result == ESP_OK && drive_result == ESP_OK && imu.valid && encoder.valid,
        .drive_healthy = encoder.ready && encoder.axis_error == 0,
    };
}

/* 等待采样通知，读取一次硬件并发布最新快照。 */
static void sensor_task(void *arg)
{
    while (1) {
        /* 一次清空累计通知，唤醒后只采样一次，避免追赶积压的采样周期。 */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        bike_state_t state;
        read_state(&state);
        xQueueOverwrite(s_state_queue, &state);
    }
}

/* 接收并解析上位机命令；命令是否可执行由控制任务进一步校验。 */
static void receive_task(void *arg)
{
    uint8_t bytes[128];
    while (1) {
        int count = pc_link_port_read(bytes, sizeof(bytes));
        /* 每次读取后重置解析器，避免把两个残缺的 UDP 数据报拼成命令。 */
        bike_command_parser_t parser = {0};
        /* 读取失败时短暂让出 CPU，避免连续失败形成忙等。 */
        if (count < 0) vTaskDelay(pdMS_TO_TICKS(20));
        for (int i = 0; i < count; ++i) {
            command_message_t message;
            if (bike_parse_command_byte(&parser, bytes[i], &message.sequence, &message.command)) {
                /* 覆盖旧命令，避免排队执行已经过时的力矩请求。 */
                xQueueOverwrite(s_command_queue, &message);
                xTaskNotify(s_control_task, EVENT_COMMAND, eSetBits);
            }
        }
    }
}

/* 将时间戳转换为微秒年龄；无效、来自未来或超出范围时返回哨兵值。 */
static uint32_t sample_age(int64_t timestamp, int64_t now_us)
{
    if (timestamp <= 0 || timestamp > now_us || now_us - timestamp > UINT32_MAX)
        return UINT32_MAX;
    return (uint32_t)(now_us - timestamp);
}

/* 统一处理状态校验、命令时效、电机停机及周期遥测上报。 */
static void control_task(void *arg)
{
    bike_link_guard_t guard;
    bike_link_guard_init(&guard, s_boot_id);
    /* 状态、时序和驱动写入故障在本次运行中锁存，恢复读数不会自动清除。 */
    uint32_t sequence = 0, latched_fault = BIKE_FAULT_NONE;
    int64_t previous_send_us = 0;
    uint32_t previous_fault = UINT32_MAX;
    while (1) {
        uint32_t events = 0;
        /* 取出并清空事件位；即使没有事件，也至多等待 20 ms 就重新检查安全状态。 */
        xTaskNotifyWait(0, UINT32_MAX, &events, pdMS_TO_TICKS(20));
        bike_state_t state = {0};
        /* 窥视队列而不移除快照，以便在下一次采样前继续检查同一份状态。 */
        if (xQueuePeek(s_state_queue, &state, 0) != pdTRUE) latched_fault |= BIKE_FAULT_SENSOR;
        /* 先取快照再读时钟：采样任务可能在读取队列期间抢占并发布新样本，
         * 提前读取时钟会把该新样本误判为来自未来。 */
        int64_t now_us = esp_timer_get_time();
        /* 检查数值、驱动状态、采样时效和倾倒角度；只对新出现的故障打印日志。 */
        uint32_t state_fault = bike_validate_state(&state, now_us);
        if (state_fault & ~latched_fault) {
            ESP_LOGE("bike", "New state fault=0x%02" PRIx32
                     " imu_age=%" PRId64 "us encoder_age=%" PRId64 "us limit=%uus",
                     state_fault, now_us - state.imu_timestamp_us,
                     now_us - state.encoder_timestamp_us, BIKE_MAX_SENSOR_AGE_US);
        }
        latched_fault |= state_fault;
        if (events & EVENT_COMMAND) {
            command_message_t message;
            if (xQueueReceive(s_command_queue, &message, 0) == pdTRUE) {
                /* 校验启动标识、帧序号、关联状态时间戳和力矩范围；
                 * 拒绝的命令不会更新 guard 中已接受的命令。 */
                bike_link_guard_accept(&guard, message.sequence, &message.command, now_us);
            }
        }
        if (events & EVENT_SEND_STATE) {
            /* 首次上报没有可比较的间隔；后续要求周期处于标称值的 50%～150%。 */
            int64_t period = previous_send_us ? now_us - previous_send_us : 0;
            previous_send_us = now_us;
            if (period && (period < BIKE_CONTROL_PERIOD_US / 2U ||
                           period > BIKE_CONTROL_PERIOD_US * 3U / 2U)) {
                if (!(latched_fault & BIKE_FAULT_TIMING)) {
                    ESP_LOGE("bike", "New timing fault: state period=%" PRId64 "us", period);
                }
                latched_fault |= BIKE_FAULT_TIMING;
            }
        }
        /* 链路故障按当前状态计算，不锁存；有效命令恢复后可自动解除该故障位。 */
        bool alive = pc_link_port_ready() && bike_link_guard_alive(&guard, now_us);
        uint32_t fault = latched_fault | (alive ? 0U : BIKE_FAULT_LINK);
        if (fault == BIKE_FAULT_NONE) {
            /* 原型没有电机解锁命令。收到的力矩仅用于链路验证，
             * 即使链路健康，当前配置下 ODrive 也保持禁用。 */
            /* 限制力矩幅值，并禁止在飞轮达到速度上限后继续同向加速。 */
            float torque = bike_limit_torque(guard.torque_nm, state.flywheel_rad_s);
            const odrive_command_t command = {
                .flywheel_torque_nm = BIKE_MOTOR_OUTPUT_ENABLED ? torque : 0.0f,
                /* 有效期从关联状态帧的时间戳计算，覆盖采样到命令返回的延迟。 */
                .valid_until_us = guard.source_timestamp_us + BIKE_LINK_TIMEOUT_US,
                .enable = BIKE_MOTOR_OUTPUT_ENABLED,
            };
            if (odrive_port_write_command(&command) != ESP_OK) latched_fault |= BIKE_FAULT_DRIVE;
        }
        /* 同时覆盖本轮新产生的驱动写入故障，确保立即请求停机。 */
        if (fault != BIKE_FAULT_NONE || latched_fault != BIKE_FAULT_NONE) odrive_port_stop();
        fault |= latched_fault;
        /* 仅在故障组合变化时打印控制链路状态，避免每轮重复输出。 */
        if (fault != previous_fault) {
            ESP_LOGI("bike", "CONTROL_LINK=%s fault=0x%02" PRIx32
                     " cmd_age=%" PRIu32 "us accepted_torque=%.4fNm applied=0Nm MOTOR_OUTPUT=OFF",
                     fault == BIKE_FAULT_NONE ? "OK" : "STOP", fault,
                     guard.seen_command ? sample_age(guard.source_timestamp_us, now_us) : UINT32_MAX,
                     fault == BIKE_FAULT_NONE ? (double)guard.torque_nm : 0.0);
            previous_fault = fault;
        }
        if (events & EVENT_SEND_STATE) {
            /* 只有周期事件发送遥测；单独收到命令时仍立即完成安全处理。 */
            ++sequence;
            bike_wire_state_t packet = {
                .boot_id = s_boot_id, .timestamp_us = now_us,
                .imu_age_us = sample_age(state.imu_timestamp_us, now_us),
                .encoder_age_us = sample_age(state.encoder_timestamp_us, now_us),
                .roll_rad = state.roll_rad, .roll_rate_rad_s = state.roll_rate_rad_s,
                .flywheel_rad_s = state.flywheel_rad_s, .fault = fault,
                /* 分别标记采样有效、驱动就绪、模拟模式、电机禁用及命令有效。 */
                .flags = (state.valid ? BIKE_STATE_VALID : 0) |
                         (state.drive_healthy ? BIKE_STATE_DRIVE_READY : 0) |
                         (CONFIG_BIKE_MOCK_HARDWARE ? BIKE_STATE_MOCK : 0) |
                         BIKE_STATE_MOTOR_DISABLED |
                         (alive ? BIKE_STATE_COMMAND_VALID : 0),
                /* 回传最近接受的命令序号、关联状态帧年龄及往返耗时。 */
                .acknowledged_sequence = guard.last_sequence,
                .command_age_us = guard.seen_command ?
                    sample_age(guard.source_timestamp_us, now_us) : UINT32_MAX,
                .round_trip_us = guard.round_trip_us,
            };
            uint8_t bytes[BIKE_PACKET_MAX_SIZE];
            size_t size = bike_encode_state(bytes, sequence, &packet);
            /* 记录帧序号与发送时间，供后续校验上位机返回命令的来源。 */
            bike_link_guard_sent(&guard, sequence, now_us);
            esp_err_t result = pc_link_port_send(bytes, size);
            if (result != ESP_OK && result != ESP_ERR_NOT_FOUND) {
                /* 当前禁用电机的原型将 Wi-Fi 断连视为可恢复故障：
                 * 使当前命令失效并停机，等待后续有效命令恢复链路。 */
                guard.valid = false;
                odrive_port_stop();
            }
        }
    }
}

/* 初始化硬件与链路，发布首帧采样，再创建任务并启动周期调度。 */
esp_err_t bike_app_start(void)
{
    /* 启动阶段先确保驱动停止；任一步失败都将错误交给调用者处理。 */
    esp_err_t result = imu_port_init();
    if (result != ESP_OK) return result;
    result = odrive_port_init();
    if (result != ESP_OK) return result;
    result = odrive_port_stop();
    if (result != ESP_OK) return result;
    /* 长度为 1 的队列配合 xQueueOverwrite，实现最新值邮箱。 */
    s_state_queue = xQueueCreate(1, sizeof(bike_state_t));
    s_command_queue = xQueueCreate(1, sizeof(command_message_t));
    if (!s_state_queue || !s_command_queue) return ESP_ERR_NO_MEM;
    s_boot_id = esp_random();
    result = pc_link_port_init();
    if (result != ESP_OK) return result;
    /* Wi-Fi 初始化可能耗时数百毫秒，完成后再采首帧，避免启动即触发过期故障。 */
    bike_state_t initial;
    read_state(&initial);
    xQueueOverwrite(s_state_queue, &initial);
#if CONFIG_FREERTOS_UNICORE
    const BaseType_t core = 0;
#else
    const BaseType_t core = 1;
#endif
    /* 采样与控制在双核设备上固定到核 1；单核设备使用核 0。
     * 接收任务固定到核 0。先创建全部任务，再启动通知它们的定时器。 */
    if (xTaskCreatePinnedToCore(sensor_task, "bike_sensor", BIKE_SENSOR_STACK_BYTES,
        NULL, BIKE_SENSOR_PRIORITY, &s_sensor_task, core) != pdPASS ||
        xTaskCreatePinnedToCore(control_task, "bike_control", BIKE_CONTROL_STACK_BYTES,
        NULL, BIKE_CONTROL_PRIORITY, &s_control_task, core) != pdPASS ||
        xTaskCreatePinnedToCore(receive_task, "bike_pc_rx", BIKE_RX_STACK_BYTES,
        NULL, BIKE_RX_PRIORITY, NULL, 0) != pdPASS) return ESP_ERR_NO_MEM;
    /* 采样按 200 Hz 唤醒，状态上报按约 60 Hz 唤醒，周期由 app_config.h 定义。 */
    esp_timer_handle_t sensor_timer, control_timer;
    result = make_timer("bike_sample", wake_sensor, BIKE_SENSOR_PERIOD_US, &sensor_timer);
    if (result != ESP_OK) return result;
    return make_timer("bike_upload", wake_control, BIKE_CONTROL_PERIOD_US, &control_timer);
}
