#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "imu_port.h"

#define BIKE_WIFI_UDP_PORT 3334

/* 在网络任务中解析命令或提交请求，reply 写入可直接返回给客户端的文本。 */
typedef bool (*wifi_command_callback_t)(const char *command, char *reply, size_t capacity);

typedef struct {
    imu_sample_t imu;
    int mode;
    bool zero_valid, encoder_valid;
    float tilt_deg, speed_turns_s, torque_nm, wheel_target;
    float kp, kd, torque_limit, flywheel_limit, wheel_limit;
    int direction;
} wifi_telemetry_t;

/* 热点和 UDP 会话服务。网络断开只影响数据传输，本地控制继续运行。 */
esp_err_t wifi_link_init(wifi_command_callback_t command);
bool wifi_link_ready(void);

/* 从调用任务向网络任务提交新快照或事件。队列满时调用方不等待。 */
void wifi_link_publish(const wifi_telemetry_t *sample);
void wifi_link_event(const char *format, ...) __attribute__((format(printf, 1, 2)));
