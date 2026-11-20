#pragma once
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "imu_port.h"

#define BIKE_WIFI_UDP_PORT 3334
typedef bool (*wifi_command_callback_t)(const char *, char *, size_t);
typedef struct {
    imu_sample_t imu;
    int mode;
    bool zero_valid, encoder_valid;
    float tilt_deg, speed_turns_s, torque_nm, wheel_target;
    float kp, kd, torque_limit, flywheel_limit, wheel_limit;
    int direction;
} wifi_telemetry_t;
/* Loss of Wi-Fi only affects data transport, never the local controller. */
esp_err_t wifi_link_init(wifi_command_callback_t command);
bool wifi_link_ready(void);
void wifi_link_publish(const wifi_telemetry_t *sample);
void wifi_link_event(const char *format, ...) __attribute__((format(printf,1,2)));
