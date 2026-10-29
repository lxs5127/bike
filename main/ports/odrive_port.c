#include "odrive_port.h"

#include <math.h>
#include "sdkconfig.h"
#include "esp_timer.h"

esp_err_t odrive_port_init(void)
{
#if CONFIG_BIKE_MOCK_HARDWARE
    return ESP_OK;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t odrive_port_read_feedback(odrive_feedback_t *feedback)
{
    if (feedback == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
#if CONFIG_BIKE_MOCK_HARDWARE
    *feedback = (odrive_feedback_t) {
        .flywheel_rad_s = 0.0f,
        .timestamp_us = esp_timer_get_time(),
        .axis_error = 0,
        .valid = true,
        .ready = true,
    };
    return ESP_OK;
#else
    *feedback = (odrive_feedback_t) {0};
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t odrive_port_write_command(const odrive_command_t *command)
{
    if (command == NULL || !isfinite(command->flywheel_torque_nm)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (command->enable && command->valid_until_us <= esp_timer_get_time()) {
        return ESP_ERR_TIMEOUT;
    }
#if CONFIG_BIKE_MOCK_HARDWARE
    /* No GPIO/CAN/UART writes. Mock backend never energizes a motor. */
    return command->enable ? ESP_ERR_NOT_SUPPORTED : ESP_OK;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t odrive_port_stop(void)
{
    const odrive_command_t stop = { .enable = false, .flywheel_torque_nm = 0.0f };
    return odrive_port_write_command(&stop);
}
