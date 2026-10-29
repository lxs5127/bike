#ifndef IMU_PORT_H
#define IMU_PORT_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    float roll_rad;              /* radians, training body-X roll */
    float roll_rate_rad_s;       /* rad/s, gyro mapped into body frame */
    int64_t timestamp_us;        /* esp_timer 时基；JY901P 使用保守的采样时间估计 */
    bool valid;
} imu_sample_t;

esp_err_t imu_port_init(void);
/* Must return promptly (cached sample or bounded transaction), no infinite wait.
 * JY901P provides fused Roll + GX via I2C; mounting must be X forward/Y left/Z up.
 * I2C read time minus one output period estimates acquisition time; it cannot
 * detect internal fusion freezing while the register interface still responds.
 * Failed reads must return an invalid sample, never a refreshed cached value. */
esp_err_t imu_port_read(imu_sample_t *sample);

#endif
