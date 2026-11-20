#ifndef LOCAL_BALANCE_LOOP_H
#define LOCAL_BALANCE_LOOP_H
#include <stdbool.h>
#include <stdint.h>

/* 参数由实车调试提供，程序没有预设可用于车辆的增益或力矩上限。 */
typedef struct {
    float kp_nm_rad;
    float kd_nm_rad_s;
    float torque_limit_nm;
    float flywheel_speed_limit_turns_s;
    int torque_direction;  /* +1 或 -1，由实际飞轮反作用方向确定 */
} balance_parameters_t;

typedef struct {
    float roll_rad, roll_rate_rad_s, flywheel_turns_s;
    int64_t imu_us, encoder_us;
    bool imu_valid, encoder_valid;
} balance_input_t;

typedef enum {
    LOCAL_BALANCE_OK = 0,
    LOCAL_BALANCE_NOT_CALIBRATED,
    LOCAL_BALANCE_BAD_PARAMETERS,
    LOCAL_BALANCE_BAD_SENSOR,
    LOCAL_BALANCE_STALE,
    LOCAL_BALANCE_FALL,
    LOCAL_BALANCE_OVERSPEED,
} balance_result_t;

balance_result_t balance_compute(const balance_parameters_t *parameters,
                                const balance_input_t *input, bool zero_valid,
                                float zero_rad, int64_t now_us, float *torque_nm);
#endif
