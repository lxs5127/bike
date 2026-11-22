#include "balance_loop.h"

#include <math.h>
#include <stddef.h>

#define MAX_ENCODER_AGE_US 100000LL
#define FALL_ANGLE_RAD 0.1745329252f  /* 距平衡零点 10°，直立调试范围 */

balance_result_t balance_compute(const balance_parameters_t *parameters,
                                const balance_input_t *input, bool zero_valid,
                                float zero_rad, int64_t now_us, float *torque_nm)
{
    if (!torque_nm) {
        return LOCAL_BALANCE_BAD_PARAMETERS;
    }
    *torque_nm = 0.f;
    if (!zero_valid || !isfinite(zero_rad)) {
        return LOCAL_BALANCE_NOT_CALIBRATED;
    }

    /* 先验证调参输入，再检查两路传感器的读数与时间戳。 */
    if (!parameters ||
        !isfinite(parameters->kp_nm_rad) || parameters->kp_nm_rad <= 0.f ||
        !isfinite(parameters->kd_nm_rad_s) || parameters->kd_nm_rad_s < 0.f ||
        !isfinite(parameters->torque_limit_nm) || parameters->torque_limit_nm <= 0.f ||
        !isfinite(parameters->flywheel_speed_limit_turns_s) ||
        parameters->flywheel_speed_limit_turns_s <= 0.f ||
        (parameters->torque_direction != 1 && parameters->torque_direction != -1)) {
        return LOCAL_BALANCE_BAD_PARAMETERS;
    }
    if (!input || !input->imu_valid || !input->encoder_valid ||
        !isfinite(input->roll_rad) || !isfinite(input->roll_rate_rad_s) ||
        !isfinite(input->flywheel_turns_s)) {
        return LOCAL_BALANCE_BAD_SENSOR;
    }
    if (input->imu_us <= 0 || input->encoder_us <= 0 ||
        input->imu_us > now_us || input->encoder_us > now_us ||
        now_us - input->imu_us > BALANCE_IMU_MAX_AGE_US ||
        now_us - input->encoder_us > MAX_ENCODER_AGE_US) {
        return LOCAL_BALANCE_STALE;
    }

    /* 将角差折回 ±π，避免 IMU 横滚角跨过 ±180° 时误判为大幅倾倒。 */
    const float roll_delta_rad = input->roll_rad - zero_rad;
    const float roll_error_rad = atan2f(sinf(roll_delta_rad), cosf(roll_delta_rad));
    if (fabsf(roll_error_rad) > FALL_ANGLE_RAD) {
        return LOCAL_BALANCE_FALL;
    }
    if (fabsf(input->flywheel_turns_s) >= parameters->flywheel_speed_limit_turns_s) {
        return LOCAL_BALANCE_OVERSPEED;
    }

    /* P 项用横滚角误差，D 项用 IMU 的 GX。
     * 编码器速度用于上面的停机判断，PD 公式只使用车身姿态。 */
    const float pd_torque_nm = (float)parameters->torque_direction *
                               (parameters->kp_nm_rad * roll_error_rad +
                                parameters->kd_nm_rad_s * input->roll_rate_rad_s);
    if (!isfinite(pd_torque_nm)) {
        return LOCAL_BALANCE_BAD_PARAMETERS;
    }
    *torque_nm = fmaxf(-parameters->torque_limit_nm,
                      fminf(parameters->torque_limit_nm, pd_torque_nm));
    return LOCAL_BALANCE_OK;
}
