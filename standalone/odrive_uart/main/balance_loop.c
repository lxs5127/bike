#include "balance_loop.h"
#include <math.h>
#include <stddef.h>

#define MAX_IMU_AGE_US 30000LL
#define MAX_ENCODER_AGE_US 100000LL
#define FALL_RAD 0.1745329252f  /* 10°，直立调试范围 */

balance_result_t balance_compute(const balance_parameters_t *p, const balance_input_t *s,
                                bool zero_valid, float zero, int64_t now, float *torque)
{
    if (!torque) return LOCAL_BALANCE_BAD_PARAMETERS;
    *torque = 0.f;
    if (!zero_valid || !isfinite(zero)) return LOCAL_BALANCE_NOT_CALIBRATED;
    if (!p || !isfinite(p->kp_nm_rad) || p->kp_nm_rad <= 0.f ||
        !isfinite(p->kd_nm_rad_s) || p->kd_nm_rad_s < 0.f ||
        !isfinite(p->torque_limit_nm) || p->torque_limit_nm <= 0.f ||
        !isfinite(p->flywheel_speed_limit_turns_s) || p->flywheel_speed_limit_turns_s <= 0.f ||
        (p->torque_direction != 1 && p->torque_direction != -1)) return LOCAL_BALANCE_BAD_PARAMETERS;
    if (!s || !s->imu_valid || !s->encoder_valid || !isfinite(s->roll_rad) ||
        !isfinite(s->roll_rate_rad_s) || !isfinite(s->flywheel_turns_s)) return LOCAL_BALANCE_BAD_SENSOR;
    if (s->imu_us <= 0 || s->encoder_us <= 0 || s->imu_us > now || s->encoder_us > now ||
        now - s->imu_us > MAX_IMU_AGE_US || now - s->encoder_us > MAX_ENCODER_AGE_US) return LOCAL_BALANCE_STALE;
    float angle = atan2f(sinf(s->roll_rad-zero), cosf(s->roll_rad-zero));
    if (fabsf(angle) > FALL_RAD) return LOCAL_BALANCE_FALL;
    if (fabsf(s->flywheel_turns_s) >= p->flywheel_speed_limit_turns_s) return LOCAL_BALANCE_OVERSPEED;
    float raw = (float)p->torque_direction * (p->kp_nm_rad*angle + p->kd_nm_rad_s*s->roll_rate_rad_s);
    if (!isfinite(raw)) return LOCAL_BALANCE_BAD_PARAMETERS;
    *torque = fmaxf(-p->torque_limit_nm, fminf(p->torque_limit_nm, raw));
    return LOCAL_BALANCE_OK;
}
