#include "bike_safety.h"

#include <math.h>
#include "app_config.h"

bike_fault_t bike_validate_state(const bike_state_t *state, int64_t now_us)
{
    if (!state->valid || !isfinite(state->roll_rad) ||
        !isfinite(state->roll_rate_rad_s) || !isfinite(state->flywheel_rad_s)) {
        return BIKE_FAULT_SENSOR;
    }
    if (!state->drive_healthy) return BIKE_FAULT_DRIVE;
    if (state->imu_timestamp_us <= 0 || state->encoder_timestamp_us <= 0 ||
        state->imu_timestamp_us > now_us || state->encoder_timestamp_us > now_us ||
        now_us - state->imu_timestamp_us > BIKE_MAX_SENSOR_AGE_US ||
        now_us - state->encoder_timestamp_us > BIKE_MAX_SENSOR_AGE_US) {
        return BIKE_FAULT_STALE;
    }
    if (fabsf(state->roll_rad) > BIKE_FALL_ANGLE_RAD) return BIKE_FAULT_FALL;
    return BIKE_FAULT_NONE;
}

float bike_limit_torque(float torque_nm, float speed_rad_s)
{
    torque_nm = fminf(BIKE_TORQUE_LIMIT_NM, fmaxf(-BIKE_TORQUE_LIMIT_NM, torque_nm));
    /* Independent hard envelope, in addition to the laptop's smooth guard. */
    if ((speed_rad_s >= BIKE_SPEED_LIMIT_RAD_S && torque_nm > 0.0f) ||
        (speed_rad_s <= -BIKE_SPEED_LIMIT_RAD_S && torque_nm < 0.0f)) {
        return 0.0f;
    }
    return torque_nm;
}
