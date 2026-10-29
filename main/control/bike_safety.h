#ifndef BIKE_SAFETY_H
#define BIKE_SAFETY_H

#include "bike_types.h"

/* Simulation limits only. Real actuator limits/calibration remain unspecified. */
#define BIKE_TORQUE_LIMIT_NM 4.9f
#define BIKE_SPEED_LIMIT_RAD_S 250.0f
#define BIKE_FALL_ANGLE_RAD 0.1745329252f

bike_fault_t bike_validate_state(const bike_state_t *state, int64_t now_us);
float bike_limit_torque(float torque_nm, float speed_rad_s);

#endif
