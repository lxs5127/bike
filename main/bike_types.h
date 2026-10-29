#ifndef BIKE_TYPES_H
#define BIKE_TYPES_H

#include <stdbool.h>
#include <stdint.h>

/* All quantities use the training frame: X forward, Y left, Z up.
 * Positive roll, body-X gyro and flywheel rotation follow the right-hand rule.
 * Drivers must convert mounting axes, signs and units before publishing. */
typedef struct {
    float roll_rad;
    float roll_rate_rad_s;
    float flywheel_rad_s;
    int64_t imu_timestamp_us;
    int64_t encoder_timestamp_us;
    bool valid;
    bool drive_healthy;
} bike_state_t;

typedef struct {
    float normalized_action;  /* after smoothing and speed guard */
    float torque_nm;          /* flywheel joint torque, NOT a speed command */
    float rotation_rad;       /* integral memory, reset on session start */
} bike_control_output_t;

typedef enum {
    BIKE_FAULT_NONE = 0,
    BIKE_FAULT_SENSOR = 1 << 0,
    BIKE_FAULT_STALE = 1 << 1,
    BIKE_FAULT_TIMING = 1 << 2,
    BIKE_FAULT_POLICY = 1 << 3,
    BIKE_FAULT_FALL = 1 << 4,
    BIKE_FAULT_DRIVE = 1 << 5,
    BIKE_FAULT_LINK = 1 << 6,
} bike_fault_t;

#endif
