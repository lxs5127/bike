#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdbool.h>

/* Training: (1/120)*2. MuJoCo: (1/240)*4. Both = 60 Hz. */
#define BIKE_CONTROL_HZ 60U
#define BIKE_CONTROL_DT_S (1.0f / (float)BIKE_CONTROL_HZ)
/* esp_timer takes integer microseconds: error vs 60 Hz is only 0.002%. */
#define BIKE_CONTROL_PERIOD_US 16667U

/* Initial acquisition target; select actual IMU ODR / ODrive telemetry later. */
#define BIKE_SENSOR_HZ 200U
#define BIKE_SENSOR_PERIOD_US 5000U
#define BIKE_MAX_SENSOR_AGE_US 20000U

/* Scripted input only; no simulated response to commanded torque. */
#define BIKE_MOCK_ROLL_AMPLITUDE_RAD 0.034906585f /* 2 degrees */
#define BIKE_MOCK_ROLL_PERIOD_US 4000000LL

/* Wi-Fi SoftAP / UDP. USB UART is now used only for flashing and diagnostics. */
#define BIKE_UDP_PORT 3333U
#define BIKE_PEER_LEASE_US 1000000U
#define BIKE_COMMAND_MAX_AGE_US 25000U
#define BIKE_LINK_TIMEOUT_US 50000U

#define BIKE_SENSOR_PRIORITY 12
#define BIKE_CONTROL_PRIORITY 10
#define BIKE_RX_PRIORITY 11
#define BIKE_SENSOR_STACK_BYTES 4096
#define BIKE_CONTROL_STACK_BYTES 4096
#define BIKE_RX_STACK_BYTES 4096

/* No motor output until driver, units, limits and explicit arming exist.
 * This version intentionally exposes no automatic arming path. */
#define BIKE_MOTOR_OUTPUT_ENABLED false

#endif
