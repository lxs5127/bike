#ifndef ODRIVE_PORT_H
#define ODRIVE_PORT_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    float flywheel_rad_s;    /* signed FLYWHEEL speed, not raw motor turns/s */
    int64_t timestamp_us;    /* actual feedback acquisition/reception time */
    uint32_t axis_error;
    bool valid;
    bool ready;             /* healthy/calibrated; not permission to arm */
} odrive_feedback_t;

typedef struct {
    float flywheel_torque_nm; /* driver must map coupling/sign/torque constant */
    int64_t valid_until_us;   /* never apply expired commands */
    bool enable;             /* false = disable; not just input_torque=0 */
} odrive_command_t;

esp_err_t odrive_port_init(void);
esp_err_t odrive_port_read_feedback(odrive_feedback_t *feedback);
esp_err_t odrive_port_write_command(const odrive_command_t *command);
esp_err_t odrive_port_stop(void);

/* TODO backend: CAN or UART, axis number, motor/load encoder mapping,
 * torque mode, direction, current/velocity limits and ODrive watchdog.
 * ODrive commonly reports turns/s: direct drive rad/s = turns/s * 2*pi.
 * All APIs require bounded communication; feedback must remain timestamped. */

#endif
