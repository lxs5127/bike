#ifndef BIKE_CONTROLLER_H
#define BIKE_CONTROLLER_H

#include "rl_policy.h"
#include "bike_safety.h"

typedef struct {
    rl_policy_t policy;
    float previous_action;
} bike_controller_t;

void bike_controller_reset(bike_controller_t *controller);
float bike_limit_action(float action, float speed_rad_s);
bool bike_controller_step(bike_controller_t *controller, const bike_state_t *state,
                          float dt_s, bike_control_output_t *output);

#endif
