#ifndef RL_POLICY_H
#define RL_POLICY_H

#include <stdbool.h>
#include "bike_types.h"

typedef struct {
    float rotation_rad;
    float previous_speed_rad_s;
    bool initialized;
} rl_policy_t;

void rl_policy_reset(rl_policy_t *policy);
/* obs = [sin(roll), cos(roll), clipped body gyro, clipped speed/420].
 * Appends clipped rotation/1000 internally. Two mirrored ELU branches. */
float rl_policy_action_from_observation(const float observation[4], float rotation_rad);
bool rl_policy_step(rl_policy_t *policy, const bike_state_t *state,
                    float dt_s, float *action);

#endif
