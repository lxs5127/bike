#include "bike_controller.h"

#include <math.h>
#include <string.h>
#include "app_config.h"
#include "policy_weights.h"

static float clip(float x, float low, float high)
{
    return fminf(high, fmaxf(low, x));
}

void bike_controller_reset(bike_controller_t *controller)
{
    memset(controller, 0, sizeof(*controller));
}

float bike_limit_action(float action, float speed_rad_s)
{
    /* Same guard as sim2sim/play.py. Inertia is taken from the supplied URDF.
     * All these limits describe the SIMULATED plant, not calibrated hardware. */
    const float speed_limit = POLICY_SIM_SPEED_LIMIT_RAD_S;
    const float maximum_speed_change =
        2.0f * POLICY_SIM_TORQUE_LIMIT_NM * BIKE_CONTROL_DT_S /
        POLICY_SIM_FLYWHEEL_INERTIA;
    const float span = fmaxf(0.2f * speed_limit, maximum_speed_change);
    const float direction = action > 0.0f ? 1.0f : (action < 0.0f ? -1.0f : 0.0f);
    action *= clip((speed_limit - speed_rad_s * direction) / span, 0.0f, 1.0f);
    const float brake = clip((fabsf(speed_rad_s) - speed_limit) / span, 0.0f, 1.0f);
    if (speed_rad_s > speed_limit) {
        action = fminf(action, -brake);
    } else if (speed_rad_s < -speed_limit) {
        action = fmaxf(action, brake);
    }
    return action;
}

bool bike_controller_step(bike_controller_t *controller, const bike_state_t *state,
                          float dt_s, bike_control_output_t *output)
{
    float action;
    if (!rl_policy_step(&controller->policy, state, dt_s, &action)) {
        return false;
    }
    action = POLICY_ACTION_ALPHA * action +
             (1.0f - POLICY_ACTION_ALPHA) * controller->previous_action;
    action = bike_limit_action(action, state->flywheel_rad_s);
    if (!isfinite(action)) {
        return false;
    }
    controller->previous_action = action;
    output->normalized_action = action;
    output->torque_nm = POLICY_SIM_TORQUE_LIMIT_NM * action;
    output->rotation_rad = controller->policy.rotation_rad;
    return true;
}
