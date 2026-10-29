#include "rl_policy.h"

#include <math.h>
#include <string.h>
#include "policy_weights.h"

static float clampf(float value, float lower, float upper)
{
    return fminf(upper, fmaxf(lower, value));
}

static float branch(const float input[POLICY_INPUT_SIZE])
{
    float first[POLICY_HIDDEN_SIZE];
    float second[POLICY_HIDDEN_SIZE];
    for (unsigned int row = 0; row < POLICY_HIDDEN_SIZE; ++row) {
        float sum = policy_fc1_bias[row];
        for (unsigned int col = 0; col < POLICY_INPUT_SIZE; ++col) {
            sum += policy_fc1_weight[row * POLICY_INPUT_SIZE + col] * input[col];
        }
        first[row] = sum > 0.0f ? sum : expm1f(sum);
    }
    for (unsigned int row = 0; row < POLICY_HIDDEN_SIZE; ++row) {
        float sum = policy_fc2_bias[row];
        for (unsigned int col = 0; col < POLICY_HIDDEN_SIZE; ++col) {
            sum += policy_fc2_weight[row * POLICY_HIDDEN_SIZE + col] * first[col];
        }
        second[row] = sum > 0.0f ? sum : expm1f(sum);
    }
    float sum = policy_actor_bias[0];
    for (unsigned int col = 0; col < POLICY_HIDDEN_SIZE; ++col) {
        sum += policy_actor_weight[col] * second[col];
    }
    return sum;
}

void rl_policy_reset(rl_policy_t *policy)
{
    memset(policy, 0, sizeof(*policy));
}

float rl_policy_action_from_observation(const float observation[4], float rotation_rad)
{
    float input[POLICY_INPUT_SIZE] = {
        observation[0], observation[1], observation[2], observation[3],
        clampf(rotation_rad / 1000.0f, -10.0f, 10.0f),
    };
    float mirrored[POLICY_INPUT_SIZE];
    for (unsigned int i = 0; i < POLICY_INPUT_SIZE; ++i) {
        mirrored[i] = i == 1 ? input[i] : -input[i];
    }
    return tanhf(0.5f * (branch(input) - branch(mirrored)));
}

bool rl_policy_step(rl_policy_t *policy, const bike_state_t *state,
                    float dt_s, float *action)
{
    if (!isfinite(state->roll_rad) || !isfinite(state->roll_rate_rad_s) ||
        !isfinite(state->flywheel_rad_s) || !isfinite(dt_s) || dt_s <= 0.0f) {
        return false;
    }
    if (policy->initialized) {
        policy->rotation_rad +=
            0.5f * (policy->previous_speed_rad_s + state->flywheel_rad_s) * dt_s;
    } else {
        policy->initialized = true;
    }
    policy->previous_speed_rad_s = state->flywheel_rad_s;
    const float observation[4] = {
        sinf(state->roll_rad), cosf(state->roll_rad),
        clampf(state->roll_rate_rad_s, -5.0f, 5.0f),
        clampf(state->flywheel_rad_s / 420.0f, -1.5f, 1.5f),
    };
    *action = rl_policy_action_from_observation(observation, policy->rotation_rad);
    return isfinite(policy->rotation_rad) && isfinite(*action);
}
