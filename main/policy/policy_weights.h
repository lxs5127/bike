/* Generated; do not edit weights by hand. */
#ifndef POLICY_WEIGHTS_H
#define POLICY_WEIGHTS_H
#define POLICY_INPUT_SIZE 5U
#define POLICY_HIDDEN_SIZE 128U
#define POLICY_SIM_TORQUE_LIMIT_NM 4.9000001f
#define POLICY_SIM_SPEED_LIMIT_RAD_S 250.0f
#define POLICY_SIM_FALL_ANGLE_RAD 0.17453292f
#define POLICY_SIM_FLYWHEEL_INERTIA 0.00738999993f
#define POLICY_ACTION_ALPHA 1.0f

extern const float policy_fc1_weight[640];
extern const float policy_fc1_bias[128];
extern const float policy_fc2_weight[16384];
extern const float policy_fc2_bias[128];
extern const float policy_actor_weight[128];
extern const float policy_actor_bias[1];

#endif
