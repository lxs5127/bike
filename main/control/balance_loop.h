#ifndef LOCAL_BALANCE_LOOP_H
#define LOCAL_BALANCE_LOOP_H

#include <stdbool.h>
#include <stdint.h>

/* 姿态采样超过这个年龄时，手动模式和平衡模式都会停机。 */
#define BALANCE_IMU_MAX_AGE_US 30000LL

/* 参数由实车调试提供，增益和力矩上限没有预设值。 */
typedef struct {
    float kp_nm_rad;                   /* 横滚角误差对应的力矩增益，N·m/rad */
    float kd_nm_rad_s;                 /* 横滚角速度对应的力矩增益，N·m/(rad/s) */
    float torque_limit_nm;
    float flywheel_speed_limit_turns_s;
    int torque_direction;              /* +1 或 -1，由实际飞轮反作用方向确定 */
} balance_parameters_t;

typedef struct {
    float roll_rad;                    /* JY901P 车身横滚角 */
    float roll_rate_rad_s;             /* JY901P 车体 X 轴角速度 GX */
    float flywheel_turns_s;            /* ODrive 轴 0 编码器速度，用于超速保护 */
    int64_t imu_us;                    /* IMU 采样时刻，ESP32 单调时钟 */
    int64_t encoder_us;                /* 编码器读回时刻，与 imu_us 使用同一时钟 */
    bool imu_valid;
    bool encoder_valid;
} balance_input_t;

typedef enum {
    LOCAL_BALANCE_OK = 0,
    LOCAL_BALANCE_NOT_CALIBRATED,
    LOCAL_BALANCE_BAD_PARAMETERS,
    LOCAL_BALANCE_BAD_SENSOR,
    LOCAL_BALANCE_STALE,
    LOCAL_BALANCE_FALL,
    LOCAL_BALANCE_OVERSPEED,
} balance_result_t;

/* 在平衡零点附近计算飞轮目标力矩。由调用方负责实际发送和停机。
 * torque_nm 指向有效地址时，任何失败都将输出清零并返回原因。 */
balance_result_t balance_compute(const balance_parameters_t *parameters,
                                const balance_input_t *input, bool zero_valid,
                                float zero_rad, int64_t now_us, float *torque_nm);

#endif
