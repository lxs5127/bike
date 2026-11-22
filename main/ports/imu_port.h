#ifndef IMU_PORT_H
#define IMU_PORT_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    float roll_rad;          /* 本地控制使用的车体 X 轴横滚角，单位 rad。 */
    float roll_rate_rad_s;    /* 车体 X 轴角速度，单位 rad/s。 */
    int64_t timestamp_us;     /* esp_timer 时基上的估计采样时刻，单位 μs。 */
    bool valid;
    float acceleration_g[3];  /* 供可视化读取的 AX/AY/AZ，单位 g。 */
    float gyro_deg_s[3];      /* GX/GY/GZ，单位 °/s。 */
    float angles_deg[3];      /* 融合后的 Roll/Pitch/Yaw，单位 °。 */
} imu_sample_t;

/* 选择真实 JY901P 或通信测试的模拟输入。在启动采样任务前调用。 */
esp_err_t imu_port_init(void);

/* 一次限时读取生成样本。失败时输出无效样本，供控制端判断停机。
 * 模块按车体 X 前、Y 左、Z 上安装。采样时间由寄存器读取时刻估计，
 * 传感器内部融合停滞需要通过其他方式判断。 */
esp_err_t imu_port_read(imu_sample_t *sample);

#endif
