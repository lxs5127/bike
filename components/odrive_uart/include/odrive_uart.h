#ifndef ODRIVE_UART_H
#define ODRIVE_UART_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* ESP32 → ODrive TTL 串口。TX/RX 交叉连接，双方 GND 共地。
 * ODrive 端须预先启用相同波特率的 ASCII UART。
 * GPIO21/22 留给 JY901P，UART0 留给 USB 调试输出。 */
#define ODRIVE_UART_NUM          2
#define ODRIVE_UART_TX_GPIO      25  /* ESP32 TX → ODrive RX */
#define ODRIVE_UART_RX_GPIO      26  /* ESP32 RX ← ODrive TX */
#define ODRIVE_UART_BAUDRATE     115200
#define ODRIVE_UART_TIMEOUT_MS   20  /* 每次锁等待、发送等待或整行接收的超时时间。 */

#define ODRIVE_MOTOR0            0U
#define ODRIVE_MOTOR1            1U
#define ODRIVE_STATE_IDLE        1U
#define ODRIVE_STATE_CLOSED_LOOP 8U

typedef struct {
    float position_turns;
    float velocity_turns_s;
    int64_t received_at_us;  /* ESP32 收到完整响应的时刻，单位 μs。 */
    bool valid;
} odrive_uart_feedback_t;

typedef struct {
    uint32_t axis_error;     /* ODrive 错误位掩码。 */
    uint32_t current_state;  /* 实际状态，供确认请求的闭环状态是否生效。 */
    bool valid;
} odrive_uart_axis_state_t;

/* 初始化 UART2（8N1，无流控），向两轴发送 IDLE 和零目标。
 * 初始化和反初始化阶段由调用任务独占。正常运行时各接口内部加锁。
 * 写入返回 ESP_OK 表示 ESP32 完成发送。实际状态可通过查询接口确认。 */
esp_err_t odrive_uart_init(void);
esp_err_t odrive_uart_deinit(void);

/* 电机 0 力矩目标，单位 N·m。发送 c 0 <torque_nm>\n，闭环由 enable 请求。 */
esp_err_t odrive_uart_set_motor0_torque(float torque_nm);

/* 电机 1 转速目标，统一以圈/s 发送 v 1 <turns_s> 0\n，力矩前馈固定为 0。 */
esp_err_t odrive_uart_set_motor1_velocity(float turns_s);
esp_err_t odrive_uart_set_motor1_rpm(float rpm);
esp_err_t odrive_uart_set_motor1_rad_s(float rad_s);

/* 显式使能：先 IDLE、清零目标、设置 PASSTHROUGH 与对应模式，再请求状态 8。
 * 轴 0 为力矩模式，轴 1 为速度模式。调用前应完成电机及编码器配置与校准。
 * 可通过 read_axis_state 检查是否实际进入 CLOSED_LOOP。 */
esp_err_t odrive_uart_enable(unsigned motor);

/* 请求状态 1（IDLE），同时清零目标。
 * stop_all 即使轴 0 停机失败，也继续尝试轴 1。 */
esp_err_t odrive_uart_stop(unsigned motor);
esp_err_t odrive_uart_stop_all(void);

/* f <motor> 返回位置（圈）和速度（圈/s）。
 * 查询超时或收到坏帧后锁存接收故障，后续查询返回 ESP_ERR_INVALID_STATE。
 * 查询锁存只影响读取。链路恢复后重新初始化串口即可重新查询。 */
esp_err_t odrive_uart_read_feedback(unsigned motor, odrive_uart_feedback_t *feedback);
/* 查询 axisN.encoder.vel_estimate，单位圈/s，供平衡控制读取。 */
esp_err_t odrive_uart_read_velocity(unsigned motor, float *turns_s, int64_t *received_at_us);
/* 查询 axisN.error 和 axisN.current_state。两个查询都成功后 valid 才有效。 */
esp_err_t odrive_uart_read_axis_state(unsigned motor, odrive_uart_axis_state_t *state);
esp_err_t odrive_uart_feed_watchdog(unsigned motor);
esp_err_t odrive_uart_clear_errors(void);  /* 发送 sc 清除 ODrive 端错误。 */

#endif
