#ifndef ODRIVE_UART_H
#define ODRIVE_UART_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* ESP32 → ODrive TTL 串口。TX/RX 交叉连接，双方 GND 共地。
 * 这些参数只配置 ESP32；ODrive 侧必须预先启用相同波特率的 ASCII UART。
 * GPIO21/22 留给 JY901P，UART0 留给 USB 调试输出。 */
#define ODRIVE_UART_NUM          2
#define ODRIVE_UART_TX_GPIO      25  /* ESP32 TX → ODrive RX */
#define ODRIVE_UART_RX_GPIO      26  /* ESP32 RX ← ODrive TX */
#define ODRIVE_UART_BAUDRATE     115200
#define ODRIVE_UART_TIMEOUT_MS   20

#define ODRIVE_MOTOR0            0U
#define ODRIVE_MOTOR1            1U
#define ODRIVE_STATE_IDLE        1U
#define ODRIVE_STATE_CLOSED_LOOP 8U

typedef struct {
    float position_turns;
    float velocity_turns_s;
    int64_t received_at_us;  /* 完整响应接收时间，不是内部编码器采样时间 */
    bool valid;
} odrive_uart_feedback_t;

typedef struct {
    uint32_t axis_error;
    uint32_t current_state;
    bool valid;
} odrive_uart_axis_state_t;

/* 初始化 UART2（8N1，无流控），发送两轴 IDLE 与零设定值，不自动闭环。
 * 初始化/反初始化须在其他任务停止使用接口时调用。正常运行接口内部互斥。
 * 写入接口的 ESP_OK 表示串口发送完成，不表示 ODrive 已执行或校准完成。 */
esp_err_t odrive_uart_init(void);
esp_err_t odrive_uart_deinit(void);

/* 电机0：c 0 <torque_nm>\n。只写目标，不会隐式使能电机。 */
esp_err_t odrive_uart_set_motor0_torque(float torque_nm);

/* 电机1：v 1 <turns_s> 0\n。直接速度输入，扭矩前馈固定为0。 */
esp_err_t odrive_uart_set_motor1_velocity(float turns_s);
esp_err_t odrive_uart_set_motor1_rpm(float rpm);
esp_err_t odrive_uart_set_motor1_rad_s(float rad_s);

/* 显式使能：先 IDLE、清零目标、设置 PASSTHROUGH 与对应模式，再请求状态8。
 * 轴0固定为力矩模式，轴1固定为速度模式。调用前应完成 ODrive 电机/编码器配置与校准。
 * 可通过 read_axis_state 检查是否实际进入 CLOSED_LOOP。 */
esp_err_t odrive_uart_enable(unsigned motor);

/* 停机请求状态1（IDLE），同时清零目标；不是仅发送零力矩/零速度。
 * stop_all 即使轴0停机失败，也继续尝试轴1。 */
esp_err_t odrive_uart_stop(unsigned motor);
esp_err_t odrive_uart_stop_all(void);

/* f <motor> 返回位置/速度。r 查询 axisN.error 与 axisN.current_state。
 * 超时、坏帧后锁存接收故障，后续查询返回 INVALID_STATE，避免把迟到响应当新数据；
 * 运动写入和停机仍可调用。重建连接前应确认 ODrive 通信已恢复。 */
esp_err_t odrive_uart_read_feedback(unsigned motor, odrive_uart_feedback_t *feedback);
/* 使用文档的 r 指令读取 axisN.encoder.vel_estimate，单位 turns/s。 */
esp_err_t odrive_uart_read_velocity(unsigned motor, float *turns_s, int64_t *received_at_us);
esp_err_t odrive_uart_read_axis_state(unsigned motor, odrive_uart_axis_state_t *state);
esp_err_t odrive_uart_feed_watchdog(unsigned motor);
esp_err_t odrive_uart_clear_errors(void);  /* sc，不自动执行 */

#endif
