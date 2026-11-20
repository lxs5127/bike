# ODrive 双轴串口控制接口

该模块由 **ESP32 通过 UART 向 ODrive 发送 ASCII 指令**，按用户提供的 `odrive串口指令文档.pdf` 封装：

- 电机 0：力矩控制，`c 0 <torque_nm>`。
- 电机 1：直接速度控制，`v 1 <velocity_turns_s> 0`，力矩前馈为 0。

实现位于 [odrive_uart.c](../components/odrive_uart/odrive_uart.c)，对外接口和引脚配置集中在 [odrive_uart.h](../components/odrive_uart/include/odrive_uart.h)。
模块已加入根目录工程的编译依赖，可以在应用中 `#include "odrive_uart.h"` 使用。

## 接线与串口参数

| ESP32 | ODrive |
|---|---|
| GPIO25，UART2 TX | UART RX |
| GPIO26，UART2 RX | UART TX |
| GND | GND |

串口为 **115200 baud、8 数据位、无校验、1 停止位、无流控**，3.3 V TTL 电平。
GPIO21 / GPIO22 继续用于 JY901P，UART0 继续用于 USB 调试。

以上只配置 ESP32。ODrive 侧也必须启用 ASCII UART，并设置相同波特率。
用户提供的 PDF 没有指定波特率，因此采用旧版 ODrive UART 默认的 115200；如果板子已经改过，修改头文件的 `ODRIVE_UART_BAUDRATE` 与之匹配。
ODrive 端的 RX / TX 物理位置按具体板卡标识确定。

本接口面向文档中支持 `motor=0/1` 的双轴 ASCII 固件。`f` 反馈查询、控制模式和状态参数补充依据官方旧版接口；不同型号或定制固件需核对这些属性是否存在。
参考：[官方 ASCII 协议](https://github.com/odriverobotics/ODrive/blob/master/docs/ascii-protocol.rst)、[UART 接口](https://github.com/odriverobotics/ODrive/blob/master/docs/uart.rst)、[模式和状态枚举](https://github.com/odriverobotics/ODrive/blob/master/tools/odrive/enums.py)。

## 接口与单位

| 接口 | 含义 |
|---|---|
| `odrive_uart_init()` | 初始化串口，发送两轴 IDLE 和零目标 |
| `odrive_uart_enable(0)` | 电机 0 配置为力矩控制、直通输入，再请求闭环 |
| `odrive_uart_enable(1)` | 电机 1 配置为速度控制、直通输入，再请求闭环 |
| `odrive_uart_set_motor0_torque(nm)` | 电机 0 目标力矩，单位 N·m |
| `odrive_uart_set_motor1_velocity(turns_s)` | 电机 1 目标速度，单位圈/s |
| `odrive_uart_set_motor1_rad_s(rad_s)` | 电机 1 目标速度，单位 rad/s，内部除以 2π |
| `odrive_uart_stop(motor)` | 对指定电机请求 IDLE，并清零目标 |
| `odrive_uart_stop_all()` | 对两台电机分别发送停机请求 |
| `odrive_uart_read_feedback(motor, &feedback)` | 读取编码器位置与速度 |
| `odrive_uart_read_velocity(motor, &turns_s, &timestamp_us)` | 通过 `r axisN.encoder.vel_estimate` 读取速度 |
| `odrive_uart_read_axis_state(motor, &state)` | 读取错误码和当前状态 |
| `odrive_uart_feed_watchdog(motor)` | 给指定轴的已有看门狗喂狗 |
| `odrive_uart_clear_errors()` | 显式发送 `sc` 清除错误 |
| `odrive_uart_deinit()` | 尝试停机，释放 UART 和互斥锁 |

力矩和速度均为电机侧量。组件直接使用电机轴的力矩（N·m）和转速（圈/s）。

“直接速度”通过 `input_mode=1`（PASSTHROUGH）实现，不使用速度斜坡输入。
模式值为电机 0 的 `control_mode=1`（TORQUE_CONTROL）和电机 1 的 `control_mode=2`（VELOCITY_CONTROL）。

## 调用流程

```c
#include "odrive_uart.h"

// 上电初始化：两轴保持 IDLE，不自动请求闭环或执行电机校准。
ESP_ERROR_CHECK(odrive_uart_init());

// 完成 ODrive 电机/编码器配置与校准后，由应用显式使能。
ESP_ERROR_CHECK(odrive_uart_enable(ODRIVE_MOTOR0));
ESP_ERROR_CHECK(odrive_uart_enable(ODRIVE_MOTOR1));

// 控制周期中更新目标。这里以零目标示例，实际值由上层应用提供。
ESP_ERROR_CHECK(odrive_uart_set_motor0_torque(0.0f));
ESP_ERROR_CHECK(odrive_uart_set_motor1_velocity(0.0f));

// 读取电机 1 的反馈，位置为圈数，速度为圈/s。
odrive_uart_feedback_t feedback;
esp_err_t result = odrive_uart_read_feedback(ODRIVE_MOTOR1, &feedback);
if (result == ESP_OK && feedback.valid) {
    // 应用直接使用 feedback.velocity_turns_s 和 feedback.position_turns。
}

// 停止运行或出现异常时，应用应检查停机接口的返回值。
esp_err_t stop_result = odrive_uart_stop_all();
(void)stop_result;
```

实际程序中应在命令发送失败后调用停机并处理错误，而不是依赖 `ESP_ERROR_CHECK` 重启来完成物理停机。
ODrive 的电流/力矩限制、速度限制、校准参数和独立看门狗由具体电机配置决定，组件不自动改写或保存这些参数。
`c`、`v` 和 `u` 指令会刷新对应轴的已有看门狗；两轴应分别周期更新命令或喂狗。

`enable()` 先 IDLE、清零目标、配置输入和控制模式，最后发送 `requested_state=8`。
由于 ASCII 写命令没有执行确认，`ESP_OK` 表示 UART 发送完成；可调用 `read_axis_state()` 检查是否实际进入状态 8，以及 `axis_error` 是否为 0。

`stop()` 请求 `requested_state=1`，含义是退出闭环并停止驱动，不保证机械瞬间停转。
保持零速与停用轴是不同操作：需要闭环保持零速时使用电机 1 的速度零目标。

## 与现有应用的关系

这是独立底层组件，根目录的 `main/CMakeLists.txt` 已添加 `odrive_uart` 依赖，应用可直接调用上述接口。
现有 `main/ports/odrive_port.*` 仍保留原来的模拟适配流程，新增组件不会在启动时自动使能电机，也不会把轴 1 速度命令混入轴 0 力矩命令。
独立 JY901P 固件只负责传感器采集，通过 Wi-Fi 向电脑发送三轴数据。

本地控制工程提供 MCU 零点持久化、手动力矩和转速设定、100 Hz 直立 PD 调试及停机命令。使用已配置好的 ODrive 即可连接。[控制工程说明](../standalone/odrive_uart/README.md)

## 通信与错误处理

- 每条命令以换行符结束。模块用互斥锁串行化写入与查询，防止两个任务把指令或响应交错。
- 互斥等待、TX 等待及每次响应读取均有有限超时。查询函数会阻塞调用任务，应按实际控制周期安排读取任务。
- 反馈速度和位置只在完整、合法且有限的数字响应收到后置为有效；失败时输出清零、`valid=false`。
- 查询超时或格式错误后锁存接收故障，后续查询返回 `ESP_ERR_INVALID_STATE`；停机和写命令仍可使用。通信恢复后，在其他调用任务停止的条件下先调用 `odrive_uart_deinit()`，再调用 `odrive_uart_init()` 重建连接。
- UART 短写后锁存发送故障，避免将下一条运动命令拼到半条旧指令上；复位通信双方后重新建立连接，不自动重发运动指令。

## 验证

```bash
python3 -m unittest discover -s tests -p 'test_odrive_uart.py' -v
```

测试编译实际组件并模拟 UART，覆盖轴号和指令、模式与使能顺序、单位换算、坏响应/超时、无效数字、互斥等待、短写和两轴停机路径。
ESP-IDF 工程已通过编译；尚未通过该组件进行 ODrive 实板通信或电机运动测试。
