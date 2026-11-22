# ODrive UART 接口

ESP32 使用 UART2 发 ASCII 指令。GPIO25（TX）接 ODrive RX，GPIO26（RX）接 ODrive TX，两端 GND 共地。双方配置为 3.3 V TTL、115200、8N1，无流控。完整接线和整车操作见 [控制工程说明](../standalone/odrive_uart/README.md)。

## 常用接口

| C 接口 | 行为 |
|---|---|
| `odrive_uart_init()` | 初始化 UART，发送两轴 IDLE 和零目标 |
| `odrive_uart_enable(0)` | 设置力矩模式和直通输入，请求电机 0 进入闭环 |
| `odrive_uart_enable(1)` | 设置速度模式和直通输入，请求电机 1 进入闭环 |
| `odrive_uart_set_motor0_torque(nm)` | 发 `c 0 <Nm>`，量值为电机轴力矩 |
| `odrive_uart_set_motor1_velocity(turns_s)` | 发 `v 1 <turns/s> 0`，量值为电机轴转速，力矩前馈为零 |
| `odrive_uart_read_velocity(motor, &turns_s, &timestamp_us)` | 通过 `r axisN.encoder.vel_estimate` 获取速度和接收时间 |
| `odrive_uart_read_axis_state(motor, &state)` | 查询轴状态和错误码 |
| `odrive_uart_stop(motor)`、`odrive_uart_stop_all()` | 请求退出闭环，并清零目标 |
| `odrive_uart_deinit()` | 发停机请求，随后释放 UART 和互斥锁 |

典型顺序为 `init → enable → 周期更新目标 → stop`。调用方应处理每一步的错误。全部接口与额外诊断命令见 [odrive_uart.h](../components/odrive_uart/include/odrive_uart.h)。

## 指令和故障

- 每条指令以换行结束。写操作成功表示 ESP32 完成传输，有需要时可读轴状态确认 ODrive 已进入闭环。
- 互斥锁保证写入和查询按顺序执行。查询有超时，完整、有限数值才会作为有效反馈。
- 查询超时或格式错误后，后续查询返回 `ESP_ERR_INVALID_STATE`。仍可发送停机。停机并恢复连接后，通过 `reconnect` 或重新初始化 UART 恢复查询。
- UART 短写后，驱动锁存发送故障，防止半条指令与新指令拼接。复位通信双方后恢复连接。
- 上电时保留 ODrive 本身的电流、速度、校准和看门狗配置。连续发送 `c` 或 `v` 会刷新对应轴已启用的看门狗。

根目录和独立控制工程复用本驱动。手动模式只发力矩或速度目标，平衡模式还会读飞轮速度用于保护。

从仓库根目录可运行 `python3 -m unittest discover -s tests -p 'test_odrive_uart.py' -v`，在主机上模拟串口验证指令编码与异常处理。
