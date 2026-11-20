# ESP32 + JY901P + ODrive 本地控制与 Wi-Fi 数据

独立 ESP-IDF v5.4.4 工程，单片机直接通过 UART 控制 ODrive v3.5 双轴板。
按用户的《odrive串口指令文档.pdf》使用 ASCII `c`、`v`、`r`、`w` 指令，不读取固件版本、不执行电机/编码器校准检测、不改写已配置的电流限制和校准参数。

## 接线

| ESP32 | 外设 |
|---|---|
| GPIO25，UART2 TX | ODrive UART RX |
| GPIO26，UART2 RX | ODrive UART TX |
| GPIO21，I²C SDA | JY901P SDA |
| GPIO22，I²C SCL | JY901P SCL |
| GND | 两个外设共地 |

ODrive UART 为 115200、8N1、3.3 V TTL。电机 0 是反作用飞轮，用 `c 0 <Nm>`；电机 1 是驱动轮，用 `v 1 <turns/s> 0`。
转速和力矩为电机轴侧量，单位分别为圈/s 和 N·m。
JY901P 采用 0x50 / 100 kHz；本工程复用真实 IMU 驱动，传感器内部输出 200 Hz，本地读取和控制目标 100 Hz。
IMU 必须按 **X 向车头、Y 向左、Z 向上** 安装，控制使用 Roll 和 GX；安装方向不同必须先改轴映射。

## 编译和启动

在同一个终端激活 ESP-IDF 环境，再从仓库根目录构建并烧录。以下是本机 ESP-IDF v5.4.4 的命令，其他机器请使用各自的激活脚本和串口。

```bash
source ~/.espressif/tools/activate_idf_v5.4.4.sh
idf.py -C standalone/odrive_uart -p /dev/ttyUSB0 build flash
```

使用 VS Code 插件烧录时，请将 `standalone/odrive_uart` 文件夹作为 ESP-IDF 工程打开。
插件按打开的工程文件夹烧录。根目录的 `balance_bike` 工程使用 UDP 3333，本查看器连接的是 UDP 3334。
烧录完成后，可在 115200 波特率的启动日志中用 `Project name: uart_balance` 确认工程。

按实际设备改串口名。USB 仅用于烧录和启动诊断，实时数据、状态和调参命令均走 Wi-Fi。
电脑连接热点 **BalanceBike-ESP32**，密码 **balance123**；地址为 **192.168.4.1:3334 / UDP**。
启动 3D 姿态窗口：

```bash
/usr/bin/python3 standalone/jy901p/view_3d.py --wifi 192.168.4.1
```

或使用 Wi-Fi 文本控制台（每秒接收目标 100 组，按 10 Hz 打印）：

```bash
/usr/bin/python3 standalone/odrive_uart/wifi_console.py
```

3D 窗口底部和文本控制台都可以输入下表命令。两个客户端择一运行，避免争用一个接收会话。
3D 窗口顶部提供“设置力矩”（电机 0，N·m）和“设置转速”（电机 1，圈/s）两个按钮。
按钮分别发送 `torque0` 和 `speed1`。ESP32 持续应用目标，直到收到新目标、手动停机或触发保护。
**Wi-Fi 断线、关闭窗口、退出文本控制台，都不会停机或重置本地控制**；本地 100 Hz 循环继续运行。
重新连接只恢复最新数据，不自动重发运动指令。需要停机时显式输入 `stop` 或点击窗口“手动停机”。

上电先向两轴发送 IDLE 和零目标，始终从 STOPPED 开始，不自动使能电机。
驱动接口及引脚常量位于 [odrive_uart.h](../../components/odrive_uart/include/odrive_uart.h)，应用命令位于 [main.c](main/main.c)，直立控制计算位于 [balance_loop.c](main/balance_loop.c)。

## 命令

| 命令 | 操作 |
|---|---|
| `stop` | 两轴退出闭环，取消等待请求 |
| `status` | 查看 MCU 当前姿态、零点、模式、速度和控制参数 |
| `stream on` / `stream off` | 开关 100 Hz Wi-Fi 状态上报，本地控制周期保持目标 100 Hz |
| `zero` | 停机时保持自行车在确认的平衡姿态，采集约 2 秒并将 Roll 零点存入 ESP32 NVS |
| `limits <Nm> <飞轮turns/s> <驱动轮turns/s>` | 设置 MCU 力矩目标上限、平衡模式的飞轮速度上限、驱动轮速度目标上限 |
| `gains <Kp> <Kd> <1或-1>` | 设置直立 PD 参数和飞轮反作用方向，Kp 单位 Nm/rad，Kd 单位 Nm/(rad/s) |
| `torque0 <Nm>` | 设置或更新电机 0 的持续力矩目标 |
| `speed1 <turns/s>` | 设置或更新电机 1 的持续速度目标 |
| `balance` | 显式启动电机 0 的直立 PD，电机 1 保持停用 |
| `reconnect` | 停机时重建 UART；不清除 ODrive 错误或重做校准 |

`limits`、`gains` 的尖括号是参数占位符，必须换成实车数值。没有内置可以直接用于这辆车的增益或限幅。
旧版客户端发送可选的 100～2000 ms 参数时，ESP32 仍会按指定时长执行并在到时停机。
这些是 MCU 本次调试的参数，与已经配置好的 ODrive 电机参数不同；重启后需要重设。零点单独保存在 NVS，正常断电、复位后保留。

## 零点与直立调试

将单车扶在确认的平衡姿态，停机后输入 `zero`。采样要求 Roll 变化不超过 0.5°、GX 不超过 2°/s。
保存的是这个安装姿态的绝对 Roll；以后上电读取它，再计算 `当前 Roll − 保存的 Roll`，不会把每次上电的倾斜姿态当作零点。
电脑 3D 查看器的 `reference_pose.json` 不会自动变成 MCU 零点，这里需要单独执行一次 `zero`。重新安装传感器或改动 IMU 的姿态基准后需要重新设置。

设置实车限幅后，可用有支撑条件下的短脉冲确认飞轮反作用方向；`gains` 最后一个参数由实际响应确定。
`balance` 使用 `direction × (Kp × 相对倾角 + Kd × Roll角速度)`，并对目标力矩限幅。
只有设置了零点、限幅和有效增益才允许启动。此处是用于直立附近调参的 PD 起点，尚未证明整车能够稳定平衡，也未实现飞轮动量卸载。
飞轮直立控制、手动力矩和手动转速是独立模式。切换模式前先停机，手动模式下可以直接更新当前电机的目标。

手动模式只写 UART 命令。ESP32 检查目标是否超过软件限幅，实际转速和电流由 ODrive 自身的配置约束。`UART TX complete` 表示 ESP32 串口已发完这条指令。
平衡模式每 50 ms 读取 `r axis0.encoder.vel_estimate`。编码器读取失败、反馈超过 100 ms、相对倾角超过 10° 或飞轮超速时，请求两轴停机。
两种模式都会在 IMU 数据无效或过期、UART 发送失败或周期计算超过 20 ms 时请求两轴停机。
`mode` 数字取值为 0（停机）、1（直立控制）、2（飞轮手动力矩）、3（驱动轮手动转速）。
`speed` 只在平衡模式反馈有效时代表实测飞轮速度。手动模式显示目标值。
`torque` 是发送的目标值，串口写入成功并不代表 ODrive 已执行，也不代表实际测得的力矩。
停机请求使驱动退出闭环，不保证机械瞬间停止；独立 ODrive 看门狗沿用现有配置，本工程不自动更改。

## 验证范围

主机测试编译实际 UART 驱动和 PD 计算源文件，验证指令、单位、异常数据、零点、方向、限幅和故障输出清零。
控制固件编译通过。主机测试覆盖 Wi-Fi 会话、3D 查看器和串口指令格式。模拟验证中，手动力矩和转速均可在没有编码器回包时持续发出指令，且可更新和停机。本版的实车电机动作尚待验证。

```bash
python3 -m unittest discover -s tests -p 'test_odrive_uart.py' -v
python3 -m unittest discover -s tests -p 'test_local_balance.py' -v
python3 -m unittest discover -s tests -p 'test_wifi_session.py' -v
/usr/bin/python3 -m unittest discover -s standalone/jy901p -p 'test_*.py' -v
```

## 无线接口

共享 Wi-Fi 组件在 `components/bike_wifi/`。网络任务与控制任务通过单槽快照队列和命令队列交互，控制周期不等待 UDP 收发。
Wi-Fi 会话保活 100 ms 一次，500 ms 没有消息只释放接收端登记，不触发任何停机操作。
数据为 JSON，含三轴 `acc_g`、`gyro_deg_s`、`angles_deg` 和 MCU 模式、零点、速度、目标力矩。
命令使用会话令牌、递增序号和 ESP32 时间基准；重复、乱序、过期或重连前的旧命令被拒绝。客户端不自动重发运动指令。
热点参数在 `menuconfig → Bike Wi-Fi data link` 中修改；本链路使用 3334 端口，与根工程 3333 的字节协议不同。
