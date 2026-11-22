# ESP32 平衡自行车

根目录固件通过 I²C 读取 JY901P，通过 UART2 控制 ODrive，并通过 Wi-Fi 向电脑发送姿态与控制状态。控制与状态发布的目标频率都是 100 Hz。

## 编译、烧录和查看

1. 用 ESP-IDF v5.4.4 打开仓库根目录。激活环境后，从根目录执行 `idf.py -p /dev/ttyUSB0 build flash monitor`，串口按实际设备修改。
2. 启动日志应显示工程名 `balance_bike` 和应用标签 `uart_balance`。ESP32 上电时先向 ODrive 发送两轴 IDLE 和零目标。
3. 电脑连接热点 `BalanceBike-ESP32`（密码 `balance123`），运行 `python3 standalone/jy901p/view_3d.py`。查看器默认连接 `192.168.4.1:3334`。

保留旧 `sdkconfig` 时，检查 `menuconfig → Component config → FreeRTOS → Kernel` 中的 `configTICK_RATE_HZ` 是否为 1000。

## 接线

| 外设 | ESP32 |
|---|---|
| JY901P SDA、SCL | GPIO21、GPIO22 |
| JY901P VCC | 按模块规格接 3.3 V |
| ODrive UART RX、TX | GPIO25（TX）、GPIO26（RX） |
| 两个外设 GND | ESP32 GND |

JY901P 默认地址为 `0x50`，I²C 为 100 kHz。按模块需要给 SDA、SCL 加 3.3 V 上拉。ODrive 使用 3.3 V UART、115200、8N1。

## 控制

- 电机 0 是飞轮，设定力矩（N·m）。电机 1 是驱动轮，设定转速（圈/s）。先按实车设置软件限幅，再在查看器中选一个电机操作。
- 切换控制模式前先点「手动停机」。**关闭查看器或 Wi-Fi 断线后，ESP32 会继续本地控制。**
- 直立调试依次设置限幅、车体零点和 PD 增益，再显式启动 `balance`。步骤与停机条件见 [控制工程说明](standalone/odrive_uart/README.md)。
- 飞轮最大转速、最大力矩和静态恢复角度见 [实车记录](standalone/odrive_uart/README.md#实车记录)。

## 工程与说明

| 位置 | 用途 |
|---|---|
| 根目录 | 整车联调入口，默认使用真实 JY901P 和 ODrive |
| `standalone/jy901p/` | 独立传感器固件和 [3D 查看器](standalone/jy901p/VIEW_3D.md) |
| `standalone/odrive_uart/` | 复用根工程控制代码的独立固件，附 Wi-Fi 文本控制台 |
| `main/control/` | 状态切换、零点与指令执行见 [uart_balance_app.c](main/control/uart_balance_app.c)，PD 计算和保护判断见 [balance_loop.c](main/control/balance_loop.c) |
| `components/` | Wi-Fi 数据链路及 [ODrive UART 接口](docs/odrive_uart.md) |
| `tests/`、`docs/` | 主机测试、协议说明与联调记录 |

根工程的 `menuconfig → Balance bike framework` 保留独立 Wi-Fi 诊断和 [旧版电脑链路测试](docs/pc_link_protocol.md)，后者使用 UDP 3333，电机输出关闭。两项均关闭时运行默认控制固件。

主机测试和编译用于核对控制逻辑与接口。根目录整合版的实板联调仍需烧录后验证。
