# ESP32 平衡自行车硬件接口与 JY901P 姿态显示

基于 ESP-IDF v5.4.4，适用于普通 ESP32。包含 JY901P 的 I²C 数据读取、100 Hz Wi-Fi 数据输出、桌面 3D 姿态查看器，以及 Wi-Fi / UDP 通信与硬件接口。

## 工程组成

| 工程 | 主要功能 | 数据频率 | USB 启动诊断波特率 |
|---|---|---|---|
| 根目录工程 | 传感器采集、Wi-Fi / UDP 通信、硬件状态检查 | 采集目标 200 Hz，状态上报约 60 Hz | 115200 |
| `standalone/jy901p/` | 独立 JY901P 读取、Wi-Fi 数据和 3D 姿态查看 | 读取与 Wi-Fi 发布目标 100 Hz | 460800 |
| `standalone/odrive_uart/` | MCU 通过 UART 控制飞轮力矩、驱动轮速度，提供直立 PD 调试入口 | 控制与 Wi-Fi 发布目标 100 Hz | 115200 |

各工程分别编译、烧录。两个独立工程均通过 Wi-Fi 提供实时数据；根目录工程另有 Wi-Fi / UDP 字节协议。
ODrive 串口工程的接线、MCU 零点保存和命令操作见 [直立调试说明](standalone/odrive_uart/README.md)。

## JY901P 接线

| JY901P | ESP32 |
|---|---|
| SDA | GPIO21 |
| SCL | GPIO22 |
| VCC | 3.3 V，按模块供电规格接入 |
| GND | GND，共地 |

SDA、SCL 各用约 4.7 kΩ 电阻上拉到 3.3 V；模块已有适当上拉时无需重复添加。
默认使用 I2C0，总线速率 100 kHz，7 位设备地址为 `0x50`。地址无需左移为 `0xA0` 或 `0xA1`。

独立模块的引脚和运行参数集中在 [jy901p.c](standalone/jy901p/main/jy901p.c) 文件顶部。
读取三轴加速度、三轴角速度、Roll / Pitch / Yaw 姿态角及磁场原始计数，通过 Wi-Fi 发送三轴加速度、角速度和姿态角。

| 数据 | 换算 | 单位 |
|---|---|---|
| 加速度 | `raw / 32768 × 16` | g |
| 角速度 | `raw / 32768 × 2000` | °/s |
| 姿态角 | `raw / 32768 × 180` | ° |

寄存器数据按低字节在前的 16 位有符号数解析。独立模块配置输出率为 100 Hz、带宽代码为 `2`（厂家 SDK 标称 94 Hz），写入后回读确认。
初始化不会自动执行校准或 Z 轴归零；按需要使用厂家工具校准。

## 独立模块构建与烧录

先激活 ESP-IDF v5.4.4 环境，再从仓库根目录运行：

```bash
cd standalone/jy901p
idf.py build
idf.py -p /dev/ttyUSB0 flash
```

按实际设备修改 `/dev/ttyUSB0`，USB 只用于烧录及启动诊断。新版传感器数据通过 Wi-Fi UDP 发送。

## 实时 3D 查看

电脑端使用 Python 3 和 Tkinter。先连接 ESP32 热点 `BalanceBike-ESP32`（密码 `balance123`），再从仓库根目录运行：

```bash
cd standalone/jy901p
python3 view_3d.py
```

- 模型跟随模块朝向旋转，红色端表示模块 +X 方向。
- 默认 Wi-Fi 接收 `192.168.4.1:3334` 的数据，不打开串口；可用 `--wifi` 指定 IP。
- Wi-Fi 断线或关闭窗口不停止本地采集及已启动的控制；重连恢复接收最新数据。
- 左键拖动调整观察视角，滚轮缩放。
- “设为零点并保存”将当前朝向作为显示基准，模型、右侧角度和轨迹同步归零，重开窗口自动加载；“显示绝对姿态”恢复原始朝向，“使用已存零点”切回保存的参考。
- 右侧实时显示角度、角速度和加速度；参考姿态操作只改变显示，不修改传感器参数。
- 右上角显示实际接收频率和软件绘制频率。数据目标为 100 Hz，绘制目标为 100 fps；屏幕实际呈现帧率取决于显示器刷新率。

完整说明见 [3D 查看器使用说明](standalone/jy901p/VIEW_3D.md)。

旧版串口固件实板已验证约 100 Hz 的 ESP32 读取与输出；本次 Wi-Fi 版本已[实板验证热点可见](standalone/jy901p/test_results/wifi/report.md)，尚未实测无线接收频率。历史结果见 [100 Hz 测试报告](standalone/jy901p/test_results/100hz/report.md)。
该测试不代表模块内部采样时序或绝对测量精度已通过验证；先前记录到的 Z 轴角速度瞬时尖峰仍需进一步确认。

查看器检查命令：

```bash
python3 -m unittest discover -s standalone/jy901p -p 'test_view_3d.py' -v
```

## 根目录工程

从仓库根目录构建、烧录并查看日志：

```bash
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

当前默认读取真实 JY901P；应用中的飞轮反馈仍为模拟，电机输出关闭。
独立的 ODrive UART 控制组件已封装，支持电机 0 力矩控制、电机 1 直接速度控制。
接线、使能、停机和调用方法见 [ODrive 串口控制接口](docs/odrive_uart.md)。
关闭 `Use real JY901P IMU over I2C` 可使用模拟传感器输入。
根目录工程的 JY901P 引脚、地址和带宽可以在 `menuconfig → Balance bike framework` 中配置。

Wi-Fi 默认配置：

| 项目 | 默认值 |
|---|---|
| 热点名称 | `BalanceBike-ESP32` |
| 热点密码 | `balance123` |
| ESP32 IP | `192.168.4.1` |
| UDP 端口 | `3333` |

ESP32 提供本地热点，不提供互联网。Wi-Fi 初始化、UDP 收发及端点登记位于 `main/ports/pc_link_port.c`。
字节协议与校验说明见 [通信协议](docs/pc_link_protocol.md)，无线延迟测试记录见 [Wi-Fi 延迟分析](docs/wifi_latency_diagnosis.md)。

启用 `Run isolated Wi-Fi diagnostic instead of bike application` 后，固件先打印 10 秒无射频心跳，再启动最小热点。
该模式用于诊断供电、射频启动和热点连接；恢复普通运行前取消该选项并重新烧录。

## 主要文件

```text
main/bike_app.c                    任务调度、状态采集、通信和本地检查
main/app_config.h                  周期、端口与超时配置
main/ports/imu_port.*              JY901P 接口与模拟输入
main/ports/odrive_port.*           ODrive 接口与模拟反馈
components/odrive_uart/           ODrive UART2 驱动与双轴控制接口
components/bike_wifi/             独立固件的 Wi-Fi UDP 收发组件
main/ports/pc_link_port.*          ESP32 热点与 UDP 通信
main/link/                        协议、端点登记和链路检查
host/protocol.py                  电脑端字节协议
host/udp_link.py                  电脑端 UDP 通信
standalone/jy901p/main/jy901p.c    独立传感器示例
standalone/jy901p/view_3d.py       桌面 3D 姿态查看器
standalone/odrive_uart/           独立串口电机控制与直立调试工程
tests/                            主机验证脚本
docs/                             协议说明与测试记录
```

## 仓库文件

保留源码、构建配置、`sdkconfig.defaults`、测试和文档。
`build/`、本地生成的 `sdkconfig`、日志、缓存和原始采集数据已通过 `.gitignore` 排除。
