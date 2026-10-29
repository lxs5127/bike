# ESP32 平衡自行车硬件接口与 JY901P 姿态显示

基于 ESP-IDF v5.4.4，适用于普通 ESP32。包含 JY901P 的 I²C 数据读取、实时串口输出、桌面 3D 姿态查看器，以及 Wi-Fi / UDP 通信与硬件接口。

## 工程组成

| 工程 | 主要功能 | 数据频率 | 串口波特率 |
|---|---|---|---|
| 根目录工程 | 传感器采集、Wi-Fi / UDP 通信、硬件状态检查 | 采集目标 200 Hz，状态上报约 60 Hz | 115200 |
| `standalone/jy901p/` | 独立 JY901P 读取、打印和 3D 姿态查看 | 读取与打印 100 Hz | 460800 |

两个工程分别编译、烧录。烧录独立 JY901P 工程后，板上运行传感器示例；需要 Wi-Fi 功能时重新烧录根目录工程。

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
读取三轴加速度、三轴角速度、Roll / Pitch / Yaw 姿态角及磁场原始计数，串口打印加速度、角速度和姿态角。

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
idf.py -p /dev/ttyUSB0 -b 460800 monitor
```

按实际设备修改 `/dev/ttyUSB0`。串口每 10 ms 输出一组读数，示例：

```text
acc[g] X=0.001 Y=0.002 Z=1.000 | gyro[deg/s] X=0.061 Y=0.000 Z=-0.061 | angle[deg] Roll=0.100 Pitch=-0.200 Yaw=12.300
```

示例数值仅说明输出格式。按 `Ctrl+]` 退出串口监视。

## 实时 3D 查看

电脑端使用 Python 3、Tkinter 和 pyserial。先退出其他串口监视器，释放设备，再从仓库根目录运行：

```bash
cd standalone/jy901p
python3 view_3d.py --port /dev/ttyUSB0
```

- 模型跟随模块朝向旋转，红色端表示模块 +X 方向。
- 左键拖动调整观察视角，滚轮缩放。
- “此刻设为参考姿态”将当前朝向作为显示基准；“显示绝对姿态”恢复原始朝向。
- 右侧实时显示角度、角速度和加速度；参考姿态操作只改变显示，不修改传感器参数。
- 右上角显示实际接收频率和软件绘制频率。数据目标为 100 Hz，绘制目标为 100 fps；屏幕实际呈现帧率取决于显示器刷新率。

完整说明见 [3D 查看器使用说明](standalone/jy901p/VIEW_3D.md)。

实板已验证约 100 Hz 的 ESP32 读取与串口输出，相关结果见 [100 Hz 测试报告](standalone/jy901p/test_results/100hz/report.md)。
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

当前默认读取真实 JY901P；飞轮反馈仍为模拟，真实 ODrive 驱动尚未接入，电机输出关闭。
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
main/ports/pc_link_port.*          ESP32 热点与 UDP 通信
main/link/                        协议、端点登记和链路检查
host/protocol.py                  电脑端字节协议
host/udp_link.py                  电脑端 UDP 通信
standalone/jy901p/main/jy901p.c    独立传感器示例
standalone/jy901p/view_3d.py       桌面 3D 姿态查看器
tests/                            主机验证脚本
docs/                             协议说明与测试记录
```

## 仓库文件

保留源码、构建配置、`sdkconfig.defaults`、测试和文档。
`build/`、本地生成的 `sdkconfig`、日志、缓存和原始采集数据已通过 `.gitignore` 排除。
