# 反作用飞轮自行车：Wi-Fi 通信 + 电脑推理

工程：`/home/lxs/esp/projects/balance_bike`，ESP-IDF v5.4.4，普通 ESP32。
默认采用 ESP32 热点 + UDP 通信，电脑推理。ESP32 固件不编译神经网络和权重；电脑直接复用训练工程中的 NumPy actor。

当前正常固件标记为 `BIKE_FW=wifi-udp-v4`，诊断模式关闭，PHY 发射功率上限保留为 10 dBm。
用户提供的 v3 实板输出已确认 `link=OK`、`fault=0x00`，电脑策略推理最大耗时 993 µs，
所贴采样行的 RTT 为 2.81–16.55 ms，`crc_bad/old/gaps` 均为 0。v3 的快照时间误判修正已在这段运行中验证。
v4 在此基础上加入动态模拟输入和可观察的断联保护。
工具通过当前热点运行 v2 实验，已观察同一 RUN 指令从有效变为 50.001 ms 过期，且通信可恢复。
2026-10-09 的无线对照发现：GNOME 的“设置 → Wi-Fi”窗口打开时，网卡每约 15 秒扫描一次。
关闭该窗口、保持无线省电关闭后，一分钟测试的非计划 STOP 从 155 次降到 4 次，期间未记录到扫描事件。
剩余 STOP 表明连续 60 Hz 稳定性仍未通过；见 `docs/wifi_latency_diagnosis.md` 和 `docs/wifi_test_v2_settings_closed.json`。

若 `sdkconfig` 中启用了 `CONFIG_BIKE_WIFI_DIAGNOSTIC=y`，当前构建为隔离诊断固件：
上电先打印 10 秒无射频心跳，再启动最小热点，不提供 UDP 策略通信。见下文“隔离诊断”；
恢复正常流程前需取消该 menuconfig 选项并重新构建、烧录。

```text
IMU/ODrive 反馈 → ESP32 → Wi-Fi / UDP → 笔记本 RL 推理
                    ↑                         ↓
             本地检查/ODrive 接口 ← 力矩指令（N·m）
```

默认配置现已接入 JY901P，通过 I²C 读取真实侧倾角和 X 轴角速度；飞轮速度仍模拟为 0。
电机输出固定关闭，没有远程解锁指令。ODrive 的真实底层驱动仍只保留接口。
未连接 JY901P 时初始化会报错；纯通信实验可在 menuconfig 中关闭 `Use real JY901P IMU over I2C`，
恢复 ±2°、周期 4 秒的正弦模拟输入。

## JY901P 接线与参数

| JY901P | ESP32 |
|---|---|
| SDA | GPIO21 |
| SCL | GPIO22 |
| GND | GND，共地 |
| VCC | 3.3 V（按模块供电规格接入） |

SDA、SCL 各用约 4.7 kΩ 电阻上拉到 **3.3 V**；模块已有适当上拉时无需重复添加。
默认占用 I2C0，引脚可在 menuconfig → Balance bike framework 修改。
安装方向为模块 X 轴朝车头、Y 轴朝左、Z 轴朝上；其他安装方向需先实现对应的坐标变换。

- `BIKE_IMU_JY901P=y`：使用真实 IMU，与 `BIKE_MOCK_HARDWARE=y` 的模拟 ODrive 独立。
- 默认 7 位地址 `0x50`，ESP-IDF 自动生成读写位，不填 `0xA0` 或 `0xA1`。
- I²C 时钟默认 100 kHz，可配置到 400 kHz；不使用串口波特率设置。
- 初始化按 `KEY(0x69)=0xB588` 解锁，设置 `RATE(0x03)=0x000B`（200 Hz），
  `BANDWIDTH(0x1F)=0x0002`（厂家 SDK 标称 94 Hz），分别回读检查；这些是运行期设置，每次上电重设，不自动写入 Flash。
- 从 `GX(0x37)` 连续读到 `Yaw(0x3F)`，每个寄存器 16 位、低字节在前。
  控制使用 `Roll(0x3D)` 和 `GX(0x37)`：`roll = raw / 32768 × π` rad，
  `roll_rate = raw / 32768 × 2000 × π / 180` rad/s，与图片中的 ±180°、±2000°/s 换算一致。
  加速度的 ±16g 换算不参与当前侧倾控制接口。
- I²C 事务有有限超时；NACK、超时或配置回读不一致会返回错误。读取失败会发布无效样本，触发原有安全检查。
  数据块没有逐帧采样时间戳，以读取开始时刻减去一个 5 ms 输出周期估计采样时间，
  不含模块内部融合滤波延迟，也不能检测“寄存器仍应答但内部融合已冻结”的故障；实板时效尚需测量。

首次连接后将模块水平静置，并用厂家工具按需要完成加速度/磁场校准。
程序不会在每次启动时自动校准或执行图片中的 Z 轴归零；Z 轴归零也不等于自行车的 Roll 零位校准。
协议与寄存器依据：[维特官方 IIC 示例及 SDK](https://github.com/WITMOTION/WitIIC_JY901/tree/main/ESP32/main)。

构建烧录后，串口应出现 `IMU=JY901P I2C SDA=21 SCL=22 addr=0x50` 和 `JY901P ready`；
`FEEDBACK_MODE=real_imu_mock_drive` 表示 IMU 为实测、飞轮反馈仍模拟，状态包的 `mock=True` 仍会置位。
保持模块接近水平，缓慢左右倾斜，观察电脑端 `roll`、`rate` 的符号和变化；倾斜超过 10° 会锁存倾倒故障，需要复位。
以下历史 Wi-Fi 动态模拟测试步骤需先关闭 JY901P 选项，不能用于证明真实传感器性能。

## 现在怎么运行

1. 先退出电脑策略程序（Ctrl+C），VS Code 打开本工程，选择 ESP-IDF v5.4.4、esp32、UART 和实际烧录串口，构建、烧录新版固件。
2. 打开 Ubuntu 的 Wi-Fi 菜单，连接下面的 ESP32 热点：

| 项目 | 默认值 |
|---|---|
| 热点名称 | `BalanceBike-ESP32` |
| 热点密码 | `balance123` |
| ESP32 IP | `192.168.4.1` |
| UDP 端口 | `3333` |

3. 确认笔记本连接的是这个热点，然后在 Ubuntu 终端运行：

```bash
cd /home/lxs/esp/projects/balance_bike
/home/lxs/miniconda3/envs/legged_gym/bin/python host/run_policy.py --esp-ip 192.168.4.1
```

这里使用你已有的 `legged_gym` Python，因为它已经有 NumPy。不用启动 ROS、Isaac Gym 或 CUDA，不需要 PyTorch 或另装 pyserial。
程序仅在自身进程设置 BLAS 为单线程，适合这个小网络，不修改系统环境。
ESP32 建立的是本地控制热点，不提供互联网；Ubuntu 显示“无互联网”时仍需保持连接。Wi-Fi 密码/名称/信道可在 menuconfig → Balance bike framework 修改，修改后重新烧录。
电脑程序不打开串口，ESP-IDF Monitor 可以同时显示启动/连接日志。USB 数据线现在用于烧录、供电和诊断；移除 USB 前需给 ESP32 提供合适的独立电源，否则热点会随断电消失。

模型默认是：

```text
/home/lxs/work_place/balance_bike/deploy/policies/best_turn.npz
```

更换模型：

```bash
/home/lxs/miniconda3/envs/legged_gym/bin/python host/run_policy.py \
  --esp-ip 192.168.4.1 --policy /绝对路径/新模型.npz
```

更换模型前，程序会检查当前网络形状、输入维度、记忆设置及速度/倾倒限值是否匹配。
这个版本只支持现有 4 维观测 + 积分记忆的 128×128 镜像 actor，未实现外部慢卸速/动量控制器。

PC 终端每 0.5 秒显示类似下面的字段（数值示意，不是实板测量结果）：

```text
n=120 link=OK mock=True infer=100us max=300us rtt=5.00ms ack=119 fault=0x00 roll=0.0035 speed=0.00 action=0.1669 torque=0.8180Nm
```

- `infer/max`：电脑状态检查、记忆更新和策略计算耗时，不包含 Wi-Fi 通信。
- `rtt`：ESP32 从生成状态包到接收并校验该包对应指令的时间，包括上传、电脑处理、回传和调度。由 ESP32 同一时钟测量，无需与电脑对时。
- `ack`：ESP32 已接受的状态序号对应指令；不代表 ODrive 已施加力矩。
- `mock=True`：仍有模拟反馈；默认 JY901P 模式下仅飞轮反馈模拟，显示的力矩只是计算结果，电机保持关闭。
- `fault=0x40`：启动等待电脑或指令已失效；模拟模式收到新鲜指令后通信状态恢复。没有自动解锁电机。
- `old`：被丢弃的重复/乱序/旧启动会话状态包；`gaps`：UDP 读取时检测到的状态序号缺失数。
- `coalesced`：一批连续到达的状态中，为只处理最新状态而主动跳过的较早状态数。
  因而 `gaps=0` 不代表每个 60 Hz 状态都交给了策略；发生批量到达时 `coalesced` 仍可增加。
- `command=RUN/STOP`：本次电脑发送的指令类型。`link` 和 `fault` 来自刚收到的 ESP32 快照，
  反映先前指令的状态，可能同时出现 `link=OK command=STOP`；尚不代表这次发送已被确认。
  `policy_stops` 累计电脑状态检查失败或计算超时而主动发送 STOP 的次数，不含退出 STOP。
- `fault` 其他位：1 无效传感器，2 数据过期，4 上传时序异常，8 策略异常保留位，16 倾倒，32 驱动错误。这些本地故障锁定到 ESP32 复位。
- `imu_age/enc_age`：当前快照中 IMU/驱动反馈的实际年龄（微秒）。`0x42` 是过期故障 `0x02` 加未持有有效运行指令 `0x40`；锁定后当前年龄恢复正常也不会自动清除故障。
- `cmd_age`：最近已接受指令所引用的 ESP32 状态年龄（微秒），包含往返时间；未收到指令时为 `4294967295`。`rate` 为模拟角速度，单位 rad/s。

按 `Ctrl+C` 退出电脑程序，尽力发送 STOP；即使电脑直接断电/进程被杀，ESP32 仍按自身时钟使旧指令失效。
再次烧录会重启 ESP32，电脑程序可能暂时收不到数据。建议先 Ctrl+C 退出，烧录完成并重新连上热点后再次运行。
串口监视器可查看 `SSID=BalanceBike-ESP32 IP=192.168.4.1 UDP=3333` 和热点连接日志；逐周期策略结果仍在电脑程序终端显示。

### 下一步：动态输入与指令超时测试

先 Ctrl+C 退出旧的 `host/run_policy.py`；同一时间只运行一个电脑控制程序。
如果串口监视器正在运行，先 Ctrl+] 退出它，再烧录：

```bash
cd /home/lxs/esp/projects/balance_bike
source /home/lxs/.espressif/tools/activate_idf_v5.4.4.sh
idf.py -p /dev/ttyUSB0 flash monitor
```

确认启动显示 `BIKE_FW=wifi-udp-v4` 和 `MOCK_INPUT=sine_roll amplitude=2deg period=4s`。
重新连接 `BalanceBike-ESP32` 热点，保留串口监视器，新开终端：

```bash
cd /home/lxs/esp/projects/balance_bike
/home/lxs/miniconda3/envs/legged_gym/bin/python host/test_wifi.py --esp-ip 192.168.4.1
```

脚本收到首帧后先运行至少 5 秒，确认连续 3 帧均为新鲜、已确认的 RUN 指令，
才进入暂停阶段。若链路抖动，额外等待最多 10 秒；仍无健康起点则 FAIL。
正常约 8.3 秒完成测试：

| 阶段 | 时长 | 检查 |
|---|---:|---|
| RUN | 至少 5 秒 | 倾角覆盖正负方向，策略动作发生变化，暂停前有健康 RUN 指令确认 |
| PAUSE | 0.3 秒 | 停发 COMMAND，不主动发送 STOP；继续 HELLO/接收状态，旧指令仍须过期 |
| RECOVER | 3 秒 | 重置电脑策略记忆，恢复新鲜指令，确认 `fault=0x00` 与有效命令标志恢复 |

正常时电脑输出 `PASS: changing input/action, command expiry while HELLO continues, and link recovery`，
并将本次实测报告保存到 `docs/wifi_test_latest.json`。失败会报告原因并返回非零状态；不会将未测结果标成 PASS。
报告另外记录 `coalesced` 和计划暂停以外的 `policy_stops_outside_pause`；若后者非零会打印 NOTE。
PASS 只表示本次动态输入/超时/恢复检查通过，不表示每一周期均按连续 60 Hz 执行。
暂停前打印 `PAUSE_BASELINE valid=1`，记录已确认 RUN 的序号和时间戳；暂停时不再发送 COMMAND，
持续检查确认序号不变、该指令年龄随 ESP32 时间增长。`cmd_age<50000us` 应仍有效，
达到期限后应为 `fault=0x40` 且有效指令标志清除。提前 STOP 或确认序号改变都判为 FAIL。
`command=HELD` 表示没有发送新的指令；暂停行中的 `torque=0` 只是电脑占位显示，不表示发出了零力矩。
同时串口可见 `CONTROL_LINK=OK → CONTROL_LINK=STOP → CONTROL_LINK=OK`；脚本退出时再发送 STOP。
串口日志的 `accepted_torque` 是本地接受的候选值，`applied=0Nm MOTOR_OUTPUT=OFF` 表示输出一直关闭。
该脚本验证模拟输入、通信与超时状态，不验证电机物理停机，也不证明车辆能平衡。

正弦输入不会根据策略力矩改变：它是信号测试，未实现车体或飞轮动力学。
需要回到固定 0.2° 测试时，先关闭 `Use real JY901P IMU over I2C`，再在 menuconfig → Balance bike framework 中关闭
`Generate a +/-2 degree, 4 second mock roll waveform`，然后重新构建、烧录；此时动态测试脚本会拒绝给出 PASS。
长期观察仍使用上面的 `host/run_policy.py`。要测试整个电脑进程暂停，可在它所在终端按 Ctrl+Z，
观察串口 `CONTROL_LINK=STOP`；执行 `fg` 恢复，观察重新变为 OK。Ctrl+C 是主动 STOP，和无指令超时不同。

### 状态批量到达：关闭 Wi-Fi 设置窗口，检查省电

连接 `BalanceBike-ESP32` 后，先关闭电脑的“设置 → Wi-Fi”窗口，再启动策略程序。
这台 Ubuntu 的 GNOME 3.36.5 会在该窗口存在时每 15 秒主动请求扫描。
实测关闭窗口后扫描消失，60 秒对照的非计划 STOP 从 155 次降到 4 次；
这说明扫描是本次卡顿的重要因素，尚不能解释所有延迟。源码与实测报告见 `docs/wifi_latency_diagnosis.md`。

同时检查接口 `wlx8c772b016deb` 的省电状态。临时关闭省电不需要重连热点，但需要用户在本机输入 sudo 密码：

```bash
sudo iw dev wlx8c772b016deb set power_save off
iw dev wlx8c772b016deb get power_save
/home/lxs/miniconda3/envs/legged_gym/bin/python /home/lxs/esp/projects/balance_bike/host/test_wifi.py --esp-ip 192.168.4.1
```

确认第二条输出 `Power save: off`，再比较 `coalesced` 和 `policy_stops_outside_pause`。
本次省电关闭后单独复测仍有 STOP，因此没有证实省电是唯一原因。
曾试验绑定热点 BSSID，但未阻止这些扫描，已恢复原连接配置：BSSID 空、省电 `default`。
当前运行期省电仍为关闭；重连后需重新检查。
恢复命令为 `sudo iw dev wlx8c772b016deb set power_save on`。

### 搜不到热点时

在 VS Code 左侧 ESP-IDF 面板选择“监视设备”（115200 baud），按开发板 EN/RESET，查看完整启动日志：

- `BIKE_FW=wifi-udp-v4`：确认运行的是当前正常 Wi-Fi 固件，另有日志显示 PHY 发射功率上限。
- `stage=...`：配置、NVS/网络、Wi-Fi 驱动、UDP socket、Wi-Fi 启动的检查点。
- `AP_STARTED: hotspot=BalanceBike-ESP32`：Wi-Fi 已报告热点启动事件。
- `Framework startup failed`：初始化未完成；保留前面的错误行一起定位。
- `Brownout`、`Guru Meditation` 或不断重复启动：先按日志定位供电或崩溃。

若看见 `AP_STARTED` 但电脑仍搜不到，用手机 Wi-Fi 扫描一次，区分热点广播和电脑扫描问题。不要在没有对应错误日志时先擦除 Flash/NVS。

### BOOT 模式稳定、运行应用就断连：隔离诊断

menuconfig → Balance bike framework → `Run isolated Wi-Fi diagnostic instead of bike application`
启用后，重新构建、烧录。此模式只编译入口和 `main/wifi_diagnostic.c`，不运行自行车任务、
UDP、传感器或电机接口。它采用本机 ESP-IDF `examples/wifi/getting_started/softAP` 的基本启动顺序，
保留现有热点名称、密码和信道，不擦除 NVS、不关闭欠压保护。

启动标记为 `BIKE_FW=wifi-diagnostic-v2`，另打印当前编译时的 PHY 发射功率上限。
先每秒打印 `RF_OFF alive=...` 共 10 秒，
然后打印 `stage=wifi_start: enabling radio now` 并启动热点。监视器使用 115200 波特率。

- 前 10 秒已经断连：问题发生在射频启动之前，继续查 USB/驱动/板级硬件。
- 前 10 秒稳定，启动射频后断连：最小热点也触发问题，尚不能单凭这一点区分供电、板上射频硬件或 IDF/PHY 问题。
- 看见 `AP_STARTED` 且心跳持续：最小热点可以运行，再检查完整应用初始化的差异。

此诊断模式不提供电脑策略通信。排查完成后取消该选项，重新构建、烧录恢复正常固件。

2026-10-09 的实板诊断日志确认：v1 的多次启动能完成 10 秒 `RF_OFF` 心跳，
随后在最小热点的 `esp_wifi_start()` / PHY 初始化附近出现 USB 串口丢失并再次启动。
其中一次在 `RF_OFF alive=8s` 后也有复位记录，未确定是否为手动复位。
这次复现没有运行自行车任务或 UDP；仍未确定是供电、板级硬件还是 IDF/PHY 问题。
v2 对照将 `sdkconfig` 中 `CONFIG_ESP_PHY_MAX_WIFI_TX_POWER` 从 20 改为 10 dBm，
隐藏的派生值 `CONFIG_ESP_PHY_MAX_TX_POWER` 同为 10。该设置用于生成 PHY 初始化数据，
不用等待 `esp_wifi_start()` 成功后才调用运行期功率 API。用户提供的 v2 实板日志确认：
10 秒心跳完成，`esp_wifi_start()` 返回成功，DHCP 启动，出现 `AP_STARTED`，
之后继续打印至少 20 秒心跳，未记录到断连。该结果说明现象对功率配置敏感，支持供电余量不足的怀疑，
但不能确定具体故障部件，也没有证明实车运行可靠性。正常固件已恢复，保留相同的 10 dBm 设置；
`sdkconfig.defaults` 也设置为 10 dBm，避免重建配置时回到 20 dBm。后续正常应用的实板输出已确认双向 UDP 收发：
报告的 RTT 为 2.34–6.48 ms，`crc_bad/old/gaps` 均为 0，但当时存在锁定的过期故障，电脑发送 STOP，尚未进入健康策略运行。
它不保证消除射频初始化的峰值负载；即使启动成功，也不能据此认定供电已满足实车运行要求。
暂时保留 10 dBm；后续如需做功率对照，可在 menuconfig → Component config → PHY →
Max WiFi TX power (dBm) 修改，再构建、烧录。提高功率可能再次复现之前的断连。

只测试电脑推理、暂不连接板子：

```bash
/home/lxs/miniconda3/envs/legged_gym/bin/python host/run_policy.py --self-test
```

该命令执行 121 次模拟状态推理，只报告电脑计算耗时，不访问网络，也不代表真实 Wi-Fi 链路已经通过测试。

## 周期与通信

| 工作 | 设置 | 原因 |
|---|---:|---|
| ESP32 读取最新状态 | 200 Hz / 5 ms | 初始采集目标，实际频率取决于 IMU 与 ODrive |
| 上传状态 / 电脑策略 | 60 Hz / 16667 µs | 训练 `(1/120)*2` 与 MuJoCo `(1/240)*4` 均为 1/60 s |
| 指令应用 | 收到有效回包立即处理 | 不再等下一次 60 Hz 上传，避免额外一整周期延迟 |
| Wi-Fi 通信 | ESP32 SoftAP + UDP，端口 3333 | 每个数据报恰好一帧，状态 68 字节、指令 36 字节；收到新状态立即处理 |
| 允许指令回复年龄 | 最多 25 ms | 根据 ESP32 状态时间戳校验；超时的结果直接丢弃 |
| 指令有效期限 | 状态生成后 50 ms | 本地任务检查到过期便停机；不是 50 ms 的精确硬实时保证 |
| PC 可读输出 | 2 Hz | 电脑终端显示统计，UDP 数据包保持固定格式 |

60 Hz 保留了训练的动作时间尺度；电脑由新状态包触发推理，不用独立 sleep 定时推理。
若一次收到多个数据报，只处理最新完整状态；按 ESP32 时间戳丢弃旧/重复包，不按积压逐帧执行旧动作。每个 UDP 数据报独立校验，截断/拼接帧不接受。
策略记忆按 ESP32 状态时间间隔作梯形积分；复位、丢帧/重复帧、明显间隔异常或无效状态会重置记忆并发送 STOP。

ESP32 使用 `esp_timer` 唤醒任务，Wi-Fi/lwIP socket 负责收发；分辨率不等于实时调度保证。Wi-Fi 重传、电脑无线网卡省电、干扰和操作系统调度都可能增加延迟。
ESP32 AP 模式让笔记本直接连接，不需要路由器或填写家中 Wi-Fi 密码。参考 [ESP-IDF Wi-Fi](https://docs.espressif.com/projects/esp-idf/en/v5.4.4/esp32/api-reference/network/esp_wifi.html)、[lwIP sockets](https://docs.espressif.com/projects/esp-idf/en/v5.4.4/esp32/api-guides/lwip.html) 和 [esp_timer](https://docs.espressif.com/projects/esp-idf/en/v5.4.4/esp32/api-reference/system/esp_timer.html)。
UDP 没有应用层重传；本框架通过序号/时间戳筛选最新状态和指令。Wi-Fi 底层仍可能重传，因此不能保证固定延迟。

接车前需要重新实测 Wi-Fi 往返延迟、抖动和掉包，并在仿真中验证这些延迟对当前策略的影响。电脑快不等于整条控制链路快。
25/50 ms 是本通信实验的工程设置，不是已经验证的实车稳定性边界。

## 文件职责

```text
main/main.c                   入口
main/bike_app.c               200 Hz 采样、60 Hz 上传、接收通知、本地检查
main/app_config.h             频率、UDP 端口、指令超时，电机输出固定 false
main/control/bike_safety.*    状态过期、倾倒检查和独立力矩/转速边界
main/link/bike_protocol.*     二进制封包、CRC32、完整 UDP 数据报校验
main/link/bike_udp_peer.*     单个电脑端点登记与 1 秒租期
main/link/bike_link_guard.*   会话/序号/时间戳匹配、过期与重放检查
main/ports/pc_link_port.*     ESP32 热点、UDP socket、电脑端点登记
main/ports/imu_port.*         JY901P I²C 驱动与动态/固定模拟反馈
main/ports/odrive_port.*      ODrive 反馈/力矩/停机接口与模拟实现
host/run_policy.py           电脑端启动命令、耗时/往返延迟显示
host/test_wifi.py            动态输入、暂停指令、恢复通信的有限时长实验
host/controller.py           复用 NumPy actor，积分、镜像策略、速度保护
host/protocol.py             与 C 一致的字节协议
host/udp_link.py             UDP 收发、新状态筛选、乱序/旧会话丢弃
```

之前的 `main/policy/*` 和 `main/control/bike_controller.*` 保留用于主机数值对照测试，**不在当前固件 CMake 源文件列表里**。
旧板上推理说明保留于 `docs/onboard_inference.md`，不是当前运行流程。
训练仓库只读复用，没有修改。

## 模型约定与 ODrive 接口

电脑读取原有 `/home/lxs/work_place/balance_bike/sim2sim/policy.py` 的 `NumpyPolicy`。
模型输入为 `sin(roll)`、`cos(roll)`、机体系 X 角速度裁剪到 `[-5,5]`、飞轮 rad/s 除以 420 裁剪到 `[-1.5,1.5]`。
第五维是飞轮速度梯形积分除以 1000，裁剪到 `[-10,10]`；镜像分支保留 cos 符号。
动作经过现有 MuJoCo 的平滑/速度保护后乘 4.9 N·m。

4.9 N·m、250 rad/s、10°、惯量 0.00739 kg·m² 来自仿真，不是已确认的实车参数。
ESP32 再独立检查力矩幅值与超速加速方向，不依赖电脑始终正确运行。

ODrive 接口仍是：

```c
esp_err_t odrive_port_init(void);
esp_err_t odrive_port_read_feedback(odrive_feedback_t *feedback);
esp_err_t odrive_port_write_command(const odrive_command_t *command);
esp_err_t odrive_port_stop(void);
```

反馈单位为飞轮侧 rad/s，命令为飞轮侧 N·m，驱动负责编码器来源、传动比、安装方向和单位转换。
ODrive 内部承担无刷电机的电流/力矩环；ESP32/电脑不承担换相。
ODrive 协议、轴号、力矩模式、限流和看门狗待型号/固件确定后接入。
真实 `stop/enable=false` 必须请求停用轴，不能只是传零力矩。ODrive 自身也应配置独立看门狗，不能只依赖电脑或 ESP32。参考 [ODrive 力矩控制](https://docs.odriverobotics.com/v/latest/manual/control.html) 和 [CAN/看门狗](https://docs.odriverobotics.com/v/latest/manual/can-protocol.html)。

此版没有真实驱动、校准或电机解锁流程；关闭模拟硬件配置会在初始化时报 `ESP_ERR_NOT_SUPPORTED`。
30pin 是排针布局，编译目标仍选 esp32；以后接 CAN 还需要收发器，不能将 GPIO 直接接 CANH/CANL。

## 字节协议与验证

协议详情：`docs/pc_link_protocol.md`。采用帧头、版本、长度、序号、ESP32 启动 ID、原状态时间戳和 CRC32。
电脑先发 HELLO 登记返回 IP/端口，并周期性刷新端点登记；HELLO 不构成控制指令，也不会延长旧力矩有效期。
只允许一个活跃 UDP 端点，其他端点不能抢占；端点无报文 1 秒后可重新登记，Wi-Fi 客户端断开时立即清除登记。
旧会话、重复/倒序、未知状态序号、错误 CRC、非有限力矩、超限/超时指令都不更新有效命令。
热点采用 WPA2 密码。CRC 用于数据完整性，不提供应用层身份认证；当前为电机关闭的单客户端实验。

验证命令：

```bash
/home/lxs/miniconda3/envs/legged_gym/bin/python -m unittest discover -s tests -p 'test_*.py' -v
```

32 项主机测试覆盖 C/Python 字节一致、CRC/截断/拼接帧拒绝、旧包/过期/非有限指令拒绝、序号回绕、独立限幅、600 步电脑策略与旧 C 控制器对照、UDP 端点登记/过期/抢占拒绝、乱序/旧启动会话丢弃，以及真实 PC 程序通过本机回环 UDP 的请求/响应与退出 STOP。
新增控制任务回归测试直接运行实际 `control_task`，在取队列样本时注入采样抢占：
新鲜样本不会被取样前的旧“当前时间”误判为未来时间戳；真实超过 20 ms 的样本仍触发并锁定过期故障。
新增测试编译实际 IMU 端口，验证波形峰值、角速度导数、长期精度和真实电脑 actor 的动态输出；
直接运行实际控制任务，验证端点仍在线时指令超时会停用、恢复新指令后通信可恢复，且始终不使能电机。
实验脚本用确定性遥测验证正常、固定输入、超时失败、恢复失败和无数据情况的判定与退出 STOP。
另覆盖暂停起点已 STOP、期限前提前 STOP、暂停期间确认序号变化三种情况，避免把原本失效的指令误判成暂停超时成功。
顺序状态批量到达的回归测试确认：即使 UDP 层没有序号缺口，策略仍可能因只取最新状态而跨过周期；
`coalesced` 会如实计数，策略仍保持原有间隔检查并发送 STOP，不靠重放积压动作掩盖延迟。
固件已通过 ESP-IDF 构建，v4 由用户烧录。本轮工具已通过当前 ESP32 热点完成动态/超时/恢复实验，
未更改固件或控制保护逻辑，未测试真实 IMU 或 ODrive 电机输出。
省电与扫描对照已完成，详细结果见 `docs/wifi_latency_diagnosis.md`；连续 60 Hz 长期稳定性仍待验证。
