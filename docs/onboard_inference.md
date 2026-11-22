# ESP32 反作用飞轮自行车：强化学习部署框架

> 本文记录早期的板上 C 推理方案，相关源码用于对照测试。当前根工程默认运行 JY901P 与 ODrive 控制固件，使用说明见 [项目 README](../README.md)。

本工程位于 `/home/lxs/esp/projects/balance_bike`，使用 ESP-IDF v5.4.4。
策略来自 `/home/lxs/work_place/balance_bike/deploy/policies/best_turn.npz`，已导出为 C 的只读 float32 权重。
它是本地默认部署 YAML 选择的 best_turn 策略；换模型时必须重新导出并做对照检查。

旧方案的默认模式为 **DRY_RUN**，使用模拟传感器输入和真实策略推理，电机输出关闭。
模拟 IMU 固定为倾角 0.2°，飞轮速度固定为 0，不是自行车动力学仿真。

## 文件从哪里看

```text
main/
  main.c                         入口，启动框架
  bike_app.c/.h                  采样任务、控制任务、日志任务、定时通知
  app_config.h                   周期、优先级、栈、数据超时
  bike_types.h                   状态、策略结果和故障码
  control/bike_controller.c/.h    动作平滑、仿真限速、力矩换算、状态检查
  policy/rl_policy.c/.h           ELU 网络、镜像分支、飞轮积分记忆
  policy/policy_weights.c/.h      从当前 NPZ 导出的只读权重
  policy/policy_manifest.json     权重来源、哈希、输入约定、仿真参数
  ports/imu_port.c/.h             IMU 接口与模拟实现
  ports/odrive_port.c/.h          ODrive 反馈/力矩/停机接口与模拟实现
  Kconfig.projbuild               模拟硬件开关
tools/export_policy_c.py          NPZ → C，后续更换策略用
tests/test_policy_runtime.py      主机上验证 C 与 NumPy 策略一致
docs/validation.json             本次检查结果；不代表实板测时
docs/task_notification_example.c 原来的任务通知实验，保留供学习
```

## 周期为什么这样设置

| 工作 | 初始设置 | 依据 |
|---|---:|---|
| RL 策略推理、动作更新 | 60 Hz，16667 µs | Isaac Gym 训练 `(1/120)*2`；MuJoCo `(1/240)*4`，都是 1/60 s |
| 状态采样/读取最新反馈 | 200 Hz，5000 µs | 工程初始目标，减少控制读取时的状态年龄；实际 IMU ODR、ODrive 推送频率尚未确定 |
| 串口状态日志 | 2 Hz，500 ms | 观察运行而不在 60 Hz 控制任务中打印 |
| ODrive 内部电流/力矩环 | 由 ODrive 执行 | ESP32 发力矩目标，不承担无刷电机换相或电流环 |

强化学习模型学习的是采样系统。把策略从 60 Hz 随意改成 200 Hz 会改变动作保持时间、状态变化和记忆积分，不是简单的“越快越好”。更换策略频率需要重新进行仿真验证，必要时重新训练。
200 Hz 是驱动层的采集目标，不是第二个策略频率；轮询缓存 200 次/秒也不能使低频设备生成新数据。驱动必须保留真实数据时间戳。

已有工程 `CONFIG_FREERTOS_HZ=100`，一个 tick 是 10 ms，不适合用固定整数 tick 精确表达 16.667 ms。因此采用 `esp_timer` 微秒定时器，回调只发送任务通知，推理在控制任务执行。定时器分辨率不代表实际任务没有调度抖动。
16667 µs 相对于 1/60 s 的偏差约 0.002%，任务仍需实板检查耗时和抖动。
CPU 已设为 240 MHz，网络推理源文件使用 `-O2`；控制/采样固定在 Core 1，日志在 Core 0。单核配置会回退到 Core 0。

## 当前模型的输入和输出

外部输入严格与训练一致：

1. `sin(roll_rad)`。
2. `cos(roll_rad)`。
3. 机体系 X 轴角速度，rad/s，裁剪到 `[-5, 5]`；不是无条件对欧拉角差分。
4. 飞轮相对车架的有符号角速度除以 **420**，裁剪到 `[-1.5, 1.5]`。

内部第五维是飞轮速度的梯形积分，除以 1000 后裁剪到 `[-10, 10]`。首次推理记忆为 0，之后每个控制周期更新，故障/新会话重置。
框架用实际控制间隔积分；无抖动时等价于仿真的 1/60 s，严重漏周期锁定故障，不连续补跑推理。

网络为 `5 → 128 → 128 → 1`、两层 ELU。
同时计算原始输入与镜像输入，再求 `tanh((f(s)-f(Ms))/2)`，与已有 NumPy 部署一致。
只导出 actor，省略 critic 和探索噪声；权重 69636 字节，无需 PSRAM 或 Python/PyTorch 在板上运行。

动作在 `[-1, 1]`，与当前 MuJoCo 部署一致做动作平滑、速度保护后乘 **4.9 N·m**。
这里的 4.9 N·m、250 rad/s、10° 和 0.00739 kg·m² 都是现有模型/URDF 的仿真参数，**不是已经确认的实车限值**。
观测速度归一化仍是 420，不能因为转速保护是 250 就改成 250。
当前元数据里 `flywheel_unload_kw=0`，外部慢卸速关闭；虽然 `flywheel_unload_ki` 非零，它被 kw 的门控关闭。纯策略之外的卸速器没有额外启用。
当前示例不控制后轮驱动和车把；该 best_turn 原本还配合仿真巡航/地形，实车动力学迁移仍需验证。

## 怎么接底层驱动

按照你的要求，尚未编写 IMU I²C/SPI、ODrive CAN/UART 驱动，也未分配 GPIO。

IMU 接口：

```c
esp_err_t imu_port_init(void);
esp_err_t imu_port_read(imu_sample_t *sample);
```

`imu_port_read` 返回融合后的侧倾角、机体系 X 角速度和实际采样时间。
驱动负责校准、安装轴转换和单位转换。训练坐标系为 X 向前、Y 向左、Z 向上，符号按右手定则。

ODrive 接口：

```c
esp_err_t odrive_port_init(void);
esp_err_t odrive_port_read_feedback(odrive_feedback_t *feedback);
esp_err_t odrive_port_write_command(const odrive_command_t *command);
esp_err_t odrive_port_stop(void);
```

反馈是 **飞轮侧** rad/s、时间戳、轴错误和就绪状态；命令是飞轮侧 N·m、失效时间和 enable。
ODrive 常见速度单位是 turn/s，直连时要乘 `2*pi`；若有传动比，还要转换转速和力矩，确认 encoder 来源及方向。
不要把模型力矩直接写成速度目标或 PWM 占空比；实际 ODrive 控制模式、限流、力矩常数和限速器应与仿真执行器模型核对。

`enable=false` 的契约是请求停用，不是只发零力矩。真实实现需结合对应固件处理轴状态、错误和驱动端看门狗。
接口不得无限阻塞；命令过期不执行，反馈过期不冒充新数据。实际 ODrive 看门狗独立于 ESP32 调度工作。

关闭 menuconfig → Balance bike framework → 模拟硬件后，目前会返回 `ESP_ERR_NOT_SUPPORTED`，不会假装已经实现实车驱动。
`BIKE_MOTOR_OUTPUT_ENABLED` 固定为 false，尚无实车校准和显式解锁流程。不要仅改成 true 就接车测试；真实驱动和解锁/故障处理需完成后再接通。

## 30 针开发板

30pin 表示排针布局，不改变这里的 `esp32` 编译目标。框架使用普通双核 ESP32，不依赖 PSRAM。
这不表示不同厂家的 30 针板引脚排列一定相同。接 IMU/ODrive 前按板上 GPIO 标记和模块型号确认；当前代码没有占用任何外部 GPIO。
CAN 接入还需要收发器，不能只把 ESP32 GPIO 接到 CANH/CANL。选择 CAN/UART 前先确定 ODrive 型号和固件。

## 烧录后看什么

VS Code → ESP32 Profile → 打开本工程 → ESP-IDF v5.4.4 → esp32 → UART → 实际串口 → 构建、烧录和监视。

启动显示：

```text
policy=60Hz sensor=200Hz logs=2Hz core=1 mock=1 MOTOR_OUTPUT=OFF
```

每 500 ms 输出状态：

- `dt`：最近两次控制任务开始之间的实际间隔，目标约 16667 µs。
- `compute` / `max`：状态检查、策略推理、命令提交耗时及最大值，必须小于周期；建议留足余量，先以小于 8 ms 为调试目标，不是已测量保证。
- `stack`：控制任务历史最小剩余栈，字节。控制任务初始分配 8192 B。
- `missed`：累计控制/采样通知积压次数；正常应该为 0/0。
- `fault=0x00`：没有故障。故障位：1 数据异常、2 过期、4 时序、8 策略、16 倾倒、32 驱动。
- `torque`：模型计算结果，**不是已施加到电机的力矩**。

故障保持到复位，电机始终关闭输出。日志是运行检查，不是闭环平衡验证。
本机目前没有 `/dev/ttyUSB0`，本次未烧录、未测量 ESP32 推理时间、未进行实车试验。

## 更换权重与验证

```bash
/home/lxs/miniconda3/envs/legged_gym/bin/python tools/export_policy_c.py \
  --source /home/lxs/work_place/balance_bike/deploy/policies/best_turn.npz \
  --urdf /home/lxs/work_place/balance_bike/assets/urdf/fixed_gear_balance_bike/fixed_gear_balance_bike.urdf \
  --output-dir main/policy

/home/lxs/miniconda3/envs/legged_gym/bin/python tests/test_policy_runtime.py
```

目前导出器只接收本策略的四维观测加内部记忆、128×128 镜像网络；不支持任意网络结构。
测试在主机 GCC 上验证数值，覆盖 512 组输入、600 步记忆轨迹、镜像对称、速度保护、无效数据/过期/倾倒/驱动错误检查。
结果见 `docs/validation.json`。主机一致性不能代替 ESP32 浮点数值、时序和真实驱动验证。

参考：
- [ESP-IDF esp_timer](https://docs.espressif.com/projects/esp-idf/en/v5.4.4/esp32/api-reference/system/esp_timer.html)
- [ODrive 力矩控制](https://docs.odriverobotics.com/v/latest/manual/control.html)
- [ODrive CAN 与看门狗](https://docs.odriverobotics.com/v/latest/manual/can-protocol.html)
