# JY901P 3D 查看器

整车联调使用仓库根目录默认固件。仅看陀螺仪时，可在仓库根目录烧录独立固件 `idf.py -C standalone/jy901p -p /dev/ttyUSB0 build flash`。烧录前先激活 ESP-IDF 环境，并按实际端口修改串口。

电脑连接热点 `BalanceBike-ESP32`（密码 `balance123`），从仓库根目录运行。

```bash
python3 standalone/jy901p/view_3d.py
```

需要 Python 3 和 Tkinter。默认源为 UDP `192.168.4.1:3334`，可用 `--wifi`、`--udp-port` 改地址和端口。右上角显示实际收包和绘制频率。桌面查看器与 `python3 standalone/odrive_uart/wifi_console.py` 择一使用。

## 界面操作

- 左键拖动视角，滚轮缩放。模型红色端为传感器 +X。右侧显示角度、角速度和加速度，底部显示最近的姿态轨迹。
- 静止在参考姿态时点击「设为零点并保存」，查看器会把该姿态存到本机 `reference_pose.json`。可用「显示绝对姿态」或「使用已存零点」切换。控制固件的 `zero` 指令单独设置 ESP32 上的平衡零点。
- 用默认根固件控制电机时，先点「设置软件限幅」，输入实车允许的力矩和转速上限。然后设置电机 0 力矩（N·m）或电机 1 转速（圈/s）。查看器显示 `UART TX complete` 时，表示 ESP32 已完成发送。
- 「手动停机」发送 `stop`。**关闭窗口和 Wi-Fi 断线后，已经启动的本地控制会继续运行。** 独立陀螺仪固件只提供姿态。操作详见 [ODrive 控制工程说明](../odrive_uart/README.md)。

## 其他数据源

| 参数 | 用途 |
|---|---|
| `--reference-file ./my_bike_reference.json` | 为不同安装方式分别保存显示参考 |
| `--port auto --baud 460800` | 读取旧版固件的 USB 串口输出，需要 pyserial |
| `--log /tmp/jy901p-live-monitor.log` | 查看旧串口日志 |

旧版串口实板记录和 Wi-Fi 热点检查分别见 [100 Hz 测试报告](test_results/100hz/report.md) 和 [Wi-Fi 测试记录](test_results/wifi/report.md)。
