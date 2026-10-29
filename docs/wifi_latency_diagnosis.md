# 2026-10-09 Wi-Fi 延迟对照

当前只有 ESP32，IMU 为脚本生成的正弦输入，电机输出固定关闭。
本轮没有修改固件、策略或保护阈值。`PASS` 只表示动态输入、同一指令过期和通信恢复检查通过，不能代替连续周期检查。

## 已观察到的改善

同一 ESP32 启动会话、同一电脑网卡 `wlx8c772b016deb`、运行期省电关闭，两个实验均为 30 秒 RUN、0.3 秒 PAUSE、30 秒 RECOVER：

| 状态 | 非计划策略 STOP | 合并跳过状态 coalesced | RTT 范围 | 最大推理耗时 |
| --- | ---: | ---: | --- | ---: |
| Wi-Fi 设置窗口打开，试验绑定热点 BSSID | 155 | 231 | 2.039–24.888 ms | 475 µs |
| 关闭 Wi-Fi 设置窗口，连接配置恢复原值 | 4 | 4 | 2.018–24.189 ms | 403 µs |

报告：`wifi_test_v2_pinned.json`、`wifi_test_v2_settings_closed.json`。
关闭窗口后的暂停检查从已确认的有效 RUN 指令开始，同一指令在源状态年龄达到 50001 µs 时失效，恢复后可继续接收新指令。

关闭窗口后，66 秒 `iw event -t` 记录没有扫描事件。窗口打开时则有约 15 秒一次、每次约 2.8 秒的扫描。
事件文件：`wifi_scan_events_pinned.log`、`wifi_scan_events_pinned_after_reset.log`、`wifi_scan_events_settings_closed.log`。
绑定 BSSID 未阻止扫描，已回滚该配置，记录在 `wifi_connection_before_bssid_test.json`。
回滚未重连；当前连接配置省电为原来的 `default`，运行期仍为 `off`。

本机 `gnome-control-center` 版本为 `1:3.36.5-0ubuntu4.1`。
[GNOME 3.36.5 对应源码](https://raw.githubusercontent.com/GNOME/gnome-control-center/3.36.5/panels/network/net-device-wifi.c)
定义了 15 秒周期，并通过 `nm_device_wifi_request_scan_async` 请求扫描。
源码机制与关闭窗口后的对照结果共同支持：Wi-Fi 设置窗口是本次周期扫描与大量卡顿的重要因素。
这不是证明其解释了全部延迟，也没有通过 D-Bus 直接识别每次扫描的请求方。

## 延迟在何处

窗口打开、省电关闭时，另做了 18 秒 HELLO-only 接收实验（不发送 COMMAND）。
用 Linux `SO_TIMESTAMPNS` 的内核收包时间与应用读取时间分别记录：

- 接收 1078 个状态，序号无缺口、CRC 错误为 0。
- 最大相邻内核到达间隔约 122.993 ms。
- 最大“内核收包到 Python 读取”延迟约 188.105 µs，没有超过 5 ms 的读取延迟。
- 57 个到达间隔超过 25 ms，其中 37 个结束于已记录的扫描期间。
- 扫描之外也有 20 个超过 25 ms 的间隔，故扫描不能解释全部异常。

报告：`wifi_rx_timing_unpinned.json`、`wifi_scan_events_unpinned.log`。
这次采样显示明显延迟发生在应用读取之前，不能把它归因于神经网络推理耗时。
内核到达时间仍不能单独区分 ESP32 发送、无线传输、网卡驱动等环节。

关闭设置窗口后另做 18 秒相同的 HELLO-only 实验：接收 1081 个状态、无序号缺口或 CRC 错误，
内核相邻到达间隔最大 41.830 ms、95 分位 20.470 ms；仍有 10 个间隔超过 25 ms。
应用读取延迟最大 190.388 µs，没有超过 5 ms 的读取延迟。
报告为 `wifi_rx_timing_settings_closed.json`。这进一步表明接收链路仍有残余抖动，不能仅凭关闭窗口判定稳定。

用户关闭省电后的默认长度实验也通过了功能检查，但仍有 39 次非计划 STOP、61 次 coalesced；
报告为 `wifi_test_v2_powersave_off_unpinned.json`。省电关闭并未单独解决问题。

## 日常操作与限制

连接 ESP32 热点后关闭 Wi-Fi 设置窗口，保持 Wi-Fi 开启，再运行 `host/run_policy.py`。
连接或重连后用 `iw dev wlx8c772b016deb get power_save` 确认省电状态；
需要时用 `sudo iw dev wlx8c772b016deb set power_save off` 临时关闭。
不需要为这些电脑端设置重新烧录。

关闭窗口后的 60 秒实验仍有 4 次非计划 STOP，不能宣称稳定连续 60 Hz，更没有验证实车平衡。
保留最新状态处理、间隔检查、指令过期和电机关闭，不通过放宽期限或重放积压指令消除 STOP 计数。
