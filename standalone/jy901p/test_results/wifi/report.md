# Wi-Fi 热点实板验证（2026-11-18）

- 问题：电脑看不到热点。读取启动日志发现板上仍是旧的根工程 `balance_bike`，使用 UDP 3333。
- 处理：通过 `/dev/ttyUSB0` 烧录 `standalone/jy901p` 的新版独立 Wi-Fi 固件，写入校验通过，未擦除 NVS。
- 新版启动日志：`Project name: jy901p_standalone`；IIC 初始化成功，`output=100Hz Wi-Fi=100Hz`；SoftAP 与 DHCP 启动成功。
- 热点：`BalanceBike-ESP32`，BSSID `08:B6:1F:9B:D1:21`，IP `192.168.4.1`，UDP `3334`。
- 电脑重新扫描实际发现该热点，NetworkManager 报告信号 100%。
- 没有发送电机控制指令；Wi-Fi 断线不触发停机的行为保持不变。

这次验证确认了固件烧录、IMU 初始化和热点可见性。电脑仍保持原有网络连接，未通过热点采集 UDP 数据，因此尚未实测无线接收频率和长时间稳定性。
