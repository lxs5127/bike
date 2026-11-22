"""供 3D 查看器和文本控制台共用的非阻塞 UDP 客户端。"""
import json
import math
import secrets
import socket
import time

DEFAULT_ADDRESS = '192.168.4.1'
DEFAULT_PORT = 3334
HELLO_PERIOD_S = 0.1
COMMAND_LINK_MAX_AGE_S = 0.3
IMU_SAMPLE_MAX_AGE_US = 30_000
MAX_PACKETS_PER_POLL = 64
COMMAND_PACKET_LIMIT = 192  # 固件接收缓冲区为 192 字节，须留出字符串终止符


def valid_sample(data):
    """姿态绘制只使用三轴读数完整且采样足够新的数据。"""
    if data.get('imu_valid') is not True:
        return False
    for name in ('acc_g', 'gyro_deg_s', 'angles_deg'):
        values = data.get(name)
        if not isinstance(values, list) or len(values) != 3:
            return False
        if not all(type(v) in (float, int) and math.isfinite(v) for v in values):
            return False
    # 两个时间均来自 ESP32。相减是设备内采样到发包的时间，无需电脑与 ESP32 对时。
    sampled_us = data.get('sample_us')
    sent_us = data.get('time_us')
    return (type(sampled_us) is int and type(sent_us) is int and
            0 < sampled_us <= sent_us and sent_us - sampled_us <= IMU_SAMPLE_MAX_AGE_US)


class WifiClient:
    """维护一个会话，向调用方提供最新状态和提示。"""

    def __init__(self, address=DEFAULT_ADDRESS, port=DEFAULT_PORT):
        socket.inet_pton(socket.AF_INET, address)
        if not 1 <= port <= 65535:
            raise ValueError('UDP port must be 1..65535')
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        # UDP connect 只固定目标并过滤其他来源，实际会话由 hello/welcome 建立。
        self.socket.connect((address, port))
        self.address, self.port = address, port
        self.client = secrets.randbits(64) or 1
        self.token = 0
        self.sequence = 0
        self.welcome_sequence = 0
        self.board_us = 0            # 最近的板端时间，随指令回传供 ESP32 检查时效
        self.latest_telemetry_us = 0  # 已接受的最新状态包时间，用于丢弃乱序状态
        self.last_received = 0.
        self.next_hello = 0.
        self.telemetry = {}
        self.last_event = '等待 Wi-Fi 数据；请连接 ESP32 热点'

    def _send(self, command, hello=False):
        # hello 没有会话令牌。其余命令携带令牌、递增序号和近期板端时间。
        self.sequence += 1
        packet = (f'BIKE1 {self.client:016x} {0 if hello else self.token:016x} '
                  f'{self.sequence} {0 if hello else self.board_us} {command}').encode('ascii')
        if len(packet) >= COMMAND_PACKET_LIMIT:
            raise ValueError('Command is too long')
        self.socket.send(packet)

    def has_fresh_link(self):
        return bool(self.token) and time.monotonic() - self.last_received <= COMMAND_LINK_MAX_AGE_S

    def send_command(self, command):
        command = command.strip()
        if not command or any(ord(c) < 32 or ord(c) > 126 for c in command):
            raise ValueError('命令只能包含可打印的 ASCII 字符')
        if not self.has_fresh_link():
            raise OSError('Wi-Fi 未连接或状态已过期；命令未发送')
        self._send(command)
        self.last_event = f'已发送：{command}（等待回执）'

    def _receive(self, packet):
        try:
            data = json.loads(packet)
            if not isinstance(data, dict):
                return None
            stamp = data.get('time_us')
            if type(stamp) is not int or stamp <= 0:
                return None
            token_text = data.get('token')
            if not isinstance(token_text, str) or len(token_text) != 16:
                return None
            token = int(token_text, 16)
            kind = data.get('type')
            if kind == 'welcome':
                seq = data.get('sequence')
                if (data.get('client') != f'{self.client:016x}' or type(seq) is not int or
                        not self.welcome_sequence < seq <= self.sequence or token == 0):
                    return None
                self.welcome_sequence = seq
                if token != self.token:
                    # ESP32 重启或重连时会换令牌，旧会话的状态和时间戳随之清空。
                    self.latest_telemetry_us = self.board_us = 0
                    self.telemetry = {}
                    self.last_event = 'Wi-Fi 已连接；断线不停止本地采集或控制'
                self.token = token
            elif not self.token or token != self.token:
                return None
            if kind not in ('welcome', 'reply', 'event', 'sample'):
                return None
            self.board_us = max(self.board_us, stamp)
            self.last_received = time.monotonic()
            if kind in ('reply', 'event'):
                # reply 是接收方回执。event 由执行任务提供，可能晚于回执到达。
                self.last_event = str(data.get('message', ''))[:300].strip()
            if kind == 'sample':
                if stamp <= self.latest_telemetry_us:
                    return None
                for name in ('tilt_deg', 'speed_turns_s', 'torque_nm', 'wheel_target'):
                    value = data.get(name, 0)
                    if type(value) not in (int, float) or not math.isfinite(value):
                        return None
                if type(data.get('mode', -1)) is not int:
                    return None
                self.latest_telemetry_us = stamp
                self.telemetry = data
                if valid_sample(data):
                    return data
                self.last_event = 'IMU 数据无效或采样已过期'
        except (ValueError, TypeError, OverflowError, UnicodeDecodeError):
            return None
        return None

    def poll(self):
        """维持会话并读取一批当前可用的包，返回最新可绘制姿态和有效帧数。"""
        now = time.monotonic()
        if now >= self.next_hello:
            try:
                self._send('hello', hello=True)
            except OSError as error:
                self.last_event = f'Wi-Fi：{error}'
            self.next_hello = now + HELLO_PERIOD_S
        latest, count = None, 0
        # 每次读取设上限，避免积压数据占满 Tk 的事件循环。
        for _ in range(MAX_PACKETS_PER_POLL):
            try:
                packet = self.socket.recv(2048)
            except BlockingIOError:
                break
            except OSError as error:
                self.last_event = f'Wi-Fi：{error}'
                break
            sample = self._receive(packet)
            if sample is not None:
                latest = sample
                count += 1
        return latest, count

    def close(self):
        # 关闭本地套接字时，ESP32 继续按已收到的目标运行。
        self.socket.close()
