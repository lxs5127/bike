"""Nonblocking Wi-Fi client shared by the 3D viewer and text console."""
import json
import math
import secrets
import socket
import time

DEFAULT_ADDRESS = '192.168.4.1'
DEFAULT_PORT = 3334


def valid_sample(data):
    if data.get('imu_valid') is not True:
        return False
    for name in ('acc_g', 'gyro_deg_s', 'angles_deg'):
        values = data.get(name)
        if not isinstance(values, list) or len(values) != 3:
            return False
        if not all(type(v) in (float, int) and math.isfinite(v) for v in values):
            return False
    stamp = data.get('sample_us')
    now = data.get('time_us')
    return (type(stamp) is int and type(now) is int and
            0 < stamp <= now and now-stamp <= 30000)


class WifiClient:
    def __init__(self, address=DEFAULT_ADDRESS, port=DEFAULT_PORT):
        socket.inet_pton(socket.AF_INET, address)
        if not 1 <= port <= 65535:
            raise ValueError('UDP port must be 1..65535')
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.socket.connect((address, port))
        self.address, self.port = address, port
        self.client = secrets.randbits(64) or 1
        self.token = 0
        self.sequence = 0
        self.welcome_sequence = 0
        self.board_us = 0
        self.sample_us = 0
        self.last_received = 0.
        self.next_hello = 0.
        self.telemetry = {}
        self.last_event = '等待 Wi-Fi 数据；请连接 ESP32 热点'

    def _send(self, command, hello=False):
        self.sequence += 1
        packet = (f'BIKE1 {self.client:016x} {0 if hello else self.token:016x} '
                  f'{self.sequence} {0 if hello else self.board_us} {command}').encode('ascii')
        if len(packet) >= 192:
            raise ValueError('Command is too long')
        self.socket.send(packet)

    def send_command(self, command):
        command = command.strip()
        if not command or any(ord(c) < 32 or ord(c) > 126 for c in command):
            raise ValueError('命令只能包含可打印的 ASCII 字符')
        if not self.token or time.monotonic()-self.last_received > .3:
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
                    self.sample_us = self.board_us = 0
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
                self.last_event = str(data.get('message', ''))[:300].strip()
            if kind == 'sample':
                if stamp <= self.sample_us:
                    return None
                for name in ('tilt_deg', 'speed_turns_s', 'torque_nm', 'wheel_target'):
                    value = data.get(name, 0)
                    if type(value) not in (int, float) or not math.isfinite(value):
                        return None
                if type(data.get('mode', -1)) is not int:
                    return None
                self.sample_us = stamp
                self.telemetry = data
                if valid_sample(data):
                    return data
                self.last_event = 'IMU 数据无效或采样已过期'
        except (ValueError, TypeError, OverflowError, UnicodeDecodeError):
            return None
        return None

    def poll(self):
        now = time.monotonic()
        if now >= self.next_hello:
            try:
                self._send('hello', hello=True)
            except OSError as error:
                self.last_event = f'Wi-Fi：{error}'
            self.next_hello = now+.1
        latest, count = None, 0
        for _ in range(64):
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
        # Closing a display/data client must never stop or reset the bicycle.
        self.socket.close()
