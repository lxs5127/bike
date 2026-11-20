#!/usr/bin/env python3
"""JY901P live desktop attitude viewer. Uses Python's built-in Tkinter.

Default: receive over Wi-Fi from 192.168.4.1:3334 (no serial port).
Wi-Fi: python3 view_3d.py --wifi 192.168.4.1
Direct serial (after closing the monitor): python3 view_3d.py --port auto
To follow a serial-monitor log instead: python3 view_3d.py --log /path/to/live.log
Drag to orbit the camera; scroll to zoom. Reference changes the display only.
"""
import argparse
from collections import deque
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import re
import tempfile
import time
import tkinter as tk

BG = '#0b1220'
PANEL = '#131f31'
TEXT = '#e6edf7'
MUTED = '#8da3bf'
AXES = ('#ff727b', '#58dfad', '#6ca9ff')
DEFAULT_BAUD = 460800
REFRESH_HZ = 100
DEFAULT_REFERENCE_FILE = Path(__file__).resolve().with_name('reference_pose.json')
FLOAT = r'([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)'
PATTERN = re.compile(
    r'acc\[g\]\s+X=' + FLOAT + r'\s+Y=' + FLOAT + r'\s+Z=' + FLOAT
    + r'\s*\|\s*gyro\[deg/s\]\s+X=' + FLOAT + r'\s+Y=' + FLOAT + r'\s+Z=' + FLOAT
    + r'\s*\|\s*angle\[deg\]\s+Roll=' + FLOAT + r'\s+Pitch=' + FLOAT + r'\s+Yaw=' + FLOAT)
IDENTITY = ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.))


@dataclass
class Sample:
    acceleration: tuple
    gyro: tuple
    angles: tuple


def parse_sample(line):
    match = PATTERN.search(re.sub(r'\x1b\[[0-9;]*m', '', line))
    if not match:
        return None
    values = tuple(map(float, match.groups()))
    if not all(math.isfinite(value) for value in values):
        return None
    return Sample(values[:3], values[3:6], values[6:])


def multiply(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3))
                       for j in range(3)) for i in range(3))


def transpose(matrix):
    return tuple(zip(*matrix))


def transform(matrix, point):
    return tuple(sum(row[i] * point[i] for i in range(3)) for row in matrix)


def rotation(angles):
    """Body to world: Rz(yaw) * Ry(pitch) * Rx(roll), right-handed, Z up."""
    roll, pitch, yaw = map(math.radians, angles)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


def matrix_angles(matrix):
    """Return Roll/Pitch/Yaw matching rotation(), including the ±90° singularity."""
    cos_pitch = math.hypot(matrix[0][0], matrix[1][0])
    pitch = math.atan2(-matrix[2][0], cos_pitch)
    if cos_pitch > 1e-8:
        roll = math.atan2(matrix[2][1], matrix[2][2])
        yaw = math.atan2(matrix[1][0], matrix[0][0])
    else:
        # At gimbal lock roll/yaw are coupled. Choose yaw=0 while preserving R.
        roll = math.atan2(-matrix[1][2], matrix[1][1])
        yaw = 0.
    values = tuple(map(math.degrees, (roll, pitch, yaw)))
    return tuple(0. if abs(value) < 1e-9 else value for value in values)


def reference_angles(values):
    """Validate a saved sensor pose before using it as a display reference."""
    if (not isinstance(values, (tuple, list)) or len(values) != 3 or
            any(type(value) not in (int, float) or not -180. <= value <= 180. or
                not math.isfinite(value) for value in values)):
        raise ValueError('零点必须包含三个有效角度，范围为 -180° 到 180°')
    return tuple(map(float, values))


def load_reference(path):
    try:
        with Path(path).open(encoding='utf-8') as handle:
            data = json.load(handle)
    except FileNotFoundError:
        return None
    if not isinstance(data, dict) or type(data.get('version')) is not int or data['version'] != 1:
        raise ValueError('零点文件格式不受支持')
    return reference_angles(data.get('reference_angles_deg'))


def save_reference(path, angles):
    """Replace atomically: an interrupted write must not destroy the old zero."""
    angles = reference_angles(angles)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                         prefix='.'+path.name+'.', suffix='.tmp', delete=False) as handle:
            temporary = Path(handle.name)
            json.dump({'version': 1, 'reference_angles_deg': angles}, handle, indent=2, allow_nan=False)
            handle.write('\n')
            handle.flush()
            os.fsync(handle.fileno())
        temporary.replace(path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def select_serial_port(requested, ports):
    """Handle USB re-enumeration without guessing between multiple devices."""
    ports = set(ports)
    if requested and requested != 'auto':
        if requested in ports:
            return requested
        # A stable /dev/serial/by-id path or other explicit port is used as-is.
        if not re.fullmatch(r'/dev/tty(?:USB|ACM)\d+', requested):
            return requested
    usb_ports = sorted(port for port in ports if re.fullmatch(r'/dev/tty(?:USB|ACM)\d+', port))
    if len(usb_ports) == 1:
        return usb_ports[0]
    if not usb_ports:
        raise FileNotFoundError('未检测到 USB 串口，请连接 ESP32 后重新启动窗口。')
    raise OSError('检测到多个 USB 串口，请用 --port 指定设备：'+', '.join(usb_ports))


class Stream:
    """Consume complete lines without blocking the GUI or replaying a backlog."""
    def __init__(self, log_path=None, port=None, baud=DEFAULT_BAUD):
        self.pending = ''
        self.handle = None
        self.log_path = Path(log_path) if log_path else None
        self.port = port
        if port:
            import serial
            from serial.tools import list_ports
            self.port = select_serial_port(port, [item.device for item in list_ports.comports()])
            self.handle = serial.Serial(port=None, baudrate=baud, timeout=0, exclusive=True)
            self.handle.dtr = False
            self.handle.rts = False
            self.handle.port = self.port
            self.handle.open()
            print(f'Serial connected: {self.port} @ {baud} baud', flush=True)
        else:
            self.handle = self.log_path.open('rb')
            self.handle.seek(0, 2)

    def poll(self):
        if self.port:
            data = self.handle.read(min(self.handle.in_waiting, 65536))
        else:
            size = self.log_path.stat().st_size
            position = self.handle.tell()
            if size < position:
                self.handle.seek(0)
                self.pending = ''
            elif size - position > 65536:
                self.handle.seek(size - 65536)
                self.handle.readline()
                self.pending = ''
            data = self.handle.read(65536)
        self.pending += data.decode('utf-8', errors='replace')
        lines = self.pending.split('\n')
        self.pending = lines.pop()[-8192:]
        latest, count = None, 0
        for line in lines:
            sample = parse_sample(line)
            if sample:
                latest = sample
                count += 1
        return latest, count

    def close(self):
        if self.handle:
            self.handle.close()


class WifiStream:
    def __init__(self, address='192.168.4.1', port=3334):
        from wifi_client import WifiClient
        self.client = WifiClient(address, port)

    def poll(self):
        data, count = self.client.poll()
        if data is None:
            return None, 0
        return Sample(tuple(data['acc_g']), tuple(data['gyro_deg_s']), tuple(data['angles_deg'])), count

    def close(self):
        self.client.close()


class Viewer:
    def __init__(self, root, stream, source_name, reference_file=DEFAULT_REFERENCE_FILE):
        self.root, self.stream = root, stream
        self.reference_file = Path(reference_file)
        self.saved_reference_angles = None
        self.sample = None
        self.reference = IDENTITY
        self.relative = False
        self.received_at = 0
        self.received_count = 0
        self.error = ''
        self.history = deque(maxlen=180)
        self.arrivals = deque(maxlen=101)
        self.frame_times = deque(maxlen=101)
        self.next_update = time.monotonic()
        self.camera_yaw, self.camera_elevation, self.zoom = -55., 28., 1.
        self.drag_position = None
        root.title('JY901P · 实时 3D 姿态')
        root.geometry('1200x800')
        root.minsize(950, 680)
        root.configure(bg=BG)
        root.protocol('WM_DELETE_WINDOW', self.close)

        header = tk.Frame(root, bg=BG, padx=26, pady=18)
        header.pack(fill='x')
        tk.Label(header, text='JY901P', bg=BG, fg=TEXT,
                 font=('Noto Sans CJK SC', 23, 'bold')).pack(side='left')
        tk.Label(header, text='  /  实时 3D 姿态', bg=BG, fg=MUTED,
                 font=('Noto Sans CJK SC', 13)).pack(side='left', padx=14)
        self.status = tk.Label(header, text='等待数据', bg=BG, fg='#f8cb72',
                               font=('Noto Sans CJK SC', 11))
        self.status.pack(side='right')

        content = tk.Frame(root, bg=BG)
        content.pack(fill='both', expand=True, padx=22)
        sidebar = tk.Frame(content, bg=PANEL, width=285, padx=18, pady=14)
        sidebar.pack(side='right', fill='y', padx=(16, 0))
        sidebar.pack_propagate(False)
        self.canvas = tk.Canvas(content, bg=PANEL, highlightthickness=0)
        self.canvas.pack(side='left', fill='both', expand=True)
        self.canvas.bind('<ButtonPress-1>', self.drag_start)
        self.canvas.bind('<B1-Motion>', self.drag)
        self.canvas.bind('<Button-4>', lambda event: self.set_zoom(1.08))
        self.canvas.bind('<Button-5>', lambda event: self.set_zoom(1 / 1.08))
        self.canvas.bind('<MouseWheel>', lambda event: self.set_zoom(1.08 if event.delta > 0 else 1 / 1.08))
        self.canvas.bind('<Configure>', lambda event: self.draw())

        self.attitude_title = tk.Label(sidebar, text='传感器原始姿态', bg=PANEL, fg=MUTED,
                                       font=('Noto Sans CJK SC', 11))
        self.attitude_title.pack(anchor='w', pady=(0, 12))
        self.angle_labels = []
        for label, subtitle, color in zip(('ROLL', 'PITCH', 'YAW'),
                                          ('横滚 · 绕 X 轴', '俯仰 · 绕 Y 轴', '航向 · 绕 Z 轴'), AXES):
            tk.Label(sidebar, text=f'{label}   {subtitle}', bg=PANEL, fg=color,
                     font=('Noto Sans CJK SC', 10)).pack(anchor='w')
            value = tk.Label(sidebar, text='— °', bg=PANEL, fg=TEXT,
                             font=('DejaVu Sans Mono', 25, 'bold'))
            value.pack(anchor='w', pady=(2, 12))
            self.angle_labels.append(value)
        tk.Label(sidebar, text='角速度  °/s', bg=PANEL, fg=MUTED,
                 font=('Noto Sans CJK SC', 10)).pack(anchor='w', pady=(8, 5))
        self.gyro_label = tk.Label(sidebar, text='X —\nY —\nZ —', bg=PANEL, fg=TEXT,
                                   justify='left', font=('DejaVu Sans Mono', 12))
        self.gyro_label.pack(anchor='w')
        tk.Label(sidebar, text='加速度  g', bg=PANEL, fg=MUTED,
                 font=('Noto Sans CJK SC', 10)).pack(anchor='w', pady=(18, 5))
        self.acc_label = tk.Label(sidebar, text='X —\nY —\nZ —', bg=PANEL, fg=TEXT,
                                  justify='left', font=('DejaVu Sans Mono', 12))
        self.acc_label.pack(anchor='w')
        self.gravity = tk.Label(sidebar, text='|a|  — g', bg=PANEL, fg=MUTED,
                                 font=('DejaVu Sans Mono', 10))
        self.gravity.pack(anchor='w', pady=9)

        controls = tk.Frame(root, bg=BG, padx=22, pady=14)
        controls.pack(fill='x')
        for text, callback in [('设为零点并保存', self.set_reference),
                               ('使用已存零点', self.restore_reference),
                               ('显示绝对姿态', self.clear_reference), ('重置视角', self.reset_camera)]:
            tk.Button(controls, text=text, command=callback, bg='#21334d', fg=TEXT,
                      activebackground='#324d70', activeforeground=TEXT,
                      relief='flat', padx=15, pady=9, cursor='hand2',
                      font=('Noto Sans CJK SC', 10)).pack(side='left', padx=(0, 9))
        self.mode = tk.Label(controls, text='绝对姿态', bg=BG, fg=MUTED,
                             font=('Noto Sans CJK SC', 10))
        self.mode.pack(side='right')
        if isinstance(stream, WifiStream):
            commands = tk.Frame(root, bg=BG, padx=22)
            commands.pack(fill='x', pady=(0, 8))
            tk.Label(commands, text='Wi-Fi 指令', bg=BG, fg=MUTED).pack(side='left', padx=(0, 8))
            self.command_entry = tk.Entry(commands, bg=PANEL, fg=TEXT, insertbackground=TEXT, width=32)
            self.command_entry.pack(side='left', padx=(0, 8))
            self.command_entry.bind('<Return>', lambda event: self.send_wifi_command())
            tk.Button(commands, text='发送', command=self.send_wifi_command).pack(side='left', padx=(0, 8))
            tk.Button(commands, text='手动停机', command=lambda: self.send_wifi_command('stop'),
                      bg='#a83540', fg='white').pack(side='left')
            self.wifi_details = tk.Label(root, text='等待 Wi-Fi 数据', bg=BG, fg=MUTED, anchor='w')
            self.wifi_details.pack(fill='x', padx=24, pady=(0, 6))
            self.wifi_event = tk.Label(root, text='', bg=BG, fg=MUTED, anchor='w', wraplength=1100)
            self.wifi_event.pack(fill='x', padx=24, pady=(0, 6))
        tk.Label(root, text=f'{source_name}   ·   左键拖动视角 / 滚轮缩放   ·   参考模式下模型、角度和轨迹同步归零',
                 bg=BG, fg=MUTED, font=('Noto Sans CJK SC', 9)).pack(anchor='w', padx=24, pady=(0, 14))
        self.restore_reference()
        root.after(10, self.update)

    def close(self):
        self.stream.close()
        self.root.destroy()

    def set_reference(self):
        if self.sample and time.monotonic() - self.received_at < 1.:
            try:
                save_reference(self.reference_file, self.sample.angles)
            except (OSError, ValueError) as error:
                self.mode.config(text='零点保存失败，请检查保存路径', fg=AXES[0])
                print(f'Cannot save reference {self.reference_file}: {error}', flush=True)
                return
            self.saved_reference_angles = tuple(self.sample.angles)
            self.apply_reference(self.saved_reference_angles, '相对姿态 · 零点已保存')
        else:
            self.mode.config(text='等待实时数据后再设置零点', fg='#f8cb72')

    def restore_reference(self):
        try:
            angles = load_reference(self.reference_file)
        except (OSError, ValueError) as error:
            self.mode.config(text='零点读取失败，请重新设置', fg=AXES[0])
            print(f'Cannot load reference {self.reference_file}: {error}', flush=True)
            return
        if angles is None:
            self.mode.config(text='尚未保存零点', fg=MUTED)
            return
        self.saved_reference_angles = angles
        self.apply_reference(angles, '相对姿态 · 已加载保存零点')

    def apply_reference(self, angles, message):
        # A relative orientation is a matrix composition, not Euler subtraction.
        self.reference = transpose(rotation(angles))
        self.relative = True
        self.mode.config(text=message, fg=MUTED)
        self.refresh_angles()
        self.reset_history()
        self.draw()

    def clear_reference(self):
        self.reference, self.relative = IDENTITY, False
        message = '绝对姿态 · 零点已保留' if self.saved_reference_angles is not None else '绝对姿态'
        self.mode.config(text=message, fg=MUTED)
        self.refresh_angles()
        self.reset_history()
        self.draw()

    def display_rotation(self):
        return multiply(self.reference, rotation(self.sample.angles)) if self.sample else IDENTITY

    def display_angles(self):
        if not self.sample:
            return None
        return matrix_angles(self.display_rotation()) if self.relative else self.sample.angles

    def refresh_angles(self):
        self.attitude_title.config(text='相对参考姿态' if self.relative else '传感器原始姿态')
        angles = self.display_angles()
        if angles is not None:
            for label, value in zip(self.angle_labels, angles):
                label.config(text=f'{value:+8.2f}°')

    def reset_history(self):
        # A reference change starts a new trace; do not mix two coordinate frames.
        self.history.clear()
        angles = self.display_angles()
        if angles is not None:
            self.history.append(angles)

    def reset_camera(self):
        self.camera_yaw, self.camera_elevation, self.zoom = -55., 28., 1.
        self.draw()

    def send_wifi_command(self, command=None):
        command = command if command is not None else self.command_entry.get()
        try:
            self.stream.client.send_command(command)
            self.command_entry.delete(0, 'end')
        except (OSError, ValueError) as error:
            self.stream.client.last_event = str(error)

    def set_zoom(self, scale):
        self.zoom = max(.55, min(1.6, self.zoom * scale))
        self.draw()

    def drag_start(self, event):
        self.drag_position = (event.x, event.y)

    def drag(self, event):
        if self.drag_position:
            x, y = self.drag_position
            self.camera_yaw -= (event.x - x) * .4
            self.camera_elevation = max(8., min(80., self.camera_elevation + (event.y - y) * .3))
            self.drag_position = event.x, event.y
            self.draw()

    def update(self):
        now = time.monotonic()
        self.frame_times.append(now)
        try:
            sample, count = self.stream.poll()
            if isinstance(self.stream, WifiStream):
                data = self.stream.client.telemetry
                if data:
                    mode = data.get('mode', -1)
                    if mode == -1:
                        details = '独立陀螺仪采集 · Wi-Fi 100 Hz · 断线继续采集'
                    else:
                        details = (f"MCU 模式 {mode}  |  倾角 {data.get('tilt_deg', 0):+.3f}°  |  "
                                   f"速度 {data.get('speed_turns_s', 0):+.3f} 圈/s  |  "
                                   f"目标力矩 {data.get('torque_nm', 0):+.4f} Nm  |  "
                                   f"MCU 零点：{'已保存' if data.get('zero_valid') else '未设置'}")
                    self.wifi_details.config(text=details)
                self.wifi_event.config(text=self.stream.client.last_event)
            if sample:
                self.sample = sample
                self.received_at = now
                self.received_count += count
                self.arrivals.append((now, self.received_count))
                self.history.append(self.display_angles())
                self.error = ''
                self.refresh_angles()
                for label, values in [(self.gyro_label, sample.gyro), (self.acc_label, sample.acceleration)]:
                    label.config(text='\n'.join(f'{axis}  {value:+8.3f}' for axis, value in zip('XYZ', values)))
                self.gravity.config(text=f'|a|  {math.sqrt(sum(v*v for v in sample.acceleration)):.3f} g')
        except (OSError, ValueError) as error:
            self.error = str(error)

        age = now - self.received_at if self.sample else float('inf')
        hz = 0.
        if len(self.arrivals) > 1:
            first, last = self.arrivals[0], self.arrivals[-1]
            hz = (last[1] - first[1]) / max(.001, last[0] - first[0])
        fps = ((len(self.frame_times)-1) / (self.frame_times[-1]-self.frame_times[0])
               if len(self.frame_times) > 1 else 0.)
        if age < 1. and not self.error:
            self.status.config(text=f'● 数据 {hz:.1f} Hz   ·   绘制 {fps:.0f} fps   /   {age*1000:.0f} ms', fg=AXES[1])
        elif self.sample:
            self.status.config(text=f'● 数据已停止   {age:.1f} s', fg=AXES[0])
        else:
            self.status.config(text='● 等待数据', fg='#f8cb72')
        self.draw()
        # Use a deadline so render/parse time does not add to the 10 ms period.
        finished = time.monotonic()
        self.next_update = max(self.next_update + 1. / REFRESH_HZ, finished)
        self.root.after(max(1, math.ceil((self.next_update-finished)*1000)), self.update)

    def draw(self):
        c = self.canvas
        c.delete('all')
        w, h = c.winfo_width(), c.winfo_height()
        if w < 10 or h < 10:
            return
        yaw, elev = math.radians(self.camera_yaw), math.radians(self.camera_elevation)
        eye = (math.cos(elev)*math.cos(yaw), math.cos(elev)*math.sin(yaw), math.sin(elev))
        right = (-math.sin(yaw), math.cos(yaw), 0.)
        up = (-math.sin(elev)*math.cos(yaw), -math.sin(elev)*math.sin(yaw), math.cos(elev))
        scale = min(w / 7.2, h / 6.0) * self.zoom
        center = (w * .5, h * .47)

        def dot(a, b):
            return sum(x*y for x, y in zip(a, b))

        def project(point):
            depth = dot(point, eye)
            perspective = 9. / (9. - depth)
            return center[0] + scale * dot(point, right) * perspective, center[1] - scale * dot(point, up) * perspective

        def line(a, b, color, width=1, arrow=False):
            c.create_line(*project(a), *project(b), fill=color, width=width,
                          arrow='last' if arrow else 'none', arrowshape=(10, 12, 4))

        for i in range(-5, 6):
            value = i * .6
            color = '#2b3c53' if i == 0 else '#1c2b40'
            line((value, -3., -1.15), (value, 3., -1.15), color)
            line((-3., value, -1.15), (3., value, -1.15), color)
        for endpoint, label in [((2.8, 0., -1.15), '世界 X'), ((0., 2.8, -1.15), '世界 Y')]:
            x, y = project(endpoint)
            c.create_text(x, y, text=label, fill=MUTED, font=('Noto Sans CJK SC', 9))

        matrix = self.display_rotation()
        faces = []

        def box(x0, x1, y0, y1, z0, z1, colors):
            vertices = [(x0,y0,z0),(x1,y0,z0),(x1,y1,z0),(x0,y1,z0),
                        (x0,y0,z1),(x1,y0,z1),(x1,y1,z1),(x0,y1,z1)]
            points = [transform(matrix, point) for point in vertices]
            for indices, color in zip([(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)], colors):
                face = [points[index] for index in indices]
                a, b, d = face[0], face[1], face[2]
                u, v = tuple(b[i]-a[i] for i in range(3)), tuple(d[i]-a[i] for i in range(3))
                normal = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
                centroid = tuple(sum(point[i] for point in face)/4 for i in range(3))
                if dot(normal, tuple(eye[i]*9-centroid[i] for i in range(3))) > 0:
                    faces.append((sum(dot(p, eye) for p in face)/4, face, color))

        # A simple PCB, a raised IC, connector and front marker establish orientation.
        box(-1.45,1.45,-.85,.85,-.09,.09,
            ('#21584e','#2caa8a','#236c60','#3c8673','#205c53','#347c68'))
        box(-.35,.5,-.38,.38,.09,.27,
            ('#142132','#304156','#1c2b40','#263950','#1b293c','#24374c'))
        box(-1.48,-1.04,-.39,.39,.09,.26,
            ('#65788f','#c5d2de','#7b8da2','#9eafc0','#8296ad','#a6b5c4'))
        box(1.08,1.4,-.55,.55,.09,.12,
            ('#b4525c','#ff727b','#c85d68','#dd6a73','#bc5660','#e27982'))
        for _, face, color in sorted(faces, key=lambda item: item[0]):
            xy = [value for p in face for value in project(p)]
            c.create_polygon(*xy, fill=color, outline='#91d5c2', width=1)

        for point, label, color in zip(((2.25,0.,0.),(0.,1.85,0.),(0.,0.,1.65)),
                                       ('X / 前','Y / 左','Z / 上'), AXES):
            endpoint = transform(matrix, point)
            line((0.,0.,0.), endpoint, color, 3, True)
            x, y = project(endpoint)
            c.create_text(x, y-15, text=label, fill=color, font=('Noto Sans CJK SC', 11, 'bold'))

        c.create_text(20, 20, anchor='nw', text='相对参考姿态' if self.relative else '绝对姿态 · Z 轴向上',
                      fill=TEXT, font=('Noto Sans CJK SC', 12, 'bold'))
        c.create_text(20, 49, anchor='nw', text='红色端为模块 +X 方向；模型按 Roll → Pitch → Yaw 旋转',
                      fill=MUTED, font=('Noto Sans CJK SC', 9))
        if self.error:
            c.create_text(20, 76, anchor='nw', text=self.error[:100], fill=AXES[0], font=('Noto Sans CJK SC', 9))
        if not self.sample or time.monotonic() - self.received_at >= 1.:
            c.create_text(w/2, h*.76, text='等待实时数据…' if not self.sample else '数据已停止 · 保留最后姿态',
                          fill='#f8cb72', font=('Noto Sans CJK SC', 13, 'bold'))

        # Recent attitude history; unwrap each trace
        # across ±180° so a wrap does not draw a spurious full-height spike.
        y0, y1, x0, x1 = h-92, h-28, 20, w-20
        c.create_rectangle(x0, y0, x1, y1, fill=BG, outline='#28394f')
        c.create_text(x0, y0-13, text='姿态轨迹 · 各轴独立缩放', anchor='w', fill=MUTED, font=('Noto Sans CJK SC', 9))
        if len(self.history) > 1:
            for axis, color in enumerate(AXES):
                values = [sample[axis] for sample in self.history]
                unwrapped = [values[0]]
                for previous, value in zip(values, values[1:]):
                    unwrapped.append(unwrapped[-1] + (value-previous+180)%360-180)
                mid = (max(unwrapped)+min(unwrapped))/2
                span = max(30., max(unwrapped)-min(unwrapped))
                points = []
                for i, value in enumerate(unwrapped):
                    points.extend((x0+(x1-x0)*i/(len(unwrapped)-1), (y0+y1)/2-(value-mid)/span*(y1-y0-12)))
                c.create_line(*points, fill=color, width=1.7)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group()
    source.add_argument('--log', default=None, help='Follow a live serial-monitor log')
    source.add_argument('--port', help='Legacy serial port or auto; close other monitors first')
    source.add_argument('--wifi', nargs='?', const='192.168.4.1', help='ESP32 Wi-Fi IP (default: 192.168.4.1)')
    parser.add_argument('--udp-port', type=int, default=3334, help='Wi-Fi UDP port (default: 3334)')
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD, help='Serial baud rate (default: 460800)')
    parser.add_argument('--reference-file', type=Path, default=DEFAULT_REFERENCE_FILE,
                        help='Saved reference pose JSON (default: reference_pose.json next to this script)')
    parser.add_argument('--close-after', type=float, default=0, help=argparse.SUPPRESS)
    args = parser.parse_args()
    root = tk.Tk()
    try:
        if args.log or args.port:
            stream = Stream(log_path=args.log, port=args.port, baud=args.baud)
            source_name = f'{stream.port} · {args.baud} baud' if stream.port else args.log
        else:
            address = args.wifi or '192.168.4.1'
            stream = WifiStream(address, args.udp_port)
            source_name = f'Wi-Fi UDP · {address}:{args.udp_port}'
    except (OSError, ImportError) as error:
        from tkinter import messagebox
        root.withdraw()
        messagebox.showerror('JY901P 数据源未打开', str(error), parent=root)
        root.destroy()
        return 1
    viewer = Viewer(root, stream, source_name, reference_file=args.reference_file)
    if args.close_after:
        root.after(int(args.close_after*1000), viewer.close)
    root.mainloop()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
