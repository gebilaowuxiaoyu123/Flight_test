#!/usr/bin/env python3
"""180 秒飞行演示：手柄 → Web 协议 → C++ 后端 → PX4，同时录屏合成成片。

链路说明（和 docs/11-cpp-rewrite.md 一致）：
  本脚本 --WebSocket(50Hz)--> rc_server(C++) --/fmu/in/manual_control_input--> PX4
  位置/姿态反馈走 ROS 订阅（控制回路需要高频反馈，不走 WebSocket）。

飞行模式用 ALTCTL：无 GPS 时 POSCTL 的水平速度控制会被飞控抑制，
而 ALTCTL 的横滚/俯仰杆直接对应姿态角，正好满足「手柄与飞行姿态相对应」。

用法：
  python3 scripts/demo_show.py --record /tmp/flight_show.mp4   # 录制成片
  python3 scripts/demo_show.py --dry-run                       # 只飞不录（调试）
"""
from __future__ import annotations

import argparse
import base64
import json
import math
import os
import socket
import struct
import subprocess
import sys
import threading
import time

import rclpy
from PIL import Image, ImageDraw, ImageFont
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, VehicleOpticalFlow
from sensor_msgs.msg import Image as RosImage
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# ---------------------------------------------------------------- 画面参数
W, H = 1920, 1080
PANEL_H = 360        # 下半屏：手柄 + 航迹 + 遥测
# 四路画中画在 1280 宽的主画面上排一行：320×180 × 4 = 1280
PIP_W, PIP_H = 320, 180
PIP_FPV = (0 * PIP_W, 720 - PIP_H)      # ① 第一视角
PIP_TOP = (1 * PIP_W, 720 - PIP_H)      # ② 俯视
PIP_DEPTH = (2 * PIP_W, 720 - PIP_H)    # ③ 深度相机（彩色图 + 最近距离）
PIP_FLOW = (3 * PIP_W, 720 - PIP_H)     # ④ 光流（下视原图 + 箭头）
FPS = 25

FONT_PATH = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
FONT_BOLD = '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'

BG = (16, 18, 24)
FG = (232, 236, 244)
DIM = (140, 150, 170)
ACCENT = (90, 200, 255)
GREEN = (80, 220, 140)
RED = (255, 92, 92)
AMBER = (255, 190, 70)


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    return ImageFont.truetype(FONT_BOLD if bold else FONT_PATH, size, index=0)


def clamp(v, lo=-1.0, hi=1.0):
    return max(lo, min(hi, v))


# ---------------------------------------------------------------- WebSocket
class WSClient:
    """只实现我们用到的最小集合：握手 / 文本帧收发。"""

    def __init__(self, host='127.0.0.1', port=8000, path='/ws'):
        self.sock = socket.create_connection((host, port), timeout=5)
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall((
            f"GET {path} HTTP/1.1\r\nHost: {host}:{port}\r\nUpgrade: websocket\r\n"
            f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n"
            f"Sec-WebSocket-Version: 13\r\n\r\n").encode())
        head = b''
        while b'\r\n\r\n' not in head:
            head += self.sock.recv(1)
        if b'101' not in head.split(b'\r\n')[0]:
            raise RuntimeError('WebSocket 握手失败')

        self.telemetry: dict = {}
        self.acks: list = []
        self._lock = threading.Lock()
        self._alive = True
        self._buf = b''
        threading.Thread(target=self._read_loop, daemon=True).start()

    def send(self, obj: dict):
        data = json.dumps(obj).encode()
        n = len(data)
        if n < 126:
            hdr = bytes([0x81, 0x80 | n])
        elif n < 65536:
            hdr = bytes([0x81, 0x80 | 126]) + struct.pack('>H', n)
        else:
            hdr = bytes([0x81, 0x80 | 127]) + struct.pack('>Q', n)
        mask = os.urandom(4)
        self.sock.sendall(hdr + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(data)))

    def sticks(self, roll, pitch, yaw, throttle, deadman=True):
        self.send({'type': 'sticks', 'roll': roll, 'pitch': pitch,
                   'yaw': yaw, 'throttle': throttle, 'deadman': deadman})

    def _read_loop(self):
        self.sock.settimeout(None)
        while self._alive:
            try:
                chunk = self.sock.recv(65536)
            except OSError:
                break
            if not chunk:
                break
            self._buf += chunk
            while len(self._buf) >= 2:
                ln, off = self._buf[1] & 0x7F, 2
                if ln == 126:
                    if len(self._buf) < 4:
                        break
                    ln = struct.unpack('>H', self._buf[2:4])[0]
                    off = 4
                elif ln == 127:
                    if len(self._buf) < 10:
                        break
                    ln = struct.unpack('>Q', self._buf[2:10])[0]
                    off = 10
                if len(self._buf) < off + ln:
                    break
                payload, self._buf = self._buf[off:off + ln], self._buf[off + ln:]
                try:
                    msg = json.loads(payload)
                except Exception:
                    continue
                with self._lock:
                    if msg.get('type') == 'telemetry':
                        self.telemetry = msg
                    elif msg.get('type') == 'ack':
                        self.acks.append(msg)

    def close(self):
        self._alive = False
        try:
            self.sock.close()
        except OSError:
            pass


# ---------------------------------------------------------------- 状态与飞控
class FlightState:
    """ROS 侧的高频反馈：位置、速度、姿态。"""

    def __init__(self):
        rclpy.init()
        self.node = Node('demo_show')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.x = self.y = self.z = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.heading = 0.0
        self.roll = self.pitch = 0.0
        self.fresh = 0.0
        # 光流：PX4 内部用下视相机算出来的像素位移（px/s）+ 质量 + 测距
        self.flow_x = self.flow_y = 0.0   # 光流角速度（°/s）
        self.flow_quality = 0
        self.flow_distance = -1.0
        self.node.create_subscription(VehicleLocalPosition,
                                      '/fmu/out/vehicle_local_position', self._on_pos, qos)
        self.node.create_subscription(VehicleAttitude,
                                      '/fmu/out/vehicle_attitude', self._on_att, qos)
        self.node.create_subscription(VehicleOpticalFlow,
                                      '/fmu/out/vehicle_optical_flow', self._on_flow, qos)
        threading.Thread(target=self._spin, daemon=True).start()

    def _spin(self):
        try:
            rclpy.spin(self.node)
        except Exception:
            pass

    def _on_pos(self, m):
        self.x, self.y, self.z = m.x, m.y, m.z
        self.vx, self.vy, self.vz = m.vx, m.vy, m.vz
        self.heading = m.heading
        self.fresh = time.time()

    def _on_att(self, m):
        w, x, y, z = m.q
        # 四元数 → 横滚/俯仰角，给姿态仪显示用
        self.roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        self.pitch = math.asin(clamp(2 * (w * y - z * x), -1.0, 1.0))

    def _on_flow(self, m):
        # pixel_flow 是「积分角度」(rad)，除以积分时长才得到角速度（°/s）
        dt = max(1e-4, float(m.integration_timespan_us) * 1e-6)
        self.flow_x = math.degrees(float(m.pixel_flow[0]) / dt)
        self.flow_y = math.degrees(float(m.pixel_flow[1]) / dt)
        self.flow_quality = int(m.quality)
        self.flow_distance = float(m.distance_m)

    @property
    def alt(self):
        return -self.z


CAM_TOPICS = {
    'third': '/world/flow/model/cam_third/link/link/sensor/third_cam/image',
    'fpv': '/world/flow/model/x500_flow_depth_0/link/camera_link/sensor/IMX214/image',
    'top': '/world/flow/model/cam_top/link/link/sensor/top_cam/image',
    # 光流用的下视相机：光流算法就是在它画面上算像素位移
    'down': '/world/flow/model/x500_flow_depth_0/link/camera_link/sensor/down_cam/image',
    # 深度相机（32FC1）：由 run_show.sh 里的 ros_gz_bridge 桥过来
    'depth': '/depth_camera',
}


def read_true_pose(max_age=0.4):
    """读 scripts/pose_writer.py 写的 Gazebo 真实位姿。

    无 GPS + 光流时 PX4 的位置估计会漂（实测 1~2 m），拿它做闭环会让
    目标点落到真实世界的错误位置 —— 轻则航线跑偏，重则撞墙。
    位置反馈改用真实位姿，姿态/模式仍走 PX4。
    """
    try:
        with open('/tmp/cam_target.json') as fh:
            d = json.load(fh)
        if time.time() - float(d.get('t', 0)) > max_age:
            return None
        return d
    except Exception:
        return None


class CameraFeed:
    """订阅三路机载/机外相机，缓存最近一帧 PIL 图。"""

    # 深度图的伪彩色量程（米）：近=红，远=蓝
    DEPTH_NEAR, DEPTH_FAR = 0.2, 6.0

    def __init__(self):
        self.latest = {k: None for k in CAM_TOPICS}
        self.depth_min = -1.0          # 画面中央最近距离（米），-1 表示没数据
        self.qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._node = None

    def attach(self, node, keys=None):
        """keys=None 表示全部订阅；否则只订阅用得到的那几路（省 CPU）。"""
        self._node = node
        for key, topic in CAM_TOPICS.items():
            if keys is not None and key not in keys:
                continue
            node.create_subscription(
                RosImage, topic, lambda m, k=key: self._on(m, k), self.qos)

    def _on(self, msg, key):
        if key == 'depth':
            self._on_depth(msg)
            return
        encoding = msg.encoding.lower()
        mode = {'rgb8': 'RGB', 'bgr8': 'RGB', 'rgba8': 'RGBA', 'bgra8': 'RGBA'}.get(encoding)
        if mode != 'RGB':
            return
        try:
            img = Image.frombytes('RGB', (msg.width, msg.height),
                                  bytes(msg.data), 'raw', 'RGB')
        except Exception:
            return
        self.latest[key] = img

    def _on_depth(self, msg):
        """32FC1 深度图 → 伪彩色（近红远蓝），顺手记中央区域的最近距离。"""
        import numpy as np

        try:
            buf = np.frombuffer(bytes(msg.data), dtype=np.float32)
        except Exception:
            return
        w, h = msg.width, msg.height
        if buf.size < w * h:
            return
        depth = buf[:w * h].reshape(h, w)

        # 中央 ROI：宽度 50%、高度 60%（和 min_depth.cpp 的口径一致）
        x0, x1 = int(w * 0.25), int(w * 0.75)
        y0, y1 = int(h * 0.2), int(h * 0.8)
        roi = depth[y0:y1, x0:x1]
        valid = roi[np.isfinite(roi) & (roi > 0.0)]
        self.depth_min = float(valid.min()) if valid.size else -1.0

        # 归一化：越近越"热"
        near, far = self.DEPTH_NEAR, self.DEPTH_FAR
        v = np.nan_to_num(depth, nan=far, posinf=far, neginf=far)
        v = np.clip(v, near, far)
        t = 1.0 - (v - near) / (far - near)              # 1=近, 0=远
        rgb = np.zeros((h, w, 3), dtype=np.uint8)
        rgb[..., 0] = (255 * t).astype(np.uint8)          # R
        rgb[..., 1] = (90 * t + 40 * (1 - t)).astype(np.uint8)
        rgb[..., 2] = (60 * t + 255 * (1 - t)).astype(np.uint8)
        self.latest['depth'] = Image.fromarray(rgb, 'RGB')

    def get(self, key, size=None):
        img = self.latest.get(key)
        if img is None:
            img = Image.new('RGB', (size or (640, 360)), (18, 20, 26))
        return img.resize(size, Image.BILINEAR) if size else img


class Controller:
    """ALTCTL 下的位置保持：位置误差 → 期望速度 → 姿态杆量。

    ALTCTL 里横滚/俯仰杆是「姿态角」，本身不保位置，
    所以要靠光流给的速度反馈做一层外环，否则松杆就会一直漂。
    """

    KP_XY = 0.7        # 位置误差 → 期望速度 (1/s)
    VMAX = 1.5         # 期望速度上限 (m/s)，小场地不需要快
    KV = 0.16          # 速度误差 → 杆量 (杆量/(m/s))
    KI = 0.045
    FF = 0.16          # 速度前馈：实测满杆约 5 m/s；留余量避免杆量打满
    KP_ALT = 0.55      # 高度误差 → 油门杆
    KYAW = 0.85

    def __init__(self):
        self.tx = self.ty = 0.0
        self.talt = 0.0
        self.t_heading = 0.0
        self.heading_follow = False
        self.xlim = (-1e9, 1e9)      # 安全边界：别飞出光流网格
        self.ylim = (-1e9, 1e9)
        self._if = self._ir = 0.0

    def reset_integrators(self):
        self._if = self._ir = 0.0

    def sticks(self, st: FlightState, dt: float):
        self.tx = max(self.xlim[0], min(self.xlim[1], self.tx))
        self.ty = max(self.ylim[0], min(self.ylim[1], self.ty))
        ex, ey = self.tx - st.x, self.ty - st.y
        vx_d = clamp(self.KP_XY * ex, -self.VMAX, self.VMAX)
        vy_d = clamp(self.KP_XY * ey, -self.VMAX, self.VMAX)
        if self.heading_follow and math.hypot(vx_d, vy_d) > 0.35:
            self.t_heading = math.atan2(vy_d, vx_d)

        ch, sh = math.cos(st.heading), math.sin(st.heading)
        vf_d, vr_d = vx_d * ch + vy_d * sh, -vx_d * sh + vy_d * ch
        vf_m, vr_m = st.vx * ch + st.vy * sh, -st.vx * sh + st.vy * ch

        ef, er = vf_d - vf_m, vr_d - vr_m
        self._if = clamp(self._if + ef * dt, -3.0, 3.0)
        self._ir = clamp(self._ir + er * dt, -3.0, 3.0)

        # 前馈 + 反馈：只有反馈时，杆量永远追不上"期望速度"，
        # 画面上就会出现"目标在动、手柄几乎不动"
        pitch = clamp(self.FF * vf_d + self.KV * ef + self.KI * self._if, -0.45, 0.45)
        roll = clamp(self.FF * vr_d + self.KV * er + self.KI * self._ir, -0.45, 0.45)

        dhead = math.atan2(math.sin(self.t_heading - st.heading),
                           math.cos(self.t_heading - st.heading))
        yaw = clamp(self.KYAW * dhead, -0.5, 0.5)

        alt_err = self.talt - st.alt
        throttle = clamp(self.KP_ALT * alt_err, -0.7, 0.8)
        return roll, pitch, yaw, throttle


# ---------------------------------------------------------------- 画面渲染
class Overlay:
    """下半屏面板：手柄 / 姿态仪 / 航迹 / 阶段与遥测。"""

    # 四列布局
    COL_A = (16, 600)      # 手柄
    COL_B = (616, 900)     # 姿态仪
    COL_C = (916, 1400)    # 航迹
    COL_D = (1416, 1904)   # 阶段 + 遥测

    def __init__(self):
        self.f_title = font(26, True)
        self.f_big = font(46, True)
        self.f_mid = font(22, True)
        self.f_txt = font(19)
        self.f_small = font(16)
        self.base = self._build_base()

    def _panel(self, d, col, fill=(24, 27, 36)):
        d.rounded_rectangle((col[0], 16, col[1], PANEL_H - 16), 16,
                            fill=fill, outline=(46, 52, 66), width=2)

    def _build_base(self) -> Image.Image:
        img = Image.new('RGB', (W, PANEL_H), BG)
        d = ImageDraw.Draw(img)
        for i in range(PANEL_H):
            k = i / PANEL_H
            d.line([(0, i), (W, i)],
                   fill=(int(16 + 8 * k), int(18 + 9 * k), int(24 + 12 * k)))
        for col in (self.COL_A, self.COL_B, self.COL_C, self.COL_D):
            self._panel(d, col)
        self._pad_body(d)
        d.text((self.COL_B[0] + 12, 26), '姿态', font=self.f_small, fill=DIM)
        d.text((self.COL_C[0] + 12, 26), '航迹（俯视 / 米）', font=self.f_small, fill=DIM)
        return img

    def _pad_body(self, d: ImageDraw.ImageDraw):
        cx, cy = 308, 140
        d.rounded_rectangle((cx - 232, cy - 62, cx - 76, cy + 66), 46,
                            fill=(38, 42, 54), outline=(72, 80, 98), width=3)
        d.rounded_rectangle((cx + 76, cy - 62, cx + 232, cy + 66), 46,
                            fill=(38, 42, 54), outline=(72, 80, 98), width=3)
        d.rounded_rectangle((cx - 104, cy - 46, cx + 104, cy + 52), 30,
                            fill=(46, 51, 66), outline=(80, 88, 108), width=3)
        d.text((cx, 40), 'RC 遥控手柄', font=self.f_title, fill=(170, 180, 198), anchor='ma')

    def _stick(self, d, cx, cy, vx, vy, label, hint, color):
        r = 44
        d.ellipse((cx - r, cy - r, cx + r, cy + r), fill=(26, 29, 38),
                  outline=(84, 92, 112), width=3)
        d.line([(cx - r + 8, cy), (cx + r - 8, cy)], fill=(60, 66, 82), width=1)
        d.line([(cx, cy - r + 8), (cx, cy + r - 8)], fill=(60, 66, 82), width=1)
        kx, ky = cx + vx * r * 0.70, cy + vy * r * 0.70
        d.ellipse((kx - 18, ky - 18, kx + 18, ky + 18), fill=color,
                  outline=(255, 255, 255), width=2)
        d.text((cx, cy + r + 8), label, font=self.f_mid, fill=FG, anchor='ma')
        d.text((cx, cy + r + 32), hint, font=self.f_small, fill=DIM, anchor='ma')

    def _horizon(self, img, d, roll, pitch):
        acx, acy, ar = 758, 132, 88
        clip = Image.new('RGB', (2 * ar, 2 * ar), (12, 15, 22))
        cd = ImageDraw.Draw(clip)
        cd.rectangle((0, 0, 2 * ar, ar), fill=(40, 88, 138))
        cd.rectangle((0, ar, 2 * ar, 2 * ar), fill=(98, 72, 46))
        cd.line([(0, ar), (2 * ar, ar)], fill=(245, 245, 245), width=2)
        for k in (-2, -1, 1, 2):
            y = ar + k * 26
            cd.line([(ar - 30, y), (ar + 30, y)], fill=(210, 214, 222), width=2)
        rot = clip.rotate(-math.degrees(roll), resample=Image.BILINEAR,
                          center=(ar, ar),
                          translate=(0, math.degrees(pitch) * 2.6))
        mask = Image.new('L', (2 * ar, 2 * ar), 0)
        ImageDraw.Draw(mask).ellipse((1, 1, 2 * ar - 1, 2 * ar - 1), fill=255)
        img.paste(rot, (acx - ar, acy - ar), mask)
        d.ellipse((acx - ar, acy - ar, acx + ar, acy + ar),
                  outline=(96, 106, 128), width=2)
        d.line([(acx - 30, acy), (acx - 8, acy)], fill=AMBER, width=3)
        d.line([(acx + 8, acy), (acx + 30, acy)], fill=AMBER, width=3)
        d.ellipse((acx - 3, acy - 3, acx + 3, acy + 3), fill=AMBER)
        d.text((758, 232), f'{math.degrees(pitch):+5.1f}°', font=self.f_mid,
               fill=FG, anchor='ma')
        d.text((758, 256), '俯仰', font=self.f_small, fill=DIM, anchor='ma')

    def _map(self, d, path, cur_x, cur_y, tgt_x, tgt_y, heading):
        x0, x1 = self.COL_C[0] + 14, self.COL_C[1] - 14
        y0, y1 = 44, PANEL_H - 30
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        scale = 52.0     # 像素 / 米（场地小，放大才看得清 8 字）

        for g in range(-4, 5):
            gx, gy = cx + g * scale, cy - g * scale
            if x0 <= gx <= x1:
                d.line([(gx, y0), (gx, y1)], fill=(36, 42, 54))
            if y0 <= gy <= y1:
                d.line([(x0, gy), (x1, gy)], fill=(36, 42, 54))
        d.line([(cx, y0), (cx, y1)], fill=(60, 68, 86))
        d.line([(x0, cy), (x1, cy)], fill=(60, 68, 86))
        d.text((x1 - 4, 50), '1 格 = 1 m', font=self.f_small, fill=DIM, anchor='ra')

        pts = [(cx + px * scale, cy - py * scale) for px, py in path[-3000:]]
        pts = [(x, y) for x, y in pts if x0 - 40 <= x <= x1 + 40 and y0 - 40 <= y <= y1 + 40]
        if len(pts) > 1:
            d.line(pts, fill=ACCENT, width=3)

        tx, ty = cx + tgt_x * scale, cy - tgt_y * scale
        d.ellipse((tx - 9, ty - 9, tx + 9, ty + 9), outline=AMBER, width=2)
        px, py = cx + cur_x * scale, cy - cur_y * scale
        d.ellipse((px - 7, py - 7, px + 7, py + 7), fill=GREEN)
        d.line([(px, py), (px + math.cos(heading) * 26, py - math.sin(heading) * 26)],
               fill=GREEN, width=4)
        d.text((x1 - 4, y1 + 4), '● 机体   ○ 目标', font=self.f_small,
               fill=DIM, anchor='ra')

    def render(self, st, ctrl, sticks, tel, path, stage, stage_i, total,
               t_now, t_stage, t_stage_total, deadman):
        img = self.base.copy()
        d = ImageDraw.Draw(img)
        roll_s, pitch_s, yaw_s, thr_s = sticks
        cx = 308

        # ---- 手柄 ----
        self._stick(d, cx - 154, 140, yaw_s, -thr_s, '左摇杆', '油门 / 偏航',
                    GREEN if deadman else (86, 92, 108))
        self._stick(d, cx + 154, 140, roll_s, -pitch_s, '右摇杆', '俯仰 / 横滚',
                    ACCENT if deadman else (86, 92, 108))
        items = [('油门', f'{thr_s:+.2f}'), ('偏航', f'{yaw_s:+.2f}'),
                 ('俯仰', f'{pitch_s:+.2f}'), ('横滚', f'{roll_s:+.2f}')]
        for i, (k, v) in enumerate(items):
            x = 44 + i * 136
            d.text((x, 258), k, font=self.f_small, fill=DIM)
            d.text((x, 278), v, font=self.f_mid, fill=FG)
        lamp = GREEN if deadman else RED
        d.ellipse((468, 26, 486, 44), fill=lamp)
        d.text((494, 26), 'DEADMAN ' + ('按住' if deadman else '松开'),
               font=self.f_small, fill=lamp)

        # ---- 姿态 ----
        self._horizon(img, d, st.roll, st.pitch)
        d.text((630, 300), '横滚', font=self.f_small, fill=DIM)
        d.text((666, 299), f'{math.degrees(st.roll):+.1f}°', font=self.f_txt, fill=FG)
        d.text((792, 300), '航向', font=self.f_small, fill=DIM)
        d.text((828, 299), f'{math.degrees(st.heading) % 360:.0f}°', font=self.f_txt, fill=FG)

        # ---- 航迹（相对起点）----
        self._map(d, path, st.x - getattr(st, 'x0', 0.0), st.y - getattr(st, 'y0', 0.0),
                  ctrl.tx - getattr(st, 'x0', 0.0), ctrl.ty - getattr(st, 'y0', 0.0),
                  st.heading)

        # ---- 阶段 ----
        d.text((1432, 34), '当前动作', font=self.f_small, fill=DIM)
        d.text((1432, 56), stage, font=self.f_big, fill=FG)
        frac = clamp(t_stage / max(t_stage_total, 0.001), 0, 1)
        d.rounded_rectangle((1432, 122, 1888, 132), 5, fill=(40, 46, 58))
        d.rounded_rectangle((1432, 122, 1432 + 456 * frac, 132), 5, fill=ACCENT)
        d.text((1432, 140), f'第 {stage_i + 1}/{total} 段   {t_stage:4.1f}s / '
                            f'{t_stage_total:.0f}s', font=self.f_small, fill=DIM)
        d.text((1888, 140), f'total {t_now:5.1f}s', font=self.f_small,
               fill=DIM, anchor='ra')

        # ---- 遥测 ----
        armed = bool(tel.get('armed'))
        batt = tel.get('battery', -1)
        rows = [
            ('模式', str(tel.get('mode', '--')), FG),
            ('状态', 'ARMED' if armed else 'DISARMED', GREEN if armed else RED),
            ('高度', f'{tel.get("alt", 0.0):5.2f} m', FG),
            ('水平速度', f'{math.hypot(st.vx, st.vy):4.2f} m/s', FG),
            ('垂直速度', f'{-st.vz:+5.2f} m/s', FG),
            ('航向', f'{math.degrees(st.heading) % 360:5.1f}°', FG),
            ('电量', f'{batt:5.1f} %' if batt and batt > 0 else '  --', FG),
        ]
        for i, (k, v, col) in enumerate(rows):
            y = 168 + i * 24
            d.text((1432, y), k, font=self.f_txt, fill=DIM)
            d.text((1888, y), v, font=self.f_txt, fill=col, anchor='ra')

        return img


# ---------------------------------------------------------------- 任务编排
def build_mission(x0, y0, alt0):
    """阶段表。

    关键点：每段的**目标都在持续移动**，不是"飞到点就停"。
    否则控制器很快回中，画面上手柄一动不动 —— 这是上一版最大的问题。
    场地限制：光流网格 ±6 m，x=+4 处有 6 m 高的测距墙。
    """
    return [
        dict(name='系统就绪', dur=8, kind='idle'),
        dict(name='解锁 ARM', dur=4, kind='arm'),
        # 起飞：爬到 1.4 m 后继续在 1.0~1.8 之间起伏，油门一直有动作
        dict(name='起飞', dur=14, kind='alt', alt=(0.0, 1.8), cycles=1.6),
        # 前进：来回推杆，1 个来回 11 s
        dict(name='前进', dur=22, kind='fwd', amp=1.2, cycles=3.4),
        # 左转：持续给偏航杆，机体连续自转约两圈
        dict(name='左转', dur=20, kind='spin', rate=0.30),
        # 俯冲 & 攀升：0.9~2.1 m 之间两次起伏
        dict(name='俯冲 & 攀升', dur=22, kind='alt', alt=(0.9, 2.1), cycles=2.0),
        dict(name='绕 8 字', dur=46, kind='fig8'),
        dict(name='返回原点', dur=16, kind='goto'),
        dict(name='降落', dur=24, kind='descend'),
        dict(name='任务完成', dur=4, kind='land'),
    ]


def figure8(center_x, center_y, u, amp=1.9, span=1.3):
    """Gerono 双纽线：x = A sin u, y = A sin u cos u。"""
    return (center_x + amp * math.sin(u),
            center_y + span * math.sin(u) * math.cos(u))


class _FramesDone(Exception):
    """--frames 达到上限时用来跳出整场（break 只能跳出内层循环）。"""


def crop_center(im, keep=0.84):
    """裁掉外圈（下视相机会扫到机臂/桨），保持尺寸不变。"""
    w, h = im.size
    nw, nh = int(w * keep), int(h * keep)
    box = ((w - nw) // 2, (h - nh) // 2, (w + nw) // 2, (h + nh) // 2)
    return im.crop(box).resize((w, h), Image.BILINEAR)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--record', default='')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--grab', default=':1.0+0,0')
    ap.add_argument('--duration', type=int, default=180)
    ap.add_argument('--sensors', default='depth,flow',
                    help='要显示哪几路传感器画中画：depth,flow / none / depth / flow')
    ap.add_argument('--frames', type=int, default=0,
                    help='渲染 N 帧后退出（0=跑完整场），便于快速验证画面')
    args = ap.parse_args()

    # 传感器画中画开关：--sensors depth,flow / none / depth / flow
    wanted = {t.strip().lower() for t in args.sensors.split(',') if t.strip()}
    if 'none' in wanted:
        wanted = set()
    show_depth = 'depth' in wanted
    show_flow = 'flow' in wanted

    ws = WSClient()
    st = FlightState()
    cams = CameraFeed()
    # 只订阅真正要显示的那几路，关掉的连 DDS 流量都省了
    extra = set()
    if show_depth:
        extra.add('depth')
    if show_flow:
        extra.add('down')
    cams.attach(st.node, keys={'third', 'fpv', 'top'} | extra)
    print('传感器画中画：深度=%s 光流=%s' % ('开' if show_depth else '关',
                                            '开' if show_flow else '关'), flush=True)
    ctrl = Controller()
    ov = Overlay()

    class PoseView:
        """位置取自真实位姿，姿态/航向仍取 PX4。"""
        __slots__ = ('x', 'y', 'z')

        def __init__(self):
            self.x = self.y = 0.0
            self.z = 0.0

        def __getattr__(self, name):
            return getattr(st, name)

        @property
        def alt(self):
            return -self.z

    pos = PoseView()
    canvas = Image.new('RGB', (W, H), (12, 14, 18))   # 整帧画布，网页那块留黑给 ffmpeg 叠
    pip_font = font(18, True)

    def draw_pip(im, pos, caption, footer=None):
        """贴一个画中画并加边框+标题，让小窗口和主画面分得开。"""
        canvas.paste(im, pos)
        d = ImageDraw.Draw(canvas)
        x, y = pos
        d.rectangle((x, y, x + PIP_W - 1, y + PIP_H - 1), outline=(90, 200, 255), width=2)
        d.rectangle((x, y, x + 140, y + 26), fill=(10, 12, 16))
        d.text((x + 8, y + 4), caption, font=pip_font, fill=(150, 215, 255))
        if footer:
            d.rectangle((x, y + PIP_H - 26, x + PIP_W - 1, y + PIP_H - 1), fill=(10, 12, 16))
            d.text((x + 8, y + PIP_H - 23), footer, font=pip_font, fill=(230, 236, 244))
        return d

    def draw_flow_arrow(d, pos, fx, fy):
        """在光流小窗里画箭头：指向画面纹理移动方向，长度随角速度（°/s）变化。"""
        x, y = pos
        cx, cy = x + PIP_W // 2, y + PIP_H // 2 + 10
        d.line(((cx - 40, cy), (cx + 40, cy)), fill=(60, 66, 82), width=1)
        d.line(((cx, cy - 40), (cx, cy + 40)), fill=(60, 66, 82), width=1)
        # 悬停时几 °/s，机动时几十 °/s；取负号 = 相机转动方向反推画面移动方向
        vx = max(-70.0, min(70.0, -fx * 2.5))
        vy = max(-70.0, min(70.0, -fy * 2.5))
        if abs(vx) < 2 and abs(vy) < 2:
            d.ellipse((cx - 4, cy - 4, cx + 4, cy + 4), outline=(120, 200, 255))
            return
        tip = (cx + vx, cy + vy)
        d.line(((cx, cy), tip), fill=(255, 210, 90), width=4)
        # 箭头两个翼
        import math as _m
        ang = _m.atan2(vy, vx)
        for da in (2.6, -2.6):
            d.line((tip, (tip[0] + 12 * _m.cos(ang + da),
                          tip[1] + 12 * _m.sin(ang + da))),
                   fill=(255, 210, 90), width=4)

    # 摇杆上行走独立线程，永远 50Hz（服务端 300ms 无帧就判使能失效）
    sp = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'throttle': -1.0}
    sp_lock = threading.Lock()
    running = True

    yielded = {'v': False}

    def sender():
        while running:
            # 让贤：发现控制权被网页拿走就立刻停手，别和真人抢杆
            if ws.telemetry.get('controller') == 'other':
                if not yielded['v']:
                    yielded['v'] = True
                    print('控制权已被网页接管，脚本停止发杆', file=sys.stderr)
                time.sleep(0.05)
                continue
            yielded['v'] = False
            with sp_lock:
                s = dict(sp)
            try:
                ws.sticks(s['roll'], s['pitch'], s['yaw'], s['throttle'])
            except OSError:
                break
            time.sleep(0.02)

    threading.Thread(target=sender, daemon=True).start()

    def apply(roll, pitch, yaw, throttle):
        with sp_lock:
            sp.update(roll=roll, pitch=pitch, yaw=yaw, throttle=throttle)

    # 等第一帧遥测/位置
    t_wait = time.time()
    while time.time() - t_wait < 20:
        apply(0, 0, 0, -1.0)
        if ws.telemetry and st.fresh > 0:
            break
        time.sleep(0.05)
    if not ws.telemetry:
        print('!! 收不到遥测，检查后端是否在 8000 端口', file=sys.stderr)
        return 1

    # 切 ALTCTL（横滚/俯仰杆 = 姿态角）。切模式要确认，不能发完就走
    for _ in range(5):
        ws.send({'type': 'mode', 'mode': 'ALTCTL'})
        time.sleep(1.6)
        if ws.telemetry.get('nav_state') == 1:
            break
    if ws.telemetry.get('nav_state') != 1:
        print('!! 无法切到 ALTCTL，中止', file=sys.stderr)
        return 2
    print(f'模式已确认 {ws.telemetry.get("mode")} (nav_state=1)')

    # 起飞前确保在地面
    if st.alt > 0.25:
        print(f'起始高度 {st.alt:.2f} m，先降落')
        apply(0, 0, 0, -1.0)
        t_end = time.time() + 25
        while time.time() < t_end and st.alt > 0.12:
            time.sleep(0.1)
        ws.send({'type': 'disarm'})
        time.sleep(2.0)
        ws.send({'type': 'mode', 'mode': 'ALTCTL'})
        time.sleep(1.5)
    x0, y0 = st.x, st.y
    st.x0, st.y0 = x0, y0
    pos.x, pos.y, pos.z = st.x, st.y, st.z
    print(f'起始位置 x={x0:.2f} y={y0:.2f} alt={st.alt:.2f} '
          f'roll={math.degrees(st.roll):+.1f} pitch={math.degrees(st.pitch):+.1f}')
    if abs(st.roll) > 0.5 or abs(st.pitch) > 0.5:
        print('!! EKF 姿态异常（可能已翻转），需重启仿真后再录', file=sys.stderr)
        return 2

    # 安全边界按世界坐标算（x=+4 有 6 m 高的墙），不要相对起点漂移
    ctrl.xlim = (-2.6, 2.4)
    ctrl.ylim = (-2.6, 2.6)
    mission = build_mission(x0, y0, 0.0)
    total = len(mission)
    path = [(0.0, 0.0)]
    log = open('/tmp/flight_show_log.jsonl', 'w')

    ff = None
    if args.record and not args.dry_run:
        ff = subprocess.Popen([
            'ffmpeg', '-loglevel', 'error', '-y',
            # 0: 网页窗口（真实 Firefox）
            '-f', 'x11grab', '-video_size', '640x720', '-framerate', str(FPS),
            '-i', args.grab,
            # 1: Python 合成的整帧（三视角 + 面板，网页区域留黑）
            '-f', 'rawvideo', '-pix_fmt', 'rgb24',
            '-video_size', f'{W}x{H}', '-framerate', str(FPS), '-i', 'pipe:0',
            '-filter_complex',
            '[1:v][0:v]overlay=1280:0:format=auto,format=yuv420p[v]',
            '-map', '[v]', '-t', str(args.duration),
            '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '20',
            '-movflags', '+faststart', args.record,
        ], stdin=subprocess.PIPE)
        print(f'录制中 → {args.record}')

    t_start = time.time()
    next_frame = t_start
    frames = 0
    try:
        for i, mg in enumerate(mission):
            t_stage0 = time.time()
            kind = mg['kind']
            # 每段的基线：起点位置/高度，后面按 kind 生成持续移动的目标
            base_x, base_y, base_alt = pos.x, pos.y, max(pos.alt, 0.0)
            ctrl.tx, ctrl.ty, ctrl.talt = base_x, base_y, base_alt
            ctrl.t_heading = st.heading
            ctrl.heading_follow = (kind == 'fig8')
            ctrl.reset_integrators()
            spin_yaw = 0.0

            if kind == 'arm':
                ws.send({'type': 'arm'})
                last_arm = time.time()
            if kind == 'land':
                ws.send({'type': 'land'})

            if i == 3 and not ws.telemetry.get('armed'):
                print('!! 解锁失败，中止录制（检查 PX4 health_and_arming_checks）',
                      file=sys.stderr)
                return 2

            t_prev = time.time()
            while True:
                now = time.time()
                t_stage = now - t_stage0
                if t_stage >= mg['dur']:
                    break
                dt = max(now - t_prev, 1e-3)
                t_prev = now

                # 位置反馈仍用 PX4 的 vehicle_local_position。
                # 曾试过换 Gazebo 真实位姿做闭环，但 PX4 的 NED 局部系与
                # Gazebo 世界系在对齐上会漂，混用会让控制器发散（实测飞出 100+ m）。
                # 真实位姿只在跟拍相机侧使用（scripts/pose_writer.py），
                # 那是"看得见飞机"的问题，不参与控制。
                pos.x, pos.y, pos.z = st.x, st.y, st.z

                # ---- 每段的目标轨迹：始终在动，杆量才不会长时间回中 ----
                phase = 2 * math.pi * mg.get('cycles', 1) * (t_stage / mg['dur'])
                if kind == 'alt':
                    lo, hi = mg['alt']
                    mid, amp = (lo + hi) / 2, (hi - lo) / 2
                    # 从低端起步，先爬后沉
                    ctrl.talt = mid + amp * math.sin(phase - math.pi / 2)
                elif kind == 'fwd':
                    # 往复推杆：幅度 1.4 m、周期约 6 s → 峰值 1.4 m/s，姿态杆约 ±0.28
                    ctrl.tx = base_x + mg['amp'] * math.sin(phase - math.pi / 2)
                    ctrl.ty = base_y
                    ctrl.talt = base_alt
                elif kind == 'spin':
                    ctrl.tx, ctrl.ty, ctrl.talt = base_x, base_y, base_alt
                    spin_yaw = mg['rate']
                elif kind == 'fig8':
                    u = 2 * math.pi * (t_stage / (mg['dur'] / 2.0))
                    ctrl.tx, ctrl.ty = figure8(base_x - 1.0, base_y, u)
                    ctrl.talt = 1.9
                elif kind == 'goto':
                    # 螺旋进近：半径 2.6→0 收缩 + 一圈绕飞，全程都在动
                    frac = t_stage / mg['dur']
                    radius = 2.6 * (1.0 - frac)
                    ang = 2 * math.pi * frac
                    ctrl.tx = x0 + radius * math.cos(ang)
                    ctrl.ty = y0 + radius * math.sin(ang)
                    ctrl.talt = 1.4
                elif kind == 'descend':
                    ctrl.tx, ctrl.ty = base_x, base_y
                    ctrl.talt = max(base_alt - 0.35 * t_stage, -0.05)

                if abs(st.roll) > 1.0 or abs(st.pitch) > 1.0:
                    print('!! 姿态异常（可能撞机/翻转），中止录制', file=sys.stderr)
                    apply(0, 0, 0, -1.0)
                    return 3
                roll, pitch, yaw, thr = ctrl.sticks(pos, dt)
                if kind == 'spin':
                    yaw = spin_yaw                      # ALTCTL 的偏航杆就是角速率指令
                if kind in ('idle', 'arm'):
                    # 解锁硬性要求：油门在最低位。这阶段绝不能交给高度环
                    roll = pitch = yaw = 0.0
                    thr = -1.0
                elif kind == 'land':
                    roll = pitch = yaw = 0.0
                    thr = -1.0            # 收尾压到底，确保落地
                if kind == 'arm' and time.time() - last_arm > 2.0 \
                        and not ws.telemetry.get('armed'):
                    ws.send({'type': 'arm'})
                    last_arm = time.time()
                apply(roll, pitch, yaw, thr)

                rel = (pos.x - x0, pos.y - y0)
                if not path or math.hypot(rel[0] - path[-1][0], rel[1] - path[-1][1]) > 0.05:
                    path.append(rel)
                if log:
                    log.write(json.dumps({
                        't': round(now - t_start, 3), 'stage': mg['name'],
                        'roll': round(roll, 4), 'pitch': round(pitch, 4),
                        'yaw': round(yaw, 4), 'throttle': round(thr, 4),
                        'x': round(pos.x, 3), 'y': round(pos.y, 3),
                        'alt': round(pos.alt, 3), 'armed': ws.telemetry.get('armed'),
                        'mode': ws.telemetry.get('mode'),
                    }) + '\n')

                # 按固定帧率出图，避免渲染时长抖动
                now = time.time()
                if ff and now >= next_frame:
                    canvas.paste(cams.get('third', (1280, 720)), (0, 0))
                    draw_pip(cams.get('fpv', (PIP_W, PIP_H)), PIP_FPV, '① 第一视角')
                    draw_pip(cams.get('top', (PIP_W, PIP_H)), PIP_TOP, '② 俯视')

                    # ③ 深度相机：伪彩色图 + 中央最近距离
                    if show_depth:
                        depth_txt = ('最近 %.2f m' % cams.depth_min) \
                            if cams.depth_min > 0 else '无数据'
                        draw_pip(cams.get('depth', (PIP_W, PIP_H)), PIP_DEPTH,
                                 '③ 深度相机', depth_txt)

                    # ④ 光流：下视原图 + 箭头 + 角速度
                    if show_flow:
                        flow_len = math.hypot(st.flow_x, st.flow_y)
                        if st.flow_quality > 0:
                            flow_txt = '光流 %.0f °/s  质量 %d' % (flow_len, st.flow_quality)
                        else:
                            flow_txt = '贴地/无纹理（质量 0）'
                        d_flow = draw_pip(crop_center(cams.get('down', (PIP_W, PIP_H))), PIP_FLOW,
                                          '④ 光流（下视）', flow_txt)
                        draw_flow_arrow(d_flow, PIP_FLOW, st.flow_x, st.flow_y)
                    canvas.paste(ov.render(pos, ctrl, (roll, pitch, yaw, thr),
                                           ws.telemetry, path, mg['name'], i, total,
                                           time.time() - t_start, t_stage,
                                           mg['dur'], True), (0, 720))
                    try:
                        ff.stdin.write(canvas.tobytes())
                    except BrokenPipeError:
                        ff = None
                    next_frame = max(next_frame + 1.0 / FPS, time.time())
                    frames += 1
                    if args.frames and frames >= args.frames:
                        print('已渲染 %d 帧，退出' % frames, flush=True)
                        raise _FramesDone
                time.sleep(0.004)

    except _FramesDone:
        print('达到 --frames 上限，收尾', flush=True)

    try:
        # 收尾：确保落地并上锁
        apply(0, 0, 0, -1.0)
        time.sleep(2.0)
        ws.send({'type': 'land'})
        time.sleep(3.0)
        ws.send({'type': 'disarm'})
        time.sleep(1.0)
    finally:
        running = False
        if ff:
            try:
                ff.stdin.close()
            except Exception:
                pass
            ff.wait()
        if log:
            log.close()
        ws.close()
    print('完成')
    sys.stdout.flush()
    os._exit(0)


if __name__ == '__main__':
    sys.exit(main())
