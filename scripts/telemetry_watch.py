#!/usr/bin/env python3
"""终端实时遥测：模式 / 解锁 / 高度 / 姿态角 / 速度。

两种用法：
  python3 scripts/telemetry_watch.py --print     # 前台，终端里刷新显示
  python3 scripts/telemetry_watch.py            # 后台，只写 /tmp/telemetry.json

--print 时用 ANSI 光标归位原地重绘，不刷屏、不占用 scrollback。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time

from rosboot import ensure_ros

ensure_ros()

import rclpy
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

JSON_OUT = '/tmp/telemetry.json'

NAV_NAMES = {
    0: 'MANUAL', 1: 'ALTCTL', 2: 'POSCTL', 3: 'AUTO_MISSION', 4: 'AUTO_LOITER',
    5: 'AUTO_RTL', 6: 'POSITION_SLOW', 10: 'ACRO', 12: 'DESCEND',
    13: 'TERMINATION', 14: 'OFFBOARD', 15: 'STABILIZED', 17: 'AUTO_TAKEOFF',
    18: 'AUTO_LAND', 22: 'AUTO_VTOL_TAKEOFF',
}
ARM_NAMES = {1: 'DISARMED', 2: 'ARMED', 3: 'STANDBY'}


def bar(v, width=21):
    """把 [-1,1] 映射成一段带标记的字符条。"""
    idx = int(round((v + 1) / 2 * (width - 1)))
    idx = max(0, min(width - 1, idx))
    cells = ['-'] * width
    cells[width // 2] = '|'
    cells[idx] = 'O'
    return ''.join(cells)


class Watch(Node):
    def __init__(self):
        super().__init__('telemetry_watch')
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.armed = None
        self.nav_state = None
        self.alt = self.vn = self.ve = self.vd = 0.0
        self.roll = self.pitch = self.heading = 0.0
        self.updates = 0
        self.last_msg = 0.0

        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 self._on_status, qos)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position',
                                 self._on_pos, qos)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude',
                                 self._on_att, qos)

    def _on_status(self, m):
        self.armed = m.arming_state
        self.nav_state = m.nav_state
        self.updates += 1
        self.last_msg = time.time()

    def _on_pos(self, m):
        self.alt = -m.z
        self.vn, self.ve, self.vd = m.vx, m.vy, m.vz
        self.updates += 1
        self.last_msg = time.time()

    def _on_att(self, m):
        w, x, y, z = m.q
        self.roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        self.pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
        # 航向从水平速度方向估计（无磁罗盘）；静止时保持上一次
        if math.hypot(self.vn, self.ve) > 0.25:
            self.heading = math.atan2(self.ve, self.vn)
        self.updates += 1
        self.last_msg = time.time()

    def as_dict(self):
        return dict(
            mode=NAV_NAMES.get(self.nav_state, f'STATE_{self.nav_state}'),
            nav_state=self.nav_state,
            armed=(self.armed == 2),
            arm_state=ARM_NAMES.get(self.armed, '?'),
            alt=round(self.alt, 2),
            roll_deg=round(math.degrees(self.roll), 1),
            pitch_deg=round(math.degrees(self.pitch), 1),
            heading_deg=round(math.degrees(self.heading) % 360, 0),
            vn=round(self.vn, 2), ve=round(self.ve, 2), vd=round(self.vd, 2),
            speed=round(math.hypot(self.vn, self.ve), 2),
            t=round(self.last_msg, 2),   # 最后一条消息时间，判过期用这个
        )


def render(d):
    armed_mark = '\033[92m● ARMED\033[0m' if d['armed'] else '\033[91m○ DISARMED\033[0m'
    alt_col = '\033[93m' if d['armed'] else '\033[90m'
    roll_bar = bar(d['roll_deg'] / 45.0)
    pitch_bar = bar(-d['pitch_deg'] / 45.0)
    lines = [
        '',
        ' \033[1m┌─ 飞行姿态 ─────────────────────────────────────────────┐\033[0m',
        f" │ 模式  \033[96m{d['mode']:<13}\033[0m {armed_mark:<20} "
        f"高度 {alt_col}{d['alt']:6.2f} m\033[0m  │",
        f" │ 横滚 {d['roll_deg']:+6.1f}°  [{roll_bar}]              │",
        f" │ 俯仰 {d['pitch_deg']:+6.1f}°  [{pitch_bar}]              │",
        f" │ 航向 {d['heading_deg']:5.0f}°   水平速度 {d['speed']:4.2f} m/s"
        f"   垂直 {-d['vd']:+5.2f} m/s │",
        ' └────────────────────────────────────────────────────────┘',
        '   Ctrl-C 退出',
    ]
    return '\n'.join(lines)


def _acquire_singleton(lock_path: str):
    """同一时刻只允许一个实例。已有实例在跑就安静退出。

    之前靠 fly.sh 杀进程清理，一旦 pidfile 丢失就会留下孤儿进程，
    两个实例抢写 /tmp/telemetry.json，读端看到的数据时新时旧。
    """
    import fcntl
    fh = open(lock_path, 'w')
    try:
        fcntl.flock(fh, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        sys.exit(0)
    return fh          # 返回值要保活，进程退出前不能回收


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--print', dest='do_print', action='store_true')
    ap.add_argument('--hz', type=float, default=8.0)
    args = ap.parse_args()

    # --print 是"前台看"，只读不写：不抢锁、不写缓存文件，
    # 这样它可以和后台那个写缓存的实例同时存在。
    _lock = None if args.do_print else _acquire_singleton("/tmp/telemetry_watch.lock")

    rclpy.init()
    node = Watch()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    period = 1.0 / max(args.hz, 1.0)
    # \033[H 归位 + \033[J 清到末尾：原地重绘，不刷屏
    prefix = '\033[H\033[J' if args.do_print else ''
    try:
        while True:
            d = node.as_dict()
            if args.do_print:
                sys.stdout.write(prefix + render(d))
                sys.stdout.flush()
                time.sleep(period)
                continue
            try:
                # 原子写：直接 open('w') 会先截断，读端可能读到空文件
                with open(JSON_OUT + '.tmp', 'w') as fh:
                    json.dump(d, fh)
                os.replace(JSON_OUT + '.tmp', JSON_OUT)
            except OSError:
                pass
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        if args.do_print:
            print()
        # spin 线程还停在 C 层时正常退出会 abort（Fatal Python error: _enter_buffered_busy），
        # 用 os._exit 直接收摊
        sys.stdout.flush()
        os._exit(0)


if __name__ == '__main__':
    main()
