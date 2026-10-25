#!/usr/bin/env python3
"""一次性读取飞行状态（不依赖任何常驻缓存进程）。

每次启动一个短命 ROS 节点，最多等 0.6 s 收齐三路遥测就打印退出。
比 ros2 topic echo 快，而且没有"缓存过期/孤儿进程"这类问题。

  python3 scripts/tstatus.py           # 一行人类可读
  python3 scripts/tstatus.py --json    # 机器可读
  python3 scripts/tstatus.py --alt     # 只输出高度
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time

from rosboot import ensure_ros

ensure_ros()

import rclpy
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

NAV_NAMES = {
    0: 'MANUAL', 1: 'ALTCTL', 2: 'POSCTL', 3: 'AUTO_MISSION', 4: 'AUTO_LOITER',
    5: 'AUTO_RTL', 6: 'POSITION_SLOW', 10: 'ACRO', 12: 'DESCEND',
    13: 'TERMINATION', 14: 'OFFBOARD', 15: 'STABILIZED', 17: 'AUTO_TAKEOFF',
    18: 'AUTO_LAND', 22: 'AUTO_VTOL_TAKEOFF',
}


def read(timeout=2.5):
    rclpy.init()
    node = Node('tstatus')
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
    d = {}

    def on_status(m):
        d['arm_state'] = m.arming_state
        d['nav_state'] = m.nav_state
        d['_got_status'] = True

    def on_pos(m):
        d['alt'] = -m.z
        d['vn'], d['ve'], d['vd'] = m.vx, m.vy, m.vz
        d['_got_pos'] = True

    def on_att(m):
        w, x, y, z = m.q
        d['roll'] = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        d['pitch'] = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
        d['_got_att'] = True

    node.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', on_status, qos)
    node.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', on_pos, qos)
    node.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', on_att, qos)

    end = time.time() + timeout
    while time.time() < end:
        if d.get('_got_status') and d.get('_got_pos') and d.get('_got_att'):
            break
        rclpy.spin_once(node, timeout_sec=0.05)

    try:
        node.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json', action='store_true')
    ap.add_argument('--alt', action='store_true')
    ap.add_argument('--armed', action='store_true')
    ap.add_argument('--timeout', type=float, default=2.5)
    args = ap.parse_args()

    d = read(args.timeout)
    if not d:
        print('  （收不到遥测：仿真没起？先跑 flight 或 fly start）')
        return 1

    if args.alt:
        print(f"{d.get('alt', 0.0):.2f}")
        return 0
    if args.armed:
        print(1 if d.get('arm_state') == 2 else 0)
        return 0

    out = dict(
        mode=NAV_NAMES.get(d.get('nav_state'), '?'),
        armed=(d.get('arm_state') == 2),
        alt=round(d.get('alt', 0.0), 2),
        roll_deg=round(math.degrees(d.get('roll', 0.0)), 1),
        pitch_deg=round(math.degrees(d.get('pitch', 0.0)), 1),
        vn=round(d.get('vn', 0.0), 2), ve=round(d.get('ve', 0.0), 2),
        vd=round(d.get('vd', 0.0), 2),
    )
    out['speed'] = round(math.hypot(out['vn'], out['ve']), 2)

    if args.json:
        print(json.dumps(out))
        return 0

    G, R, C = '\033[92m', '\033[91m', '\033[96m'
    Z = '\033[0m'
    armed = f'{G}ARMED   {Z}' if out['armed'] else f'{R}DISARMED{Z}'
    print(f"  模式 {C}{out['mode']:<13}{Z} 状态 {armed} "
          f"高度 {out['alt']:6.2f} m  横滚 {out['roll_deg']:+5.1f}°  "
          f"俯仰 {out['pitch_deg']:+5.1f}°  速度 {out['speed']:4.2f} m/s")
    return 0


if __name__ == '__main__':
    sys.exit(main())
