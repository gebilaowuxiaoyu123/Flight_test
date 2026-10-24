#!/usr/bin/env python3
"""相机动捕：每帧把 cam_third（跟拍）/ cam_top（俯视）摆到机体的相对位置。

为什么不用 gz-gui 的 CameraTracking：实测 `follow_target` 设了也不跟。
改用 gz-transport 直接 set_pose，实测 0.2 ms/次，25 Hz 毫无压力。

两个机位都是「世界朝向固定、位置随机体平移」：
画面不会跟着机体打转，但无人机始终在正中 —— 观感是"静态机位 + 轨道推车"。

单独进程运行，不 import rclpy：gz 的 pb2 与 ROS 2 Humble 的 protobuf 版本冲突。
"""
from __future__ import annotations

import json
import math
import os
import time

# gz 的 pb2 是旧版 protoc 生成的，必须走纯 Python 实现
os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')

from gz.msgs10.boolean_pb2 import Boolean          # noqa: E402
from gz.msgs10.pose_pb2 import Pose               # noqa: E402
from gz.transport13 import Node                   # noqa: E402

WORLD = 'flow'
TARGET_FILE = '/tmp/cam_target.json'

# 第三人称：固定世界朝向 + 沿视线后撤 dist
THIRD = dict(name='cam_third', yaw=0.95, pitch=0.32, dist=3.0)
# 俯视：机体正上方 height 米，镜头朝下
TOP = dict(name='cam_top', yaw=0.0, pitch=math.pi / 2, height=3.2)


def quat(yaw: float, pitch: float):
    """Rz(yaw) * Ry(pitch) 的四元数（gz 坐标系：X 前 / Y 左 / Z 上）。"""
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    return (-sy * sp, cy * sp, sy * cp, cy * cp)


def forward(yaw: float, pitch: float):
    """镜头朝向在世界系下的单位向量。"""
    return (math.cos(pitch) * math.cos(yaw),
            math.cos(pitch) * math.sin(yaw),
            -math.sin(pitch))


def main():
    node = Node()
    target = {'x': 0.0, 'y': 0.0, 'z': 0.5, 'ok': False}

    # 机体位置由 demo_show.py（ROS 侧，高频）写进小文件。
    # 不走 gz 的 pose/info：那是全实体大消息，纯 Python protobuf 解析太慢，
    # 回调几乎不刷新，相机就会一直停在初始位置。
    def read_target():
        try:
            with open(TARGET_FILE) as fh:
                d = json.load(fh)
            target.update(x=float(d['x']), y=float(d['y']), z=float(d['z']),
                          ok=True)
        except Exception:
            pass

    print('相机动捕启动：cam_third(跟拍) + cam_top(俯视)', flush=True)

    fx, fy, fz = forward(THIRD['yaw'], THIRD['pitch'])
    qx, qy, qz, qw = quat(THIRD['yaw'], THIRD['pitch'])
    tx, ty, tz, tw = quat(TOP['yaw'], TOP['pitch'])

    last = time.time()
    while True:
        read_target()
        if target['ok']:
            third = Pose()
            third.name = THIRD['name']
            third.position.x = target['x'] - fx * THIRD['dist']
            third.position.y = target['y'] - fy * THIRD['dist']
            third.position.z = target['z'] - fz * THIRD['dist']
            third.orientation.x, third.orientation.y = qx, qy
            third.orientation.z, third.orientation.w = qz, qw
            node.request(f'/world/{WORLD}/set_pose', third, Pose, Boolean, 300)

            top = Pose()
            top.name = TOP['name']
            top.position.x = target['x']
            top.position.y = target['y']
            top.position.z = target['z'] + TOP['height']
            top.orientation.x, top.orientation.y = tx, ty
            top.orientation.z, top.orientation.w = tz, tw
            node.request(f'/world/{WORLD}/set_pose', top, Pose, Boolean, 300)

        wait = 1.0 / 50 - (time.time() - last)
        last = time.time()
        if wait > 0:
            time.sleep(wait)
        else:
            time.sleep(0.002)


if __name__ == '__main__':
    main()
