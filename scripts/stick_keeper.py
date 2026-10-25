#!/usr/bin/env python3
"""摇杆守护进程：以 50 Hz 持续发布 /fmu/in/manual_control_input。

读 /tmp/stick.json，格式 {"roll":0,"pitch":0,"yaw":0,"throttle":-1}。
文件不存在或读失败就发油门最低的安全值。

为什么要有这个守护：PX4 超过 0.5 s 收不到手动控制帧就判为失效（触发 failsafe），
所以不能"发一次就停"。让一个常驻进程一直发，其它脚本只改那个 4 个数的文件。
"""
import json
import os
import sys
import time

from rosboot import ensure_ros

ensure_ros()

import rclpy
from px4_msgs.msg import ManualControlSetpoint
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

STICK_FILE = '/tmp/stick.json'
# 过期后发什么：回中 = ALTCTL 里零爬升率 → 悬停。
# 不能用油门最低（那就是往下掉），更不能沿用上一条杆量（会一直爬）。
HOVER = dict(roll=0.0, pitch=0.0, yaw=0.0, throttle=0.0)
# 文件超过这个时间没更新，就认为"没人操控了"，回中悬停。
# 没有这条保护时，操控脚本被 Ctrl-C 打断会把最后一条杆量一直发下去
# —— 实测油门 0.45 被留在文件里，飞机一口气爬到 17 m。
STALE_SEC = 2.0


def read_stick():
    try:
        age = time.time() - os.stat(STICK_FILE).st_mtime
        if age > STALE_SEC:
            return dict(HOVER), True       # 过期 → 回中悬停
        with open(STICK_FILE) as fh:
            d = json.load(fh)
        return dict(roll=float(d.get('roll', 0.0)),
                    pitch=float(d.get('pitch', 0.0)),
                    yaw=float(d.get('yaw', 0.0)),
                    throttle=float(d.get('throttle', -1.0))), False
    except Exception:
        return dict(HOVER), True


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
    _lock = _acquire_singleton("/tmp/stick_keeper.lock")
    rclpy.init()
    node = Node('stick_keeper')
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(ManualControlSetpoint, '/fmu/in/manual_control_input', qos)
    print('stick_keeper 已启动（50 Hz）', flush=True)

    stale_announced = False
    while rclpy.ok():
        s, stale = read_stick()
        if stale and not stale_announced:
            print(f'⚠ 杆量文件 {STALE_SEC:.0f}s 未更新，已回中悬停', flush=True)
            stale_announced = True
        elif not stale:
            stale_announced = False
        msg = ManualControlSetpoint()
        msg.timestamp = node.get_clock().now().nanoseconds // 1000
        msg.timestamp_sample = msg.timestamp
        msg.valid = True
        msg.data_source = ManualControlSetpoint.SOURCE_MAVLINK_0
        msg.roll = s['roll']
        msg.pitch = s['pitch']
        msg.yaw = s['yaw']
        msg.throttle = s['throttle']
        pub.publish(msg)
        time.sleep(0.02)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
