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
# 控制权租约：和网页后端（rc_server）抢同一个话题时，同一时刻只有一方发杆量。
# 文件格式和 C++ 侧（control_lease.cpp）一致：<owner> <prio> <pid> <unix_ts>
LEASE_FILE = '/tmp/flight_control.lease'
LEASE_NAME = 'terminal'
LEASE_PRIO = 5              # 网页是 10，优先级更高，网页一连上就抢走
LEASE_STALE_SEC = 2.0
# 过期后发什么：回中 = ALTCTL 里零爬升率 → 悬停。
# 不能用油门最低（那就是往下掉），更不能沿用上一条杆量（会一直爬）。
HOVER = dict(roll=0.0, pitch=0.0, yaw=0.0, throttle=0.0, valid=True)
# 暂停：valid=False 让 PX4 知道"没有人在打杆"，
# 否则它检测到杆量变动会把飞机从 AUTO_LAND 踢回手动模式。
PAUSED = dict(roll=0.0, pitch=0.0, yaw=0.0, throttle=0.0, valid=False)
# 文件超过这个时间没更新，就认为"没人操控了"，回中悬停。
# 没有这条保护时，操控脚本被 Ctrl-C 打断会把最后一条杆量一直发下去
# —— 实测油门 0.45 被留在文件里，飞机一口气爬到 17 m。
STALE_SEC = 2.0


def read_stick():
    try:
        age = time.time() - os.stat(STICK_FILE).st_mtime
        if age > STALE_SEC:
            return dict(HOVER), True       # 过期 → 回中悬停
        if os.path.getsize(STICK_FILE) == 0:
            return dict(HOVER), True
        with open(STICK_FILE) as fh:
            d = json.load(fh)
        return dict(roll=float(d.get('roll', 0.0)),
                    pitch=float(d.get('pitch', 0.0)),
                    yaw=float(d.get('yaw', 0.0)),
                    throttle=float(d.get('throttle', -1.0)),
                    valid=bool(d.get('valid', True))), False
    except Exception:
        return dict(HOVER), True


def _pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def _read_lease():
    try:
        with open(LEASE_FILE) as f:
            owner, prio, pid, ts = f.readline().split()
        return owner, int(prio), int(pid), int(ts)
    except (OSError, ValueError):
        return None


def _write_lease():
    tmp = LEASE_FILE + '.tmp'
    try:
        with open(tmp, 'w') as f:
            f.write(f'{LEASE_NAME} {LEASE_PRIO} {os.getpid()} {int(time.time() * 1000)}\n')
        os.replace(tmp, LEASE_FILE)      # 原子替换，读端不会读到半行
    except OSError:
        pass


def try_acquire_lease() -> bool:
    """能不能发杆量。能就顺手刷心跳，不能就返回 False。"""
    cur = _read_lease()
    now_ms = int(time.time() * 1000)
    free = (cur is None) or (now_ms - cur[3] > LEASE_STALE_SEC * 1000) or (not _pid_alive(cur[2]))
    if free or cur[0] == LEASE_NAME or cur[1] < LEASE_PRIO:
        _write_lease()
        return True
    return False


def release_lease():
    cur = _read_lease()
    if cur and cur[0] == LEASE_NAME:
        try:
            os.remove(LEASE_FILE)
        except OSError:
            pass


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
    lease_ok = False
    lease_checked = 0.0
    while rclpy.ok():
        now = time.time()
        if now - lease_checked > 0.5:
            lease_checked = now
            was = lease_ok
            lease_ok = try_acquire_lease()
            if was and not lease_ok:
                cur = _read_lease()
                print(f'⚠ 控制权被 {cur[0] if cur else "?"} 接管，暂停发杆（网页在操控）',
                      flush=True)
            elif not was and lease_ok:
                print('控制权已取得，恢复发杆', flush=True)
        if not lease_ok:
            time.sleep(0.02)
            continue

        s, stale = read_stick()
        if stale and not stale_announced:
            print(f'⚠ 杆量文件 {STALE_SEC:.0f}s 未更新，已回中悬停', flush=True)
            stale_announced = True
        elif not stale:
            stale_announced = False
        msg = ManualControlSetpoint()
        msg.timestamp = node.get_clock().now().nanoseconds // 1000
        msg.timestamp_sample = msg.timestamp
        msg.valid = s.get('valid', True)
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
