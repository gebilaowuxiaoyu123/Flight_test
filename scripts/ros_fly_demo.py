#!/usr/bin/env python3
"""纯 ROS 2 话题操控无人机（不经过 Web 界面）。

只用两个话题：
  /fmu/in/vehicle_command        离散命令（切模式 / 解锁 / 上锁 / 降落）
  /fmu/in/manual_control_input   50 Hz 摇杆帧（连续量）

为什么用 ALTCTL 而不用 OFFBOARD：本机无 GPS，光流给不出有效水平位置
（实测 vehicle_local_position.xy_valid = false），PX4 会以
"invalid setpoints" 拒绝 OFFBOARD。ALTCTL 的横滚/俯仰杆直接对应姿态角，
不依赖位置估计，所以能飞。

用法：
  python3 scripts/ros_fly_demo.py            # 完整飞一轮（约 40 s）
  python3 scripts/ros_fly_demo.py --land     # 只降落上锁
"""
from __future__ import annotations

import argparse
import os
import sys
import threading
import time

import rclpy
from px4_msgs.msg import ManualControlSetpoint, VehicleCommand, VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

CMD_ARM_DISARM = 400     # param1: 1=解锁 0=上锁
CMD_DO_SET_MODE = 176    # param1=1 自定义模式, param2=主模式号
MAIN_MODE_ALTCTL = 2


class RosPilot(Node):
    def __init__(self):
        super().__init__('ros_fly_demo')
        qos_out = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_in = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_in)
        self.stick_pub = self.create_publisher(
            ManualControlSetpoint, '/fmu/in/manual_control_input', qos_in)

        self.alt = 0.0
        self.armed = False
        self.nav_state = -1
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position',
                                 self._on_pos, qos_out)
        self._load_vehicle_status(qos_out)

        # 摇杆帧必须是连续流：PX4 超过 0.5 s 收不到就把手动控制判为失效
        self._stick = dict(roll=0.0, pitch=0.0, yaw=0.0, throttle=-1.0)
        self._run = True
        threading.Thread(target=self._stick_loop, daemon=True).start()

    def _load_vehicle_status(self, qos):
        from px4_msgs.msg import VehicleStatus
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 self._on_status, qos)

    def _on_pos(self, m):
        self.alt = -m.z                    # NED 向下为正，取负才是高度

    def _on_status(self, m):
        self.armed = (m.arming_state == 2)
        self.nav_state = m.nav_state

    def _stick_loop(self):
        while self._run:
            msg = ManualControlSetpoint()
            msg.timestamp = self.get_clock().now().nanoseconds // 1000
            msg.timestamp_sample = msg.timestamp
            msg.valid = True
            msg.data_source = ManualControlSetpoint.SOURCE_MAVLINK_0
            msg.roll = self._stick['roll']
            msg.pitch = self._stick['pitch']
            msg.yaw = self._stick['yaw']
            msg.throttle = self._stick['throttle']
            self.stick_pub.publish(msg)
            time.sleep(0.02)               # 50 Hz

    # ---------------- 对外接口 ----------------
    def sticks(self, roll=0.0, pitch=0.0, yaw=0.0, throttle=-1.0):
        self._stick.update(roll=roll, pitch=pitch, yaw=yaw, throttle=throttle)

    def send(self, command, p1=0.0, p2=0.0, p7=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.command = command
        # param1/2/7 必须是 float，传 int 会触发生成的 setter 断言
        msg.param1, msg.param2, msg.param7 = float(p1), float(p2), float(p7)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

    def close(self):
        self._run = False
        time.sleep(0.05)


def wait(cond, timeout, tick=0.05):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(tick)
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--land', action='store_true')
    args = ap.parse_args()

    rclpy.init()
    p = RosPilot()
    threading.Thread(target=rclpy.spin, args=(p,), daemon=True).start()
    time.sleep(1.0)

    def report(tag):
        print(f'  {tag:10s} 高度 {p.alt:5.2f} m   模式 nav_state={p.nav_state}   '
              f'{"ARMED" if p.armed else "DISARMED"}')

    try:
        if args.land:
            p.sticks(throttle=-0.5)
            time.sleep(5)
            p.send(CMD_ARM_DISARM, 0.0)
            time.sleep(1)
            report('已降落')
            return 0

        print('1) 切 ALTCTL')
        p.send(CMD_DO_SET_MODE, 1.0, MAIN_MODE_ALTCTL)
        wait(lambda: p.nav_state == 1, 5)
        report('切模式')

        print('2) 解锁（油门必须最低）')
        p.sticks(throttle=-1.0)
        for _ in range(3):
            p.send(CMD_ARM_DISARM, 1.0)
            if wait(lambda: p.armed, 4):
                break
        if not p.armed:
            print('  ✗ 解锁失败，看 PX4 日志的 Preflight Fail')
            return 1
        report('解锁')

        print('3) 爬升到约 1.2 m')
        p.sticks(throttle=0.45)
        wait(lambda: p.alt > 1.2, 12)
        p.sticks(throttle=0.0)
        report('爬升')

        print('4) 前进 3 秒（俯仰前推）')
        p.sticks(pitch=0.30, throttle=0.0)
        time.sleep(3)
        p.sticks(throttle=0.0)
        time.sleep(1)
        report('前进')

        print('5) 左移 2 秒（横滚左推）')
        p.sticks(roll=-0.30, throttle=0.0)
        time.sleep(2)
        p.sticks(throttle=0.0)
        time.sleep(1)
        report('左移')

        print('6) 左偏航 2 秒')
        p.sticks(yaw=0.40, throttle=0.0)
        time.sleep(2)
        p.sticks(throttle=0.0)
        time.sleep(1)
        report('偏航')

        print('7) 降落并上锁')
        p.sticks(throttle=-0.45)
        wait(lambda: p.alt < 0.25, 20)
        p.sticks(throttle=-1.0)
        time.sleep(2)
        p.send(CMD_ARM_DISARM, 0.0)
        time.sleep(1)
        report('完成')
        return 0
    finally:
        p.sticks(throttle=-1.0)
        time.sleep(0.3)
        p.close()
        # spin 线程还停在 C 层时直接返回会 abort（terminate called ...），
        # 用 os._exit 干净退出
        sys.stdout.flush()
        os._exit(0)


if __name__ == '__main__':
    sys.exit(main())
