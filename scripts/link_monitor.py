#!/usr/bin/env python3
"""通信链路监视：每 2 秒打印一次 ROS 节点清单 + PX4 关键话题频率。

配合 `MicroXRCEAgent` 用（`flight link` 会自动起它）：
    终端里能看到 PX4 是否连上、话题通不通、频率对不对。

单独跑：python3 scripts/link_monitor.py
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import BatteryStatus
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus


QOS = QoSProfile(depth=1,
                 reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST)

# (显示名, 话题, 期望频率下限)
WATCH = [
    ('状态', '/fmu/out/vehicle_status_v1', 1.0),
    ('位置', '/fmu/out/vehicle_local_position', 20.0),
    ('电池', '/fmu/out/battery_status', 5.0),
]

MODEL = {
    '状态': VehicleStatus,
    '位置': VehicleLocalPosition,
    '电池': BatteryStatus,
}


class Monitor(Node):
    def __init__(self):
        super().__init__('link_monitor')
        self.count = {name: 0 for name, *_ in WATCH}
        self.last_nodes = []
        for name, topic, _ in WATCH:
            self.create_subscription(MODEL[name], topic,
                                     lambda _msg, n=name: self._bump(n), QOS)

    def _bump(self, name):
        self.count[name] += 1


def main():
    rclpy.init()
    node = Monitor()
    period = 2.0
    print('[通信] 开始监视（Ctrl-C 停止）', flush=True)
    last = time.monotonic()
    try:
        while rclpy.ok():
            # 短超时快速轮询，频率按真实经过时间算（不能拿固定的 period 当除数）
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now - last < period:
                continue
            dt, last = now - last, now

            # 节点清单：只在新节点出现/消失时打一行，避免刷屏
            nodes = sorted(n for n in node.get_node_names() if not n.startswith('_'))
            if nodes != node.last_nodes:
                node.last_nodes = nodes
                print(f'[节点] {len(nodes)} 个: ' + ', '.join(nodes), flush=True)

            parts, ok = [], True
            for name, _topic, minimum in WATCH:
                hz = node.count[name] / dt
                node.count[name] = 0
                if hz < minimum:
                    ok = False
                parts.append(f'{name} {hz:5.1f} Hz')
            flag = '链路正常' if ok else '等 PX4 数据 ...'
            print(f'[通信] {flag}  ' + ' | '.join(parts), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
