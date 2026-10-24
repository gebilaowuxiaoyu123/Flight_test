#!/usr/bin/env python3
"""把机体在 Gazebo 里的**真实位姿**写进 /tmp/cam_target.json。

为什么不直接用 PX4 的 vehicle_local_position：光流定位会漂，
实测飞完一圈 PX4 估计的 (x,y) 和 Gazebo 真实位置能差 2 m 以上。
相机跟拍要的是「画面里能看见飞机」，所以必须用真实位姿。

为什么单独一个进程：gz 的 pb2 是旧 protoc 生成的，纯 Python 解析
`/world/flow/pose/info` 很慢；和 25 Hz 的 set_pose 循环放在同一进程会互相拖累。
"""
from __future__ import annotations

import json
import os
import time

os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')

from gz.msgs10.pose_v_pb2 import Pose_V           # noqa: E402
from gz.transport13 import Node                   # noqa: E402

WORLD = 'flow'
DRONE = 'x500_flow_depth_0'
OUT = '/tmp/cam_target.json'
TMP = '/tmp/.cam_target.tmp'


def main():
    node = Node()

    def on_pose(msg, *_args):
        for p in msg.pose:
            if p.name == DRONE:
                with open(TMP, 'w') as fh:
                    json.dump({'x': round(p.position.x, 3),
                               'y': round(p.position.y, 3),
                               'z': round(p.position.z, 3),
                               't': round(time.time(), 3)}, fh)
                os.replace(TMP, OUT)      # 原子替换，读端不会读到半截
                break
        return True

    # 用 dynamic_pose/info 而不是 pose/info：后者含 2300+ 块静态网格，
    # 纯 Python protobuf 解析不过来，跟拍会明显滞后
    node.subscribe(Pose_V, f'/world/{WORLD}/dynamic_pose/info', on_pose)
    print('真实位姿写入器启动 →', OUT, flush=True)

    while True:
        time.sleep(1.0)


if __name__ == '__main__':
    main()
