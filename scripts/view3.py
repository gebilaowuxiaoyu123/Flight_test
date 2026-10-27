#!/usr/bin/env python3
"""三视角实时预览：第一人称 / 第三人称跟拍 / 俯视。

用的就是录制成片时的同一批相机（Gazebo 相机传感器 → ros_gz_bridge → sensor_msgs/Image），
所以画面和视频里的三个视口完全一致。

    python3 scripts/view3.py              # 三个窗口
    python3 scripts/view3.py --fps 15     # 限制每个窗口的刷新率

依赖：tkinter（python3-tk）、Pillow、ros_gz_bridge 已在桥接这三路图像。
"""

import argparse
import signal
import sys
import threading
import time
import tkinter as tk

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image as RosImage

from PIL import Image, ImageTk


# (键, 显示名, 窗口坐标, 窗口宽, 窗口高)
VIEWS = [
    ('fpv',   '① 第一人称', 0,    40, 640, 360),
    ('third', '② 第三人称', 660,  40, 640, 360),
    ('top',   '③ 俯视',     1320, 40, 640, 360),
]

TOPIC = {
    'fpv':   '/world/{world}/model/{model}/link/camera_link/sensor/IMX214/image',
    'third': '/world/{world}/model/cam_third/link/link/sensor/third_cam/image',
    'top':   '/world/{world}/model/cam_top/link/link/sensor/top_cam/image',
}

BG = '#101014'
FG = '#d8d8d8'


class ViewNode(Node):
    def __init__(self, world, model):
        super().__init__('view3')
        # gz 侧是传感器数据，best_effort 订阅对 reliable / best_effort 发布方都兼容
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.frames = {k: None for k, *_ in VIEWS}
        self.counts = {k: 0 for k, *_ in VIEWS}
        self.sizes = {k: None for k, *_ in VIEWS}
        for key, *_ in VIEWS:
            topic = TOPIC[key].format(world=world, model=model)
            self.create_subscription(RosImage, topic,
                                     lambda msg, k=key: self._on_image(k, msg), qos)
            self.get_logger().info(f'订阅 {topic}')

    def _on_image(self, key, msg):
        mode = {('rgb8', 3): 'RGB', ('bgr8', 3): 'RGB', ('rgba8', 4): 'RGBA',
                ('bgra8', 4): 'RGBA', ('mono8', 1): 'L'}.get(
                    (msg.encoding, msg.step // msg.width), None)
        try:
            if mode == 'RGB' and msg.encoding == 'bgr8':
                img = Image.frombytes('RGB', (msg.width, msg.height),
                                      bytes(msg.data), 'raw', 'BGR', msg.step, 1)
            elif mode:
                img = Image.frombytes(mode, (msg.width, msg.height),
                                      bytes(msg.data), 'raw', mode, msg.step, 1)
            else:
                return
        except (ValueError, TypeError):
            return
        self.frames[key] = img
        self.sizes[key] = (msg.width, msg.height)
        self.counts[key] += 1


def main():
    ap = argparse.ArgumentParser(description='三视角实时预览')
    ap.add_argument('--world', default='flow')
    ap.add_argument('--model', default='x500_flow_depth_0')
    ap.add_argument('--fps', type=float, default=15.0, help='每个窗口刷新上限')
    args = ap.parse_args()

    rclpy.init()
    node = ViewNode(args.world, args.model)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    root = tk.Tk()
    root.withdraw()

    windows, labels, last_shown = {}, {}, {}
    interval = int(1000 / max(args.fps, 1.0))

    for key, name, x, y, w, h in VIEWS:
        win = tk.Toplevel(root)
        win.title(name)
        win.geometry(f'{w}x{h}+{x}+{y}')
        win.configure(bg=BG)
        label = tk.Label(win, bg=BG, fg=FG, text='等待画面 ...',
                         font=('Sans', 12))
        label.pack(expand=True, fill='both')
        windows[key] = win
        labels[key] = label
        last_shown[key] = 0.0
        win.lift()          # 别被 VS Code 压住

        def on_close():
            root.quit()

        win.protocol('WM_DELETE_WINDOW', on_close)

    print('三视角窗口已打开，Ctrl-C 或关掉任一窗口退出', flush=True)

    # tkinter 的 mainloop 不响应 Ctrl-C，所以自己接管信号，转成退出
    def _stop(*_):
        root.quit()

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    def tick():
        now = time.monotonic()
        for key, *_ in VIEWS:
            if now - last_shown[key] < interval / 1000.0:
                continue
            img = node.frames[key]
            if img is None:
                continue
            last_shown[key] = now
            w = windows[key].winfo_width()
            h = windows[key].winfo_height()
            if w > 1 and h > 1:
                scale = min(w / img.width, h / img.height)
                size = (max(1, int(img.width * scale)), max(1, int(img.height * scale)))
                photo = ImageTk.PhotoImage(img.resize(size, Image.BILINEAR))
                labels[key].configure(image=photo, text='')
                labels[key].image = photo        # 防止被 GC
        root.after(30, tick)

    def report():
        parts = []
        for key, name, *_ in VIEWS:
            fps = node.counts[key] / 2.0
            node.counts[key] = 0
            if fps > 0.1:
                w, h = node.sizes[key]
                parts.append(f'{name.split()[0]} {fps:4.1f} fps {w}x{h}')
            else:
                parts.append(f'{name.split()[0]} 无数据')
        print('[三视角] ' + ' | '.join(parts), flush=True)
        root.after(2000, report)

    root.after(30, tick)
    root.after(2000, report)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
