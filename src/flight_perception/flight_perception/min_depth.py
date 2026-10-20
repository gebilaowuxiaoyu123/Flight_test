"""Turn a 32FC1 depth image into a single forward obstacle distance."""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


class MinDepth(Node):
    def __init__(self):
        super().__init__('min_depth')
        self.declare_parameter('image_topic', '/depth_camera')
        self.declare_parameter('output_topic', '/obstacle_distance')
        self.declare_parameter('max_range', 20.0)
        self.declare_parameter('roi_width', 0.5)
        self.declare_parameter('roi_height', 0.6)

        self._image_topic = self.get_parameter('image_topic').value
        self._max_range = float(self.get_parameter('max_range').value)
        self._roi_w = float(self.get_parameter('roi_width').value)
        self._roi_h = float(self.get_parameter('roi_height').value)
        self._warned = False

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(Image, self._image_topic, self._on_image, qos)
        self._pub = self.create_publisher(
            Float32, self.get_parameter('output_topic').value, 10)
        self._last_log = 0.0
        self.get_logger().info(f'listening {self._image_topic}')

    def _on_image(self, msg):
        if msg.encoding not in ('32FC1', '32FC1;'):
            if not self._warned:
                self.get_logger().warn(f'unsupported encoding: {msg.encoding}')
                self._warned = True
            return

        row_len = msg.step // 4
        if row_len < msg.width:
            return
        arr = np.frombuffer(msg.data, dtype=np.float32)
        if arr.size < msg.height * row_len:
            return
        img = arr.reshape(msg.height, row_len)[:, :msg.width]

        h, w = img.shape
        rw = max(1, int(w * self._roi_w))
        rh = max(1, int(h * self._roi_h))
        x0 = max(0, (w - rw) // 2)
        y0 = max(0, (h - rh) // 2)
        roi = img[y0:y0 + rh, x0:x0 + rw]

        valid = roi[np.isfinite(roi)]
        valid = valid[(valid > 0.01) & (valid < self._max_range)]
        if valid.size == 0:
            return

        distance = float(np.min(valid))
        out = Float32()
        out.data = distance
        self._pub.publish(out)

        now = time.monotonic()
        if now - self._last_log > 0.5:
            self.get_logger().info(f'obstacle distance: {distance:.2f} m')
            self._last_log = now


def main(args=None):
    rclpy.init(args=args)
    node = MinDepth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
