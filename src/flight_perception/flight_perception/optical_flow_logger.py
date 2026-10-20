"""Log PX4 optical flow and compare it with EKF2 velocity."""

import csv
import math
import time

import rclpy
from px4_msgs.msg import (
    DistanceSensor,
    VehicleLocalPosition,
    VehicleOpticalFlow,
)
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray


class OpticalFlowLogger(Node):
    def __init__(self):
        super().__init__('optical_flow_logger')
        self.declare_parameter('output', '/tmp/optical_flow.csv')
        self.declare_parameter('flush_every', 50)

        self._output = self.get_parameter('output').value
        self._flush_every = int(self.get_parameter('flush_every').value)
        self._rows = 0
        self._vx = 0.0
        self._vy = 0.0
        self._distance = math.nan

        self._file = open(self._output, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow([
            't', 'flow_x', 'flow_y', 'dt', 'distance', 'quality',
            'ekf_vx', 'ekf_vy', 'flow_vx', 'flow_vy',
        ])

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            VehicleOpticalFlow, '/fmu/out/vehicle_optical_flow',
            self._on_flow, qos)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._on_position, qos)
        self.create_subscription(
            DistanceSensor, '/fmu/out/distance_sensor',
            self._on_distance, qos)
        self._pub = self.create_publisher(
            Float32MultiArray, '/optical_flow/velocity', 10)
        self.get_logger().info(f'writing {self._output}')

    def _on_position(self, msg):
        self._vx = float(msg.vx)
        self._vy = float(msg.vy)

    def _on_distance(self, msg):
        if math.isfinite(msg.current_distance) and msg.current_distance > 0.01:
            self._distance = float(msg.current_distance)

    def _on_flow(self, msg):
        dt = float(msg.integration_timespan_us) * 1e-6
        if dt <= 0.0:
            return
        flow_x = float(msg.pixel_flow[0])
        flow_y = float(msg.pixel_flow[1])
        distance = float(msg.distance_m)
        if not math.isfinite(distance) or distance <= 0.01:
            distance = self._distance
        if not math.isfinite(distance) or distance <= 0.01:
            return

        flow_vx = flow_x / dt * distance
        flow_vy = flow_y / dt * distance

        self._writer.writerow([
            f'{msg.timestamp * 1e-6:.6f}', f'{flow_x:.8f}', f'{flow_y:.8f}',
            f'{dt:.6f}', f'{distance:.4f}', int(msg.quality),
            f'{self._vx:.4f}', f'{self._vy:.4f}',
            f'{flow_vx:.4f}', f'{flow_vy:.4f}',
        ])
        self._rows += 1
        if self._rows % self._flush_every == 0:
            self._file.flush()

        out = Float32MultiArray()
        out.data = [flow_vx, flow_vy, self._vx, self._vy, distance]
        self._pub.publish(out)

    def destroy_node(self):
        try:
            self._file.flush()
            self._file.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OpticalFlowLogger()
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
