"""Thin adapter between the UI and px4_msgs.

This is the only module in flight_ui that should know about px4_msgs field
names.  If PX4 changes a message, fix it here.
"""

import threading
import time

from px4_msgs.msg import (ManualControlSetpoint, TimesyncStatus,
                          VehicleCommand, VehicleLocalPosition)
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

MANUAL_CONTROL_TOPIC = '/fmu/in/manual_control_input'
VEHICLE_COMMAND_TOPIC = '/fmu/in/vehicle_command'

MODE_IDS = {
    'MANUAL': 1,
    'ALTCTL': 2,
    'POSCTL': 3,
    'AUTO': 4,
    'ACRO': 5,
    'OFFBOARD': 6,
    'STABILIZED': 7,
}

VEHICLE_CMD_NAV_LAND = 21
VEHICLE_CMD_NAV_TAKEOFF = 22
VEHICLE_CMD_DO_SET_MODE = 176
VEHICLE_CMD_DO_FLIGHTTERMINATION = 185
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400


def clamp(value, low=-1.0, high=1.0):
    return max(low, min(high, float(value)))


class Px4Adapter:
    def __init__(self, node):
        self._node = node
        self._lock = threading.Lock()

        # PX4 uXRCE examples use default (reliable) QoS for /fmu/in.
        qos_in = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._manual_pub = node.create_publisher(
            ManualControlSetpoint, MANUAL_CONTROL_TOPIC, qos_in)
        self._command_pub = node.create_publisher(
            VehicleCommand, VEHICLE_COMMAND_TOPIC, qos_in)

        # PX4 validates manual input timestamps against its own boot time.
        # ROS wall-clock microseconds are in the future from PX4's point of
        # view, so track the PX4 clock from timesync_status and interpolate.
        self._px4_time_us = None
        self._px4_recv_monotonic = 0.0
        self._ref_alt = None
        self._z = 0.0
        qos_telem = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        node.create_subscription(
            TimesyncStatus, '/fmu/out/timesync_status',
            self._on_timesync, qos_telem)
        node.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._on_local_position, qos_telem)

    def _on_timesync(self, msg):
        if msg.timestamp > 0:
            self._px4_time_us = int(msg.timestamp)
            self._px4_recv_monotonic = time.monotonic()

    def _on_local_position(self, msg):
        # PX4 NAV_TAKEOFF.param7 is an absolute (AMSL) altitude, not a
        # height above home.  SITL's home altitude is not necessarily zero,
        # so convert the requested relative height with the EKF reference.
        if msg.ref_alt != 0.0:
            with self._lock:
                self._ref_alt = float(msg.ref_alt)
                self._z = float(msg.z)

    def now_us(self):
        if self._px4_time_us is None:
            # Before the first timesync sample, use ROS time as a fallback.
            return int(self._node.get_clock().now().nanoseconds // 1000)
        elapsed = int((time.monotonic() - self._px4_recv_monotonic) * 1e6)
        # Keep the sample slightly in PX4's past: the selector rejects
        # samples whose timestamp_sample is newer than its own clock.
        return self._px4_time_us + elapsed - 100_000

    def publish_manual(self, roll, pitch, yaw, throttle, valid, deadman):
        msg = ManualControlSetpoint()
        msg.timestamp = self.now_us()
        msg.timestamp_sample = msg.timestamp
        msg.valid = bool(valid and deadman)
        msg.data_source = ManualControlSetpoint.SOURCE_MAVLINK_0
        msg.roll = clamp(roll)
        msg.pitch = clamp(pitch)
        msg.yaw = clamp(yaw)
        msg.throttle = clamp(throttle)
        msg.flaps = 0.0
        for name in ('aux1', 'aux2', 'aux3', 'aux4', 'aux5', 'aux6'):
            setattr(msg, name, 0.0)
        msg.sticks_moving = any(
            abs(v) > 0.01 for v in (msg.roll, msg.pitch, msg.yaw, msg.throttle))
        msg.buttons = 0
        with self._lock:
            self._manual_pub.publish(msg)

    def send_command(self, command, param1=0.0, param2=0.0, param3=0.0,
                     param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.now_us()
        msg.command = int(command)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.confirmation = 0
        msg.from_external = True
        with self._lock:
            self._command_pub.publish(msg)

    def arm(self, arm=True):
        self.send_command(
            VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0 if arm else 0.0,
        )

    def set_mode(self, mode):
        mode_id = MODE_IDS.get(str(mode).upper())
        if mode_id is None:
            raise ValueError(f'unknown mode: {mode}')
        # param1=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, param2=PX4 main mode
        self.send_command(
            VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=float(mode_id),
            param3=0.0,
        )

    def land(self):
        self.send_command(VEHICLE_CMD_NAV_LAND)

    def takeoff(self, altitude_m=2.5):
        with self._lock:
            ref_alt = self._ref_alt
            z = self._z
        # param7 is AMSL: current global altitude + requested relative height.
        target_alt = (ref_alt - z + float(altitude_m)
                      if ref_alt is not None else float(altitude_m))
        self.send_command(
            VEHICLE_CMD_NAV_TAKEOFF,
            param7=float(target_alt),
        )

    def kill(self):
        # Disarm and request flight termination; either one stops the vehicle.
        self.send_command(VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.send_command(VEHICLE_CMD_DO_FLIGHTTERMINATION, param1=1.0)
