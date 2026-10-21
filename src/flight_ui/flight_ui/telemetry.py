"""PX4 telemetry aggregation for the web HUD."""

import threading

from px4_msgs.msg import (BatteryStatus, VehicleLandDetected,
                          VehicleLocalPosition, VehicleStatus)
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

NAV_STATE_NAMES = {
    0: 'MANUAL',
    1: 'ALTCTL',
    2: 'POSCTL',
    3: 'AUTO_MISSION',
    4: 'AUTO_LOITER',
    5: 'AUTO_RTL',
    6: 'POSITION_SLOW',
    10: 'ACRO',
    12: 'DESCEND',
    13: 'TERMINATION',
    14: 'OFFBOARD',
    15: 'STABILIZED',
    17: 'AUTO_TAKEOFF',
    18: 'AUTO_LAND',
    19: 'AUTO_FOLLOW',
    20: 'AUTO_PRECLAND',
    21: 'ORBIT',
    22: 'AUTO_VTOL_TAKEOFF',
}


class Telemetry:
    def __init__(self, node):
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._lock = threading.Lock()
        self._state = {
            'armed': False,
            'mode': 'UNKNOWN',
            'nav_state': -1,
            'alt': 0.0,
            'vx': 0.0,
            'vy': 0.0,
            'vz': 0.0,
            'battery': -1.0,
            'voltage': 0.0,
            'xy_valid': False,
            'z_valid': False,
            'landed': True,
        }

        # PX4 appends the uORB message version when MESSAGE_VERSION > 0.
        # v1.16 publishes VehicleStatus on /fmu/out/vehicle_status_v1.
        node.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self._on_status, qos)
        node.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self._on_status, qos)
        node.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._on_position, qos)
        node.create_subscription(
            BatteryStatus, '/fmu/out/battery_status', self._on_battery, qos)
        node.create_subscription(
            VehicleLandDetected, '/fmu/out/vehicle_land_detected',
            self._on_landed, qos)

    def _on_status(self, msg):
        with self._lock:
            self._state['armed'] = (msg.arming_state == 2)
            self._state['nav_state'] = int(msg.nav_state)
            self._state['mode'] = NAV_STATE_NAMES.get(
                int(msg.nav_state), f'STATE_{msg.nav_state}')

    def _on_position(self, msg):
        with self._lock:
            self._state['xy_valid'] = bool(msg.xy_valid)
            self._state['z_valid'] = bool(msg.z_valid)
            self._state['alt'] = float(-msg.z) if msg.z_valid else 0.0
            self._state['vx'] = float(msg.vx)
            self._state['vy'] = float(msg.vy)
            self._state['vz'] = float(msg.vz)

    def _on_battery(self, msg):
        with self._lock:
            self._state['voltage'] = float(msg.voltage_v)
            if msg.connected and msg.remaining >= 0.0:
                self._state['battery'] = float(msg.remaining * 100.0)
            else:
                self._state['battery'] = -1.0

    def _on_landed(self, msg):
        with self._lock:
            self._state['landed'] = bool(msg.landed)

    def snapshot(self):
        with self._lock:
            return dict(self._state)
