"""FastAPI + rclpy Web RC transmitter."""

import argparse
import asyncio
import threading
import time
from contextlib import asynccontextmanager
from pathlib import Path

import rclpy
import uvicorn
from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .px4_adapter import Px4Adapter
from .telemetry import Telemetry

WEB_DIR = Path(get_package_share_directory('flight_ui')) / 'web'


class RosWorker:
    def __init__(self):
        self._lock = threading.Lock()
        self._sticks = {
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
            'throttle': -1.0,
            'deadman': False,
        }
        self._last_update = 0.0
        self._node = None
        self._executor = None
        self._thread = None
        self._telemetry = None
        self._adapter = None
        self.ok = False

    def start(self):
        rclpy.init()
        self._node = Node('flight_ui')
        self._adapter = Px4Adapter(self._node)
        self._telemetry = Telemetry(self._node)
        self._node.create_timer(0.02, self._tick)
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()
        self.ok = True

    def _spin(self):
        try:
            self._executor.spin()
        except Exception:  # pragma: no cover - shutdown path
            pass

    def stop(self):
        self.ok = False
        if self._executor is not None:
            self._executor.shutdown()
        if self._node is not None:
            self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def set_sticks(self, roll, pitch, yaw, throttle, deadman):
        with self._lock:
            self._sticks.update({
                'roll': float(roll),
                'pitch': float(pitch),
                'yaw': float(yaw),
                'throttle': float(throttle),
                'deadman': bool(deadman),
            })
            self._last_update = time.monotonic()

    def _tick(self):
        now = time.monotonic()
        with self._lock:
            sticks = dict(self._sticks)
            stale = (now - self._last_update) > 0.3
        deadman = sticks['deadman'] and not stale
        if deadman:
            roll, pitch, yaw = sticks['roll'], sticks['pitch'], sticks['yaw']
            throttle = sticks['throttle']
        else:
            roll = pitch = yaw = 0.0
            throttle = -1.0
        self._adapter.publish_manual(
            roll, pitch, yaw, throttle, valid=not stale, deadman=deadman)

    def telemetry(self):
        return self._telemetry.snapshot() if self._telemetry else {}

    def arm(self):
        with self._lock:
            sticks = dict(self._sticks)
        if not sticks['deadman']:
            return False, 'deadman not held'
        if sticks['throttle'] > -0.8:
            return False, 'throttle must be at minimum'
        self._adapter.arm(True)
        return True, 'arm command sent'

    def disarm(self):
        self._adapter.arm(False)
        return True, 'disarm command sent'

    def set_mode(self, mode):
        try:
            self._adapter.set_mode(mode)
        except ValueError as exc:
            return False, str(exc)
        return True, f'mode {mode} requested'

    def takeoff(self, altitude=2.5):
        with self._lock:
            sticks = dict(self._sticks)
        if not sticks['deadman']:
            return False, 'deadman not held'
        if sticks['throttle'] > -0.8:
            return False, 'throttle must be at minimum'
        # PX4 rejects a NAV_TAKEOFF sent in the same cycle as ARM/mode change.
        # Arm, wait for the armed state, switch to POSCTL, then request takeoff.
        threading.Thread(
            target=self._takeoff_sequence, args=(float(altitude),),
            daemon=True).start()
        return True, f'takeoff to {altitude} m requested'

    def _takeoff_sequence(self, altitude):
        self._adapter.arm(True)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if self.telemetry().get('armed'):
                break
            time.sleep(0.1)
        else:
            return
        time.sleep(0.3)
        self._adapter.set_mode('POSCTL')
        time.sleep(0.8)
        self._adapter.takeoff(altitude)

    def land(self):
        self._adapter.land()
        return True, 'land command sent'

    def kill(self):
        self._adapter.kill()
        with self._lock:
            self._sticks['deadman'] = False
            self._sticks['throttle'] = -1.0
        return True, 'kill command sent'


worker = RosWorker()


@asynccontextmanager
async def lifespan(_app):
    worker.start()
    try:
        yield
    finally:
        worker.stop()


app = FastAPI(title='flight_ui', lifespan=lifespan)
# --symlink-install creates symlinks into the source tree; Starlette refuses
# to follow them unless follow_symlink is enabled.
app.mount('/static', StaticFiles(directory=str(WEB_DIR), follow_symlink=True), name='static')


@app.get('/health')
async def health():
    return {'status': 'ok', 'ros': worker.ok, 'telemetry': worker.telemetry()}


@app.get('/')
async def index():
    return FileResponse(WEB_DIR / 'index.html')


@app.websocket('/ws')
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()

    async def sender():
        while True:
            await ws.send_json({'type': 'telemetry', **worker.telemetry()})
            await asyncio.sleep(0.1)

    sender_task = asyncio.create_task(sender())
    try:
        while True:
            data = await ws.receive_json()
            kind = data.get('type')
            if kind == 'sticks':
                worker.set_sticks(
                    data.get('roll', 0.0),
                    data.get('pitch', 0.0),
                    data.get('yaw', 0.0),
                    data.get('throttle', -1.0),
                    data.get('deadman', False),
                )
            elif kind == 'arm':
                ok, message = worker.arm()
                await ws.send_json({'type': 'ack', 'action': 'arm',
                                    'ok': ok, 'message': message})
            elif kind == 'disarm':
                ok, message = worker.disarm()
                await ws.send_json({'type': 'ack', 'action': 'disarm',
                                    'ok': ok, 'message': message})
            elif kind == 'mode':
                ok, message = worker.set_mode(data.get('mode', 'POSCTL'))
                await ws.send_json({'type': 'ack', 'action': 'mode',
                                    'ok': ok, 'message': message})
            elif kind == 'takeoff':
                ok, message = worker.takeoff(data.get('altitude', 2.5))
                await ws.send_json({'type': 'ack', 'action': 'takeoff',
                                    'ok': ok, 'message': message})
            elif kind == 'land':
                ok, message = worker.land()
                await ws.send_json({'type': 'ack', 'action': 'land',
                                    'ok': ok, 'message': message})
            elif kind == 'kill':
                ok, message = worker.kill()
                await ws.send_json({'type': 'ack', 'action': 'kill',
                                    'ok': ok, 'message': message})
    except WebSocketDisconnect:
        pass
    finally:
        sender_task.cancel()


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8000)
    parsed, _ = parser.parse_known_args(args)
    uvicorn.run(app, host=parsed.host, port=parsed.port, log_level='info')


if __name__ == '__main__':
    main()
