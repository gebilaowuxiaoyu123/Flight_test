#!/usr/bin/env python3
"""端到端验收脚本：通过 WebSocket 遥控 PX4，自动完成 解锁 -> 起飞 -> 降落 -> 上锁。

用法：
    python3 scripts/ws_smoke_test.py            # 默认 http://127.0.0.1:8000
    python3 scripts/ws_smoke_test.py --url ws://127.0.0.1:8000/ws --altitude 2.0

退出码 0 表示全流程通过；非 0 表示某一步失败（会打印最后一条遥测）。
依赖：python3 -m pip install --user websockets
"""

import argparse
import asyncio
import json
import sys

import websockets

STICK_INTERVAL = 0.05
STICK_TIMEOUT = 5.0


class SmokeTest:
    def __init__(self, url, altitude, timeout):
        self.url = url
        self.altitude = altitude
        self.timeout = timeout
        self.state = {}
        self.acks = []

    async def _sticks(self, ws, throttle=-1.0, pitch=0.0, roll=0.0, yaw=0.0):
        """持续发送摇杆帧。PX4 在 300 ms 收不到输入后会失效。"""
        while True:
            await ws.send(json.dumps({
                'type': 'sticks',
                'roll': roll,
                'pitch': pitch,
                'yaw': yaw,
                'throttle': throttle,
                'deadman': True,
            }))
            await asyncio.sleep(STICK_INTERVAL)

    async def _receiver(self, ws):
        while True:
            msg = json.loads(await ws.recv())
            if msg.get('type') == 'telemetry':
                self.state.update(msg)
            elif msg.get('type') == 'ack':
                self.acks.append(msg)
                print(f"  ACK {msg.get('action')}: "
                      f"{'OK' if msg.get('ok') else 'REJECTED'} "
                      f"{msg.get('message', '')}", flush=True)

    async def _wait_for(self, predicate, label):
        deadline = asyncio.get_running_loop().time() + self.timeout
        while asyncio.get_running_loop().time() < deadline:
            if predicate(self.state):
                print(f"[PASS] {label}", flush=True)
                return True
            await asyncio.sleep(0.1)
        print(f"[FAIL] {label}（超时 {self.timeout:.0f}s）", flush=True)
        print(f"       最后遥测：{self.state}", flush=True)
        return False

    async def run(self):
        print(f"连接 {self.url} ...", flush=True)
        async with websockets.connect(self.url, max_size=None) as ws:
            sticks_task = asyncio.create_task(self._sticks(ws))
            recv_task = asyncio.create_task(self._receiver(ws))
            try:
                await asyncio.sleep(1.5)
                print(f"起始遥测：{self.state}", flush=True)

                if not await self._wait_for(
                        lambda s: 'armed' in s and 'mode' in s, 'Web 遥测已就绪'):
                    return 1

                await ws.send(json.dumps({'type': 'takeoff',
                                          'altitude': self.altitude}))
                if not await self._wait_for(lambda s: s.get('armed'),
                                            '解锁成功'):
                    return 1

                if not await self._wait_for(
                        lambda s: s.get('alt', 0) > self.altitude * 0.6,
                        f'爬升到约 {self.altitude} m'):
                    return 1

                await ws.send(json.dumps({'type': 'land'}))
                if not await self._wait_for(
                        lambda s: s.get('alt', 0) < 0.4, '降落到地面附近'):
                    return 1

                # PX4 会拒绝“还没落地”时的上锁请求，等 land detector 确认。
                if not await self._wait_for(lambda s: s.get('landed'),
                                            'PX4 检测到已落地'):
                    return 1

                await ws.send(json.dumps({'type': 'mode', 'mode': 'POSCTL'}))
                await asyncio.sleep(1.0)
                await ws.send(json.dumps({'type': 'disarm'}))
                await self._wait_for(lambda s: not s.get('armed'), '上锁成功')
                return 0
            finally:
                for task in (sticks_task, recv_task):
                    task.cancel()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--url', default='ws://127.0.0.1:8000/ws')
    parser.add_argument('--altitude', type=float, default=2.0)
    parser.add_argument('--timeout', type=float, default=20.0)
    args = parser.parse_args()

    test = SmokeTest(args.url, args.altitude, args.timeout)
    try:
        sys.exit(asyncio.run(test.run()))
    except OSError as exc:
        print(f'[FAIL] 连不上 Web UI：{exc}', file=sys.stderr)
        print('       先确认 ./scripts/run_demo.sh 已跑起来，端口 8000 可访问。',
              file=sys.stderr)
        sys.exit(2)


if __name__ == '__main__':
    main()
