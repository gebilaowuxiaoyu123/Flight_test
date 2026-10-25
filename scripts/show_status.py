#!/usr/bin/env python3
"""一行式状态输出，读 /tmp/telemetry.json（由 telemetry_watch.py 维护）。

不直接调 ros2 CLI，是因为每次调用要启动一次 ROS 2 节点，耗时 1~2 s；
连续调几次飞控状态命令就会明显卡顿。
"""
import json
import sys
import time

GREEN, RED, CYAN, GREY, RESET = '\033[92m', '\033[91m', '\033[96m', '\033[90m', '\033[0m'

try:
    with open('/tmp/telemetry.json') as fh:
        d = json.load(fh)
except Exception:
    print('  （还没有遥测数据，先跑 ./scripts/fly.sh start）')
    sys.exit(0)

age = time.time() - d.get('t', 0)
stale = ''
if age > 2.0:
    stale = f'  {GREY}(数据 {age:.0f}s 前，守护可能已停){RESET}'

armed = f'{GREEN}ARMED   {RESET}' if d['armed'] else f'{RED}DISARMED{RESET}'
print(f"  模式 {CYAN}{d['mode']:<13}{RESET} 状态 {armed} "
      f"高度 {d['alt']:6.2f} m  横滚 {d['roll_deg']:+5.1f}°  "
      f"俯仰 {d['pitch_deg']:+5.1f}°  航向 {d['heading_deg']:3.0f}°{stale}")
