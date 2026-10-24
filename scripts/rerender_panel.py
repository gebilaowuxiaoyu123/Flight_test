#!/usr/bin/env python3
"""用飞行日志重新渲染下半屏面板，并覆盖到已录好的画面上。

这样调整面板排版/比例不需要重新飞一次（重飞有 EKF 发散风险）。
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import subprocess
import sys

from demo_show import FPS, PANEL_H, W, Controller, Overlay


class Snap:
    """把日志里的一行包装成渲染器需要的对象。"""

    __slots__ = ('x', 'y', 'z', 'vx', 'vy', 'vz', 'heading', 'roll', 'pitch',
                 'alt', 'x0', 'y0')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--video', required=True)
    ap.add_argument('--log', default='/tmp/flight_show_log.jsonl')
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    rows = [json.loads(line) for line in open(args.log)]
    t0 = rows[0]['t']
    x0 = min(r['x'] for r in rows[:200])
    y0 = min(r['y'] for r in rows[:200])
    # 起点的精确值用第一行
    x0, y0 = rows[0]['x'], rows[0]['y']

    ts = [r['t'] - t0 for r in rows]
    dur = ts[-1]
    n_frames = int(dur * FPS)

    # 姿态角由位置微分的粗略近似代替（日志没存四元数）
    def at(t):
        i = bisect.bisect_left(ts, t)
        i = max(0, min(len(rows) - 1, i))
        r = rows[i]
        s = Snap()
        s.x, s.y, s.alt = r['x'], r['y'], r['alt']
        s.z = -r['alt']
        # 用相邻样本估计速度
        j = min(len(rows) - 1, i + 1)
        dt = max(ts[j] - ts[i], 1e-3)
        s.vx = (rows[j]['x'] - r['x']) / dt
        s.vy = (rows[j]['y'] - r['y']) / dt
        s.vz = -(rows[j]['alt'] - r['alt']) / dt
        s.heading = math.atan2(s.vy, s.vx) if math.hypot(s.vx, s.vy) > 0.15 else 0.0
        # ALTCTL 里杆量就是姿态角命令，按满杆 40° 折算，比用速度反推可靠
        s.roll = r['roll'] * 0.70
        s.pitch = r['pitch'] * 0.70
        s.x0, s.y0 = x0, y0
        return s, r

    # 预处理路径（相对起点）
    path = []
    for r in rows:
        path.append((r['x'] - x0, r['y'] - y0))

    ov = Overlay()
    ctrl = Controller()

    ff = subprocess.Popen([
        'ffmpeg', '-loglevel', 'error', '-y',
        '-i', args.video,
        '-f', 'rawvideo', '-pix_fmt', 'rgb24',
        '-video_size', f'{W}x{PANEL_H}', '-framerate', str(FPS), '-i', 'pipe:0',
        '-filter_complex', '[0:v][1:v]overlay=0:720,format=yuv420p[v]',
        '-map', '[v]', '-c:v', 'libx264', '-preset', 'medium', '-crf', '20',
        '-movflags', '+faststart', args.out,
    ], stdin=subprocess.PIPE)

    stages = []
    for r in rows:
        if not stages or stages[-1] != r['stage']:
            stages.append(r['stage'])
    # 每个阶段的起止时间
    bounds = []
    for name in stages:
        idx = [i for i, r in enumerate(rows) if r['stage'] == name]
        bounds.append((name, ts[idx[0]], ts[idx[-1]]))

    cur_path = []
    pi = 0
    for f in range(n_frames):
        t = f / FPS
        s, r = at(t)
        while pi < len(path) and ts[pi] <= t:
            cur_path.append(path[pi])
            pi += 1

        stage, t_s0, t_s1 = bounds[0]
        si = 0
        for k, (nm, a, b) in enumerate(bounds):
            if a <= t <= b:
                stage, t_s0, t_s1, si = nm, a, b, k
                break

        target = (r['x'] - x0, r['y'] - y0)
        tgt_r = None
        for nm, a, b in bounds:
            if nm == stage:
                # 用该阶段最后一行作为目标近似
                idx = [i for i, rr in enumerate(rows) if rr['stage'] == nm]
                tgt_r = (rows[idx[-1]]['x'] - x0, rows[idx[-1]]['y'] - y0)
                break
        ctrl.tx = tgt_r[0] + x0 if tgt_r else s.x
        ctrl.ty = tgt_r[1] + y0 if tgt_r else s.y

        tel = {'armed': r['armed'], 'mode': r['mode'], 'alt': r['alt'],
               'battery': None}
        # 电量在日志里没有，用固定值占位（原视频里已有真实值，这里只显示文字）
        tel['battery'] = 99.0

        img = ov.render(s, ctrl, (r['roll'], r['pitch'], r['yaw'], r['throttle']),
                        tel, cur_path, stage, si, len(bounds), t,
                        t - t_s0, max(t_s1 - t_s0, 0.1), True)
        ff.stdin.write(img.tobytes())

    ff.stdin.close()
    ff.wait()
    print(f'重渲染完成 → {args.out}  ({n_frames} 帧)')


if __name__ == '__main__':
    sys.exit(main())
