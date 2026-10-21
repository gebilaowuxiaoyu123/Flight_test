#!/usr/bin/env bash
# 等待 SITL 真正可用：IMU 数据在流 + EKF 有有效高度估计。
#
# 为什么需要它：冷启动时 Gazebo 有时会在模型传感器就绪前接受 gz_bridge 的订阅，
# IMU 只传一帧就停，PX4 会一直停在 "ekf2 missing data"，解锁必然失败。
# 用本脚本可以区分“还在初始化”和“假启动”。
#
# 用法：./scripts/wait_ready.sh [超时秒数]
# 退出码：0 就绪 / 1 超时
set -uo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"
set +e

TIMEOUT="${1:-60}"
ros_setup
workspace_setup

imu_ok() {
  local out rate
  out="$(timeout 6 ros2 topic hz /fmu/out/sensor_combined 2>/dev/null \
        | grep 'average rate' | tail -1)"
  rate="${out##*average rate: }"
  rate="${rate%% *}"
  [[ -n "${rate}" ]] && awk -v r="${rate}" 'BEGIN { exit !(r > 50) }'
}

ekf_ok() {
  timeout 4 ros2 topic echo /fmu/out/vehicle_local_position --once \
    --qos-reliability best_effort 2>/dev/null | grep -q 'z_valid: true'
}

deadline=$((SECONDS + TIMEOUT))
while ((SECONDS < deadline)); do
  if imu_ok && ekf_ok; then
    log "PX4 已就绪（IMU 在流、EKF 高度有效）"
    exit 0
  fi
  sleep 2
done

warn "等待 ${TIMEOUT}s 仍未就绪。多半是冷启动时 IMU 只传了一帧。"
warn "处理：tmux kill-session -t sitl && ./scripts/run_demo.sh"
warn "自检：PX4 控制台执行 listener sensor_combined，若 timestamp 是几十秒前即可确认。"
exit 1
