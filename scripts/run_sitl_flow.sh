#!/usr/bin/env bash
# 启动 PX4 SITL + Gazebo Harmonic 的 x500_flow（下视光流 + LW20 测距，GPS 关闭）。
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

ensure_px4
JOBS_N="$(jobs_count)"
export CMAKE_BUILD_PARALLEL_LEVEL="${JOBS_N}"

if [[ "${HEADLESS:-0}" == "1" ]]; then
  log "HEADLESS=1：不启动 Gazebo GUI"
fi

log "启动 gz_x500_flow ..."
cd "${WORKSPACE_ROOT}/PX4-Autopilot"
exec make px4_sitl gz_x500_flow
