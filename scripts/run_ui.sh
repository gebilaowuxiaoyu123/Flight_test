#!/usr/bin/env bash
# 启动 Web 遥控器（P5 完成后可用）。
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

ros_setup
workspace_setup

if ! ros2 pkg prefix flight_ui >/dev/null 2>&1; then
  die "flight_ui 尚未编译。请先完成 P5，并运行 ./scripts/build_ros.sh"
fi

log "启动 Web 遥控器 ..."
exec ros2 launch flight_ui ui.launch.py
