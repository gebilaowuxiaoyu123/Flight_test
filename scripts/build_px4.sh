#!/usr/bin/env bash
# 编译 PX4 SITL 目标。
# 用法：./scripts/build_px4.sh [gz_x500_flow|gz_x500_depth|gz_x500|...]
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

TARGET="${1:-gz_x500_flow}"
ensure_px4

JOBS_N="$(jobs_count)"
export CMAKE_BUILD_PARALLEL_LEVEL="${JOBS_N}"

log "编译 PX4 SITL：px4_sitl ${TARGET}（并行度 ${JOBS_N}）"
cd "${WORKSPACE_ROOT}/PX4-Autopilot"
make px4_sitl "${TARGET}"

log "完成：build/px4_sitl_default 下的 ${TARGET} 可执行文件已生成"
