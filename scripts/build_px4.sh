#!/usr/bin/env bash
# 编译 PX4 SITL 目标。
# 用法：./scripts/build_px4.sh [gz_x500_flow|gz_x500_depth|gz_x500|...]
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

TARGET="${1:-gz_x500_flow}"
ensure_px4

# 重放本项目对 PX4 的最小补丁。补丁已应用时 git apply 会失败，用 --reverse 判断。
PATCH="${PROJECT_ROOT}/px4_overlay/px4_patches/gz_bridge_downward_lidar.patch"
if [[ -f "${PATCH}" ]]; then
  if git -C "${WORKSPACE_ROOT}/PX4-Autopilot" apply --check "${PATCH}" 2>/dev/null; then
    git -C "${WORKSPACE_ROOT}/PX4-Autopilot" apply "${PATCH}"
    log "已应用 gz_bridge 下视激光补丁"
  elif git -C "${WORKSPACE_ROOT}/PX4-Autopilot" apply --reverse --check "${PATCH}" 2>/dev/null; then
    log "gz_bridge 下视激光补丁已存在，跳过"
  else
    warn "gz_bridge 补丁无法应用；请检查 PX4 版本是否仍为 ${PX4_TAG}"
  fi
fi
TOPICS="${PROJECT_ROOT}/px4_overlay/px4_patches/dds_topics.yaml"
if [[ -f "${TOPICS}" ]]; then
  cp "${TOPICS}" "${WORKSPACE_ROOT}/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml"
  log "已同步 uXRCE dds_topics.yaml（光流/测距转发）"
fi

JOBS_N="$(jobs_count)"
export CMAKE_BUILD_PARALLEL_LEVEL="${JOBS_N}"

log "编译 PX4 SITL：px4_sitl ${TARGET}（并行度 ${JOBS_N}）"
cd "${WORKSPACE_ROOT}/PX4-Autopilot"
make px4_sitl "${TARGET}"

log "完成：build/px4_sitl_default 下的 ${TARGET} 可执行文件已生成"
