#!/usr/bin/env bash
# 启动自定义 x500_flow_depth（光流 + 深度相机），依赖 px4_overlay/models。
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

ensure_px4
MODEL="${PX4_GZ_MODEL:-x500_flow_depth}"
MODEL_DIR="${PROJECT_ROOT}/px4_overlay/models/${MODEL}"

if [[ ! -d "${MODEL_DIR}" ]]; then
  die "找不到自定义模型 ${MODEL_DIR}，请先完成 docs/07-integration.md 的模型合并步骤"
fi

export GZ_SIM_RESOURCE_PATH="${PROJECT_ROOT}/px4_overlay/models:${GZ_SIM_RESOURCE_PATH:-}"
export PX4_GZ_MODEL="${MODEL}"
JOBS_N="$(jobs_count)"
export CMAKE_BUILD_PARALLEL_LEVEL="${JOBS_N}"

log "启动 ${MODEL}（airframe 仍用 gz_x500_flow 的光流/GPS 配置）..."
cd "${WORKSPACE_ROOT}/PX4-Autopilot"
exec make px4_sitl gz_x500_flow
