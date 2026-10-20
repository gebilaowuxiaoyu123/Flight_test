#!/usr/bin/env bash
# 克隆 PX4 v1.16.2 并安装依赖（不会编译）。
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

require_cmd git
PX4_DIR="${WORKSPACE_ROOT}/PX4-Autopilot"

if [[ ! -d "${PX4_DIR}/.git" ]]; then
  log "克隆 PX4 ${PX4_TAG} 到 ${PX4_DIR}"
  git clone --recursive --branch "${PX4_TAG}" \
    https://github.com/PX4/PX4-Autopilot.git "${PX4_DIR}"
else
  log "PX4 已存在，切换到 ${PX4_TAG}"
  git -C "${PX4_DIR}" fetch --tags --force
  git -C "${PX4_DIR}" checkout "${PX4_TAG}"
  git -C "${PX4_DIR}" submodule update --init --recursive
fi

log "安装 PX4 依赖（容错版：跳过 NuttX/Java，忽略无关 apt 源错误）"
bash "${PROJECT_ROOT}/scripts/install_deps.sh"

log "完成。首次编译："
log "  ./scripts/build_px4.sh gz_x500_flow"
