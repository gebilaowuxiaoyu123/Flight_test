#!/usr/bin/env bash
# 克隆 PX4 v1.16.2 并安装依赖（不会编译）。
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

require_cmd git
PX4_DIR="${WORKSPACE_ROOT}/PX4-Autopilot"

# 只有这些子模块是 SITL + uXRCE + Gazebo 必需的。
# 不要用 --recursive：NuttX（~1.5GB）、Gazebo Classic、jsbsim、FlightGear、jMAVSim
# 对四旋翼 Gazebo 仿真完全用不上，递归克隆会卡几十分钟甚至超时。
SUBMODULES=(
  Tools/simulation/gz
  src/modules/mavlink/mavlink
  src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client
  src/drivers/gps/devices
  src/lib/crypto/libtomcrypt
  src/lib/crypto/libtommath
  src/lib/crypto/monocypher
  src/lib/events/libevents
  src/lib/heatshrink/heatshrink
)

init_submodules() {
  local dir="$1"
  git -C "${dir}" submodule update --init --depth 1 -- "${SUBMODULES[@]}"
}

if [[ ! -d "${PX4_DIR}/.git" ]]; then
  log "克隆 PX4 ${PX4_TAG} 到 ${PX4_DIR}（不递归）"
  git clone --branch "${PX4_TAG}" \
    https://github.com/PX4/PX4-Autopilot.git "${PX4_DIR}"
else
  log "PX4 已存在，切换到 ${PX4_TAG}"
  git -C "${PX4_DIR}" fetch --tags --force
  git -C "${PX4_DIR}" checkout "${PX4_TAG}"
fi

log "初始化 SITL 必需子模块（${#SUBMODULES[@]} 个，不含 NuttX/Classic）"
init_submodules "${PX4_DIR}" || warn "部分子模块下载失败，可重试 ./scripts/install_px4.sh"

log "安装 PX4 依赖（容错版：跳过 NuttX/Java，忽略无关 apt 源错误）"
bash "${PROJECT_ROOT}/scripts/install_deps.sh"

log "完成。首次编译："
log "  ./scripts/build_px4.sh gz_x500_flow"
