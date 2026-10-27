#!/usr/bin/env bash
# 启动自定义 x500_flow_depth（光流 + 深度相机），依赖 px4_overlay/models。
#
# PX4 v1.16 起 PX4_GZ_MODEL 已废弃，模型由 PX4_SIM_MODEL=gz_<model> 选择；
# PX4 只会到 <PX4>/Tools/simulation/gz/models/<model>/model.sdf 查找模型，
# 因此这里把 overlay 模型软链进 PX4 模型目录，再直接启动 px4 可执行文件。
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

ensure_px4
MODEL="${PX4_GZ_MODEL:-${MODEL_NAME:-x500_flow_depth}}"
MODEL_DIR="${PROJECT_ROOT}/px4_overlay/models/${MODEL}"
PX4_DIR="${WORKSPACE_ROOT}/PX4-Autopilot"
PX4_BIN="${PX4_DIR}/build/px4_sitl_default/bin/px4"
GZ_BRIDGE_DIR="${PX4_DIR}/build/px4_sitl_default/src/modules/simulation/gz_bridge"

[[ -d "${MODEL_DIR}" ]] \
  || die "找不到自定义模型 ${MODEL_DIR}，请先完成 docs/07-integration.md 的模型合并步骤"
[[ -x "${PX4_BIN}" ]] \
  || die "找不到 ${PX4_BIN}，请先运行 ./scripts/build_px4.sh gz_x500_flow"

# PX4 的 px4-rc.gzsim 用 ${PX4_GZ_MODELS}/<model>/model.sdf 生成 spawn 请求，
# 所以 overlay 模型必须出现在 PX4 自己的模型目录中（软链即可）。
ln -sfn "${MODEL_DIR}" "${PX4_DIR}/Tools/simulation/gz/models/${MODEL}"
# flow 世界（棋盘格 + 深度墙）也放进 PX4 的 world 搜索路径。
ln -sfn "${PROJECT_ROOT}/px4_overlay/worlds/flow.sdf" \
  "${PX4_DIR}/Tools/simulation/gz/worlds/flow.sdf"

export PX4_SIM_MODEL="gz_${MODEL}"
export PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-4021}"
export PX4_GZ_WORLD="${PX4_GZ_WORLD:-flow}"
export HEADLESS="${HEADLESS:-1}"
# SITL without a real RC/GCS: disable the GCS-link arming requirement and
# bypass the power/USB/safety-button checks.  These can also be set at the
# pxh> prompt, but env vars make every restart reproducible.
export PX4_PARAM_NAV_DLL_ACT=0
export PX4_PARAM_NAV_RCL_ACT=0
export PX4_PARAM_COM_RC_IN_MODE="${PX4_PARAM_COM_RC_IN_MODE:-2}"
export PX4_PARAM_CBRK_SUPPLY_CHK=894281
export PX4_PARAM_CBRK_USB_CHK=197848
export PX4_PARAM_CBRK_IO_SAFETY=22027
# 4021 has no GPS.  Use barometric height and the downward rangefinder;
# keeping RNG_CTRL enabled also gives optical flow the metric scale it needs.
export PX4_PARAM_EKF2_HGT_REF=0
export PX4_PARAM_EKF2_BARO_CTRL=1
export PX4_PARAM_EKF2_RNG_CTRL=1
# Keep the SITL battery from draining in seconds and triggering a low-battery failsafe.
export PX4_PARAM_SIM_BAT_DRAIN=3600
export PX4_PARAM_COM_LOW_BAT_ACT=0
export PX4_PARAM_BAT_LOW_THR=0.05
export PX4_PARAM_COM_DISARM_LAND=0
export PX4_PARAM_COM_DISARM_PRFLT=0
# PX4 会以 100 Hz 写 .ulg 到 build/px4_sitl_default/rootfs/log/，只要 SITL 不关就一直涨
# （实测跑 8 小时 ≈ 9.6 GB，曾把根分区吃到剩 3.7 G）。SITL 不需要黑匣子，默认关掉。
# 真要留日志：PX4_PARAM_SDLOG_MODE=0 ./scripts/fly.sh sim
export PX4_PARAM_SDLOG_MODE="${PX4_PARAM_SDLOG_MODE:--1}"
export GZ_SIM_RESOURCE_PATH="${PROJECT_ROOT}/px4_overlay/models:${GZ_SIM_RESOURCE_PATH:-}"

log "启动 ${MODEL}（airframe ${PX4_SYS_AUTOSTART}，HEADLESS=${HEADLESS}）..."
if [ "${PX4_PARAM_SDLOG_MODE}" = "-1" ]; then
  log "ulog 黑匣子已关闭（想开：PX4_PARAM_SDLOG_MODE=0 ./scripts/fly.sh sim）"
fi
cd "${GZ_BRIDGE_DIR}"
exec "${PX4_BIN}"
