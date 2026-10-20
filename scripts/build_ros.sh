#!/usr/bin/env bash
# 编译 ROS 2 工作区。
# 注意：ros_gz 在 Humble 上没有 Harmonic 二进制包，必须带 GZ_VERSION=harmonic 源码编译。
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

ros_setup
cd "${PROJECT_ROOT}"

if ! find src -mindepth 2 -maxdepth 2 -name package.xml -print -quit 2>/dev/null | grep -q .; then
  warn "src/ 下还没有 ROS 2 包。先运行：./scripts/fetch_deps.sh"
  exit 0
fi

export GZ_VERSION
log "GZ_VERSION=${GZ_VERSION}"

log "安装 rosdep 依赖（失败时请检查网络/rosdep init）"
rosdep install --from-paths src --ignore-src -r -y \
  || warn "rosdep 有缺失项，继续尝试编译；如失败见 docs/09-troubleshooting.md"

JOBS_N="$(jobs_count)"
COLCON_EXTRA=()
if [[ "${WORKSPACE_ROOT}" != "${PROJECT_ROOT}" ]]; then
  log "构建目录迁移到 WORKSPACE_ROOT=${WORKSPACE_ROOT}"
  COLCON_EXTRA+=(
    --build-base "${WORKSPACE_ROOT}/build"
    --install-base "${WORKSPACE_ROOT}/install"
    --log-base "${WORKSPACE_ROOT}/log"
  )
fi

# 只构建本项目需要的包及其依赖，避免把 ros_gz 的 demo 包全部编译一遍。
PACKAGES="${PACKAGES:-px4_msgs flight_ui flight_perception ros_gz_bridge}"
log "colcon build（并行度 ${JOBS_N}，包：${PACKAGES}）"
# shellcheck disable=SC2086
colcon build \
  --symlink-install \
  --parallel-workers "${JOBS_N}" \
  "${COLCON_EXTRA[@]}" \
  --packages-up-to ${PACKAGES} \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

log "完成。source install/setup.bash 后即可使用 px4_msgs / ros_gz。"
