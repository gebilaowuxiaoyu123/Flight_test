#!/usr/bin/env bash
# 拉取 ROS 2 依赖源码：px4_msgs 与 ros_gz。
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

require_cmd git
mkdir -p "${PROJECT_ROOT}/src"

# ---- px4_msgs ----
if [[ ! -d "${PROJECT_ROOT}/src/px4_msgs/.git" ]]; then
  log "克隆 px4_msgs（${PX4_MSGS_BRANCH}）"
  git clone --branch "${PX4_MSGS_BRANCH}" \
    https://github.com/PX4/px4_msgs.git "${PROJECT_ROOT}/src/px4_msgs"
else
  log "更新 px4_msgs（${PX4_MSGS_BRANCH}）"
  git -C "${PROJECT_ROOT}/src/px4_msgs" fetch origin
  git -C "${PROJECT_ROOT}/src/px4_msgs" checkout "${PX4_MSGS_BRANCH}"
  git -C "${PROJECT_ROOT}/src/px4_msgs" pull --ff-only
fi

# ---- ros_gz ----
if [[ ! -d "${PROJECT_ROOT}/src/ros_gz/.git" ]]; then
  log "克隆 ros_gz（${ROS_GZ_BRANCH}）"
  git clone --branch "${ROS_GZ_BRANCH}" \
    https://github.com/gazebosim/ros_gz.git "${PROJECT_ROOT}/src/ros_gz"
else
  log "更新 ros_gz（${ROS_GZ_BRANCH}）"
  git -C "${PROJECT_ROOT}/src/ros_gz" fetch origin
  git -C "${PROJECT_ROOT}/src/ros_gz" checkout "${ROS_GZ_BRANCH}"
  git -C "${PROJECT_ROOT}/src/ros_gz" pull --ff-only
fi

log "完成。下一步：GZ_VERSION=${GZ_VERSION} ./scripts/build_ros.sh"
