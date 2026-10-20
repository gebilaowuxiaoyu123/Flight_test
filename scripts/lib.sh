#!/usr/bin/env bash
# 公共函数库。用法：source scripts/lib.sh，不要直接执行。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# shellcheck source=../versions.env
source "${PROJECT_ROOT}/versions.env"

if [[ -z "${WORKSPACE_ROOT}" ]]; then
  WORKSPACE_ROOT="${PROJECT_ROOT}"
fi
export PROJECT_ROOT WORKSPACE_ROOT

log()  { printf '\033[1;34m[%s]\033[0m %s\n' "$(date +%H:%M:%S)" "$*"; }
warn() { printf '\033[1;33m[WARN]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31m[ERROR]\033[0m %s\n' "$*" >&2; exit 1; }

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || die "缺少命令: $1"
}

ros_setup() {
  local setup="/opt/ros/${ROS_DISTRO}/setup.bash"
  [[ -f "${setup}" ]] || die "找不到 ${setup}，请先安装 ROS 2 ${ROS_DISTRO}"
  # shellcheck disable=SC1090
  source "${setup}"
}

workspace_setup() {
  local setup="${WORKSPACE_ROOT}/install/setup.bash"
  if [[ -f "${setup}" ]]; then
    # shellcheck disable=SC1090
    source "${setup}"
  fi
}

jobs_count() {
  if [[ "${JOBS}" == "auto" ]]; then
    nproc
  else
    echo "${JOBS}"
  fi
}

ensure_px4() {
  [[ -d "${WORKSPACE_ROOT}/PX4-Autopilot" ]] \
    || die "PX4-Autopilot 不存在，请先运行 scripts/install_px4.sh"
}
