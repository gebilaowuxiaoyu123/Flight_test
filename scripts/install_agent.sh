#!/usr/bin/env bash
# 编译并安装 Micro-XRCE-DDS-Agent（PX4 <-> ROS 2 的 DDS 桥）。
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

require_cmd git
AGENT_DIR="${WORKSPACE_ROOT}/Micro-XRCE-DDS-Agent"

if [[ ! -d "${AGENT_DIR}/.git" ]]; then
  log "克隆 Micro-XRCE-DDS-Agent"
  git clone "${XRCE_AGENT_REPO}" "${AGENT_DIR}"
fi

git -C "${AGENT_DIR}" fetch origin
git -C "${AGENT_DIR}" checkout "${XRCE_AGENT_REF}"
git -C "${AGENT_DIR}" pull --ff-only

log "安装编译依赖"
sudo apt-get install -y cmake build-essential libssl-dev libasio-dev

JOBS_N="$(jobs_count)"
log "编译 Micro-XRCE-DDS-Agent（并行度 ${JOBS_N}）"
cmake -S "${AGENT_DIR}" -B "${AGENT_DIR}/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "${AGENT_DIR}/build" -j "${JOBS_N}"

log "安装到 /usr/local（需要 sudo）"
sudo cmake --install "${AGENT_DIR}/build"
sudo ldconfig

command -v MicroXRCEAgent >/dev/null 2>&1 \
  || die "MicroXRCEAgent 未找到，请检查安装路径"

log "完成：$(command -v MicroXRCEAgent)"
log "运行：MicroXRCEAgent udp4 -p 8888"
