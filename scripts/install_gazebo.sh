#!/usr/bin/env bash
# 安装 Gazebo Harmonic（gz-sim8），供 PX4 v1.16+ 使用。
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

require_cmd lsb_release
require_cmd curl

if command -v gz >/dev/null 2>&1 && gz sim --version 2>/dev/null | grep -q 'version 8\.'; then
  log "Gazebo Harmonic 已安装：$(gz sim --version 2>/dev/null | head -1)"
  exit 0
fi

log "添加 OSRF Gazebo 软件源（$(lsb_release -cs)）"
sudo apt-get update
sudo apt-get install -y curl gnupg lsb-release
sudo install -d -m 0755 /usr/share/keyrings
curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
  | sudo tee /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg >/dev/null

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null

sudo apt-get update
log "安装 ${GZ_PKG} 与 libunwind-dev"
sudo apt-get install -y "${GZ_PKG}" libunwind-dev

if ! gz sim --version 2>/dev/null | grep -q 'version 8\.'; then
  die "Gazebo Harmonic 安装后仍未找到 gz sim 8.x，请检查 /etc/apt/sources.list.d/gazebo-stable.list"
fi

log "完成：$(gz sim --version 2>/dev/null | head -1)"
warn "如果 GUI 黑屏或使用核显，尝试："
warn "  __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia gz sim"
