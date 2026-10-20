#!/usr/bin/env bash
# 安装 Gazebo Harmonic + PX4 编译依赖。
#
# 与 PX4 自带 ubuntu.sh 的区别：
#   - 不因无关 apt 源（如 llvm-apt 缺 GPG key）而整体退出
#   - 跳过 NuttX、Java、jMAVSim
#   - 只安装 SITL 编译/运行需要的包
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

require_cmd lsb_release
require_cmd curl

SUDO=()
if [[ "${EUID}" -ne 0 ]]; then
  SUDO=(sudo)
fi

log "1/4 配置 OSRF Gazebo 源"
if command -v gz >/dev/null 2>&1 && gz sim --version 2>/dev/null | grep -q 'version 8\.'; then
  log "Gazebo Harmonic 已安装，跳过源配置"
else
  "${SUDO[@]}" install -d -m 0755 /usr/share/keyrings
  curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
    | "${SUDO[@]}" tee /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg >/dev/null
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | "${SUDO[@]}" tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
fi

log "2/4 apt-get update（无关源报错会继续）"
if ! "${SUDO[@]}" apt-get update -y --quiet; then
  warn "apt-get update 返回非 0，继续安装。"
  warn "常见原因：/etc/apt/sources.list.d/llvm-apt.list 缺 GPG key。"
  warn "彻底修复方法见 docs/09-troubleshooting.md。"
fi

log "3/4 安装系统依赖"
"${SUDO[@]}" env DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
  gz-harmonic libunwind-dev bc \
  build-essential cmake ninja-build git pkg-config file rsync unzip zip \
  libxml2-dev libxml2-utils libeigen3-dev libopencv-dev protobuf-compiler \
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-ugly gstreamer1.0-libav libgstreamer-plugins-base1.0-dev \
  libimage-exiftool-perl \
  python3 python3-dev python3-pip python3-setuptools python3-wheel

log "4/4 安装 PX4 Python 依赖"
REQ="${WORKSPACE_ROOT}/PX4-Autopilot/Tools/setup/requirements.txt"
python3 -m pip install --user 'empy>=3.3,<4' 'jinja2>=2.8' kconfiglib \
  || warn "关键 Python 依赖安装失败"

if [[ -f "${REQ}" ]]; then
  python3 -m pip install --user -r "${REQ}" \
    || warn "部分 Python 依赖安装失败，编译时再按报错补装"
else
  warn "找不到 ${REQ}，跳过完整 requirements"
fi

log "验证"
if ! command -v gz >/dev/null 2>&1; then
  die "gz 未安装成功；检查 /etc/apt/sources.list.d/gazebo-stable.list"
fi
gz sim --version | head -1

python3 - <<'EOF' || die "关键 Python 依赖缺失：jinja2 / empy / kconfiglib"
import jinja2
import em
import kconfiglib
print("python deps: ok")
EOF

log "完成。下一步："
log "  ./scripts/build_px4.sh gz_x500_flow"
