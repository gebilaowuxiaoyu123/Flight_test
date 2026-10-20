#!/usr/bin/env bash
# 环境与链路的冒烟测试。任何阶段都可以运行；未安装/未启动的项记为 SKIP。
set -uo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"
set +e

PASS=0
FAIL=0
SKIP=0

pass() { printf '\033[1;32m[PASS]\033[0m %s\n' "$*"; PASS=$((PASS + 1)); }
fail() { printf '\033[1;31m[FAIL]\033[0m %s\n' "$*"; FAIL=$((FAIL + 1)); }
skip() { printf '\033[1;33m[SKIP]\033[0m %s\n' "$*"; SKIP=$((SKIP + 1)); }

echo "== 版本与源码 =="
if [[ -f "${PROJECT_ROOT}/versions.env" ]]; then
  pass "versions.env 存在（PX4_TAG=${PX4_TAG}, ROS_DISTRO=${ROS_DISTRO}, GZ_VERSION=${GZ_VERSION}）"
else
  fail "versions.env 缺失"
fi

echo
echo "== Gazebo =="
if command -v gz >/dev/null 2>&1; then
  GZ_VER="$(timeout 5 gz sim --version 2>/dev/null | head -1)"
  if [[ "${GZ_VER}" == *"version 8."* ]]; then
    pass "Gazebo Harmonic：${GZ_VER}"
  else
    fail "gz 已安装但不是 8.x：${GZ_VER:-无法读取版本}"
  fi
else
  skip "Gazebo 未安装 → ./scripts/install_gazebo.sh"
fi

echo
echo "== PX4 =="
if [[ -d "${WORKSPACE_ROOT}/PX4-Autopilot/.git" ]]; then
  PX4_DESC="$(git -C "${WORKSPACE_ROOT}/PX4-Autopilot" describe --tags --always 2>/dev/null)"
  if [[ "${PX4_DESC}" == "${PX4_TAG}"* ]]; then
    pass "PX4 ${PX4_DESC}"
  else
    fail "PX4 版本不匹配：期望 ${PX4_TAG}，实际 ${PX4_DESC}"
  fi
else
  skip "PX4 未安装 → ./scripts/install_px4.sh"
fi

echo
echo "== ROS 2 =="
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f "${ROS_SETUP}" ]]; then
  pass "ROS 2 ${ROS_DISTRO} 已安装"
else
  fail "找不到 ${ROS_SETUP}"
fi

if command -v MicroXRCEAgent >/dev/null 2>&1; then
  pass "MicroXRCEAgent：$(command -v MicroXRCEAgent)"
else
  skip "MicroXRCEAgent 未安装 → ./scripts/install_agent.sh"
fi

if [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  if (source "${ROS_SETUP}"; source "${WORKSPACE_ROOT}/install/setup.bash"; \
      ros2 pkg prefix px4_msgs >/dev/null 2>&1); then
    pass "ROS 工作区可用（px4_msgs）"
  else
    fail "install/ 存在，但 px4_msgs 不可用；重新运行 ./scripts/build_ros.sh"
  fi
else
  skip "ROS 工作区未编译 → ./scripts/fetch_deps.sh && ./scripts/build_ros.sh"
fi

echo
echo "== 运行时链路 =="
TOPICS=""
if [[ -f "${ROS_SETUP}" && -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  TOPICS="$(source "${ROS_SETUP}"; source "${WORKSPACE_ROOT}/install/setup.bash"; \
            timeout 5 ros2 topic list 2>/dev/null)"
fi

if [[ -n "${TOPICS}" ]] && grep -qx '/fmu/out/vehicle_status' <<<"${TOPICS}"; then
  pass "PX4 遥测话题在线（/fmu/out/vehicle_status）"
else
  skip "未检测到 PX4 遥测话题（SITL 未运行或 uXRCE 未连接）"
fi

if [[ -n "${TOPICS}" ]] && grep -qx '/fmu/out/vehicle_optical_flow' <<<"${TOPICS}"; then
  pass "光流话题在线（/fmu/out/vehicle_optical_flow）"
else
  skip "未检测到光流话题（运行 gz_x500_flow 后再试）"
fi

GZ_TOPICS="$(timeout 5 gz topic -l 2>/dev/null)"
if [[ -n "${GZ_TOPICS}" ]] && grep -qi 'depth_camera' <<<"${GZ_TOPICS}"; then
  pass "Gazebo 深度相机话题在线"
else
  skip "未检测到 Gazebo 深度相机话题（运行 gz_x500_depth 后再试）"
fi

if curl -fsS --max-time 2 http://127.0.0.1:8000/health >/dev/null 2>&1; then
  pass "Web 遥控器在线（:8000/health）"
else
  skip "Web 遥控器未运行（P5 完成后用 ./scripts/run_ui.sh）"
fi

echo
echo "== 汇总 =="
printf 'PASS=%d  FAIL=%d  SKIP=%d\n' "${PASS}" "${FAIL}" "${SKIP}"

if ((FAIL > 0)); then
  echo
  echo "建议的下一步："
  command -v gz >/dev/null 2>&1 || echo "  ./scripts/install_gazebo.sh"
  [[ -d "${WORKSPACE_ROOT}/PX4-Autopilot" ]] || echo "  ./scripts/install_px4.sh"
  command -v MicroXRCEAgent >/dev/null 2>&1 || echo "  ./scripts/install_agent.sh"
  [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]] || echo "  ./scripts/fetch_deps.sh && ./scripts/build_ros.sh"
fi

[[ "${FAIL}" -eq 0 ]]
