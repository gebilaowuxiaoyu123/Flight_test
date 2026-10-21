#!/usr/bin/env bash
# 一键启动完整演示：Agent + PX4/Gazebo + 深度桥 + 障碍距离 + 光流日志 + Web UI。
# 重复运行是安全的：已存在的 tmux 会话会跳过。
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

require_cmd tmux
ros_setup
workspace_setup

ROS_ENV="source /opt/ros/${ROS_DISTRO}/setup.bash && source ${WORKSPACE_ROOT}/install/setup.bash"

start_session() {
  local name="$1" cmd="$2"
  if tmux has-session -t "${name}" 2>/dev/null; then
    log "tmux 会话 ${name} 已存在，跳过"
  else
    tmux new-session -d -s "${name}" "${cmd}"
    log "已启动 tmux 会话 ${name}"
  fi
}

start_session agent "MicroXRCEAgent udp4 -p 8888 > /tmp/flight_agent.log 2>&1"
start_session sitl "cd ${PROJECT_ROOT} && HEADLESS=1 ./scripts/run_sitl_combined.sh > /tmp/flight_sitl.log 2>&1"
log "等待 PX4/Gazebo 启动 ..."
sleep 8

# 冷启动偶发：gz_bridge 抢在模型传感器之前订阅，IMU 只传一帧，
# PX4 会卡在 "ekf2 missing data"。检测到就自动重启一次 SITL。
if ! ./scripts/wait_ready.sh 45; then
  warn "SITL 未就绪，自动重启一次 ..."
  tmux kill-session -t sitl 2>/dev/null
  PIDS="$(ps -eo pid,cmd | awk '/px4_sitl_default\/bin\/px4|gz sim/ && !/awk/ {print $1}')"
  for pid in ${PIDS}; do kill -9 "${pid}" 2>/dev/null; done
  sleep 3
  start_session sitl "cd ${PROJECT_ROOT} && HEADLESS=1 ./scripts/run_sitl_combined.sh > /tmp/flight_sitl.log 2>&1"
  sleep 12
  ./scripts/wait_ready.sh 60 || warn "仍未就绪，请查看 /tmp/flight_sitl.log"
fi

start_session bridge "${ROS_ENV} && ros2 run ros_gz_bridge parameter_bridge \
  /depth_camera@sensor_msgs/msg/Image@gz.msgs.Image \
  /depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked \
  /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo > /tmp/flight_bridge.log 2>&1"
start_session depth "${ROS_ENV} && ros2 run flight_perception min_depth --ros-args -p image_topic:=/depth_camera > /tmp/flight_depth.log 2>&1"
start_session flowlog "${ROS_ENV} && ros2 run flight_perception optical_flow_logger > /tmp/flight_flowlog.log 2>&1"
start_session ui "${ROS_ENV} && ros2 run flight_ui ui_server > /tmp/flight_ui.log 2>&1"

log "等待 ROS 话题 ..."
sleep 5
if curl -fsS --max-time 3 http://127.0.0.1:8000/health >/dev/null; then
  log "Web UI: http://127.0.0.1:8000/"
else
  warn "Web UI 还没就绪，查看 /tmp/flight_ui.log"
fi
log '停止全部：for s in agent sitl bridge depth flowlog ui; do tmux kill-session -t "$s"; done'
