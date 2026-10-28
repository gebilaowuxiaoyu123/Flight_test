#!/usr/bin/env bash
# 一条命令拉起"演示录制"需要的全部服务，然后（可选）直接开始录制。
#
#   ./scripts/run_show.sh                        # 只起服务，不录（自己手动跑 demo_show.py）
#   ./scripts/run_show.sh --record               # 起服务 + 录一条 180 s 成片（含深度/光流画中画）
#   ./scripts/run_show.sh --record --sensors none    # 只录三视角，不要深度/光流
#   ./scripts/run_show.sh --record --sensors depth   # 只加深度相机
#   ./scripts/run_show.sh --stop                 # 停掉全部
#
# 和 run_demo.sh 的区别：run_demo.sh 起的是"Python 版 Web 遥控器 + 感知节点"，
# 本脚本起的是"C++ 后端 + 录制用的三路相机 + 跟拍相机"，专为出片服务。
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

require_cmd tmux
ros_setup
workspace_setup

ROS_ENV="source /opt/ros/${ROS_DISTRO}/setup.bash && source ${WORKSPACE_ROOT}/install/setup.bash"
WORLD=flow
MODEL=x500_flow_depth_0
XDISPLAY=:1
# 传感器画中画：depth,flow / none / depth / flow
#   none → 只录三视角（跟拍 + 第一视角 + 俯视），最省 CPU
SENSORS="depth,flow"
# 视频编码器：auto=有 GPU 就用 NVENC（省 7 倍 CPU），x264=强制软编
ENCODER="auto"

_args=()
while [ $# -gt 0 ]; do
  case "$1" in
    --sensors) SENSORS="${2:-depth,flow}"; shift 2 ;;
    --encoder) ENCODER="${2:-auto}"; shift 2 ;;
    --sensors=*) SENSORS="${1#*=}"; shift ;;
    *) _args+=("$1"); shift ;;
  esac
done
set -- ${_args[@]+"${_args[@]}"}
log "传感器画中画：${SENSORS}（none=只录三视角）"
OUT="${WORKSPACE_ROOT}/artifacts/flight_show_final.mp4"

# ---------------------------------------------------------------- 停止
stop_all() {
  log "停止全部演示服务 ..."
  tmux kill-session -t show-ui 2>/dev/null || true
  for p in $(ps -eo pid,cmd | awk '/cam_follow\.py|pose_writer\.py|flight_ui_cpp\/rc_server/ && !/awk/ {print $1}'); do
    kill "$p" 2>/dev/null || true
  done
  # 只杀我们起的那个 Xephyr（:1），别动用户桌面
  for p in $(ps -eo pid,cmd | awk '/Xephyr :1/ && !/awk/ {print $1}'); do
    kill "$p" 2>/dev/null || true
  done
  log "完成（仿真本体请用 run_demo.sh 的停止方法）"
}

if [[ "${1:-}" == "--stop" ]]; then
  stop_all
  exit 0
fi

# ---------------------------------------------------------------- 依赖检查
[[ -x "${WORKSPACE_ROOT}/install/flight_ui_cpp/lib/flight_ui_cpp/rc_server" ]] \
  || die "rc_server 未编译，先执行: colcon build --packages-select flight_ui_cpp"
require_cmd Xephyr
require_cmd xdotool
require_cmd ffmpeg

# ---------------------------------------------------------------- 1. 仿真本体
if ! pgrep -f 'px4_sitl_default/bin/px4' >/dev/null 2>&1; then
  log "仿真未运行，先起 PX4 + Gazebo（HEADLESS=1，图形走相机不走 GUI）..."
  tmux has-session -t agent 2>/dev/null \
    || tmux new-session -d -s agent "MicroXRCEAgent udp4 -p 8888 > /tmp/flight_agent.log 2>&1"
  tmux has-session -t sitl 2>/dev/null \
    || tmux new-session -d -s sitl \
         "cd ${PROJECT_ROOT} && HEADLESS=1 ./scripts/run_sitl_combined.sh > /tmp/flight_sitl.log 2>&1"
  sleep 12
  ./scripts/wait_ready.sh 80 || die "SITL 未就绪，看 /tmp/flight_sitl.log"
else
  log "仿真已在运行，跳过启动"
fi

# ---------------------------------------------------------------- 2. C++ 后端
for p in $(ps -eo pid,cmd | awk '/flight_ui_cpp\/rc_server/ && !/awk/ {print $1}'); do
  kill "$p" 2>/dev/null || true
done
sleep 1
setsid "${WORKSPACE_ROOT}/install/flight_ui_cpp/lib/flight_ui_cpp/rc_server" \
  > /tmp/rc_full.log 2>&1 &
log "C++ 后端 :8000 已启动"

# ---------------------------------------------------------------- 3. 三路相机桥
tmux has-session -t cams 2>/dev/null && tmux kill-session -t cams
# 相机桥：跟拍 / 第一视角 / 俯视 必建；下视(光流) 和 深度 按 --sensors 决定
CAM_TOPICS=(
  "/world/${WORLD}/model/${MODEL}/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image"
  "/world/${WORLD}/model/cam_third/link/link/sensor/third_cam/image@sensor_msgs/msg/Image@gz.msgs.Image"
  "/world/${WORLD}/model/cam_top/link/link/sensor/top_cam/image@sensor_msgs/msg/Image@gz.msgs.Image"
)
case "${SENSORS}" in
  *flow*) CAM_TOPICS+=("/world/${WORLD}/model/${MODEL}/link/camera_link/sensor/down_cam/image@sensor_msgs/msg/Image@gz.msgs.Image") ;;
esac
case "${SENSORS}" in
  *depth*) CAM_TOPICS+=("/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image") ;;
esac
tmux new-session -d -s cams "${ROS_ENV} && ros2 run ros_gz_bridge parameter_bridge ${CAM_TOPICS[*]} > /tmp/cams.log 2>&1"
log "相机桥已启动（${#CAM_TOPICS[@]} 路）"

# ---------------------------------------------------------------- 4. 真实位姿 + 相机动捕
# gz 的 Python 绑定与 ROS 2 的 protobuf 版本冲突，必须走纯 Python 实现
for p in $(ps -eo pid,cmd | awk '/cam_follow\.py|pose_writer\.py/ && !/awk/ {print $1}'); do
  kill "$p" 2>/dev/null || true
done
sleep 1
setsid env PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python \
  python3 "${PROJECT_ROOT}/scripts/pose_writer.py" > /tmp/posewriter.log 2>&1 &
sleep 3
setsid env PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python \
  python3 "${PROJECT_ROOT}/scripts/cam_follow.py" > /tmp/camfollow.log 2>&1 &
log "相机动捕已启动"

# ---------------------------------------------------------------- 5. 嵌套 X + 浏览器
# 关键：网页必须跑在 Xephyr 里，否则桌面一锁屏，x11grab 抓到的就是一整片黑
if ! DISPLAY=${XDISPLAY} xdpyinfo >/dev/null 2>&1; then
  setsid Xephyr ${XDISPLAY} -screen 1280x800 -ac +extension GLX \
    -noreset -no-host-grab > /tmp/xephyr.log 2>&1 &
  sleep 4
fi
PROFILE="${HOME}/snap/firefox/common/flight_show_profile"
mkdir -p "${PROFILE}"
[[ -f "${PROFILE}/user.js" ]] || cat > "${PROFILE}/user.js" <<'JS'
user_pref("browser.shell.checkDefaultBrowser", false);
user_pref("browser.startup.homepage_override.mstone", "ignore");
user_pref("datareporting.policy.dataSubmissionEnabled", false);
user_pref("toolkit.telemetry.enabled", false);
user_pref("browser.aboutwelcome.enabled", false);
JS

tmux has-session -t show-ui 2>/dev/null && tmux kill-session -t show-ui
tmux new-session -d -s show-ui "DISPLAY=${XDISPLAY} HOME=${HOME} \
  firefox --no-remote --new-instance --profile '${PROFILE}' \
  http://127.0.0.1:8000/ > /tmp/firefox_show.log 2>&1"
log "等待浏览器就绪 ..."
for _ in $(seq 1 30); do
  WIN=$(DISPLAY=${XDISPLAY} xdotool search --name "PX4 Web RC" 2>/dev/null | tail -1 || true)
  [[ -n "${WIN}" ]] && break
  sleep 2
done
[[ -n "${WIN:-}" ]] || die "浏览器窗口未出现，看 /tmp/firefox_show.log"
DISPLAY=${XDISPLAY} xdotool windowsize "${WIN}" 640 720
sleep 1
DISPLAY=${XDISPLAY} xdotool windowmove "${WIN}" 0 0
log "网页已就位"

# ---------------------------------------------------------------- 6. 就绪检查
sleep 3
curl -fsS --max-time 5 http://127.0.0.1:8000/health >/dev/null || die "后端 /health 不通"
log "全部就绪"

# ---------------------------------------------------------------- 7. 录制
if [[ "${1:-}" == "--record" ]]; then
  log "开始录制 → ${OUT}（约 180 s，期间请勿锁屏/切换 Xephyr 窗口大小）"
  mkdir -p "${WORKSPACE_ROOT}/artifacts"
  DISPLAY=${XDISPLAY} python3 -u "${PROJECT_ROOT}/scripts/demo_show.py" \
    --record "${OUT}" --grab ":1.0+0,0" --sensors "${SENSORS}" --encoder "${ENCODER}"
  log "录制完成: ${OUT}"
else
  log "就绪：浏览器看 http://127.0.0.1:8000/ ；录片用 ./scripts/run_show.sh --record"
fi
