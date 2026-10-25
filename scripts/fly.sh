#!/usr/bin/env bash
# 终端飞飞机：把 ROS 话题的细节全部包起来，你只要敲几个词。
#
#   ./scripts/fly.sh start            起仿真 + 后端 + 摇杆守护
#   ./scripts/fly.sh stop             全部停掉
#   ./scripts/fly.sh status           看当前模式/解锁/高度
#   ./scripts/fly.sh watch            终端实时刷新姿态（模式/横滚/俯仰/航向/速度）
#   ./scripts/fly.sh mode ALTCTL      切模式
#   ./scripts/fly.sh arm              解锁
#   ./scripts/fly.sh takeoff          解锁并爬到 1.2 m
#   ./scripts/fly.sh up 2             爬升 2 秒（默认 2 秒）
#   ./scripts/fly.sh down 2           下降 2 秒
#   ./scripts/fly.sh fwd 3            前进 3 秒
#   ./scripts/fly.sh back 3           后退
#   ./scripts/fly.sh left 2           左移
#   ./scripts/fly.sh right 2          右移
#   ./scripts/fly.sh spin 2           左转
#   ./scripts/fly.sh land             降落并上锁
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

STICK=/tmp/stick.json
KEEPER_LOG=/tmp/stick_keeper.log

ros_setup
workspace_setup

# ---------------------------------------------------------------- 工具
pub_cmd() {  # pub_cmd <command> <param1> <param2>
  # 连发 3 次：ros2 topic pub 每次都要新建节点，DDS 发现没完成时 --once 会丢包
  ros2 topic pub --times 3 --rate 10 /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
"{command: $1, param1: $2, param2: $3, target_system: 1, target_component: 1, source_system: 1, source_component: 1, from_external: true}" \
    >/dev/null 2>&1
}

set_stick() {  # set_stick roll pitch yaw throttle
  printf '{"roll": %s, "pitch": %s, "yaw": %s, "throttle": %s}\n' "$1" "$2" "$3" "$4" > "${STICK}"
}

neutral() { set_stick 0 0 0 0; }

hold() {  # hold <秒> <roll> <pitch> <yaw> <throttle>：期间不断刷新，防止被判过期
  local left="$1"
  set_stick "$2" "$3" "$4" "$5"
  while [ "$(echo "${left} > 0" | bc)" = "1" ]; do
    set_stick "$2" "$3" "$4" "$5"
    sleep 0.2
    left=$(echo "${left} - 0.2" | bc)
  done
}

altitude() {
  timeout 6 ros2 topic echo /fmu/out/vehicle_local_position \
    --qos-reliability best_effort --once 2>/dev/null \
    | awk '/^z:/ {printf "%.2f", -$2; exit}'
}

nav_state() {
  timeout 6 ros2 topic echo /fmu/out/vehicle_status_v1 \
    --qos-reliability best_effort --once 2>/dev/null \
    | awk '/^nav_state:/ {print $2; exit}'
}

arming_state() {
  timeout 6 ros2 topic echo /fmu/out/vehicle_status_v1 \
    --qos-reliability best_effort --once 2>/dev/null \
    | awk '/^arming_state:/ {print $2; exit}'
}

need_keeper() {
  pgrep -f 'stick_keeper.py' >/dev/null 2>&1 \
    || die "摇杆守护没在跑，先执行: ./scripts/fly.sh start"
}

# ---------------------------------------------------------------- 动作
case "${1:-}" in

start)
  # 仿真
  if pgrep -f 'px4_sitl_default/bin/px4' >/dev/null 2>&1; then
    log "仿真已在运行"
  else
    log "启动 PX4 + Gazebo ..."
    tmux has-session -t agent 2>/dev/null || \
      tmux new-session -d -s agent "MicroXRCEAgent udp4 -p 8888 > /tmp/flight_agent.log 2>&1"
    tmux has-session -t sitl 2>/dev/null || \
      tmux new-session -d -s sitl "cd ${PROJECT_ROOT} && HEADLESS=1 ./scripts/run_sitl_combined.sh > /tmp/flight_sitl.log 2>&1"
    sleep 12
    ./scripts/wait_ready.sh 80 || die "SITL 未就绪，看 /tmp/flight_sitl.log"
  fi
  # 摇杆守护 + 遥测缓存：先清掉残留实例，避免旧进程写出"看似新鲜"的过期数据
  for p in $(ps -eo pid,cmd | awk '/stick_keeper\.py|telemetry_watch\.py/ && !/awk/ {print $1}'); do
    kill "$p" 2>/dev/null || true
  done
  sleep 1
  setsid python3 "${PROJECT_ROOT}/scripts/stick_keeper.py" > "${KEEPER_LOG}" 2>&1 &
  setsid python3 "${PROJECT_ROOT}/scripts/telemetry_watch.py" > /tmp/telemetry_watch.log 2>&1 &
  sleep 2
  log "摇杆守护(50 Hz) + 遥测缓存已启动"
  # 网页界面（可选，不影响终端飞）
  if curl -fsS --max-time 2 http://127.0.0.1:8000/health >/dev/null 2>&1; then
    log "后端已在运行"
  else
    setsid "${WORKSPACE_ROOT}/install/flight_ui_cpp/lib/flight_ui_cpp/rc_server" \
      > /tmp/rc_full.log 2>&1 &
    sleep 2
    log "后端已启动"
  fi
  log "就绪。试：./scripts/fly.sh status"
  ;;

stop)
  log "停止 ..."
  for p in $(ps -eo pid,cmd | awk '/stick_keeper\.py|flight_ui_cpp\/rc_server/ && !/awk/ {print $1}'); do
    kill "$p" 2>/dev/null || true
  done
  for s in agent sitl; do tmux kill-session -t "$s" 2>/dev/null || true; done
  for p in $(ps -eo pid,cmd | awk '/px4_sitl_default\/bin\/px4|gz sim/ && !/awk/ {print $1}'); do
    kill -9 "$p" 2>/dev/null || true
  done
  log "已停止"
  ;;

status)
  python3 "${PROJECT_ROOT}/scripts/show_status.py"
  ;;

watch)
  need_keeper
  exec python3 "${PROJECT_ROOT}/scripts/telemetry_watch.py" --print
  ;;

mode)
  want="${2:-ALTCTL}"
  case "${want^^}" in
    MANUAL) id=1;; ALTCTL) id=2;; POSCTL) id=3;; AUTO) id=4;;
    ACRO) id=5;; OFFBOARD) id=6;; STABILIZED) id=7;;
    *) die "未知模式: ${want}（可用 MANUAL/ALTCTL/POSCTL/AUTO/ACRO/OFFBOARD/STABILIZED）";;
  esac
  need_keeper
  set_stick 0 0 0 -1
  pub_cmd 176 1.0 "${id}"
  sleep 1.5
  ./scripts/fly.sh status
  ;;

arm)
  need_keeper
  set_stick 0 0 0 -1                  # 解锁必须油门最低
  sleep 0.3
  for _ in 1 2 3; do
    pub_cmd 400 1.0 0.0
    sleep 1.5
    [ "$(arming_state || echo 0)" = "2" ] && break
  done
  ./scripts/fly.sh status
  ;;

takeoff)
  target="${2:-1.2}"
  need_keeper
  set_stick 0 0 0 -1
  sleep 0.3
  for _ in 1 2 3; do
    pub_cmd 400 1.0 0.0
    sleep 1.5
    [ "$(arming_state || echo 0)" = "2" ] && break
  done
  [ "$(arming_state || echo 0)" = "2" ] || die "解锁失败，看 PX4 日志的 Preflight Fail"
  for _ in $(seq 1 40); do
    set_stick 0 0 0 0.45
    h=$(altitude)
    [ -n "${h}" ] && [ "$(echo "${h} >= ${target}" | bc)" = "1" ] && break
    sleep 0.3
  done
  neutral
  sleep 1
  ./scripts/fly.sh status
  ;;

up|down|fwd|back|left|right|spin)
  dur="${2:-2}"
  need_keeper
  case "$1" in
    up)    hold "${dur}" 0 0 0 0.40 ;;
    down)  hold "${dur}" 0 0 0 -0.40 ;;
    fwd)   hold "${dur}" 0 0.30 0 0 ;;
    back)  hold "${dur}" 0 -0.30 0 0 ;;
    left)  hold "${dur}" -0.30 0 0 0 ;;
    right) hold "${dur}" 0.30 0 0 0 ;;
    spin)  hold "${dur}" 0 0 0.40 0 ;;
  esac
  neutral
  sleep 1
  ./scripts/fly.sh status
  ;;

land)
  need_keeper
  set_stick 0 0 0 -0.45
  sleep 1
  for _ in $(seq 1 30); do
    h=$(altitude); [ -n "${h}" ] && [ "$(echo "${h} <= 0.15" | bc)" = "1" ] && break
    sleep 0.5
  done
  set_stick 0 0 0 -1
  sleep 1.5
  for _ in 1 2 3; do
    pub_cmd 400 0.0 0.0
    sleep 1
    [ "$(arming_state || echo 0)" = "1" ] && break
  done
  ./scripts/fly.sh status
  ;;

*)
  sed -n '3,19p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
  ;;
esac
