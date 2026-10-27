#!/usr/bin/env bash
# 终端飞飞机 + 一键拉起各个控制台。把 ROS 话题的细节全部包起来，你只要敲几个词。
#
# ---- 四个终端（每个前台跑一路，日志直接打在终端里，Ctrl-C 只停这一个）----
#   ./scripts/fly.sh link             ① 通信连接：MicroXRCEAgent + 节点清单/话题频率
#   ./scripts/fly.sh sim              ② 仿真：PX4 + Gazebo（默认带 3D 窗口）
#   ./scripts/fly.sh remote           ③ 网页遥控：rc_server 日志 + 自动开浏览器
#   ./scripts/fly.sh views            ④ 三视角：第一人称 / 第三人称 / 俯视 三个窗口
#
# ---- 一把梭（全部都丢后台 tmux，适合无人值守）----
#   ./scripts/fly.sh start            起仿真 + 后端 + 摇杆守护
#   ./scripts/fly.sh stop             全部停掉
#
# ---- 飞 ----
#   ./scripts/fly.sh status           看当前模式/解锁/高度
#   ./scripts/fly.sh watch            终端实时刷新姿态（模式/横滚/俯仰/航向/速度）
#   ./scripts/fly.sh gui              开 Gazebo 3D 窗口 + 浏览器（想亲眼看时用）
#   ./scripts/fly.sh logs <名字>      看后台日志：agent / sitl / stick
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

pause_sticks() {  # 暂停发杆：让 PX4 自己降落，不被杆量变动打断
  printf '{"valid": false, "roll": 0, "pitch": 0, "yaw": 0, "throttle": 0}\n' > "${STICK}"
}

hold() {  # hold <秒> <roll> <pitch> <yaw> <throttle>：期间不断刷新，防止被判过期
  local left="$1"
  set_stick "$2" "$3" "$4" "$5"
  while [ "$(echo "${left} > 0" | bc)" = "1" ]; do
    set_stick "$2" "$3" "$4" "$5"
    sleep 0.2
    left=$(echo "${left} - 0.2" | bc)
  done
}

altitude() {  # 一次性读高度（约 1 s），不依赖任何常驻缓存
  # 只放行纯数字：调用方要拿它做 bc 比较，混进注释文字会让 bc 报 illegal character
  python3 "${PROJECT_ROOT}/scripts/tstatus.py" --alt --timeout 1.5 2>/dev/null \
    | awk 'NF == 1 && $1 ~ /^-?[0-9]+(\.[0-9]+)?$/ { v = $1 } END { if (v != "") print v }' || true
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

# ---------------------------------------------------------------- 前台控制台用的零件
# tmux 里跑东西要自己带环境，这里拼一份给 tmux 用
ROS_ENV="source /opt/ros/${ROS_DISTRO}/setup.bash && source ${WORKSPACE_ROOT}/install/setup.bash"

# 三路相机（和录制成片用的是同一批传感器）
CAM_TOPICS=(
  "/world/flow/model/x500_flow_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image"
  "/world/flow/model/cam_third/link/link/sensor/third_cam/image@sensor_msgs/msg/Image@gz.msgs.Image"
  "/world/flow/model/cam_top/link/link/sensor/top_cam/image@sensor_msgs/msg/Image@gz.msgs.Image"
)

# MicroXRCEAgent = PX4 和 ROS 2 之间的 DDS 桥。没有它，/fmu/* 话题一条都不存在，
# 网页上点 ARM/TAKEOFF 会像"没反应"一样（命令发出去没人收）。
ensure_agent() {
  if pgrep -f 'MicroXRCEAgent' >/dev/null 2>&1; then
    log "① 通信链路已在跑（MicroXRCEAgent）"
    return
  fi
  tmux has-session -t agent 2>/dev/null || \
    tmux new-session -d -s agent "MicroXRCEAgent udp4 -p 8888 > /tmp/flight_agent.log 2>&1"
  sleep 2
  if pgrep -f 'MicroXRCEAgent' >/dev/null 2>&1; then
    log "① 通信链路已自动启动（没跑 flight link 时兜底；想看日志用 flight link）"
  else
    warn "MicroXRCEAgent 起不来，看 /tmp/flight_agent.log（msg 会发不出去）"
  fi
}

open_browser() {
  if pgrep -f 'snap/firefox.*--no-remote' >/dev/null 2>&1; then
    log "浏览器已在运行"
    return
  fi
  local profile="${HOME}/snap/firefox/common/flight_show_profile"
  mkdir -p "${profile}"
  if ! grep -q 'network.proxy.type' "${profile}/user.js" 2>/dev/null; then
    cat >> "${profile}/user.js" <<'JS'
user_pref("browser.sessionstore.resume_from_crash", false);
user_pref("browser.startup.page", 1);
// 本机开着 127.0.0.1:7890 代理，Firefox 会把 127.0.0.1:8000 也丢给代理 → 502
user_pref("network.proxy.type", 0);
user_pref("network.proxy.no_proxies_on", "localhost, 127.0.0.1, ::1");
JS
  fi
  rm -f "${profile}"/sessionstore* "${profile}"/recovery.jsonlz4 2>/dev/null || true
  log "启动浏览器 → http://127.0.0.1:8000/"
  tmux kill-session -t web 2>/dev/null || true
  tmux new-session -d -s web \
    "DISPLAY=${FLY_DISPLAY:-:0} HOME=${HOME} \
     firefox --no-remote --new-instance --profile ${profile} \
     http://127.0.0.1:8000/ > /tmp/firefox.log 2>&1"
  sleep 20
}

raise_windows() {
  for name in "Gazebo Sim" "PX4 Web RC"; do
    local w
    # 末尾必须 || true：xdotool 会被 head 提前关管道打成 SIGPIPE(141)，
    # 在 set -e 下会让整个脚本静默退出（表现为"窗口没起来但也没报错"）
    w=$(DISPLAY=${FLY_DISPLAY:-:0} xdotool search --name "${name}" 2>/dev/null | head -1 || true)
    if [ -n "${w}" ]; then
      DISPLAY=${FLY_DISPLAY:-:0} xdotool windowactivate "${w}" 2>/dev/null || true
      DISPLAY=${FLY_DISPLAY:-:0} xdotool windowraise "${w}" 2>/dev/null || true
    fi
  done
}

# 停掉某类进程（按命令行正则匹配，不含自己）
kill_by_cmd() {
  local pat="$1" sig="${2:--TERM}" p
  for p in $(ps -eo pid,args | awk -v pat="${pat}" '$0 ~ pat && !/awk/ {print $1}'); do
    kill "${sig}" "${p}" 2>/dev/null || true
  done
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
  # 摇杆守护 + 遥测缓存。
  # 必须杀干净残留实例：仿真重启后旧实例的订阅已失效，它还占着单实例锁
  # 并不断写出过期数据（表现为 status 一直报"数据 N 秒前"）。
  tmux kill-session -t stick 2>/dev/null || true
  tmux kill-session -t tele  2>/dev/null || true
  rm -f /tmp/telemetry.json
  for p in $(ps -eo pid,args | awk '/scripts\/(stick_keeper|telemetry_watch)\.py/ && !/awk/ {print $1}'); do
    kill -9 "$p" 2>/dev/null || true
  done
  sleep 1
  # 用 tmux 托管：和 agent/sitl 一致，不会随终端退出被回收
  tmux new-session -d -s stick \
    "python3 ${PROJECT_ROOT}/scripts/stick_keeper.py > ${KEEPER_LOG} 2>&1"
  sleep 2
  if [ -z "$(python3 "${PROJECT_ROOT}/scripts/tstatus.py" --alt --timeout 1.5 2>/dev/null)" ]; then
    warn "收不到遥测，检查 tmux attach -t sitl 或 /tmp/flight_sitl.log"
  else
    log "摇杆守护(50 Hz) 已启动（tmux: stick）"
  fi
  # 网页界面（可选，不影响终端飞）
  if curl -fsS --noproxy "*" --max-time 2 http://127.0.0.1:8000/health >/dev/null 2>&1; then
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
  for s in stick tele; do tmux kill-session -t "$s" 2>/dev/null || true; done
  for p in $(ps -eo pid,args | awk '/scripts\/(stick_keeper|telemetry_watch|view3|link_monitor)\.py|flight_ui_cpp\/rc_server/ && !/awk/ {print $1}'); do
    kill "$p" 2>/dev/null || true
  done
  for s in agent sitl stick gzgui web cams; do tmux kill-session -t "$s" 2>/dev/null || true; done
  # flight link 是前台起的 agent，没有 tmux 会话，得按进程名收
  kill_by_cmd '[M]icroXRCEAgent'
  for p in $(ps -eo pid,args | awk '/gz sim -g/ && !/awk/ {print $1}'); do
    kill -9 "$p" 2>/dev/null || true
  done
  # 浏览器也关掉（想留着看网页可加 --keep-browser）
  if [ "${2:-}" != "--keep-browser" ]; then
    for p in $(ps -eo pid,args | awk '/snap\/firefox.*--no-remote/ && !/awk/ {print $1}'); do
      kill "$p" 2>/dev/null || true
    done
    for p in $(ps -eo pid,args | awk '/Xephyr :1/ && !/awk/ {print $1}'); do
      kill "$p" 2>/dev/null || true
    done
  fi
  for p in $(ps -eo pid,cmd | awk '/px4_sitl_default\/bin\/px4|gz sim/ && !/awk/ {print $1}'); do
    kill -9 "$p" 2>/dev/null || true
  done
  log "已停止"
  ;;

status)
  python3 "${PROJECT_ROOT}/scripts/tstatus.py"
  ;;

clean)
  # PX4 的 .ulg 黑匣子日志是这台机器掉盘的唯一大坑，一步清掉
  "${PROJECT_ROOT}/scripts/clean_logs.sh" "${@:2}"
  ;;

# ---------------------------------------------------------------- 前台控制台
# 每个终端一个前台进程，日志直接打在你眼前；Ctrl-C 只停这一个。

link)
  # 接管 8888：tmux 里如果已经起了 agent，先请它退出，否则端口被占起不来
  tmux kill-session -t agent 2>/dev/null || true
  kill_by_cmd '[M]icroXRCEAgent'
  sleep 1
  log "① 通信连接  MicroXRCEAgent udp4 :8888（verbose）"
  log "   下面持续打印 ROS 节点清单 + PX4 话题频率；Ctrl-C 停止"
  python3 "${PROJECT_ROOT}/scripts/link_monitor.py" &
  mon=$!
  trap 'kill '"${mon}"' 2>/dev/null || true' EXIT
  MicroXRCEAgent udp4 -p 8888 -v 4
  ;;

sim)
  ensure_agent
  tmux kill-session -t sitl 2>/dev/null || true
  kill_by_cmd '[p]x4_sitl_default/bin/px4' -9
  kill_by_cmd '[g]z sim' -9
  sleep 2
  log "② 仿真  PX4 + Gazebo  世界 flow / 模型 x500_flow_depth"
  if [ "${HEADLESS:-0}" = "1" ]; then
    log "   HEADLESS=1（不开 3D 窗口，录片时用）"
  else
    log "   HEADLESS=0（带 Gazebo 3D 窗口）"
  fi
  HEADLESS="${HEADLESS:-0}" exec "${PROJECT_ROOT}/scripts/run_sitl_combined.sh"
  ;;

remote)
  # 网页遥控：后端日志直接打在这个终端，后端通了再开浏览器
  kill_by_cmd '[f]light_ui_cpp/rc_server'
  # rc_server 和 stick_keeper 都往 /fmu/in/manual_control_input 发 50 Hz 杆量，
  # 同时跑会互相覆盖（表现为网页打杆飞机不动）。网页接管时必须停掉守护。
  if pgrep -f 'stick_keeper.py' >/dev/null 2>&1; then
    tmux kill-session -t stick 2>/dev/null || true
    kill_by_cmd '[s]tick_keeper.py'
    log "   已停掉终端摇杆守护 → 控制权交给网页（要回终端飞：flight stop && flight start）"
  fi
  sleep 1
  ensure_agent
  log "③ 网页遥控  rc_server :8000"
  ros2 launch flight_ui remote.launch.py &
  launch_pid=$!
  trap 'kill "${launch_pid}" 2>/dev/null || true' EXIT
  for _ in $(seq 1 30); do
    if curl -fsS --noproxy '*' --max-time 1 http://127.0.0.1:8000/health >/dev/null 2>&1; then
      log "   后端已就绪 http://127.0.0.1:8000/"
      break
    fi
    sleep 1
  done
  open_browser
  raise_windows
  wait "${launch_pid}"
  ;;

views)
  if pgrep -f 'parameter_bridge.*IMX214' >/dev/null 2>&1; then
    log "④ 相机桥已在运行"
  else
    tmux kill-session -t cams 2>/dev/null || true
    tmux new-session -d -s cams \
      "${ROS_ENV} && ros2 run ros_gz_bridge parameter_bridge ${CAM_TOPICS[*]} 2>&1 | tee /tmp/cams.log"
    log "④ 相机桥已启动（tmux: cams）"
    sleep 5
  fi
  log "   打开三视角窗口：第一人称 / 第三人称 / 俯视"
  exec python3 "${PROJECT_ROOT}/scripts/view3.py" "${@:2}"
  ;;

gui)
  # 窗口进程必须用 tmux 托管：用 setsid & 起的进程会随命令结束被回收，
  # 表现为"窗口一闪就没了"。
  if pgrep -f 'gz sim -g' >/dev/null 2>&1; then
    log "Gazebo 3D 窗口已在运行"
  else
    log "启动 Gazebo 3D 窗口 ..."
    tmux kill-session -t gzgui 2>/dev/null || true
    tmux new-session -d -s gzgui \
      "DISPLAY=${FLY_DISPLAY:-:0} __GLX_VENDOR_LIBRARY_NAME=nvidia \
       __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json \
       GZ_SIM_RESOURCE_PATH=${PROJECT_ROOT}/px4_overlay/models:${PROJECT_ROOT}/PX4-Autopilot/Tools/simulation/gz/models \
       gz sim -g > /tmp/gzgui.log 2>&1"
    sleep 28
  fi

  if pgrep -f 'snap/firefox.*--no-remote' >/dev/null 2>&1; then
    log "浏览器已在运行"
  else
    open_browser
  fi

  raise_windows

  if pgrep -f 'gz sim -g' >/dev/null 2>&1 && pgrep -f 'snap/firefox.*--no-remote' >/dev/null 2>&1; then
    log "两个窗口都已就绪（Gazebo 3D + 浏览器）"
  else
    warn "有窗口没起来，看: tmux attach -t gzgui / tmux attach -t web"
  fi
  ;;

logs)
  name="${2:-}"
  case "${name}" in
    agent) tmux attach -t agent ;;
    sitl)  tmux attach -t sitl ;;
    stick) tmux attach -t stick ;;
    *)     log "用法: ./scripts/fly.sh logs agent|sitl|stick"; log "或看文件: /tmp/flight_agent.log /tmp/flight_sitl.log /tmp/stick_keeper.log" ;;
  esac
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
  for _ in $(seq 1 8); do        # 冷启动 EKF 收敛要十几秒，多试几次
    pub_cmd 400 1.0 0.0
    sleep 1.5
    if [ "$(arming_state || echo 0)" = "2" ]; then break; fi
  done
  ./scripts/fly.sh status
  ;;

takeoff)
  target="${2:-1.2}"
  need_keeper
  set_stick 0 0 0 -1
  sleep 0.3
  for _ in $(seq 1 8); do        # 冷启动 EKF 收敛要十几秒，多试几次
    pub_cmd 400 1.0 0.0
    sleep 1.5
    if [ "$(arming_state || echo 0)" = "2" ]; then break; fi
  done
  if [ "$(arming_state || echo 0)" != "2" ]; then
    warn "解锁被拒，PX4 给的原因："
    grep -E 'Preflight Fail|Arming denied' /tmp/flight_sitl.log 2>/dev/null | tail -3 | sed 's/^/    /'
    warn "常见：机身在倾斜/翻转状态（撞过机）→ 需要 ./scripts/fly.sh stop && ./scripts/fly.sh start"
    exit 1
  fi
  for _ in $(seq 1 40); do
    set_stick 0 0 0 0.45
    h=$(altitude)
    if [ -n "${h}" ] && [ "$(echo "${h} >= ${target}" | bc)" = "1" ]; then break; fi
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
    fwd)   hold "${dur}" 0 0.30 0 0.12 ;;
    back)  hold "${dur}" 0 -0.30 0 0.12 ;;
    left)  hold "${dur}" -0.30 0 0 0.10 ;;
    right) hold "${dur}" 0.30 0 0 0.10 ;;
    spin)  hold "${dur}" 0 0 0.40 0 ;;
  esac
  neutral
  sleep 1
  ./scripts/fly.sh status
  ;;

land)
  need_keeper
  # 用 PX4 自带的 NAV_LAND(21)：它自己管下降曲线和落地检测，
  # 比"手动压油门到底"稳得多（后者触地后会把飞机弹起来，导致上锁被拒）。
  pause_sticks          # 降落期间别打杆，否则 PX4 会退出 AUTO_LAND
  pub_cmd 21 0 0
  for _ in $(seq 1 40); do
    sleep 1
    a=$(python3 "${PROJECT_ROOT}/scripts/tstatus.py" --armed --timeout 1.2 2>/dev/null || echo -1)
    if [ "${a}" = "0" ]; then break; fi
  done
  a=$(python3 "${PROJECT_ROOT}/scripts/tstatus.py" --armed --timeout 1.2 2>/dev/null || echo -1)
  if [ "${a}" != "0" ]; then
    h=$(altitude)
    if [ -n "${h}" ] && [ "$(echo "${h} < 0.6" | bc)" = "1" ]; then
      # 无 GPS 时高度有漂移，PX4 的落地检测可能永远不触发。
      # 低空用 force disarm（param2=21196）兜底，高度 <0.6 m 才允许。
      pub_cmd 400 0.0 21196.0
      sleep 1
    else
      warn "仍在 ${h} m 空中，不做强制上锁"
    fi
  fi
  ./scripts/fly.sh status
  ;;

*)
  sed -n '3,19p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
  ;;
esac
