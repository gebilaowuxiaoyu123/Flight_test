# flight_project 短命令定义（由 ~/.bashrc source）
#
# 这个文件会被反复 source（改完代码重开/重载都走它），所以每次都要能安全覆盖旧定义。

# 旧版本把 flight / remote 定义成 alias。bash 在解析 `flight() {` 时会先把
# 函数名当命令名做 alias 展开，于是变成 `ros2 launch ...() {` →
# 报 "未预期的记号 "(" 附近有语法错误"，而且后面的定义全部不执行。
# 所以这里必须先把残留 alias 清掉。
unalias flight remote fly tele flystatus 2>/dev/null
unset -f flight remote _flight_env 2>/dev/null

export FLIGHT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

_flight_env() {
  source /opt/ros/humble/setup.bash 2>/dev/null
  source "${FLIGHT_ROOT}/install/setup.bash" 2>/dev/null
}

# flight 三种用法：
#   flight                    起整条链路（ros2 launch，一把梭全后台）
#   flight link|sim|remote|views|start|...  等价于 fly <动作>，透传给 scripts/fly.sh
#   flight headless:=false    参数原样传给 ros2 launch
flight() {
  local sub="${1:-}"
  case "${sub}" in
    ''|launch)
      _flight_env
      ros2 launch flight_ui flight.launch.py "${@:2}"
      ;;
    link|sim|remote|views|perc|clean|start|stop|status|watch|gui|logs|mode|arm|takeoff|up|down|fwd|back|left|right|spin|land)
      "${FLIGHT_ROOT}/scripts/fly.sh" "$@"
      ;;
    *':='*)
      _flight_env
      ros2 launch flight_ui flight.launch.py "$@"
      ;;
    -h|--help|help)
      echo "用法："
      echo "  flight                      起整条链路（后台）"
      echo "  flight link                 ① 通信连接控制台"
      echo "  flight sim                  ② 仿真控制台（带 3D 窗口）"
      echo "  flight remote               ③ 网页遥控控制台 + 浏览器"
      echo "  flight views                ④ 三视角窗口"
      echo "  flight perc                 ⑤ 感知节点（min_depth / 光流 CSV）"
      echo "  flight clean [--all]        清 PX4 飞行日志（掉盘元凶）"
      echo "  flight start|stop|status|...  等价于 fly <动作>"
      echo "  flight headless:=false      参数传给 ros2 launch"
      ;;
    *)
      echo "未知参数: ${sub}"
      echo "想看用法: flight help"
      return 1
      ;;
  esac
}

remote() { "${FLIGHT_ROOT}/scripts/fly.sh" remote "$@"; }

fly()       { "${FLIGHT_ROOT}/scripts/fly.sh" "$@"; }
tele()      { "${FLIGHT_ROOT}/scripts/fly.sh" watch; }
flystatus() { "${FLIGHT_ROOT}/scripts/fly.sh" status; }
