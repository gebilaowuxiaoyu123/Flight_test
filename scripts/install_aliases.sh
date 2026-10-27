#!/usr/bin/env bash
# 安装短命令到 ~/.bashrc（幂等，重复执行只更新那一块）
#
#   ./scripts/install_aliases.sh          安装
#   ./scripts/install_aliases.sh --remove 卸载
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASHRC="${HOME}/.bashrc"
MARK_BEGIN='# >>> flight_project >>>'
MARK_END='# <<< flight_project <<<'

remove_block() {
  if grep -qF "${MARK_BEGIN}" "${BASHRC}" 2>/dev/null; then
    python3 - "${BASHRC}" "${MARK_BEGIN}" "${MARK_END}" <<'PY'
import sys
path, begin, end = sys.argv[1], sys.argv[2], sys.argv[3]
lines = open(path).read().split('\n')
out, skip = [], False
for ln in lines:
    if ln.strip() == begin:
        skip = True
        continue
    if ln.strip() == end:
        skip = False
        continue
    if not skip:
        out.append(ln)
open(path, 'w').write('\n'.join(out))
PY
  fi
}

remove_block
if [ "${1:-}" == "--remove" ]; then
  echo "已从 ${BASHRC} 移除 flight_project 短命令"
  exit 0
fi

cat >> "${BASHRC}" <<'EOF'
# >>> flight_project >>>
# ---- flight_project 短命令 ----
export FLIGHT_ROOT="/home/wiki_wu/flight_project"

_flight_env() {
  source /opt/ros/humble/setup.bash 2>/dev/null
  source "${FLIGHT_ROOT}/install/setup.bash" 2>/dev/null
}

# flight 同时支持两种用法：
#   flight                     起整条链路（ros2 launch）
#   flight start / stop / gui  等价于 fly start / stop / gui
#   flight headless:=false     参数原样传给 ros2 launch
flight() {
  local sub="${1:-}"
  case "${sub}" in
    ''|launch)
      _flight_env
      ros2 launch flight_ui flight.launch.py "${@:2}"
      ;;
    start|stop|status|watch|gui|logs|mode|arm|takeoff|up|down|fwd|back|left|right|spin|land)
      "${FLIGHT_ROOT}/scripts/fly.sh" "$@"
      ;;
    *':='*)
      _flight_env
      ros2 launch flight_ui flight.launch.py "$@"
      ;;
    -h|--help|help)
      echo "用法："
      echo "  flight                      起整条链路"
      echo "  flight start|stop|gui|...   等价于 fly <动作>"
      echo "  flight headless:=false      参数传给 ros2 launch"
      ;;
    *)
      echo "未知参数: ${sub}"
      echo "想看用法: flight help"
      return 1
      ;;
  esac
}

remote() {
  _flight_env
  ros2 launch flight_ui remote.launch.py "$@"
}
alias fly='${FLIGHT_ROOT}/scripts/fly.sh'
alias tele='${FLIGHT_ROOT}/scripts/fly.sh watch'
alias flystatus='${FLIGHT_ROOT}/scripts/fly.sh status'
# <<< flight_project <<<
EOF
echo "已写入 ${BASHRC}"
