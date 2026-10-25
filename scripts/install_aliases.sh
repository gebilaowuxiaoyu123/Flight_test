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
  echo "已从 ${BASHRC} 移除 flight_project 别名"
  exit 0
fi

cat >> "${BASHRC}" <<EOF
${MARK_BEGIN}
# ---- flight_project 短命令 ----
export FLIGHT_ROOT="${ROOT}"
_flight_env() {
  source /opt/ros/humble/setup.bash 2>/dev/null
  source "\${FLIGHT_ROOT}/install/setup.bash" 2>/dev/null
}
alias flight='_flight_env && ros2 launch flight_ui flight.launch.py'
alias remote='_flight_env && ros2 launch flight_ui remote.launch.py'
alias fly='\${FLIGHT_ROOT}/scripts/fly.sh'
alias tele='\${FLIGHT_ROOT}/scripts/fly.sh watch'
alias flystatus='\${FLIGHT_ROOT}/scripts/fly.sh status'
${MARK_END}
EOF
echo "已写入 ${BASHRC}："
echo "  flight      起整条链路（= ros2 launch flight_ui flight.launch.py）"
echo "  remote      只起 Web 遥控器后端"
echo "  fly <动作>  终端飞：fly start / fly takeoff / fly land / fly fwd 3"
echo "  tele        终端实时看姿态"
echo "  flystatus   一行状态"
echo
echo "生效： source ${BASHRC}   或重开终端"
