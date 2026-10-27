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

cat >> "${BASHRC}" <<EOF
# >>> flight_project >>>
# 具体定义在 scripts/flight_env.sh，改命令不用再动 .bashrc
[ -f "${ROOT}/scripts/flight_env.sh" ] && source "${ROOT}/scripts/flight_env.sh"
# <<< flight_project <<<
EOF
echo "已写入 ${BASHRC}"
echo
echo "下一步：当前终端执行下面两条（清掉旧 alias 再加载）："
echo "  unalias flight remote 2>/dev/null; source ~/.bashrc"
echo "或者直接开一个新终端。"
