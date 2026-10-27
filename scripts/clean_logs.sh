#!/usr/bin/env bash
# 清理 PX4 SITL 的飞行日志（.ulg）。
#
# PX4 每次启动都会往 build/px4_sitl_default/rootfs/log/ 写 .ulg，
# 只要 SITL 不关就一直写——跑一整夜能写十几个 G。这是本机掉盘的主因。
#
#   ./scripts/clean_logs.sh             只保留今天的（删 24 小时前的）
#   ./scripts/clean_logs.sh --days 7    删 7 天前的
#   ./scripts/clean_logs.sh --all       全删（只保留目录）
#   ./scripts/clean_logs.sh --dry-run   只看不删
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

LOG_DIR="${WORKSPACE_ROOT}/PX4-Autopilot/build/px4_sitl_default/rootfs/log"
[[ -d "${LOG_DIR}" ]] || die "没找到 ${LOG_DIR}（PX4 还没编译过？）"

DAYS=0          # 0 = 只保留最近 24 小时的
MODE=days
DRY=0
while [ $# -gt 0 ]; do
  case "$1" in
    --all)     MODE=all ;;
    --days)    DAYS="${2:-0}"; shift ;;
    --dry-run) DRY=1 ;;
    -h|--help) sed -n '2,10p' "$0"; exit 0 ;;
    *)         die "未知参数: $1" ;;
  esac
  shift
done

before=$(du -sh "${LOG_DIR}" | cut -f1)

if [ "${MODE}" = all ]; then
  find "${LOG_DIR}" -type f -name '*.ulg' -print -delete | wc -l | xargs -I{} echo "删除 {} 个日志"
else
  n=$(find "${LOG_DIR}" -type f -name '*.ulg' -mtime "+${DAYS}" | wc -l)
  if [ "${DRY}" = 1 ]; then
    echo "将删除 ${n} 个（${DAYS} 天以前，即不保留最近 24 小时的）："
    find "${LOG_DIR}" -type f -name '*.ulg' -mtime "+${DAYS}" -printf '  %s\t%p\n' \
      | sort -rn | head -20
    echo "当前占用: ${before}"
    exit 0
  fi
  find "${LOG_DIR}" -type f -name '*.ulg' -mtime "+${DAYS}" -delete
  echo "删除 ${n} 个（${DAYS} 天以前的）日志"
fi

# 清掉删空的日期目录
find "${LOG_DIR}" -mindepth 1 -maxdepth 1 -type d -empty -delete 2>/dev/null || true

after=$(du -sh "${LOG_DIR}" | cut -f1)
log "日志目录: ${before} → ${after}"
df -h / | tail -1 | awk '{printf "根分区剩余: %s / %s（已用 %s）\n", $4, $2, $5}'
