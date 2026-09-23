#!/bin/bash
# 将每次 gcc/g++ 调用轮询绑定到 RK3588 大核 4-7，避免全部挤在同一颗核上
set -euo pipefail

FIRST=${LEJU_BUILD_CORE_FIRST:-4}
LAST=${LEJU_BUILD_CORE_LAST:-7}
STATE_FILE="${TMPDIR:-/tmp}/leju_bigcore_launcher.${FIRST}_${LAST}.state"
LOCK_FILE="${STATE_FILE}.lock"

mkdir -p "$(dirname "$STATE_FILE")"
exec 9>"$LOCK_FILE"
flock 9
core=$(cat "$STATE_FILE" 2>/dev/null || echo "$FIRST")
next=$((core + 1))
if [ "$next" -gt "$LAST" ]; then
    next=$FIRST
fi
echo "$next" > "$STATE_FILE"
flock -u 9

if command -v ccache >/dev/null 2>&1; then
    exec taskset -c "$core" ccache "$@"
fi
exec taskset -c "$core" "$@"
