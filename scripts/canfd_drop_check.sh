#!/bin/bash
#
# CANFD 丢帧检测脚本
#
# 使用 candump 抓取 CAN 总线帧，统计帧率和丢帧情况。
# 需要在 hardware_node 运行期间执行。
#
# 用法:
#   sudo bash scripts/canfd_drop_check.sh [选项]
#
# 选项:
#   -i <接口>     CAN 接口 (默认 bcan0)
#   -d <秒>       采集时长 (默认 10)
#   -e <ms>       期望帧间隔 (默认 2.0, 即 500Hz)
#   -t <倍数>     丢帧判定阈值 = 期望间隔 * 倍数 (默认 1.8)

set -euo pipefail

CAN_IF="bcan0"
DURATION=10
EXPECTED_INTERVAL_MS=2.0
THRESHOLD_MULT=1.8

while [[ $# -gt 0 ]]; do
    case "$1" in
        -i) CAN_IF="$2"; shift 2 ;;
        -d) DURATION="$2"; shift 2 ;;
        -e) EXPECTED_INTERVAL_MS="$2"; shift 2 ;;
        -t) THRESHOLD_MULT="$2"; shift 2 ;;
        -h|--help)
            sed -n '2,/^$/p' "$0" | grep '^#' | sed 's/^# \?//'
            exit 0 ;;
        *) echo "未知参数: $1"; exit 1 ;;
    esac
done

# 检查接口
if ! ip link show "$CAN_IF" &>/dev/null; then
    echo "错误: $CAN_IF 接口不存在"
    exit 1
fi

TMPFILE=$(mktemp /tmp/canfd_dump_XXXXXX.log)
trap "rm -f $TMPFILE" EXIT

THRESH_MS=$(echo "$EXPECTED_INTERVAL_MS * $THRESHOLD_MULT" | bc)

echo "=== CANFD 丢帧检测 ==="
echo "接口: $CAN_IF, 采集: ${DURATION}s, 期望间隔: ${EXPECTED_INTERVAL_MS}ms"
echo "丢帧阈值: ${EXPECTED_INTERVAL_MS}ms × ${THRESHOLD_MULT} = ${THRESH_MS}ms"
echo ""
echo "采集中 (请确保 hardware_node 正在运行)..."

# candump -t d: 打印 delta 时间戳
timeout "$DURATION" candump -t d "$CAN_IF" > "$TMPFILE" 2>/dev/null || true

TOTAL_FRAMES=$(wc -l < "$TMPFILE")
if [[ "$TOTAL_FRAMES" -lt 10 ]]; then
    echo "错误: 只收到 $TOTAL_FRAMES 帧，hardware_node 可能未运行"
    exit 1
fi

echo "采集完成: $TOTAL_FRAMES 帧"
echo ""

# 用 awk 分析 (兼容 mawk/gawk)
awk -v expected_ms="$EXPECTED_INTERVAL_MS" \
    -v thresh_mult="$THRESHOLD_MULT" \
    -v duration="$DURATION" \
'{
    # candump -t d 格式: "(xxx.xxxxxx)  bcan0  ID   [len]  DATA"
    # 提取括号内的 delta 时间
    s = $1
    gsub(/[()]/, "", s)
    dt_s = s + 0
    dt_ms = dt_s * 1000.0

    # 提取 CAN ID
    can_id = $3

    # 按 CAN ID 分组统计
    id_count[can_id]++
    id_sum_dt[can_id] += dt_ms

    # 跟踪每个 ID 的最大间隔
    if (!(can_id in id_max_dt) || dt_ms > id_max_dt[can_id]) {
        id_max_dt[can_id] = dt_ms
    }

    if (total > 0 && dt_ms > 0.001) {
        if (dt_ms > max_gap_ms) {
            max_gap_ms = dt_ms
            max_gap_id = can_id
        }
    }

    total++
}
END {
    printf "========================================\n"
    printf "  CANFD 通信统计 (%ds)\n", duration
    printf "========================================\n"
    printf "总帧数: %d (%.1f fps)\n", total, total / duration
    printf "最大帧间隔: %.3f ms (ID: %s)\n", max_gap_ms, max_gap_id
    printf "\n"

    # 按 ID 统计
    printf "%-10s %8s %10s %10s %10s\n", "CAN_ID", "帧数", "帧率(Hz)", "平均dt(ms)", "最大dt(ms)"
    printf "%-10s %8s %10s %10s %10s\n", "------", "----", "-------", "--------", "--------"

    # 收集所有 ID 并排序
    n = 0
    for (id in id_count) {
        ids[n] = id
        n++
    }
    # 简单冒泡排序
    for (i = 0; i < n-1; i++)
        for (j = i+1; j < n; j++)
            if (ids[i] > ids[j]) { tmp=ids[i]; ids[i]=ids[j]; ids[j]=tmp }

    for (i = 0; i < n; i++) {
        id = ids[i]
        cnt = id_count[id]
        rate = cnt / duration
        avg_dt = id_sum_dt[id] / cnt
        mdt = id_max_dt[id]
        printf "%-10s %8d %10.1f %10.3f %10.3f\n", id, cnt, rate, avg_dt, mdt
    }
    printf "========================================\n"
}' "$TMPFILE"

echo ""

# 第二遍: 检测大间隔事件
echo "=== 大间隔事件 (> ${THRESH_MS}ms) ==="
awk -v thresh_ms="$THRESH_MS" -v expected_ms="$EXPECTED_INTERVAL_MS" \
'{
    s = $1
    gsub(/[()]/, "", s)
    dt_ms = s * 1000.0
    can_id = $3
    if (dt_ms > thresh_ms + 0) {
        count++
        if (count <= 30) {
            printf "  dt=%.3fms  ID=%-8s  (%.1fx 期望)\n", dt_ms, can_id, dt_ms / expected_ms
        }
    }
}
END {
    if (count > 30) printf "  ... 还有 %d 条\n", count - 30
    if (count + 0 == 0) printf "  无大间隔事件\n"
    printf "\n总计: %d 次大间隔\n", count + 0
}' "$TMPFILE"
