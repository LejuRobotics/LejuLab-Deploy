#!/bin/bash
# ============================================================
# 全身电机在线扫描脚本
# ============================================================
#
# 功能:
#   扫描所有 CAN 总线上的电机是否在线，用于快速排查接线/上电问题。
#
# 扫描范围:
#   bcan0: 7 个 CANFD 电机 (ID 1-7), 左腿6 + 腰1
#   bcan1: 6 个 CANFD 电机 (ID 1-6), 右腿6
#   bcan2: 6 个标准CAN电机 (ID 1,2,3,4,9,10), 左臂4 + 头2
#   bcan3: 4 个标准CAN电机 (ID 5,6,7,8), 右臂4
#
# 协议说明:
#   CANFD (bcan0/bcan1):
#     发送: <0x600+ID>##1<670C000000000476> (读KP, CANFD+BRS)
#     响应: 同CAN ID, 首字节=电机ID (发送帧首字节=67, 以此区分)
#
#   标准CAN (bcan2/bcan3):
#     发送: <ID>#FF.FF.FF.FF.FF.FF.FF.FC (使能命令, 标准CAN 8字节)
#     响应: 同CAN ID, 电机回复反馈帧
#
# 用法:
#   sudo ./canfd_scan_motors.sh                 # 默认 300ms 超时
#   sudo ./canfd_scan_motors.sh --timeout 500   # 自定义超时
#   sudo ./canfd_scan_motors.sh --debug         # 显示原始 candump 输出
#
# 退出码:
#   0 = 所有电机在线
#   1 = 有电机无响应
# ============================================================

set -uo pipefail

TIMEOUT_MS=300
DEBUG=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --timeout) TIMEOUT_MS="$2"; shift 2 ;;
        --debug)   DEBUG=true; shift ;;
        *)         shift ;;
    esac
done

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

total_found=0
total_missing=0

echo -e "${CYAN}============================================${NC}"
echo -e "${CYAN}  全身电机扫描${NC}"
echo -e "${CYAN}============================================${NC}"
echo -e "超时: ${TIMEOUT_MS}ms"
echo ""

# ============================================================
# 通用扫描函数
# 参数: bus_name, can_id_hex, frame, label, filter_pattern
# ============================================================
scan_motor() {
    local bus_name="$1"
    local can_id_hex="$2"
    local frame="$3"
    local label="$4"
    local filter_pattern="$5"   # grep -v 过滤发送帧

    local tmpfile
    tmpfile=$(mktemp)

    stdbuf -oL candump "$bus_name" -n 2 > "$tmpfile" 2>/dev/null &
    local dump_pid=$!
    sleep 0.05
    cansend "$bus_name" "$frame" 2>/dev/null || true
    sleep "$(awk "BEGIN{printf \"%.2f\", ${TIMEOUT_MS}/1000.0}")"
    kill "$dump_pid" 2>/dev/null || true
    wait "$dump_pid" 2>/dev/null || true

    if $DEBUG; then
        echo -e "  \033[90m[debug] ${frame} → tmpfile:\033[0m"
        cat "$tmpfile" | sed 's/^/    /'
    fi

    local resp
    resp=$(grep -i "${can_id_hex}" "$tmpfile" 2>/dev/null \
        | grep -v "${filter_pattern}" \
        | head -1 || true)

    if [[ -n "$resp" ]]; then
        local data
        data=$(echo "$resp" | sed 's/.*\]//')
        echo -e "  ${label}: ${GREEN}在线${NC}  <-${data}"
        total_found=$((total_found + 1))
    else
        echo -e "  ${label}: ${RED}无响应${NC}"
        total_missing=$((total_missing + 1))
    fi

    rm -f "$tmpfile"
}

# ============================================================
# CANFD 总线扫描 (bcan0/bcan1)
# ============================================================
CANFD_CMD_DATA="670C000000000476"

canfd_scan_bus() {
    local bus_name="$1"
    shift
    local motor_ids=("$@")

    echo -e "${YELLOW}--- ${bus_name} [CANFD] (${#motor_ids[@]} 电机) ---${NC}"

    if ! ip link show "$bus_name" &>/dev/null; then
        echo -e "  ${RED}接口 ${bus_name} 不存在, 跳过${NC}"
        echo ""
        total_missing=$((total_missing + ${#motor_ids[@]}))
        return
    fi

    for motor_id in "${motor_ids[@]}"; do
        local can_id
        can_id=$(printf "%03X" $((0x600 + motor_id)))
        local frame="${can_id}##1${CANFD_CMD_DATA}"
        scan_motor "$bus_name" "$can_id" "$frame" "M${motor_id} (0x${can_id})" "67 0C"
    done
    echo ""
}

# ============================================================
# 标准 CAN 总线扫描 (bcan2/bcan3)
# ============================================================
CAN_CMD_DATA="670C000000000476"

can_scan_bus() {
    local bus_name="$1"
    local bus_label="$2"
    shift 2
    local motor_ids=("$@")

    echo -e "${YELLOW}--- ${bus_name} [CAN] ${bus_label} (${#motor_ids[@]} 电机) ---${NC}"

    if ! ip link show "$bus_name" &>/dev/null; then
        echo -e "  ${RED}接口 ${bus_name} 不存在, 跳过${NC}"
        echo ""
        total_missing=$((total_missing + ${#motor_ids[@]}))
        return
    fi

    for motor_id in "${motor_ids[@]}"; do
        local can_id
        can_id=$(printf "%03X" $((0x600 + motor_id)))
        local frame="${can_id}#${CAN_CMD_DATA}"
        scan_motor "$bus_name" "$can_id" "$frame" "M${motor_id} (0x${can_id})" "67 0C"
    done
    echo ""
}

# ============================================================
# 执行扫描
# ============================================================

# CANFD 腿/腰
canfd_scan_bus "bcan0" 1 2 3 4 5 6 7
canfd_scan_bus "bcan1" 1 2 3 4 5 6

# 标准 CAN 手臂/头
can_scan_bus "bcan2" "左臂+头" 1 2 3 4 9 10
can_scan_bus "bcan3" "右臂"    5 6 7 8

# ============================================================
# 汇总
# ============================================================
echo -e "${CYAN}============================================${NC}"
if [[ $total_missing -eq 0 ]]; then
    echo -e "  扫描完成: ${GREEN}${total_found} 在线${NC}, 全部正常"
else
    echo -e "  扫描完成: ${GREEN}${total_found} 在线${NC}, ${RED}${total_missing} 无响应${NC}"
fi
echo -e "${CYAN}============================================${NC}"

[[ $total_missing -eq 0 ]]
