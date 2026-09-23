#!/bin/bash
# ============================================================
# CANFD CST 阶跃力矩测试脚本
# ============================================================
#
# 对指定电机执行多阶段力矩阶跃序列，全程 candump 记录反馈到 CSV。
# CSV 格式兼容 keyboard_ctrl / analyze_step_response.py。
#
# 注意: shell 循环实际频率约 150-200Hz (非实时), 适合稳态测试。
# 需要精确 500Hz 控制请使用 C++ 工具 (canfd_motor_test / motor_test_runner)。
#
# 用法:
#   sudo ./canfd_cst_step_test.sh <bus> <motor_id> [选项]
#
# 选项:
#   --steps <t1,t2,...>   力矩阶跃序列 Nm (默认 0,1,2,0)
#   --hold <秒>           每阶段保持时长 (默认 10)
#   --pos <rad>           位置指令 (默认 0)
#   --csv <file>          CSV 路径
#   --name <name>         电机名称 (CSV 列名)
#
# 示例:
#   sudo ./canfd_cst_step_test.sh bcan0 4 --steps 0,1,2,0 --hold 10
# ============================================================

set -uo pipefail

POS_MIN=-12.5; POS_MAX=12.5
VEL_MIN=-10.0; VEL_MAX=10.0
KP_MIN=0.0;    KP_MAX=250.0
KD_MIN=0.0;    KD_MAX=50.0
TOR_MIN=-50.0; TOR_MAX=50.0

CANFD_BRS="##1"
CAN_ID_MIT="020"
CAN_ID_CTRL="010"
ZERO_FRAME="7F.FF.7F.F0.00.00.07.FF"

if [[ $# -lt 2 ]]; then
    echo "用法: sudo $0 <bus> <motor_id> [--steps 0,1,2,0] [--hold 10] [--csv <file>]"
    exit 1
fi

BUS="$1"; MOTOR_ID="$2"; shift 2

STEPS_STR="0,1,2,0"
HOLD_S=10
POS_RAD=0.0
CSV_FILE=""
MOTOR_NAME="motor_${MOTOR_ID}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --steps) STEPS_STR="$2"; shift 2 ;;
        --hold)  HOLD_S="$2"; shift 2 ;;
        --pos)   POS_RAD="$2"; shift 2 ;;
        --csv)   CSV_FILE="$2"; shift 2 ;;
        --name)  MOTOR_NAME="$2"; shift 2 ;;
        *)       shift ;;
    esac
done

IFS=',' read -ra STEPS <<< "$STEPS_STR"
NUM_STEPS=${#STEPS[@]}
TOTAL_TIME=$((HOLD_S * NUM_STEPS))

if [[ -z "$CSV_FILE" ]]; then
    CSV_FILE="cst_step_${BUS}_m${MOTOR_ID}_$(date +%Y%m%d_%H%M%S).csv"
fi

echo "============================================"
echo "  CANFD CST 阶跃力矩测试"
echo "============================================"
echo "总线: ${BUS}  电机: ${MOTOR_ID} (${MOTOR_NAME})"
echo "阶跃序列 (Nm): ${STEPS_STR}"
echo "每阶段: ${HOLD_S}s"
echo "总时长: ${TOTAL_TIME}s"
echo "CSV: ${CSV_FILE}"
echo ""

# ---- 工具函数 ----
float_to_uint() {
    python3 -c "
v=max(min($1,$3),$2);span=$3-$2
print(int((v-$2)*((1<<$4)-1)/span))
"
}

build_mit_frame() {
    local pr=$(float_to_uint "$1" $POS_MIN $POS_MAX 16)
    local vr=$(float_to_uint "$2" $VEL_MIN $VEL_MAX 12)
    local kr=$(float_to_uint "$3" $KP_MIN $KP_MAX 12)
    local dr=$(float_to_uint "$4" $KD_MIN $KD_MAX 12)
    local tr=$(float_to_uint "$5" $TOR_MIN $TOR_MAX 12)
    python3 -c "
p=$pr&0xFFFF;v=$vr&0xFFF;k=$kr&0xFFF;d=$dr&0xFFF;t=$tr&0xFFF
b=[p>>8,p&0xFF,v>>4,((v&0xF)<<4)|(k>>8),k&0xFF,d>>4,((d&0xF)<<4)|(t>>8),t&0xFF]
print('.'.join(f'{x:02X}' for x in b))
"
}

build_broadcast() {
    local tid=$1 tf=$2 f=""
    for i in $(seq 1 8); do
        [[ -n "$f" ]] && f="${f}."
        [[ $i -eq $tid ]] && f="${f}${tf}" || f="${f}${ZERO_FRAME}"
    done
    echo "$f"
}

send_canfd() { cansend "$BUS" "${1}${CANFD_BRS}${2}"; }

# ---- 预编译各阶段的广播帧 (避免循环里重复调 python) ----
declare -a BROADCASTS
for si in $(seq 0 $((NUM_STEPS - 1))); do
    tau="${STEPS[$si]}"
    MIT_FRAME=$(build_mit_frame "$POS_RAD" 0 0 0 "$tau")
    BROADCASTS[$si]=$(build_broadcast "$MOTOR_ID" "$MIT_FRAME")
done

# ---- 使能 ----
echo "[使能] motor ${MOTOR_ID}..."
send_canfd "$CAN_ID_CTRL" "$(build_broadcast "$MOTOR_ID" "FF.FF.FF.FF.FF.FF.FF.FC")"
sleep 0.05

# ---- 启动 candump 全程记录 ----
DUMP_FILE=$(mktemp)
stdbuf -oL candump "$BUS" -ta > "$DUMP_FILE" 2>/dev/null &
DUMP_PID=$!
sleep 0.05

# ---- 按真实时间执行阶跃序列 (不依赖帧数, 用 date 计时) ----
CMD_LOG=$(mktemp)
echo "# timestamp_s tau_cmd" > "$CMD_LOG"

T_START=$(date +%s.%N)

for si in $(seq 0 $((NUM_STEPS - 1))); do
    tau="${STEPS[$si]}"
    BCAST="${BROADCASTS[$si]}"
    T_PHASE_END=$(awk "BEGIN{printf \"%.6f\", $T_START + ($si + 1) * $HOLD_S}")

    echo -n "[阶段 $((si+1))/${NUM_STEPS}] tau=${tau}Nm × ${HOLD_S}s ..."

    while true; do
        T_NOW=$(date +%s.%N)
        # 检查是否超过当前阶段结束时间
        if awk "BEGIN{exit(!($T_NOW >= $T_PHASE_END))}"; then
            break
        fi
        send_canfd "$CAN_ID_MIT" "$BCAST"
        echo "$T_NOW $tau" >> "$CMD_LOG"
        sleep 0.002
    done

    echo " 完成"
done

# ---- 停止 candump ----
sleep 0.1
kill "$DUMP_PID" 2>/dev/null || true
wait "$DUMP_PID" 2>/dev/null || true

# ---- 失能 ----
echo "[失能]..."
send_canfd "$CAN_ID_CTRL" "$(build_broadcast "$MOTOR_ID" "FF.FF.FF.FF.FF.FF.FF.FD")"
sleep 0.05

echo "[发送完成] 解析反馈..."

# ---- 解析: 用 cmd_log 的时间戳匹配反馈帧 ----
python3 -c "
import bisect

motor_id = ${MOTOR_ID}
motor_name = '${MOTOR_NAME}'
pos_cmd = ${POS_RAD}

# 读取 cmd_log: [(timestamp, tau_cmd), ...]
cmd_times = []
cmd_taus = []
with open('${CMD_LOG}') as f:
    for line in f:
        if line.startswith('#'):
            continue
        parts = line.strip().split()
        if len(parts) >= 2:
            cmd_times.append(float(parts[0]))
            cmd_taus.append(float(parts[1]))

# 读取 candump 反馈
lines = open('${DUMP_FILE}').readlines()

header = 'time_ms,state_seq,dt_ms'
for pfx in ['cmd_p','cmd_v','cmd_kp','cmd_kd','cmd_tau','fb_p','fb_v','fb_tau']:
    header += f',{pfx}/{motor_name}'

rows = []
seq = 0
t0 = None
t_prev = None

for line in lines:
    parts = line.strip().split()
    if len(parts) < 12:
        continue
    try:
        ts = float(parts[0].strip('()'))
        can_id = int(parts[2], 16)
        data = [int(x, 16) for x in parts[4:12]]
    except (ValueError, IndexError):
        continue

    if can_id != motor_id or len(data) < 8:
        continue

    pos_raw = (data[0] << 8) | data[1]
    vel_raw = (data[2] << 4) | (data[3] >> 4)
    tor_raw = ((data[3] & 0xF) << 8) | data[4]

    fb_pos = pos_raw * 25.0 / 65535 + (-12.5)
    fb_vel = vel_raw * 130.0 / 4095 + (-65.0)
    fb_tau = tor_raw * 60.0 / 4095 + (-30.0)

    if t0 is None:
        t0 = ts
    t_ms = (ts - t0) * 1000.0
    dt_ms = (ts - t_prev) * 1000.0 if t_prev is not None else 0.0
    t_prev = ts

    # 用反馈帧的绝对时间戳在 cmd_log 中找最近的 tau_cmd
    idx = bisect.bisect_right(cmd_times, ts) - 1
    tau_cmd = cmd_taus[max(0, idx)] if cmd_taus else 0.0

    row = f'{t_ms:.3f},{seq},{dt_ms:.3f}'
    row += f',{pos_cmd:.6f},0.000000,0.000000,0.000000,{tau_cmd:.6f}'
    row += f',{fb_pos:.6f},{fb_vel:.6f},{fb_tau:.6f}'
    rows.append(row)
    seq += 1

if rows:
    with open('${CSV_FILE}', 'w') as f:
        f.write(header + '\n')
        for r in rows:
            f.write(r + '\n')
    t_total = float(rows[-1].split(',')[0]) / 1000
    print(f'[CSV] {len(rows)} 条反馈, {t_total:.1f}s -> ${CSV_FILE}')
    # 统计
    tau_set = sorted(set(cmd_taus))
    print(f'  tau 序列: {tau_set}')
    print(f'  实际频率: {len(rows)/t_total:.0f} Hz')
else:
    print('[CSV] 未收到反馈帧')
"

rm -f "$DUMP_FILE" "$CMD_LOG"
echo "[完成]"
