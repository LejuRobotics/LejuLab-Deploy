#!/bin/bash
# ============================================================
# CANFD CST 模式力矩下发 + 反馈记录脚本
# ============================================================
#
# 通过 cansend 发送 MIT 帧 (CAN ID 0x20) 实现 CST 控制:
#   kp=0, kd=0, vel=0, pos=指定值, tor=指定值
# 同时 candump 捕获反馈帧并解析为 CSV (格式兼容 keyboard_ctrl)。
#
# CSV 格式 (与 keyboard_ctrl 一致):
#   time_ms,state_seq,dt_ms,cmd_p/<name>,cmd_v/<name>,cmd_kp/<name>,
#   cmd_kd/<name>,cmd_tau/<name>,fb_p/<name>,fb_v/<name>,fb_tau/<name>
#
# 用法:
#   sudo ./canfd_cst_send.sh <bus> <motor_id> <torque_nm> [选项]
#
# 选项:
#   --pos <rad>     位置指令 (默认 0)
#   --count <n>     发送帧数 (默认 1)
#   --interval <s>  帧间隔 (默认 0.002)
#   --csv <file>    CSV 输出路径
#   --no-csv        不保存 CSV
#   --enable        发送前使能
#   --disable       发送后失能
#   --name <name>   电机名称 (CSV 列名, 默认 motor_<id>)
#
# 示例:
#   sudo ./canfd_cst_send.sh bcan0 1 0.5 --count 500 --enable --disable
#   sudo ./canfd_cst_send.sh bcan0 3 2.0 --count 1000 --csv test.csv --name leg_l3
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

if [[ $# -lt 3 ]]; then
    echo "用法: sudo $0 <bus> <motor_id> <torque_nm> [--pos <rad>] [--count <n>] [--csv <file>] [--enable] [--disable]"
    exit 1
fi

BUS="$1"; MOTOR_ID="$2"; TORQUE="$3"; shift 3

POS_RAD=0.0; COUNT=1; INTERVAL=0.002
CSV_FILE=""; NO_CSV=false; DO_ENABLE=false; DO_DISABLE=false
MOTOR_NAME="motor_${MOTOR_ID}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --pos)      POS_RAD="$2"; shift 2 ;;
        --count)    COUNT="$2"; shift 2 ;;
        --interval) INTERVAL="$2"; shift 2 ;;
        --csv)      CSV_FILE="$2"; shift 2 ;;
        --no-csv)   NO_CSV=true; shift ;;
        --enable)   DO_ENABLE=true; shift ;;
        --disable)  DO_DISABLE=true; shift ;;
        --name)     MOTOR_NAME="$2"; shift 2 ;;
        *)          shift ;;
    esac
done

if [[ -z "$CSV_FILE" ]] && ! $NO_CSV; then
    CSV_FILE="cst_${BUS}_m${MOTOR_ID}_$(date +%Y%m%d_%H%M%S).csv"
fi

# ---- 工具函数 ----
float_to_uint() {
    python3 -c "
v = max(min($1, $3), $2)
span = $3 - $2
print(int((v - $2) * ((1 << $4) - 1) / span))
"
}

build_mit_frame() {
    local pos_raw=$(float_to_uint "$1" $POS_MIN $POS_MAX 16)
    local vel_raw=$(float_to_uint "$2" $VEL_MIN $VEL_MAX 12)
    local kp_raw=$(float_to_uint "$3" $KP_MIN $KP_MAX 12)
    local kd_raw=$(float_to_uint "$4" $KD_MIN $KD_MAX 12)
    local tor_raw=$(float_to_uint "$5" $TOR_MIN $TOR_MAX 12)
    python3 -c "
pos=$pos_raw&0xFFFF;vel=$vel_raw&0xFFF;kp=$kp_raw&0xFFF;kd=$kd_raw&0xFFF;tor=$tor_raw&0xFFF
b=[pos>>8,pos&0xFF,vel>>4,((vel&0xF)<<4)|(kp>>8),kp&0xFF,kd>>4,((kd&0xF)<<4)|(tor>>8),tor&0xFF]
print('.'.join(f'{x:02X}' for x in b))
"
}

build_broadcast() {
    local tid=$1 tf=$2 frame=""
    for i in $(seq 1 8); do
        [[ -n "$frame" ]] && frame="${frame}."
        [[ $i -eq $tid ]] && frame="${frame}${tf}" || frame="${frame}${ZERO_FRAME}"
    done
    echo "$frame"
}

send_canfd() { cansend "$BUS" "${1}${CANFD_BRS}${2}"; }

# ---- 使能 ----
if $DO_ENABLE; then
    echo "[使能] motor ${MOTOR_ID}..."
    send_canfd "$CAN_ID_CTRL" "$(build_broadcast "$MOTOR_ID" "FF.FF.FF.FF.FF.FF.FF.FC")"
    sleep 0.02
fi

# ---- 构建 CST 帧 ----
MIT_FRAME=$(build_mit_frame "$POS_RAD" 0 0 0 "$TORQUE")
BROADCAST=$(build_broadcast "$MOTOR_ID" "$MIT_FRAME")

echo "[CST] bus=${BUS} motor=${MOTOR_ID}(${MOTOR_NAME}) tau=${TORQUE}Nm pos=${POS_RAD}rad kp=0 kd=0"
echo "  帧数=${COUNT} 间隔=${INTERVAL}s"
! $NO_CSV && echo "  CSV: ${CSV_FILE}"

# ---- 启动 candump ----
DUMP_FILE=$(mktemp)
stdbuf -oL candump "$BUS" -ta > "$DUMP_FILE" 2>/dev/null &
DUMP_PID=$!
sleep 0.05

# ---- 发送 ----
for i in $(seq 1 "$COUNT"); do
    send_canfd "$CAN_ID_MIT" "$BROADCAST"
    [[ "$COUNT" -gt 1 ]] && sleep "$INTERVAL"
done

sleep 0.1
kill "$DUMP_PID" 2>/dev/null || true
wait "$DUMP_PID" 2>/dev/null || true

echo "[CST] 发送完成"

# ---- 解析反馈 → CSV (keyboard_ctrl 兼容格式) ----
if ! $NO_CSV && [[ -s "$DUMP_FILE" ]]; then
    python3 -c "
import sys

motor_id = ${MOTOR_ID}
motor_name = '${MOTOR_NAME}'
tor_cmd = ${TORQUE}
pos_cmd = ${POS_RAD}

lines = open('${DUMP_FILE}').readlines()

# 表头 (keyboard_ctrl 格式)
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

    # 只处理该电机的反馈帧 (CAN ID = 电机 ID)
    if can_id != motor_id:
        continue
    if len(data) < 8:
        continue

    # 解析反馈
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

    # keyboard_ctrl 格式: cmd_p, cmd_v, cmd_kp, cmd_kd, cmd_tau, fb_p, fb_v, fb_tau
    row = f'{t_ms:.3f},{seq},{dt_ms:.3f}'
    row += f',{pos_cmd:.6f},0.000000,0.000000,0.000000,{tor_cmd:.6f}'
    row += f',{fb_pos:.6f},{fb_vel:.6f},{fb_tau:.6f}'
    rows.append(row)
    seq += 1

if rows:
    with open('${CSV_FILE}', 'w') as f:
        f.write(header + '\n')
        for r in rows:
            f.write(r + '\n')
    print(f'[CSV] {len(rows)} 条反馈 → ${CSV_FILE}')
    # 打印首尾
    print(f'  首帧: {rows[0].split(\",\")[0]}ms')
    print(f'  末帧: {rows[-1].split(\",\")[0]}ms')
else:
    print('[CSV] 未收到反馈帧')
"
fi

rm -f "$DUMP_FILE"

# ---- 失能 ----
if $DO_DISABLE; then
    sleep 0.05
    echo "[失能]..."
    send_canfd "$CAN_ID_CTRL" "$(build_broadcast "$MOTOR_ID" "FF.FF.FF.FF.FF.FF.FF.FD")"
    echo "[失能] 完成"
fi
