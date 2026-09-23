#!/usr/bin/env bash
# candump_record.sh — 录制并解析 SocketCAN 数据
#
# 用法:
#   ./scripts/candump_record.sh [选项] [CAN接口...]
#
# 示例:
#   ./scripts/candump_record.sh                     # 录制所有 bcan* 接口, 5秒
#   ./scripts/candump_record.sh bcan0 bcan1         # 指定接口
#   ./scripts/candump_record.sh -t 10 bcan0         # 录制 10 秒
#   ./scripts/candump_record.sh -p record.txt       # 解析已有录制文件
#   ./scripts/candump_record.sh -o mylog            # 自定义输出前缀
#
# 输出:
#   <prefix>_raw.txt   — candump 原始日志
#   <prefix>_parsed.csv — 解析后的 CSV (可直接用 pandas 读取)

set -euo pipefail

# ── 默认参数 ──
DURATION=5
OUTPUT_PREFIX=""
PARSE_ONLY=""
CAN_INTERFACES=()

# ── 参数解析 ──
while [[ $# -gt 0 ]]; do
    case "$1" in
        -t|--time)     shift; DURATION="$1" ;;
        -o|--output)   shift; OUTPUT_PREFIX="$1" ;;
        -p|--parse)    shift; PARSE_ONLY="$1" ;;
        -h|--help)
            sed -n '2,/^$/s/^# \?//p' "$0"
            exit 0
            ;;
        *)  CAN_INTERFACES+=("$1") ;;
    esac
    shift
done

# ── 仅解析模式 ──
if [[ -n "${PARSE_ONLY}" ]]; then
    if [[ ! -f "${PARSE_ONLY}" ]]; then
        echo "文件不存在: ${PARSE_ONLY}" >&2
        exit 1
    fi
    CSV_OUT="${PARSE_ONLY%.txt}_parsed.csv"
    echo "解析: ${PARSE_ONLY} → ${CSV_OUT}"
    python3 - "${PARSE_ONLY}" "${CSV_OUT}" <<'PYEOF'
import sys, re, struct

def int_to_float(val, bits, min_v, max_v):
    max_int = (1 << bits) - 1
    return (val / max_int) * (max_v - min_v) + min_v

# CAN FD 广播帧 TX (64字节): 每 8 字节一个电机 slot
# MIT 帧 (0x20): pos[16] vel[12] kp[12] kd[12] tor[12]
def parse_mit_slot(data):
    """解析 MIT 帧中一个 8 字节 slot"""
    pos_raw = (data[0] << 8) | data[1]
    vel_raw = (data[2] << 4) | (data[3] >> 4)
    kp_raw  = ((data[3] & 0x0F) << 8) | data[4]
    kd_raw  = (data[5] << 4) | (data[6] >> 4)
    tor_raw = ((data[6] & 0x0F) << 8) | data[7]
    return {
        'pos': int_to_float(pos_raw, 16, -12.5, 12.5),
        'vel': int_to_float(vel_raw, 12, -65.0, 65.0),
        'kp':  int_to_float(kp_raw,  12, 0.0, 500.0),
        'kd':  int_to_float(kd_raw,  12, 0.0, 5.0),
        'tor': int_to_float(tor_raw, 12, -30.0, 30.0),
    }

# 位置-速度帧 (0x40): 2 个 float32 (little-endian)
def parse_posvel_slot(data):
    pos = struct.unpack_from('<f', bytes(data), 0)[0]
    vel = struct.unpack_from('<f', bytes(data), 4)[0]
    return {'pos': pos, 'vel': vel, 'kp': 0, 'kd': 0, 'tor': 0}

# 控制帧 (0x10): byte7 = 命令
CTRL_CMDS = {0xFC: 'ENABLE', 0xFD: 'DISABLE', 0xFE: 'SAVE_ZERO', 0xFB: 'CLEAR_ERR'}

# 反馈帧 (8字节, CAN ID = motor_id)
# 旧协议: [motor_id, pos_H, pos_L, vel_H, vel[11:4]|tor[11:8], tor_L, err, temp]
# FD协议: [pos_H, pos_L, vel_H, vel[3:0]|tor[11:8], tor_L, temp_raw, status_H, status_L]
def parse_feedback_old(data):
    """旧协议反馈 (byte0=motor_id)"""
    pos_raw = (data[1] << 8) | data[2]
    vel_raw = (data[3] << 4) | (data[4] >> 4)
    tor_raw = ((data[4] & 0x0F) << 8) | data[5]
    return {
        'motor_id': data[0],
        'pos': int_to_float(pos_raw, 16, -12.5, 12.5),
        'vel': int_to_float(vel_raw, 12, -10.0, 10.0),
        'tor': int_to_float(tor_raw, 12, -50.0, 50.0),
        'err': data[6],
        'temp': data[7] - 40 if len(data) > 7 else 0,
    }

def parse_feedback_fd(can_id, data):
    """FD协议反馈 (can_id=motor_id, 无byte0的motor_id)"""
    pos_raw = (data[0] << 8) | data[1]
    vel_raw = (data[2] << 4) | (data[3] >> 4)
    tor_raw = ((data[3] & 0x0F) << 8) | data[4]
    return {
        'motor_id': can_id,
        'pos': int_to_float(pos_raw, 16, -12.5, 12.5),
        'vel': int_to_float(vel_raw, 12, -65.0, 65.0),
        'tor': int_to_float(tor_raw, 12, -30.0, 30.0),
        'temp': data[5] - 40 if len(data) > 5 else 0,
        'status': ((data[6] << 8) | data[7]) if len(data) > 7 else 0,
    }

# 支持两种 candump 输出格式:
#
# 1) -L 日志格式 (推荐):
#    CAN FD: (1710000000.123456) bcan0 020##17FFF7FF000...
#    CAN:    (1710000000.123456) bcan0 001#7FFF7FF000007FF
#
# 2) -ta 可读格式:
#    (1710000000.123456)  bcan0  020   [64]  7F FF 7F F0 ...
#    (1710000000.123456)  bcan0  001   [08]  7F FF 7F F0 ...

# -L 格式: ##后flags和data之间可能有空格也可能没有
CANFD_LINE = re.compile(
    r'\((\d+\.\d+)\)\s+(\S+)\s+([0-9A-Fa-f]+)##([0-9A-Fa-f])\s*([0-9A-Fa-f ]+)')
CAN_LINE = re.compile(
    r'\((\d+\.\d+)\)\s+(\S+)\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]+)')
# -ta 格式: CAN_ID [DLC] HH HH HH ...
TA_LINE = re.compile(
    r'\((\d+\.\d+)\)\s+(\S+)\s+([0-9A-Fa-f]+)\s+\[(\d+)\]\s+([0-9A-Fa-f ]+)')

raw_file = sys.argv[1]
csv_file = sys.argv[2]

rows = []

with open(raw_file) as f:
    for line in f:
        line = line.strip()
        if not line:
            continue

        # 尝试三种格式
        ts = iface = can_id_hex = data_hex = None
        is_fd = False

        m = CANFD_LINE.match(line)
        if m:
            ts, iface, can_id_hex, flags, data_hex = m.groups()
            is_fd = True
        if not m:
            m = CAN_LINE.match(line)
            if m:
                ts, iface, can_id_hex, data_hex = m.groups()
        if not m:
            m = TA_LINE.match(line)
            if m:
                ts, iface, can_id_hex, dlc_str, data_hex = m.groups()
                is_fd = (int(dlc_str) > 8)
        if not m:
            continue

        ts = float(ts)
        can_id = int(can_id_hex, 16)
        clean_hex = data_hex.replace(' ', '').strip()
        data = [int(clean_hex[i:i+2], 16) for i in range(0, len(clean_hex), 2)]
        data_len = len(data)

        if can_id == 0x20 and data_len == 64:
            # MIT 广播帧 → 解析 8 个 slot
            for slot in range(8):
                chunk = data[slot*8:(slot+1)*8]
                # 跳过全零 slot
                if all(b == 0 for b in chunk):
                    continue
                vals = parse_mit_slot(chunk)
                rows.append({
                    'time': ts, 'iface': iface, 'dir': 'TX',
                    'type': 'MIT', 'motor_id': slot + 1,
                    **vals, 'temp': 0, 'err': 0, 'status': 0,
                })

        elif can_id == 0x40 and data_len == 64:
            for slot in range(8):
                chunk = data[slot*8:(slot+1)*8]
                if all(b == 0 for b in chunk):
                    continue
                vals = parse_posvel_slot(chunk)
                rows.append({
                    'time': ts, 'iface': iface, 'dir': 'TX',
                    'type': 'POSVEL', 'motor_id': slot + 1,
                    **vals, 'temp': 0, 'err': 0, 'status': 0,
                })

        elif can_id == 0x10 and data_len == 64:
            for slot in range(8):
                cmd_byte = data[slot*8 + 7]
                cmd_name = CTRL_CMDS.get(cmd_byte, '')
                if cmd_name:
                    rows.append({
                        'time': ts, 'iface': iface, 'dir': 'TX',
                        'type': 'CTRL_' + cmd_name, 'motor_id': slot + 1,
                        'pos': 0, 'vel': 0, 'kp': 0, 'kd': 0, 'tor': 0,
                        'temp': 0, 'err': 0, 'status': 0,
                    })

        elif 0x01 <= can_id <= 0x0A and data_len == 8:
            # 反馈帧
            if is_fd or data[0] != can_id:
                # FD 协议: payload 不含 motor_id
                fb = parse_feedback_fd(can_id, data)
            else:
                # 旧协议: byte0 = motor_id
                fb = parse_feedback_old(data)
            rows.append({
                'time': ts, 'iface': iface, 'dir': 'RX',
                'type': 'FEEDBACK', 'motor_id': fb['motor_id'],
                'pos': fb['pos'], 'vel': fb['vel'], 'tor': fb['tor'],
                'kp': 0, 'kd': 0,
                'temp': fb['temp'],
                'err': fb.get('err', 0),
                'status': fb.get('status', 0),
            })

if not rows:
    print("未解析到任何 CAN 帧", file=sys.stderr)
    sys.exit(1)

# 写 CSV
cols = ['time', 'iface', 'dir', 'type', 'motor_id', 'pos', 'vel', 'tor', 'kp', 'kd', 'temp', 'err', 'status']
with open(csv_file, 'w') as out:
    out.write(','.join(cols) + '\n')
    for r in rows:
        out.write(','.join(str(r.get(c, '')) for c in cols) + '\n')

print(f"解析完成: {len(rows)} 帧 → {csv_file}")

# 统计摘要
from collections import Counter
iface_cnt = Counter((r['iface'], r['dir'], r['type']) for r in rows)
print("\n帧统计:")
for (iface, direction, ftype), cnt in sorted(iface_cnt.items()):
    print(f"  {iface} {direction:2s} {ftype:15s} {cnt:6d} 帧")

# 反馈频率估算
for iface_name in sorted(set(r['iface'] for r in rows)):
    fb_rows = [r for r in rows if r['iface'] == iface_name and r['dir'] == 'RX']
    if len(fb_rows) >= 2:
        dt = fb_rows[-1]['time'] - fb_rows[0]['time']
        motors = set(r['motor_id'] for r in fb_rows)
        for mid in sorted(motors):
            motor_fb = [r for r in fb_rows if r['motor_id'] == mid]
            if len(motor_fb) >= 2:
                motor_dt = motor_fb[-1]['time'] - motor_fb[0]['time']
                rate = (len(motor_fb) - 1) / motor_dt if motor_dt > 0 else 0
                print(f"  {iface_name} motor {mid}: {rate:.1f} Hz ({len(motor_fb)} 帧 / {motor_dt:.3f}s)")
PYEOF
    echo "完成: ${CSV_OUT}"
    exit 0
fi

# ── 检查 candump ──
if ! command -v candump &>/dev/null; then
    echo "candump 未安装, 请运行: sudo apt install can-utils" >&2
    exit 1
fi

# ── 自动发现 CAN 接口 ──
if [[ ${#CAN_INTERFACES[@]} -eq 0 ]]; then
    mapfile -t CAN_INTERFACES < <(ip -br link show type can 2>/dev/null | awk '{print $1}' | sort)
    if [[ ${#CAN_INTERFACES[@]} -eq 0 ]]; then
        echo "未发现 CAN 接口" >&2
        exit 1
    fi
fi

echo "CAN 接口: ${CAN_INTERFACES[*]}"
echo "录制时长: ${DURATION} 秒"

# ── 生成输出文件名 ──
if [[ -z "${OUTPUT_PREFIX}" ]]; then
    OUTPUT_PREFIX="candump_$(date +%Y%m%d_%H%M%S)"
fi
RAW_FILE="${OUTPUT_PREFIX}_raw.txt"
CSV_FILE="${OUTPUT_PREFIX}_parsed.csv"

echo "输出文件: ${RAW_FILE}"

# ── 录制 ──
echo "开始录制... (${DURATION}s)"
timeout "${DURATION}" candump -L "${CAN_INTERFACES[@]}" > "${RAW_FILE}" 2>/dev/null || true

FRAME_COUNT=$(wc -l < "${RAW_FILE}")
echo "录制完成: ${FRAME_COUNT} 帧"

if [[ "${FRAME_COUNT}" -eq 0 ]]; then
    echo "未捕获到任何帧" >&2
    exit 1
fi

# ── 解析 ──
echo "解析中..."
exec "$0" -p "${RAW_FILE}"
