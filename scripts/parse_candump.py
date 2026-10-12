#!/usr/bin/env python3
"""
解析 candump 日志为 CSV, 供 PlotJuggler 可视化。

用法:
  python3 parse_candump.py <candump_log> [-o output.csv]

candump 格式:
  (timestamp)  bus  CAN_ID  [DLC]  B0 B1 B2 B3 B4 B5 B6 B7

输出 CSV 列:
  time, 0x01/cmd_pos, 0x01/cmd_vel, 0x01/cmd_kp, 0x01/cmd_kd, 0x01/cmd_torque,
        0x01/fb_pos, 0x01/fb_vel, 0x01/fb_torque, 0x01/fb_errcode, 0x01/fb_temp, ...
"""

import argparse
import re
import sys

# MIT 协议参数范围
POS_MIN, POS_MAX = -12.5, 12.5
VEL_MIN, VEL_MAX = -10.0, 10.0
KP_MIN,  KP_MAX  =  0.0, 250.0
KD_MIN,  KD_MAX  =  0.0,  50.0
TOR_MIN, TOR_MAX = -50.0, 50.0

SPECIAL_FRAMES = {
    bytes([0xFF]*7 + [0xFC]): "enable",
    bytes([0xFF]*7 + [0xFD]): "disable",
    bytes([0xFF]*7 + [0xFE]): "set_zero",
}


def int_to_float(raw, min_val, max_val, bits):
    max_int = (1 << bits) - 1
    return raw / max_int * (max_val - min_val) + min_val


def decode_command(data):
    """解码 MIT/PTM 控制帧"""
    pos_raw = (data[0] << 8) | data[1]
    vel_raw = (data[2] << 4) | ((data[3] >> 4) & 0x0F)
    kp_raw  = ((data[3] & 0x0F) << 8) | data[4]
    kd_raw  = (data[5] << 4) | ((data[6] >> 4) & 0x0F)
    tor_raw = ((data[6] & 0x0F) << 8) | data[7]

    return {
        'cmd_pos':    int_to_float(pos_raw, POS_MIN, POS_MAX, 16),
        'cmd_vel':    int_to_float(vel_raw, VEL_MIN, VEL_MAX, 12),
        'cmd_kp':     int_to_float(kp_raw,  KP_MIN,  KP_MAX,  12),
        'cmd_kd':     int_to_float(kd_raw,  KD_MIN,  KD_MAX,  12),
        'cmd_torque': int_to_float(tor_raw, TOR_MIN, TOR_MAX, 12),
    }


def decode_feedback(data):
    """解码反馈帧 (Byte0=MotorID, Byte1-2=pos, ...)"""
    pos_raw = (data[1] << 8) | data[2]
    vel_raw = (data[3] << 4) | ((data[4] >> 4) & 0x0F)
    tor_raw = ((data[4] & 0x0F) << 8) | data[5]

    return {
        'fb_pos':     int_to_float(pos_raw, POS_MIN, POS_MAX, 16),
        'fb_vel':     int_to_float(vel_raw, VEL_MIN, VEL_MAX, 12),
        'fb_torque':  int_to_float(tor_raw, TOR_MIN, TOR_MAX, 12),
        'fb_errcode': data[6],
        'fb_temp':    data[7] - 40 if len(data) > 7 else 0,  # 温度偏移 -40°C
    }


def classify_frame(can_id, data):
    """分类帧类型: 'special', 'command', 'feedback'"""
    data_bytes = bytes(data)
    if data_bytes in SPECIAL_FRAMES:
        return 'special', SPECIAL_FRAMES[data_bytes]

    # 反馈帧: Byte0 == CAN ID
    if data[0] == can_id:
        return 'feedback', None

    # 其他: 控制帧
    return 'command', None


# candump 行解析正则
LINE_RE = re.compile(
    r'\((\d+\.\d+)\)\s+(\w+)\s+([0-9A-Fa-f]+)\s+\[(\d)\]\s+((?:[0-9A-Fa-f]{2}\s*)+)'
)


def parse_line(line):
    m = LINE_RE.match(line.strip())
    if not m:
        return None
    timestamp = float(m.group(1))
    bus = m.group(2)
    can_id = int(m.group(3), 16)
    dlc = int(m.group(4))
    data = [int(x, 16) for x in m.group(5).strip().split()]
    return timestamp, bus, can_id, dlc, data


def main():
    parser = argparse.ArgumentParser(description="解析 candump 日志为 PlotJuggler CSV")
    parser.add_argument("input", help="candump 日志文件")
    parser.add_argument("-o", "--output", default="", help="输出 CSV (默认: 输入文件名.csv)")
    parser.add_argument("--auto-zero", action="store_true", default=True,
                        help="自动以每个电机的首个反馈位置为零点 (默认开启)")
    parser.add_argument("--no-auto-zero", dest="auto_zero", action="store_false",
                        help="不减零点, 输出原始 MIT 值")
    args = parser.parse_args()

    output = args.output or args.input.rsplit('.', 1)[0] + '.csv'

    # 第一遍: 收集所有电机 ID
    motor_ids = set()
    with open(args.input) as f:
        for line in f:
            parsed = parse_line(line)
            if parsed:
                _, _, can_id, _, data = parsed
                frame_type, _ = classify_frame(can_id, data)
                if frame_type in ('command', 'feedback'):
                    motor_ids.add(can_id)

    motor_ids = sorted(motor_ids)
    print(f"检测到 {len(motor_ids)} 个电机: {', '.join(f'0x{m:02X}' for m in motor_ids)}")

    # CSV 列定义
    fields = ['cmd_pos', 'cmd_vel', 'cmd_kp', 'cmd_kd', 'cmd_torque',
              'fb_pos', 'fb_vel', 'fb_torque', 'fb_errcode', 'fb_temp']
    columns = []
    for mid in motor_ids:
        for f in fields:
            columns.append(f"0x{mid:02X}/{f}")

    # 状态: 每个电机最新的 cmd/fb 值 (float)
    state = {}
    for mid in motor_ids:
        for f in fields:
            state[(mid, f)] = ''

    # auto-zero: 记录每个电机首个反馈位置作为零点
    zero_pos = {mid: None for mid in motor_ids}
    pos_fields = {'cmd_pos', 'fb_pos'}

    # 第二遍: 解析并输出
    first_ts = None
    row_count = 0

    with open(args.input) as fin, open(output, 'w') as fout:
        fout.write("time," + ",".join(columns) + "\n")

        for line in fin:
            parsed = parse_line(line)
            if not parsed:
                continue

            timestamp, bus, can_id, dlc, data = parsed
            if can_id not in motor_ids:
                continue

            frame_type, special_name = classify_frame(can_id, data)

            if frame_type == 'special':
                continue

            if first_ts is None:
                first_ts = timestamp

            t = timestamp - first_ts

            if frame_type == 'command':
                decoded = decode_command(data)
                for k, v in decoded.items():
                    state[(can_id, k)] = v
            elif frame_type == 'feedback':
                decoded = decode_feedback(data)
                for k, v in decoded.items():
                    state[(can_id, k)] = v
                # 记录首个反馈位置作为零点
                if args.auto_zero and zero_pos[can_id] is None:
                    zero_pos[can_id] = decoded['fb_pos']

            # 每帧输出一行
            vals = []
            for mid in motor_ids:
                zp = zero_pos[mid] or 0.0
                for f in fields:
                    v = state[(mid, f)]
                    if v == '':
                        vals.append('')
                    elif isinstance(v, float):
                        # 位置字段减零点
                        if args.auto_zero and f in pos_fields:
                            vals.append(f"{v - zp:.6f}")
                        else:
                            vals.append(f"{v:.6f}")
                    else:
                        vals.append(str(v))
            fout.write(f"{t:.6f}," + ",".join(vals) + "\n")
            row_count += 1

    print(f"已写入 {row_count} 行到 {output}")


if __name__ == "__main__":
    main()
