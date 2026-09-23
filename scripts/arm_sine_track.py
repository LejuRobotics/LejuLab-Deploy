#!/usr/bin/env python3
"""
手臂电机正弦跟踪控制脚本 — 通过 SocketCAN 直接发送 CAN 帧

参考: keyboard_controller 控制逻辑 + motorevo PTM 协议
协议: CAN 单帧 (8 bytes), CAN ID = 电机 ID

MIT/PTM 帧编码 (8 bytes):
  Byte0-1:  position  (16-bit)  range [-12.5, 12.5] rad
  Byte2:    velocity  [11:4]
  Byte3:    velocity  [3:0] | kp [11:8]
  Byte4:    kp        [7:0]   (12-bit)  range [0, 250]
  Byte5:    kd        [11:4]
  Byte6:    kd        [3:0] | torque [11:8]
  Byte7:    torque    [7:0]   (12-bit)  range [-50, 50] Nm

反馈帧 (8 bytes, V3.5 手册 6.5.1):
  Byte0:   Status / Motor ID
  Byte1-2: position (16-bit)
  Byte3:   velocity high (8 bits)
  Byte4:   velocity low 4 bits | torque high 4 bits
  Byte5:   torque low (8 bits)
  Byte6:   error code
  Byte7:   temperature

用法:
  sudo python3 arm_sine_track.py [选项]

  --can-bus     CAN 接口名称 (默认 bcan2)
  --motor-id    电机 CAN ID, 十六进制 (默认 0x01, 左臂关节1)
  --motor-ids   多电机 CAN ID 列表, 逗号分隔, 例如 0x01,0x02,0x03
  --amplitude   正弦振幅, 度 (默认 20)
  --frequency   正弦频率, Hz (默认 0.5)
  --duration    运行时长, 秒 (默认 30)
  --rate        控制频率, Hz (默认 250)
  --kp          位置刚度 (默认 14.25)
  --kd          阻尼 (默认 0.907)
  --negtive     电机方向取反 (默认 false)
  --zero-offset 零点偏移, rad (默认 0.0)
  --csv         保存 CSV 文件路径
  --dry-run     仅打印帧数据，不实际发送
"""

import argparse
import math
import os
import signal
import socket
import struct
import sys
import time

# ── CAN 常量 ──────────────────────────────────────────────
PF_CAN = 29
CAN_RAW = 1
CAN_MTU = 16  # sizeof(struct can_frame) = 16 bytes on Linux

# MIT 协议参数范围
POS_MIN, POS_MAX = -12.5, 12.5       # rad
VEL_MIN, VEL_MAX = -10.0, 10.0       # rad/s
KP_MIN,  KP_MAX  =   0.0, 250.0
KD_MIN,  KD_MAX  =   0.0,  50.0
TOR_MIN, TOR_MAX = -50.0,  50.0      # Nm

# 特殊帧
ENABLE_FRAME  = bytes([0xFF]*7 + [0xFC])
DISABLE_FRAME = bytes([0xFF]*7 + [0xFD])

running = True


def signal_handler(sig, frame):
    global running
    running = False


def float_to_int(value: float, min_val: float, max_val: float, bits: int) -> int:
    """浮点数线性量化为 N-bit 无符号整数"""
    clamped = max(min_val, min(max_val, value))
    max_int = (1 << bits) - 1
    return int((clamped - min_val) / (max_val - min_val) * max_int)


def int_to_float(raw: int, min_val: float, max_val: float, bits: int) -> float:
    """N-bit 无符号整数反量化为浮点数"""
    max_int = (1 << bits) - 1
    return raw / max_int * (max_val - min_val) + min_val


def encode_mit_frame(pos: float, vel: float, torque: float, kp: float, kd: float) -> bytes:
    """编码 MIT/PTM 控制帧 (8 bytes)"""
    pos_raw = float_to_int(pos, POS_MIN, POS_MAX, 16)
    vel_raw = float_to_int(vel, VEL_MIN, VEL_MAX, 12)
    kp_raw  = float_to_int(kp,  KP_MIN,  KP_MAX,  12)
    kd_raw  = float_to_int(kd,  KD_MIN,  KD_MAX,  12)
    tor_raw = float_to_int(torque, TOR_MIN, TOR_MAX, 12)

    buf = bytearray(8)
    buf[0] = (pos_raw >> 8) & 0xFF
    buf[1] = pos_raw & 0xFF
    buf[2] = (vel_raw >> 4) & 0xFF
    buf[3] = ((vel_raw & 0x0F) << 4) | ((kp_raw >> 8) & 0x0F)
    buf[4] = kp_raw & 0xFF
    buf[5] = (kd_raw >> 4) & 0xFF
    buf[6] = ((kd_raw & 0x0F) << 4) | ((tor_raw >> 8) & 0x0F)
    buf[7] = tor_raw & 0xFF
    return bytes(buf)


def decode_command(data: bytes) -> dict:
    """解码 MIT/PTM 控制帧 → dict (用于记录实际发送的量化值)"""
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


def decode_feedback(data: bytes) -> dict:
    """解码反馈帧 (8 bytes) → dict

    Byte0 是 Status (V3.5) 或 Motor ID (旧版), 不影响 Byte1+ 解析
    """
    if len(data) < 6:
        return None
    pos_raw = (data[1] << 8) | data[2]
    vel_raw = (data[3] << 4) | ((data[4] >> 4) & 0x0F)
    tor_raw = ((data[4] & 0x0F) << 8) | data[5]
    return {
        'fb_pos':     int_to_float(pos_raw, POS_MIN, POS_MAX, 16),
        'fb_vel':     int_to_float(vel_raw, VEL_MIN, VEL_MAX, 12),
        'fb_torque':  int_to_float(tor_raw, TOR_MIN, TOR_MAX, 12),
        'fb_errcode': data[6] if len(data) > 6 else 0,
        'fb_temp':    data[7] if len(data) > 7 else 0,
    }


def open_can_socket(interface: str) -> socket.socket:
    """打开 SocketCAN raw socket (与 hardware_node 的 SocketCanWrapper::init 一致)"""
    sock = socket.socket(PF_CAN, socket.SOCK_RAW, CAN_RAW)
    sock.bind((interface,))
    # 禁止接收自己发出的帧 (避免命令帧被当作反馈处理)
    sock.setsockopt(socket.SOL_CAN_RAW, 0x4, struct.pack("i", 0))  # CAN_RAW_RECV_OWN_MSGS=4
    # 增大发送缓冲区 (防止高频发送时 EAGAIN)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1048576)
    sock.setblocking(False)
    return sock


def send_can_frame(sock: socket.socket, can_id: int, data: bytes):
    """发送 8 字节标准 CAN 帧 (与 C++ writeCanMessage 一致: EAGAIN 静默跳过)"""
    frame = struct.pack("=IB3x8s", can_id, len(data), data.ljust(8, b'\x00'))
    try:
        sock.send(frame)
    except BlockingIOError:
        pass  # TX buffer 满, 跳过 (与 C++ EAGAIN 处理一致)


def recv_can_frame(sock: socket.socket):
    """尝试接收一个 CAN 帧, 返回 (can_id, data) 或 None"""
    try:
        raw = sock.recv(CAN_MTU)
        if len(raw) >= CAN_MTU:
            can_id, dlc = struct.unpack_from("=IB", raw, 0)
            data = raw[8:8 + min(dlc, 8)]
            return can_id, data
    except BlockingIOError:
        pass
    return None


def main():
    parser = argparse.ArgumentParser(description="手臂电机正弦跟踪 (SocketCAN)")
    parser.add_argument("--can-bus",     default="bcan2",  help="CAN 接口 (默认 bcan2)")
    parser.add_argument("--motor-id",    default="0x01",   help="单电机 CAN ID, 十六进制 (兼容参数, 默认 0x01)")
    parser.add_argument("--motor-ids",   default="",       help="多电机 CAN ID 列表, 逗号分隔, 例如 0x01,0x02,0x03")
    parser.add_argument("--amplitude",   type=float, default=20.0,   help="振幅 (度, 默认 20)")
    parser.add_argument("--frequency",   type=float, default=0.5,    help="频率 (Hz, 默认 0.5)")
    parser.add_argument("--duration",    type=float, default=30.0,   help="时长 (秒, 默认 30)")
    parser.add_argument("--rate",        type=int,   default=250,    help="控制频率 (Hz, 默认 250)")
    parser.add_argument("--kp",          type=float, default=14.25,  help="位置 Kp (默认 14.25)")
    parser.add_argument("--kd",          type=float, default=0.907,  help="位置 Kd (默认 0.907)")
    parser.add_argument("--negtive",     action="store_true",        help="电机方向取反")
    parser.add_argument("--zero-offset", type=float, default=0.0,    help="零点偏移 rad (默认 0.0)")
    parser.add_argument("--csv",         default="",                 help="保存 CSV 文件路径")
    parser.add_argument("--dry-run",     action="store_true",        help="仅打印不发送")
    args = parser.parse_args()

    # 兼容旧参数 --motor-id; 若提供 --motor-ids 则优先使用
    motor_ids_str = args.motor_ids.strip()
    if motor_ids_str:
        motor_ids = [int(x.strip(), 16) for x in motor_ids_str.split(",") if x.strip()]
    else:
        motor_ids = [int(args.motor_id, 16)]
    # 去重并保持顺序
    motor_ids = list(dict.fromkeys(motor_ids))
    if not motor_ids:
        print("错误: 请至少提供一个电机 ID")
        sys.exit(1)

    amplitude_rad = args.amplitude * math.pi / 180.0
    interval = 1.0 / args.rate

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print(f"=== 手臂电机正弦跟踪 ===")
    print(f"CAN 接口:  {args.can_bus}")
    print(f"电机 IDs:  {', '.join(f'0x{x:02X}' for x in motor_ids)}")
    print(f"振幅:      {args.amplitude}° ({amplitude_rad:.4f} rad)")
    print(f"频率:      {args.frequency} Hz")
    print(f"时长:      {args.duration} s")
    print(f"控制频率:  {args.rate} Hz")
    print(f"Kp={args.kp}, Kd={args.kd}")
    print(f"方向取反:  {args.negtive}, 零点偏移: {args.zero_offset} rad")
    print()

    if args.dry_run:
        print("[dry-run] 模拟发送 3 帧后退出\n")
        for i in range(3):
            t = i * interval
            sine_pos = amplitude_rad * math.sin(2 * math.pi * args.frequency * t)
            sine_vel = amplitude_rad * 2 * math.pi * args.frequency * math.cos(2 * math.pi * args.frequency * t)
            raw_pos = sine_pos + args.zero_offset
            raw_vel = sine_vel
            if args.negtive:
                raw_pos = -(sine_pos + args.zero_offset)
                raw_vel = -sine_vel
            frame = encode_mit_frame(raw_pos, raw_vel, 0.0, args.kp, args.kd)
            print(f"  t={t:.4f}s  pos={raw_pos:+.4f} rad  vel={raw_vel:+.4f}  frame={frame.hex(' ')}")
            print(f"    -> 发送到: {', '.join(f'0x{x:02X}' for x in motor_ids)}")
        return

    # ── 打开 SocketCAN ──
    try:
        sock = open_can_socket(args.can_bus)
    except OSError as e:
        print(f"打开 {args.can_bus} 失败: {e}")
        print(f"请确认接口存在且已启动: sudo ip link set {args.can_bus} up type can bitrate 1000000")
        sys.exit(1)

    print(f"SocketCAN {args.can_bus} 已打开")

    # ── Step 1: 使能电机 ──
    print(f"发送使能命令 → 电机 {', '.join(f'0x{x:02X}' for x in motor_ids)} ...")
    for motor_id in motor_ids:
        send_can_frame(sock, motor_id, ENABLE_FRAME)
        time.sleep(0.02)
    time.sleep(0.1)

    # ── Step 2: 读取当前位置作为 sine 中心 ──
    # 发一帧零力矩指令触发反馈 (不做标零, 不发 FF..FE)
    zero_frame = encode_mit_frame(0.0, 0.0, 0.0, 0.0, 0.0)
    for motor_id in motor_ids:
        send_can_frame(sock, motor_id, zero_frame)
        time.sleep(0.01)
    time.sleep(0.1)

    center_positions = {mid: 0.0 for mid in motor_ids}
    feedback_count = 0
    collect_deadline = time.monotonic() + 0.3
    while time.monotonic() < collect_deadline and feedback_count < len(motor_ids):
        feedback = recv_can_frame(sock)
        if not feedback:
            time.sleep(0.001)
            continue
        fb_id, fb_data = feedback
        decoded = decode_feedback(fb_data)
        if decoded and fb_id in center_positions:
            center_positions[fb_id] = decoded['fb_pos']
            feedback_count += 1
    print(f"读取到 {feedback_count}/{len(motor_ids)} 个电机反馈")
    print("各电机当前位置 (sine 中心):")
    for mid in motor_ids:
        print(f"  0x{mid:02X}: {center_positions[mid]:+.4f} rad")

    # ── CSV 准备 ──
    csv_file = None
    csv_fields = ['cmd_pos', 'cmd_vel', 'cmd_kp', 'cmd_kd', 'cmd_torque',
                  'fb_pos', 'fb_vel', 'fb_torque', 'fb_errcode', 'fb_temp']
    if args.csv:
        csv_file = open(args.csv, 'w')
        header = "time"
        for mid in motor_ids:
            for f in csv_fields:
                header += f",0x{mid:02X}/{f}"
        csv_file.write(header + "\n")
        print(f"CSV 保存到: {args.csv}")

    # ── Step 3: 正弦跟踪控制 ──
    print(f"\n开始正弦跟踪 (Ctrl+C 停止)...\n")

    start_time = time.monotonic()
    cycle_count = 0
    last_print_time = 0.0
    last_raw_pos = {mid: 0.0 for mid in motor_ids}
    last_cmd = {}      # {mid: decode_command dict}
    last_fb = {}       # {mid: decode_feedback dict} — 保留最新反馈

    global running
    try:
        while running:
            now = time.monotonic()
            elapsed = now - start_time

            if elapsed >= args.duration:
                print(f"\n已达到 {args.duration}s, 停止.")
                break

            # 正弦位置 + 速度前馈 (导数)
            sine_pos = amplitude_rad * math.sin(2 * math.pi * args.frequency * elapsed)
            sine_vel = amplitude_rad * 2 * math.pi * args.frequency * math.cos(2 * math.pi * args.frequency * elapsed)

            # 共享同一正弦轨迹，但叠加各自零点
            target_vel = sine_vel

            # 每个周期给所有电机下发一帧
            for motor_id in motor_ids:
                target_pos = center_positions[motor_id] + sine_pos
                if args.negtive:
                    raw_pos = -(target_pos + args.zero_offset)
                    raw_vel = -target_vel
                else:
                    raw_pos = target_pos + args.zero_offset
                    raw_vel = target_vel
                frame = encode_mit_frame(raw_pos, raw_vel, 0.0, args.kp, args.kd)
                send_can_frame(sock, motor_id, frame)
                last_raw_pos[motor_id] = raw_pos
                last_cmd[motor_id] = decode_command(frame)
            cycle_count += 1

            # 读取反馈 (非阻塞, 尽量读完)
            for _ in range(len(motor_ids) * 2):
                fb = recv_can_frame(sock)
                if not fb:
                    break
                fb_id, fb_data = fb
                # 前10个周期打印原始帧 hex, 确认反馈内容
                if cycle_count <= 10 and fb_id in center_positions:
                    print(f"  [rx] ID=0x{fb_id:03X} data={fb_data.hex(' ')}")
                # 只处理属于目标电机的帧
                if fb_id not in center_positions:
                    continue
                decoded = decode_feedback(fb_data)
                if decoded:
                    last_fb[fb_id] = decoded

            # 写 CSV (位置相对零点)
            if csv_file:
                line = f"{elapsed:.6f}"
                for mid in motor_ids:
                    cmd = last_cmd.get(mid, {})
                    fb = last_fb.get(mid, {})
                    zp = center_positions[mid]
                    for f in csv_fields:
                        val = cmd.get(f, fb.get(f, ''))
                        if isinstance(val, float):
                            if f in ('cmd_pos', 'fb_pos'):
                                val -= zp
                            line += f",{val:.6f}"
                        elif val != '':
                            line += f",{val}"
                        else:
                            line += ","
                csv_file.write(line + "\n")

            # 每 200ms 打印一次 (位置相对零点)
            if elapsed - last_print_time >= 0.2:
                first_mid = motor_ids[0]
                zp = center_positions[first_mid]
                cmd_p = last_cmd.get(first_mid, {}).get('cmd_pos', zp) - zp
                fb_p = last_fb.get(first_mid, {}).get('fb_pos')
                actual_str = f"{fb_p - zp:+.4f}" if fb_p is not None else "N/A"
                print(
                    f"  t={elapsed:6.2f}s  cmd[{first_mid:#04x}]={cmd_p:+.4f} rad"
                    f"  actual[{first_mid:#04x}]={actual_str} rad  vel_cmd={raw_vel:+.4f}"
                )
                last_print_time = elapsed

            # 精确控制周期
            next_time = start_time + (cycle_count) * interval
            sleep_time = next_time - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)

    except Exception as e:
        print(f"\n错误: {e}")

    # ── Step 4: 失能电机 ──
    print(f"发送失能命令 → 电机 {', '.join(f'0x{x:02X}' for x in motor_ids)}")
    for motor_id in motor_ids:
        send_can_frame(sock, motor_id, DISABLE_FRAME)
    time.sleep(0.1)

    sock.close()
    if csv_file:
        csv_file.close()
        print(f"数据已保存到 {args.csv}")
    print(f"完成. 共发送 {cycle_count} 帧控制命令.")


if __name__ == "__main__":
    main()
