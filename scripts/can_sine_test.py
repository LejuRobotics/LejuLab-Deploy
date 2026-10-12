#!/usr/bin/env python3
"""
CAN 直连正弦测试 — 绕过 DDS/hardware_node, 直接发 PTM 帧到电机

用法:
  sudo python3 can_sine_test.py --bus bcan2 --motors 1,2,3,4       # 4电机 (无头)
  sudo python3 can_sine_test.py --bus bcan2 --motors 1,2,3,4,9,10  # 6电机 (含头)
  sudo python3 can_sine_test.py --bus bcan3 --motors 5,6,7,8       # 右臂对照

  # 全部选项
  sudo python3 can_sine_test.py --bus bcan2 --motors 1,2,3,4,9,10 \
       --freq 250 --sine-hz 0.833 --amp 0.125 --kp 14.25 --kd 0.907 \
       --duration 15 --csv /tmp/can_sine_result.csv

注意: 运行前必须确保 hardware_node 已停止, 否则两边同时发帧会冲突!
"""

import socket
import struct
import time
import math
import csv
import sys
import signal
import argparse
import threading
import os
from collections import defaultdict

# ── Motorevo PTM 协议编码参数 ──────────────────────────
THETA_MIN, THETA_MAX = -12.5, 12.5
VEL_MIN,   VEL_MAX   = -10.0, 10.0
KP_MIN,    KP_MAX    = 0.0, 250.0
KD_MIN,    KD_MAX    = 0.0, 50.0
TAU_MIN,   TAU_MAX   = -50.0, 50.0

ENABLE_FRAME  = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
DISABLE_FRAME = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])


def float_to_uint(x, x_min, x_max, bits):
    span = x_max - x_min
    x = max(x_min, min(x_max, x))
    return int((x - x_min) / span * ((1 << bits) - 1))


def uint_to_float(x_int, x_min, x_max, bits):
    span = x_max - x_min
    return x_int / ((1 << bits) - 1) * span + x_min


def encode_ptm(pos, vel, kp, kd, torque):
    """编码 PTM 控制帧 (8 字节), 与 motor_ctrl.cpp:controlPTM 一致"""
    theta = float_to_uint(pos, THETA_MIN, THETA_MAX, 16)
    v     = float_to_uint(vel, VEL_MIN, VEL_MAX, 12)
    k_p   = float_to_uint(kp,  KP_MIN,  KP_MAX,  12)
    k_d   = float_to_uint(kd,  KD_MIN,  KD_MAX,  12)
    t     = float_to_uint(torque, TAU_MIN, TAU_MAX, 12)

    payload = bytearray(8)
    payload[0] = (theta >> 8) & 0xFF
    payload[1] = theta & 0xFF
    payload[2] = (v >> 4) & 0xFF
    payload[3] = ((v & 0x0F) << 4) | ((k_p >> 8) & 0x0F)
    payload[4] = k_p & 0xFF
    payload[5] = (k_d >> 4) & 0xFF
    payload[6] = ((k_d & 0x0F) << 4) | ((t >> 8) & 0x0F)
    payload[7] = t & 0xFF
    return bytes(payload)


def decode_feedback(data):
    """解码电机反馈帧 (8 字节) → (pos, vel, torque)"""
    if len(data) < 8:
        return None
    theta = (data[0] << 8) | data[1]
    v     = (data[2] << 4) | (data[3] >> 4)
    t     = ((data[6] & 0x0F) << 8) | data[7]

    pos    = uint_to_float(theta, THETA_MIN, THETA_MAX, 16)
    vel    = uint_to_float(v, VEL_MIN, VEL_MAX, 12)
    torque = uint_to_float(t, TAU_MIN, TAU_MAX, 12)
    return pos, vel, torque


# ── SocketCAN 封装 ──────────────────────────────────
CAN_RAW = 1
CAN_EFF_FLAG = 0x80000000
SOL_CAN_RAW = 101
CAN_RAW_FILTER = 1
CAN_RAW_RECV_OWN_MSGS = 4

def open_can_socket(iface):
    """打开 SocketCAN 原始套接字"""
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, CAN_RAW)
    s.bind((iface,))
    # 接收自发帧 (看到自己发的 TX 帧, 用于精确计时)
    s.setsockopt(SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, struct.pack("i", 1))
    s.setblocking(False)
    return s


def can_send(sock, can_id, data):
    """发送 CAN 帧, 返回发送时间戳"""
    # struct can_frame: __u32 can_id, __u8 dlc, __u8 pad, __u8 res0, __u8 res1, __u8 data[8]
    frame = struct.pack("=IB3x8s", can_id, len(data), data)
    t = time.monotonic()
    try:
        sock.send(frame)
    except BlockingIOError:
        return None  # TX buffer full
    return t


def can_recv(sock):
    """非阻塞接收, 返回 (can_id, data, recv_time) 或 None"""
    try:
        frame = sock.recv(16)
    except BlockingIOError:
        return None
    if len(frame) < 16:
        return None
    can_id, dlc = struct.unpack_from("=IB", frame, 0)
    data = frame[8:8 + dlc]
    return (can_id & 0x1FFFFFFF, data, time.monotonic())


# ── 主测试逻辑 ──────────────────────────────────────
running = True

def signal_handler(sig, frame):
    global running
    running = False

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


def main():
    global running

    parser = argparse.ArgumentParser(description="CAN 直连正弦测试")
    parser.add_argument("--bus", default="bcan2", help="CAN 接口 (默认 bcan2)")
    parser.add_argument("--motors", default="1,2,3,4",
                        help="电机 ID 列表, 逗号分隔 (默认 1,2,3,4)")
    parser.add_argument("--freq", type=int, default=250, help="控制频率 Hz (默认 250)")
    parser.add_argument("--sine-hz", type=float, default=0.833, help="正弦频率 Hz (默认 0.833)")
    parser.add_argument("--amp", type=float, default=0.125, help="正弦振幅 rad (默认 0.125)")
    parser.add_argument("--kp", type=float, default=14.25, help="位置 Kp (默认 14.25)")
    parser.add_argument("--kd", type=float, default=0.907, help="位置 Kd (默认 0.907)")
    parser.add_argument("--duration", type=float, default=15, help="测试时长 秒 (默认 15)")
    parser.add_argument("--csv", default="", help="CSV 输出路径 (默认自动生成)")
    parser.add_argument("--sine-motors", default="",
                        help="只对这些电机做正弦, 其余保持位置 (默认全部)")
    parser.add_argument("--head-kp", type=float, default=10.0, help="头部电机 Kp (默认 10)")
    parser.add_argument("--head-kd", type=float, default=1.0, help="头部电机 Kd (默认 1)")
    args = parser.parse_args()

    motor_ids = [int(x.strip()) for x in args.motors.split(",")]
    sine_ids = set(motor_ids)  # 默认全部做正弦
    if args.sine_motors:
        sine_ids = set(int(x.strip()) for x in args.sine_motors.split(","))
    head_ids = {9, 10}

    if not args.csv:
        ts = time.strftime("%Y%m%d_%H%M%S")
        args.csv = f"/tmp/can_sine_{args.bus}_{len(motor_ids)}m_{ts}.csv"

    print(f"╔══════════════════════════════════════════╗")
    print(f"║  CAN 直连正弦测试                         ║")
    print(f"╠══════════════════════════════════════════╣")
    print(f"║  总线: {args.bus:<10s}  电机: {args.motors:<16s} ║")
    print(f"║  频率: {args.freq}Hz    正弦: {args.sine_hz}Hz  振幅: {args.amp}rad ║")
    print(f"║  Kp: {args.kp:<6.2f}  Kd: {args.kd:<5.3f}  时长: {args.duration}s      ║")
    print(f"║  正弦电机: {','.join(str(i) for i in sorted(sine_ids)):<28s} ║")
    print(f"║  CSV: {args.csv}  ║")
    print(f"╚══════════════════════════════════════════╝\n")

    # ── 打开 CAN socket ──
    sock = open_can_socket(args.bus)
    print(f"[OK] SocketCAN {args.bus} 已打开")

    # ── 使能电机 ──
    print("使能电机...")
    for mid in motor_ids:
        can_send(sock, mid, ENABLE_FRAME)
        time.sleep(0.002)
    time.sleep(0.5)
    # 清空 RX buffer
    while can_recv(sock):
        pass
    print(f"[OK] {len(motor_ids)} 个电机已使能\n")

    # ── 读取初始位置 ──
    # 发一帧零力矩命令获取反馈
    init_pos = {}
    for mid in motor_ids:
        ptm = encode_ptm(0, 0, 0, 0, 0)
        can_send(sock, mid, ptm)
        time.sleep(0.005)
    time.sleep(0.05)
    while True:
        rx = can_recv(sock)
        if rx is None:
            break
        cid, data, _ = rx
        if cid in motor_ids and len(data) >= 8:
            fb = decode_feedback(data)
            if fb:
                init_pos[cid] = fb[0]

    for mid in motor_ids:
        pos = init_pos.get(mid, 0.0)
        print(f"  电机 0x{mid:02X}: 初始位置 = {pos:.4f} rad ({math.degrees(pos):.2f}°)")
    print()

    # ── 数据记录 ──
    tx_log = []   # (time, motor_id, cmd_pos)
    rx_log = []   # (time, motor_id, fb_pos, fb_vel, fb_tau)
    tx_drop = 0

    # ── RX 接收线程 ──
    rx_lock = threading.Lock()

    def rx_thread_func():
        while running:
            rx = can_recv(sock)
            if rx:
                cid, data, t_rx = rx
                if cid in motor_ids and len(data) >= 8:
                    fb = decode_feedback(data)
                    if fb:
                        with rx_lock:
                            rx_log.append((t_rx, cid, fb[0], fb[1], fb[2]))
            else:
                time.sleep(0.0001)

    rx_thread = threading.Thread(target=rx_thread_func, daemon=True)
    rx_thread.start()

    # ── 控制循环 ──
    period = 1.0 / args.freq
    t0 = time.monotonic()
    next_time = t0
    cycle = 0

    print(f"开始正弦测试 ({args.duration}s)... Ctrl+C 停止\n")

    try:
        while running and (time.monotonic() - t0) < args.duration:
            next_time += period
            now = time.monotonic()
            t_elapsed = now - t0

            for mid in motor_ids:
                center = init_pos.get(mid, 0.0)

                if mid in sine_ids:
                    cmd_pos = center + args.amp * math.sin(2 * math.pi * args.sine_hz * t_elapsed)
                else:
                    cmd_pos = center  # 非正弦电机保持位置

                kp = args.head_kp if mid in head_ids else args.kp
                kd = args.head_kd if mid in head_ids else args.kd

                ptm = encode_ptm(cmd_pos, 0.0, kp, kd, 0.0)
                t_tx = can_send(sock, mid, ptm)
                if t_tx:
                    tx_log.append((t_tx, mid, cmd_pos))
                else:
                    tx_drop += 1

            cycle += 1

            # 进度显示
            if cycle % (args.freq * 2) == 0:
                fb_counts = defaultdict(int)
                with rx_lock:
                    for _, cid, *_ in rx_log[-args.freq*2*len(motor_ids):]:
                        fb_counts[cid] += 1
                fb_str = " ".join(f"0x{k:02X}:{v}" for k, v in sorted(fb_counts.items()))
                print(f"\r  {t_elapsed:.1f}s  tx={cycle*len(motor_ids)}  "
                      f"rx={len(rx_log)}  drop={tx_drop}  fb=[{fb_str}]", end="", flush=True)

            # 精确定时
            sleep_time = next_time - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_time = time.monotonic()  # 超时重新同步

    except Exception as e:
        print(f"\n错误: {e}")

    print(f"\n\n测试结束: {cycle} 周期, TX={len(tx_log)}, RX={len(rx_log)}, drop={tx_drop}")

    # ── 停止电机 ──
    print("停用电机...")
    for _ in range(10):  # 多发几帧零力矩
        for mid in motor_ids:
            center = init_pos.get(mid, 0.0)
            ptm = encode_ptm(center, 0, args.kp, args.kd, 0)
            can_send(sock, mid, ptm)
        time.sleep(0.004)
    for mid in motor_ids:
        can_send(sock, mid, DISABLE_FRAME)
        time.sleep(0.002)
    print("[OK] 电机已停用")

    # ── 保存 CSV ──
    print(f"\n保存数据到 {args.csv} ...")

    # 合并 TX/RX 按时间排序
    with open(args.csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time_ms", "direction", "motor_id", "pos_rad", "vel_rad_s", "torque_Nm"])

        for t_tx, mid, pos in tx_log:
            w.writerow([f"{(t_tx - t0)*1000:.3f}", "TX", f"0x{mid:02X}", f"{pos:.6f}", "", ""])

        with rx_lock:
            for t_rx, mid, pos, vel, tau in rx_log:
                w.writerow([f"{(t_rx - t0)*1000:.3f}", "RX", f"0x{mid:02X}",
                            f"{pos:.6f}", f"{vel:.6f}", f"{tau:.6f}"])

    print(f"[OK] {len(tx_log) + len(rx_log)} 条记录已保存")

    # ── 快速分析 ──
    print(f"\n{'='*60}")
    print("快速分析: 每电机 cmd→fb 相位差")
    print(f"{'='*60}")

    # 按电机 ID 分组 TX 和 RX
    tx_by_motor = defaultdict(list)
    rx_by_motor = defaultdict(list)
    for t_tx, mid, pos in tx_log:
        tx_by_motor[mid].append((t_tx - t0, pos))
    with rx_lock:
        for t_rx, mid, pos, vel, tau in rx_log:
            rx_by_motor[mid].append((t_rx - t0, pos))

    for mid in sorted(motor_ids):
        txs = tx_by_motor.get(mid, [])
        rxs = rx_by_motor.get(mid, [])
        if not txs or not rxs:
            print(f"  0x{mid:02X}: TX={len(txs)} RX={len(rxs)} — 数据不足")
            continue

        # TX 帧间隔
        tx_dts = [txs[i+1][0] - txs[i][0] for i in range(len(txs)-1)]
        # RX 帧间隔
        rx_dts = [rxs[i+1][0] - rxs[i][0] for i in range(len(rxs)-1)]

        tx_hz = 1.0 / (sum(tx_dts) / len(tx_dts)) if tx_dts else 0
        rx_hz = 1.0 / (sum(rx_dts) / len(rx_dts)) if rx_dts else 0
        rx_max_gap = max(rx_dts) * 1000 if rx_dts else 0

        # 丢帧检测: RX 间隔 > 2.5x 期望值
        expected_dt = 1.0 / args.freq
        gaps = sum(1 for dt in rx_dts if dt > expected_dt * 2.5) if rx_dts else 0

        status = "OK" if gaps == 0 else f"gap={gaps}"
        if mid not in sine_ids:
            status += " (保持)"

        print(f"  0x{mid:02X}: TX={len(txs):>5} ({tx_hz:.0f}Hz)  "
              f"RX={len(rxs):>5} ({rx_hz:.0f}Hz)  "
              f"RX_max_gap={rx_max_gap:.1f}ms  {status}")

    print(f"\n完整数据: {args.csv}")
    print("用 PlotJuggler 或 Python 分析 TX pos vs RX pos 的相位差")

    sock.close()


if __name__ == "__main__":
    main()
