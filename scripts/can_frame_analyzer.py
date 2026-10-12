#!/usr/bin/env python3
"""
CAN 帧丢帧分析工具 — 离线分析 candump 日志

用法:
  # 1. 先抓包
  candump -ta bcan2 > /tmp/bcan2.log
  # 2. 分析
  python3 can_frame_analyzer.py /tmp/bcan2.log

  # 或实时抓包+分析
  candump -ta bcan2 | python3 can_frame_analyzer.py -

分析内容:
  1. 每个电机的 cmd/fb 帧数和帧率
  2. cmd 帧间隔统计: avg/max/std/jitter
  3. 丢帧检测: 帧间隔 > N 倍期望值
  4. cmd→fb round-trip 时间
  5. 每个周期内的帧发送顺序和间隔
  6. 帧间隔直方图
"""

import sys
import re
from collections import defaultdict
import math

# ── Motorevo PTM 协议 ──
THETA_MIN, THETA_MAX = -12.5, 12.5
VEL_MIN, VEL_MAX = -10.0, 10.0
TAU_MIN, TAU_MAX = -50.0, 50.0
KP_MIN, KP_MAX = 0.0, 250.0
KD_MIN, KD_MAX = 0.0, 50.0

MOTOR_NAMES = {
    0x01: "arm_l1", 0x02: "arm_l2", 0x03: "arm_l3", 0x04: "arm_l4",
    0x05: "arm_r1", 0x06: "arm_r2", 0x07: "arm_r3", 0x08: "arm_r4",
    0x09: "head_y", 0x0A: "head_p",
}

ENABLE_FRAME  = "FF FF FF FF FF FF FF FC"
DISABLE_FRAME = "FF FF FF FF FF FF FF FD"


def uint_to_float(x, x_min, x_max, bits):
    return x / ((1 << bits) - 1) * (x_max - x_min) + x_min


def decode_ptm_cmd(data_bytes):
    """解码 PTM 命令帧 → (pos, vel, kp, kd, tau)"""
    d = data_bytes
    theta = (d[0] << 8) | d[1]
    v = (d[2] << 4) | (d[3] >> 4)
    kp = ((d[3] & 0x0F) << 8) | d[4]
    kd = (d[5] << 4) | (d[6] >> 4)
    tau = ((d[6] & 0x0F) << 8) | d[7]
    return (
        uint_to_float(theta, THETA_MIN, THETA_MAX, 16),
        uint_to_float(v, VEL_MIN, VEL_MAX, 12),
        uint_to_float(kp, KP_MIN, KP_MAX, 12),
        uint_to_float(kd, KD_MIN, KD_MAX, 12),
        uint_to_float(tau, TAU_MIN, TAU_MAX, 12),
    )


def decode_ptm_fb(data_bytes):
    """解码 PTM 反馈帧 → (motor_id_byte, pos, vel, tau)"""
    d = data_bytes
    mid = d[0]
    theta = (d[1] << 8) | d[2]
    v = (d[3] << 4) | (d[4] >> 4)
    tau = ((d[6] & 0x0F) << 8) | d[7]
    return (
        mid,
        uint_to_float(theta, THETA_MIN, THETA_MAX, 16),
        uint_to_float(v, VEL_MIN, VEL_MAX, 12),
        uint_to_float(tau, TAU_MIN, TAU_MAX, 12),
    )


def classify_frame(can_id, data_hex, data_bytes):
    """分类帧: 'enable'/'disable'/'cmd'/'fb'/'unknown'"""
    if data_hex == ENABLE_FRAME:
        return 'enable'
    if data_hex == DISABLE_FRAME:
        return 'disable'
    if len(data_bytes) < 8:
        return 'unknown'
    # 反馈帧: byte[0] = motor_id (低值 0x01-0x0F)
    # 命令帧: byte[0] = position high byte (通常 0x70-0x90)
    if data_bytes[0] == can_id and can_id <= 0x10:
        return 'fb'
    if data_bytes[0] > 0x10:
        return 'cmd'
    # 边界情况: byte[0] 恰好等于 can_id 但也可能是位置值
    return 'fb' if data_bytes[0] <= 0x10 else 'cmd'


def parse_candump_line(line):
    """解析 candump -ta 格式"""
    m = re.match(r'\((\d+\.\d+)\)\s+\S+\s+([0-9A-Fa-f]+)\s+\[(\d+)\]\s+(.*)', line.strip())
    if not m:
        return None
    ts = float(m.group(1))
    cid = int(m.group(2), 16)
    dlc = int(m.group(3))
    data_hex = m.group(4).strip()
    data_bytes = [int(x, 16) for x in data_hex.split()] if data_hex else []
    return ts, cid, dlc, data_hex, data_bytes


def percentile(data, p):
    if not data:
        return 0
    k = (len(data) - 1) * p / 100
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return data[int(k)]
    return data[f] * (c - k) + data[c] * (k - f)


def analyze(frames):
    if not frames:
        print("无数据!")
        return

    t0 = frames[0][0]
    duration = frames[-1][0] - t0
    print(f"总帧数: {len(frames)}  时间: {duration:.1f}s\n")

    # ── 分类统计 ──
    cmd_by_id = defaultdict(list)   # cid → [(ts, pos, vel, kp, kd, tau)]
    fb_by_id = defaultdict(list)    # cid → [(ts, pos, vel, tau)]
    enable_count = defaultdict(int)
    disable_count = defaultdict(int)

    for ts, cid, dlc, data_hex, data_bytes in frames:
        ftype = classify_frame(cid, data_hex, data_bytes)
        if ftype == 'cmd':
            decoded = decode_ptm_cmd(data_bytes)
            cmd_by_id[cid].append((ts,) + decoded)
        elif ftype == 'fb':
            decoded = decode_ptm_fb(data_bytes)
            fb_by_id[cid].append((ts,) + decoded[1:])  # skip motor_id byte
        elif ftype == 'enable':
            enable_count[cid] += 1
        elif ftype == 'disable':
            disable_count[cid] += 1

    # ── 1. 帧率总览 ──
    print(f"{'='*80}")
    print(f"  1. 帧率总览")
    print(f"{'='*80}")
    print(f"  {'ID':<12} {'cmd数':>6} {'cmd_Hz':>7} {'fb数':>6} {'fb_Hz':>7} {'en':>3} {'dis':>3} {'丢帧率':>7}")
    print(f"  {'-'*65}")

    all_ids = sorted(set(list(cmd_by_id.keys()) + list(fb_by_id.keys())))
    for cid in all_ids:
        if cid > 0xFF:
            continue
        cmds = cmd_by_id[cid]
        fbs = fb_by_id[cid]
        c_hz = len(cmds) / duration if duration > 0 else 0
        f_hz = len(fbs) / duration if duration > 0 else 0
        # 丢帧率: 1 - fb/cmd (每个 cmd 应有一个 fb)
        loss = 1.0 - len(fbs) / len(cmds) if len(cmds) > 0 else 0
        name = MOTOR_NAMES.get(cid, f'0x{cid:02X}')
        print(f"  0x{cid:02X} {name:<5} {len(cmds):>6} {c_hz:>7.1f} {len(fbs):>6} {f_hz:>7.1f} "
              f"{enable_count[cid]:>3} {disable_count[cid]:>3} {loss*100:>6.1f}%")

    # ── 2. CMD 帧间隔分析 ──
    print(f"\n{'='*80}")
    print(f"  2. CMD 帧间隔分析 (检测丢帧)")
    print(f"{'='*80}")
    print(f"  {'ID':<12} {'avg':>7} {'p50':>7} {'p95':>7} {'p99':>7} {'max':>8} {'std':>7} "
          f"{'gap>2x':>6} {'gap>4x':>6} {'gap>8x':>6}")
    print(f"  {'-'*80}")

    for cid in all_ids:
        if cid > 0xFF:
            continue
        cmds = cmd_by_id[cid]
        if len(cmds) < 20:
            continue
        # 跳过前后 10% (init/cleanup)
        margin = len(cmds) // 10
        ts_list = [c[0] for c in cmds[margin:-margin]] if margin > 5 else [c[0] for c in cmds]
        dts = sorted([(ts_list[i+1] - ts_list[i]) * 1000 for i in range(len(ts_list) - 1)])

        if not dts:
            continue

        avg = sum(dts) / len(dts)
        std = (sum((x - avg)**2 for x in dts) / len(dts)) ** 0.5
        p50 = percentile(dts, 50)
        p95 = percentile(dts, 95)
        p99 = percentile(dts, 99)
        mx = dts[-1]

        gap2x = sum(1 for d in dts if d > avg * 2.5)
        gap4x = sum(1 for d in dts if d > avg * 4)
        gap8x = sum(1 for d in dts if d > avg * 8)

        name = MOTOR_NAMES.get(cid, f'0x{cid:02X}')
        print(f"  0x{cid:02X} {name:<5} {avg:>6.2f}ms {p50:>6.2f}ms {p95:>6.2f}ms {p99:>6.2f}ms "
              f"{mx:>7.1f}ms {std:>6.2f}ms {gap2x:>6} {gap4x:>6} {gap8x:>6}")

    # ── 3. FB 帧间隔分析 ──
    print(f"\n{'='*80}")
    print(f"  3. FB (反馈) 帧间隔分析")
    print(f"{'='*80}")
    print(f"  {'ID':<12} {'avg':>7} {'p50':>7} {'p95':>7} {'p99':>7} {'max':>8} {'gap>2x':>6}")
    print(f"  {'-'*60}")

    for cid in all_ids:
        if cid > 0xFF:
            continue
        fbs = fb_by_id[cid]
        if len(fbs) < 20:
            continue
        margin = len(fbs) // 10
        ts_list = [f[0] for f in fbs[margin:-margin]] if margin > 5 else [f[0] for f in fbs]
        dts = sorted([(ts_list[i+1] - ts_list[i]) * 1000 for i in range(len(ts_list) - 1)])

        if not dts:
            continue

        avg = sum(dts) / len(dts)
        p50 = percentile(dts, 50)
        p95 = percentile(dts, 95)
        p99 = percentile(dts, 99)
        mx = dts[-1]
        gap2x = sum(1 for d in dts if d > avg * 2.5)

        name = MOTOR_NAMES.get(cid, f'0x{cid:02X}')
        print(f"  0x{cid:02X} {name:<5} {avg:>6.2f}ms {p50:>6.2f}ms {p95:>6.2f}ms {p99:>6.2f}ms "
              f"{mx:>7.1f}ms {gap2x:>6}")

    # ── 4. CMD→FB round-trip ──
    print(f"\n{'='*80}")
    print(f"  4. CMD→FB Round-Trip Time")
    print(f"{'='*80}")
    print(f"  {'ID':<12} {'avg':>7} {'p50':>7} {'p95':>7} {'max':>7} {'n':>7}")
    print(f"  {'-'*50}")

    for cid in all_ids:
        if cid > 0xFF:
            continue
        cmds = cmd_by_id[cid]
        fbs = fb_by_id[cid]
        if len(cmds) < 10 or len(fbs) < 10:
            continue

        cmd_ts = [c[0] for c in cmds]
        fb_ts = [f[0] for f in fbs]
        rtts = []
        fi = 0
        for ct in cmd_ts:
            while fi < len(fb_ts) and fb_ts[fi] <= ct:
                fi += 1
            if fi < len(fb_ts):
                rtt = (fb_ts[fi] - ct) * 1000
                if 0 < rtt < 50:
                    rtts.append(rtt)

        if not rtts:
            continue
        rtts.sort()
        avg = sum(rtts) / len(rtts)
        p50 = percentile(rtts, 50)
        p95 = percentile(rtts, 95)

        name = MOTOR_NAMES.get(cid, f'0x{cid:02X}')
        print(f"  0x{cid:02X} {name:<5} {avg:>6.3f}ms {p50:>6.3f}ms {p95:>6.3f}ms {rtts[-1]:>6.3f}ms {len(rtts):>7}")

    # ── 5. 周期内帧序 (取中间 200 个周期) ──
    min_cid = min(cmd_by_id.keys()) if cmd_by_id else None
    if min_cid is not None and len(cmd_by_id[min_cid]) > 100:
        print(f"\n{'='*80}")
        print(f"  5. 周期内帧发送顺序 (以 0x{min_cid:02X} 为周期起点)")
        print(f"{'='*80}")

        cycle_ts = [c[0] for c in cmd_by_id[min_cid]]
        # 取中间的周期
        mid = len(cycle_ts) // 2
        start = max(0, mid - 100)
        end = min(len(cycle_ts) - 1, mid + 100)

        # 所有 cmd 帧按时间排序
        all_cmds = []
        for cid, clist in cmd_by_id.items():
            for c in clist:
                all_cmds.append((c[0], cid, c[1]))  # (ts, cid, pos)
        all_cmds.sort()

        # 统计周期内各电机的偏移
        offsets = defaultdict(list)
        for ci in range(start, end):
            t_cycle = cycle_ts[ci]
            t_next = cycle_ts[ci + 1] if ci + 1 < len(cycle_ts) else t_cycle + 0.02
            for ts, cid, pos in all_cmds:
                if ts < t_cycle - 0.0001:
                    continue
                if ts >= t_next - 0.0001:
                    break
                offsets[cid].append((ts - t_cycle) * 1000)

        print(f"  (统计 {end - start} 个周期)")
        print(f"  {'ID':<12} {'avg_offset':>10} {'min':>8} {'max':>8} {'std':>8} {'count/cycle':>11}")
        print(f"  {'-'*55}")
        for cid in sorted(offsets.keys()):
            offs = offsets[cid]
            avg = sum(offs) / len(offs)
            mn = min(offs)
            mx = max(offs)
            std = (sum((x - avg)**2 for x in offs) / len(offs)) ** 0.5
            per_cycle = len(offs) / (end - start)
            name = MOTOR_NAMES.get(cid, f'0x{cid:02X}')
            print(f"  0x{cid:02X} {name:<5} {avg:>9.3f}ms {mn:>7.3f}ms {mx:>7.3f}ms {std:>7.3f}ms {per_cycle:>10.2f}")

    # ── 6. CMD 帧位置值变化检测 ──
    print(f"\n{'='*80}")
    print(f"  6. CMD 帧位置值变化 (检测重复/过期命令)")
    print(f"{'='*80}")

    for cid in all_ids:
        if cid > 0xFF:
            continue
        cmds = cmd_by_id[cid]
        if len(cmds) < 20:
            continue
        margin = len(cmds) // 10
        cmds_mid = cmds[margin:-margin] if margin > 5 else cmds

        pos_values = [c[1] for c in cmds_mid]
        # 统计连续相同位置值的次数 (位置不变 = 可能是重复帧)
        same_count = 0
        for i in range(1, len(pos_values)):
            if abs(pos_values[i] - pos_values[i-1]) < 1e-6:
                same_count += 1

        pos_range = max(pos_values) - min(pos_values)
        name = MOTOR_NAMES.get(cid, f'0x{cid:02X}')
        print(f"  0x{cid:02X} {name:<5}: pos range={pos_range:.4f}rad  "
              f"连续相同pos={same_count}/{len(pos_values)} ({100*same_count/len(pos_values):.1f}%)")


def main():
    if len(sys.argv) < 2:
        print("用法: python3 can_frame_analyzer.py <candump_log>")
        print("      candump -ta bcan2 | python3 can_frame_analyzer.py -")
        sys.exit(1)

    source = sys.argv[1]
    frames = []

    if source == '-':
        print("从 stdin 读取 candump 数据 (Ctrl+C 停止)...\n")
        try:
            for line in sys.stdin:
                result = parse_candump_line(line)
                if result:
                    frames.append(result)
        except KeyboardInterrupt:
            pass
    else:
        with open(source) as f:
            for line in f:
                result = parse_candump_line(line)
                if result:
                    frames.append(result)

    analyze(frames)


if __name__ == "__main__":
    main()
