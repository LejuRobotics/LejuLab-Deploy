#!/usr/bin/env python3
"""
CAN RX 反馈帧监控脚本 — 排查 SPI-CAN RX FIFO 溢出 / 反馈丢帧

用法:
  sudo python3 can_rx_monitor.py [bcan2] [--duration 10]

功能:
  1. 监控指定 CAN 总线上每个电机 ID 的帧间隔
  2. 检测反馈丢帧 (间隔异常大 = FIFO 溢出丢了帧)
  3. 统计 TX/RX 帧数对比 (理论上每个 TX 命令帧对应一个 RX 反馈帧)
  4. 检测 CAN error frames (RX overflow indicator)
"""

import subprocess
import sys
import time
import signal
import re
from collections import defaultdict

# ── 配置 ──────────────────────────────────────────────
CAN_IFACE = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith('-') else "bcan2"
DURATION = 10  # 默认监控秒数
for i, a in enumerate(sys.argv):
    if a == '--duration' and i + 1 < len(sys.argv):
        DURATION = int(sys.argv[i + 1])

# Motorevo 电机 ID 映射 (根据 roban2_full_canfd_cofig.yaml)
MOTOR_NAMES = {
    0x01: "arm_l1", 0x02: "arm_l2", 0x03: "arm_l3", 0x04: "arm_l4",
    0x05: "arm_r1", 0x06: "arm_r2", 0x07: "arm_r3", 0x08: "arm_r4",
    0x09: "head_yaw", 0x0A: "head_pitch",
}

# 期望帧率 (Hz) — 手臂 CAN 单帧 250Hz, 每个电机每周期 1 cmd + 1 fb
EXPECTED_HZ = 250


# ── 数据结构 ──────────────────────────────────────────
class MotorStats:
    def __init__(self):
        self.frame_count = 0
        self.last_ts = None
        self.intervals = []       # 帧间隔 (ms)
        self.gap_count = 0        # 间隔 > 2x 期望值的次数
        self.max_interval = 0.0

    def update(self, ts):
        self.frame_count += 1
        if self.last_ts is not None:
            dt = (ts - self.last_ts) * 1000  # ms
            self.intervals.append(dt)
            if dt > self.max_interval:
                self.max_interval = dt
            # 每个电机期望 ~4ms 间隔 (250Hz), gap = 超过 2 倍
            expected_ms = 1000.0 / EXPECTED_HZ
            if dt > expected_ms * 2.5:
                self.gap_count += 1
        self.last_ts = ts


stats = defaultdict(MotorStats)
error_frame_count = 0
total_frames = 0
start_time = None
running = True


def signal_handler(sig, frame):
    global running
    running = False

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


# ── 解析 candump 输出 ────────────────────────────────
# candump -ta 格式: "(1713500000.123456)  bcan2  001   [8]  FF FF FF FF FF FF FF FC"
# candump -L  格式: "(1713500000.123456) bcan2 001#FFFFFFFFFFFFFFFF"
TS_PATTERN = re.compile(r'\((\d+\.\d+)\)')
ID_PATTERN_TA = re.compile(r'\s+([0-9A-Fa-f]+)\s+\[(\d+)\]')
ID_PATTERN_L = re.compile(r'\s+([0-9A-Fa-f]+)#')
ERR_PATTERN = re.compile(r'[0-9A-Fa-f]{8}#', re.IGNORECASE)  # 29-bit = error frame


def parse_candump_line(line):
    """解析一行 candump 输出, 返回 (timestamp, can_id, is_error)"""
    global error_frame_count

    # 时间戳
    ts_match = TS_PATTERN.search(line)
    if not ts_match:
        return None
    ts = float(ts_match.group(1))

    # 检测 error frame
    if 'ERR' in line.upper() or '20000004' in line or '20000008' in line:
        error_frame_count += 1
        return None

    # CAN ID (-ta 格式)
    id_match = ID_PATTERN_TA.search(line[ts_match.end():])
    if id_match:
        can_id = int(id_match.group(1), 16)
        return (ts, can_id, False)

    # CAN ID (-L 格式)
    id_match = ID_PATTERN_L.search(line[ts_match.end():])
    if id_match:
        can_id = int(id_match.group(1), 16)
        return (ts, can_id, False)

    return None


# ── 实时打印 ──────────────────────────────────────────
def print_stats(elapsed):
    print(f"\033[H\033[J", end="")  # 清屏
    print(f"═══ CAN RX Monitor: {CAN_IFACE}  ({elapsed:.1f}s / {DURATION}s) ═══")
    print(f"总帧数: {total_frames}   Error frames: {error_frame_count}")
    print()

    # 表头
    print(f"{'Motor':<12} {'帧数':>6} {'帧率Hz':>7} {'间隔avg':>8} {'间隔max':>8} "
          f"{'间隔std':>8} {'gap>2x':>6} {'状态'}")
    print("─" * 78)

    expected_ms = 1000.0 / EXPECTED_HZ

    for can_id in sorted(stats.keys()):
        if can_id > 0xFF:
            continue  # 跳过非标准 ID
        s = stats[can_id]
        name = MOTOR_NAMES.get(can_id, f"0x{can_id:02X}")

        if elapsed > 0:
            hz = s.frame_count / elapsed
        else:
            hz = 0

        if s.intervals:
            avg_ms = sum(s.intervals) / len(s.intervals)
            max_ms = s.max_interval
            if len(s.intervals) > 1:
                mean = avg_ms
                std_ms = (sum((x - mean) ** 2 for x in s.intervals) / len(s.intervals)) ** 0.5
            else:
                std_ms = 0
        else:
            avg_ms = max_ms = std_ms = 0

        # 状态判定
        status = "\033[32mOK\033[0m"
        if s.gap_count > 0:
            gap_pct = 100 * s.gap_count / max(1, len(s.intervals))
            if gap_pct > 5:
                status = f"\033[31m丢帧! {s.gap_count}次({gap_pct:.1f}%)\033[0m"
            else:
                status = f"\033[33m偶发gap {s.gap_count}次\033[0m"
        if max_ms > expected_ms * 5:
            status = f"\033[31m严重延迟 max={max_ms:.1f}ms\033[0m"
        if elapsed > 2 and hz < EXPECTED_HZ * 0.5:
            status = f"\033[31m帧率过低 {hz:.0f}Hz\033[0m"

        print(f"{name:<12} {s.frame_count:>6} {hz:>7.1f} {avg_ms:>7.2f}ms {max_ms:>7.2f}ms "
              f"{std_ms:>7.2f}ms {s.gap_count:>6} {status}")

    # 关键对比
    arm_ids = [i for i in [0x01, 0x02, 0x03, 0x04] if i in stats]
    head_ids = [i for i in [0x09, 0x0A] if i in stats]
    if arm_ids:
        arm_total_gaps = sum(stats[i].gap_count for i in arm_ids)
        arm_total_frames = sum(len(stats[i].intervals) for i in arm_ids)
        arm_max = max(stats[i].max_interval for i in arm_ids) if arm_ids else 0
        print(f"\n  手臂汇总: gap={arm_total_gaps}/{arm_total_frames}  max_interval={arm_max:.1f}ms")
    if head_ids:
        head_total_gaps = sum(stats[i].gap_count for i in head_ids)
        head_total_frames = sum(len(stats[i].intervals) for i in head_ids)
        head_max = max(stats[i].max_interval for i in head_ids) if head_ids else 0
        print(f"  头部汇总: gap={head_total_gaps}/{head_total_frames}  max_interval={head_max:.1f}ms")

    # TX/RX 对比提示
    if arm_ids and elapsed > 2:
        arm_hz = sum(stats[i].frame_count for i in arm_ids) / elapsed / len(arm_ids)
        print(f"\n  每电机平均帧率: {arm_hz:.0f}Hz (期望 ≥{EXPECTED_HZ * 2}Hz = TX+RX)")
        # 每个电机期望 TX+RX = 500Hz (250Hz cmd + 250Hz fb)
        # 如果只看到 ~250Hz, 说明只收到了一半 (可能 TX 被 loopback 过滤了)
        # 如果看到 ~500Hz, 说明 TX+RX 都在
        if arm_hz < EXPECTED_HZ * 1.5:
            print(f"  \033[33m提示: 帧率 ~{arm_hz:.0f}Hz ≈ 只有 RX (正常, candump 默认不含自发帧)\033[0m")


# ── 主循环 ────────────────────────────────────────────
def main():
    global total_frames, start_time, running

    print(f"启动 candump {CAN_IFACE} 监控 {DURATION}s ...")
    print(f"(Ctrl+C 提前结束)\n")

    # 启动 candump (带绝对时间戳)
    try:
        proc = subprocess.Popen(
            ["candump", "-ta", CAN_IFACE],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
    except FileNotFoundError:
        print("\033[31mcandump 未找到, 请安装 can-utils: apt install can-utils\033[0m")
        sys.exit(1)

    start_time = time.monotonic()
    last_print = start_time

    try:
        while running:
            line = proc.stdout.readline()
            if not line:
                break

            result = parse_candump_line(line)
            if result:
                ts, can_id, _ = result
                stats[can_id].update(ts)
                total_frames += 1

            now = time.monotonic()
            elapsed = now - start_time

            # 每 0.5 秒刷新显示
            if now - last_print >= 0.5:
                print_stats(elapsed)
                last_print = now

            if elapsed >= DURATION:
                break

    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()

    # 最终报告
    elapsed = time.monotonic() - start_time
    print_stats(elapsed)
    print(f"\n监控结束, 共 {elapsed:.1f}s")

    # 保存详细数据
    log_path = f"/tmp/can_rx_monitor_{CAN_IFACE}.log"
    with open(log_path, "w") as f:
        f.write(f"CAN RX Monitor: {CAN_IFACE}, duration={elapsed:.1f}s\n")
        f.write(f"total_frames={total_frames}, error_frames={error_frame_count}\n\n")
        for can_id in sorted(stats.keys()):
            s = stats[can_id]
            name = MOTOR_NAMES.get(can_id, f"0x{can_id:02X}")
            hz = s.frame_count / elapsed if elapsed > 0 else 0
            if s.intervals:
                avg_ms = sum(s.intervals) / len(s.intervals)
                max_ms = s.max_interval
            else:
                avg_ms = max_ms = 0
            f.write(f"{name} (0x{can_id:02X}): frames={s.frame_count} hz={hz:.1f} "
                    f"avg={avg_ms:.2f}ms max={max_ms:.2f}ms gaps={s.gap_count}\n")

        # 写入原始间隔数据 (用于离线分析)
        f.write("\n\n=== Raw intervals (ms) per motor ===\n")
        for can_id in sorted(stats.keys()):
            s = stats[can_id]
            name = MOTOR_NAMES.get(can_id, f"0x{can_id:02X}")
            if s.intervals:
                # 只写前 2000 个和最后 500 个
                sample = s.intervals[:2000]
                if len(s.intervals) > 2500:
                    sample += s.intervals[-500:]
                f.write(f"\n[{name}] ({len(s.intervals)} total, showing {len(sample)}):\n")
                for iv in sample:
                    f.write(f"{iv:.3f}\n")

    print(f"详细日志已保存: {log_path}")


if __name__ == "__main__":
    main()
