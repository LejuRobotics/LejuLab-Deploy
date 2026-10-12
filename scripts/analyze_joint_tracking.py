#!/usr/bin/env python3
"""
关节跟踪性能分析工具

分析 keyboard_ctrl CSV 中 cmd vs fb 的相位延迟、幅度衰减、跟踪误差。
支持频域分析(Bode图)来定量评估控制带宽。

用法:
    python3 analyze_joint_tracking.py <csv_file> [joint_name]
    python3 analyze_joint_tracking.py <csv_file> --all

示例:
    python3 analyze_joint_tracking.py keyboard_ctrl.csv zarm_l1_joint
    python3 analyze_joint_tracking.py keyboard_ctrl.csv leg_l1_joint
    python3 analyze_joint_tracking.py keyboard_ctrl.csv --all
"""

import sys
import os
import numpy as np
import pandas as pd
from dataclasses import dataclass
from typing import Optional


@dataclass
class TrackingResult:
    joint_name: str
    # 时域
    phase_delay_ms: float        # 互相关相位延迟
    amplitude_ratio: float       # fb/cmd 幅度比
    correlation: float           # 归一化互相关峰值
    mean_abs_error_rad: float    # 平均绝对跟踪误差
    max_abs_error_rad: float     # 最大绝对跟踪误差
    rms_error_rad: float         # RMS 跟踪误差
    # 频域
    bandwidth_hz: Optional[float]  # -3dB 带宽
    phase_at_1hz_deg: Optional[float]  # 1Hz 处相位
    gain_at_1hz_db: Optional[float]    # 1Hz 处增益
    # 元数据
    sample_rate_hz: float
    active_duration_s: float
    cmd_range_deg: float
    fb_range_deg: float


def find_joint_columns(df, joint_name):
    """查找 cmd 和 fb 列名 (兼容多种 CSV 格式)

    格式1 (PlotJuggler): cmd_p/Lleg_joint_01, fb_p/Lleg_joint_01
    格式2 (旧): cmd_zarm_l1_joint, fb_zarm_l1_joint
    """
    # 格式1: cmd_p/<name> / fb_p/<name>
    cmd_col = f"cmd_p/{joint_name}"
    fb_col = f"fb_p/{joint_name}"
    if cmd_col in df.columns and fb_col in df.columns:
        return cmd_col, fb_col

    # 格式2 (旧): cmd_<name> / fb_<name>
    cmd_col = f"cmd_{joint_name}"
    fb_col = f"fb_{joint_name}"
    if cmd_col in df.columns and fb_col in df.columns:
        return cmd_col, fb_col

    return None, None


def get_active_segment(cmd, window=50, threshold_ratio=0.05):
    """找到信号活跃变化的区段 (用滑动窗口检测，适合慢变信号)"""
    cmd_range = cmd.max() - cmd.min()
    if cmd_range < 0.001:  # < 0.06 deg
        return None, None

    # 滑动窗口内的变化幅度
    n = len(cmd)
    if n < window * 2:
        return None, None

    local_range = np.array([
        cmd[max(0, i - window):min(n, i + window)].ptp()
        for i in range(n)
    ])
    threshold = cmd_range * threshold_ratio
    active = local_range > threshold
    indices = np.where(active)[0]
    if len(indices) < 50:
        return None, None
    return max(0, indices[0] - 10), min(n, indices[-1] + 10)


def cross_correlation_delay(cmd, fb, dt_s, max_lag_ms=500):
    """互相关法计算相位延迟"""
    cmd_d = cmd - np.mean(cmd)
    fb_d = fb - np.mean(fb)

    norm = np.linalg.norm(cmd_d) * np.linalg.norm(fb_d)
    if norm < 1e-12:
        return 0.0, 0.0

    corr = np.correlate(cmd_d, fb_d, mode='full')
    corr /= norm
    lags = np.arange(-len(cmd_d) + 1, len(cmd_d))

    max_lag_samples = int(max_lag_ms / 1000.0 / dt_s)
    center = len(cmd_d) - 1
    lo = max(0, center - max_lag_samples)
    hi = min(len(corr), center + max_lag_samples)

    valid_lags = lags[lo:hi]
    valid_corr = corr[lo:hi]

    best_idx = np.argmax(valid_corr)
    best_lag = valid_lags[best_idx]
    best_corr = valid_corr[best_idx]

    # 抛物线插值提高精度
    if 0 < best_idx < len(valid_corr) - 1:
        y0 = valid_corr[best_idx - 1]
        y1 = valid_corr[best_idx]
        y2 = valid_corr[best_idx + 1]
        denom = 2.0 * (2 * y1 - y0 - y2)
        if abs(denom) > 1e-12:
            offset = (y0 - y2) / denom
            best_lag = best_lag + offset

    delay_ms = best_lag * dt_s * 1000.0
    return delay_ms, best_corr


def frequency_analysis(cmd, fb, dt_s):
    """频域分析: 计算传递函数 H(f) = FB(f) / CMD(f)"""
    n = len(cmd)
    if n < 128:
        return None, None, None, None, None, None

    # 去均值 + 窗函数
    cmd_d = (cmd - np.mean(cmd)) * np.hanning(n)
    fb_d = (fb - np.mean(fb)) * np.hanning(n)

    CMD = np.fft.rfft(cmd_d)
    FB = np.fft.rfft(fb_d)
    freqs = np.fft.rfftfreq(n, d=dt_s)

    # 传递函数 H = FB / CMD (避免除零)
    mag_cmd = np.abs(CMD)
    threshold = np.max(mag_cmd) * 0.01  # 只分析信噪比足够的频率
    valid = mag_cmd > threshold

    gain_db = np.full_like(freqs, np.nan)
    phase_deg = np.full_like(freqs, np.nan)

    H = np.zeros_like(CMD, dtype=complex)
    H[valid] = FB[valid] / CMD[valid]
    gain_db[valid] = 20 * np.log10(np.abs(H[valid]) + 1e-12)
    phase_deg[valid] = np.degrees(np.angle(H[valid]))

    # 展开相位 (避免 ±180° 跳变)
    valid_mask = ~np.isnan(phase_deg)
    if valid_mask.sum() > 3:
        phase_deg[valid_mask] = np.unwrap(np.radians(phase_deg[valid_mask]))
        phase_deg[valid_mask] = np.degrees(phase_deg[valid_mask])

    # 找 -3dB 带宽
    bandwidth = None
    dc_gain = gain_db[valid][0] if valid.sum() > 0 else 0
    for i in range(len(freqs)):
        if valid[i] and freqs[i] > 0.1 and gain_db[i] < dc_gain - 3:
            bandwidth = freqs[i]
            break

    # 1Hz 处的增益和相位
    gain_1hz = None
    phase_1hz = None
    idx_1hz = np.argmin(np.abs(freqs - 1.0))
    if valid[idx_1hz]:
        gain_1hz = gain_db[idx_1hz]
        phase_1hz = phase_deg[idx_1hz]

    return freqs, gain_db, phase_deg, bandwidth, gain_1hz, phase_1hz


def analyze_joint(df, joint_name):
    """分析单个关节的跟踪性能"""
    cmd_col, fb_col = find_joint_columns(df, joint_name)
    if cmd_col is None:
        return None

    time_s = df['time_ms'].values / 1000.0
    cmd = df[cmd_col].values
    fb = df[fb_col].values

    # 检查是否有活跃信号
    cmd_range = cmd.max() - cmd.min()
    if cmd_range < 0.001:  # < 0.06 deg, 几乎没动
        return None

    start, end = get_active_segment(cmd)
    if start is None:
        return None

    cmd_seg = cmd[start:end]
    fb_seg = fb[start:end]
    time_seg = time_s[start:end]
    dt_s = np.mean(np.diff(time_seg))
    sample_rate = 1.0 / dt_s

    # 互相关延迟
    delay_ms, corr = cross_correlation_delay(cmd_seg, fb_seg, dt_s)

    # 幅度比
    cmd_amp = cmd_seg.max() - cmd_seg.min()
    fb_amp = fb_seg.max() - fb_seg.min()
    amp_ratio = fb_amp / cmd_amp if cmd_amp > 1e-9 else 1.0

    # 跟踪误差
    error = fb_seg - cmd_seg
    mean_abs_err = np.abs(error).mean()
    max_abs_err = np.abs(error).max()
    rms_err = np.sqrt(np.mean(error ** 2))

    # 频域分析
    freqs, gain_db, phase_deg, bandwidth, gain_1hz, phase_1hz = \
        frequency_analysis(cmd_seg, fb_seg, dt_s)

    return TrackingResult(
        joint_name=joint_name,
        phase_delay_ms=delay_ms,
        amplitude_ratio=amp_ratio,
        correlation=corr,
        mean_abs_error_rad=mean_abs_err,
        max_abs_error_rad=max_abs_err,
        rms_error_rad=rms_err,
        bandwidth_hz=bandwidth,
        phase_at_1hz_deg=phase_1hz,
        gain_at_1hz_db=gain_1hz,
        sample_rate_hz=sample_rate,
        active_duration_s=time_seg[-1] - time_seg[0],
        cmd_range_deg=np.degrees(cmd_amp),
        fb_range_deg=np.degrees(fb_amp),
    )


def print_result(r: TrackingResult):
    """打印单个关节分析结果"""
    print(f"\n{'='*60}")
    print(f"  关节: {r.joint_name}")
    print(f"{'='*60}")
    print(f"  采样率:       {r.sample_rate_hz:.0f} Hz")
    print(f"  活跃时长:     {r.active_duration_s:.1f} s")
    print(f"  cmd 幅度:     {r.cmd_range_deg:.2f} deg")
    print(f"  fb  幅度:     {r.fb_range_deg:.2f} deg")
    print()
    print(f"  --- 时域 ---")
    print(f"  相位延迟:     {r.phase_delay_ms:.1f} ms  {'⚠️ >50ms' if abs(r.phase_delay_ms) > 50 else '✓'}")
    print(f"  幅度衰减:     {(1-r.amplitude_ratio)*100:.1f}%  {'⚠️ >20%' if (1-r.amplitude_ratio) > 0.2 else '✓'}")
    print(f"  互相关系数:   {r.correlation:.4f}  {'⚠️ <0.95' if r.correlation < 0.95 else '✓'}")
    print(f"  平均跟踪误差: {np.degrees(r.mean_abs_error_rad):.2f} deg ({r.mean_abs_error_rad:.4f} rad)")
    print(f"  最大跟踪误差: {np.degrees(r.max_abs_error_rad):.2f} deg ({r.max_abs_error_rad:.4f} rad)")
    print(f"  RMS 误差:     {np.degrees(r.rms_error_rad):.2f} deg ({r.rms_error_rad:.4f} rad)")
    print()
    print(f"  --- 频域 ---")
    if r.bandwidth_hz is not None:
        print(f"  -3dB 带宽:    {r.bandwidth_hz:.2f} Hz  {'⚠️ <2Hz' if r.bandwidth_hz < 2 else '✓'}")
    else:
        print(f"  -3dB 带宽:    未检测到 (信号带宽内未衰减 3dB)")
    if r.gain_at_1hz_db is not None:
        print(f"  1Hz 处增益:   {r.gain_at_1hz_db:.1f} dB")
    if r.phase_at_1hz_deg is not None:
        print(f"  1Hz 处相位:   {r.phase_at_1hz_deg:.1f} deg")


def print_summary_table(results):
    """汇总表"""
    print(f"\n{'='*100}")
    print(f"  汇总")
    print(f"{'='*100}")
    header = f"{'关节':<22} {'延迟(ms)':>9} {'衰减%':>7} {'相关':>7} {'误差(deg)':>10} {'带宽(Hz)':>9} {'评价':<8}"
    print(header)
    print("-" * 100)
    for r in results:
        bw_str = f"{r.bandwidth_hz:.2f}" if r.bandwidth_hz else "N/A"
        # 评价
        issues = []
        if abs(r.phase_delay_ms) > 50:
            issues.append("延迟大")
        if (1 - r.amplitude_ratio) > 0.2:
            issues.append("衰减大")
        if r.correlation < 0.95:
            issues.append("相关低")
        verdict = ", ".join(issues) if issues else "OK"

        print(f"  {r.joint_name:<20} {r.phase_delay_ms:>8.1f} "
              f"{(1-r.amplitude_ratio)*100:>6.1f} {r.correlation:>7.4f} "
              f"{np.degrees(r.mean_abs_error_rad):>9.2f} {bw_str:>9} {verdict}")


def try_plot(df, joint_name, result, output_dir):
    """尝试画图 (matplotlib 可选)"""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("  [跳过绘图: matplotlib 未安装]")
        return

    cmd_col, fb_col = find_joint_columns(df, joint_name)
    if cmd_col is None:
        return

    time_s = df['time_ms'].values / 1000.0
    cmd = df[cmd_col].values
    fb = df[fb_col].values

    start, end = get_active_segment(cmd)
    if start is None:
        return

    cmd_seg = cmd[start:end]
    fb_seg = fb[start:end]
    time_seg = time_s[start:end]
    dt_s = np.mean(np.diff(time_seg))
    error = fb_seg - cmd_seg

    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=False)
    fig.suptitle(f'{joint_name} Tracking Analysis', fontsize=14)

    # ---- 子图1: cmd vs fb ----
    ax = axes[0]
    ax.plot(time_seg, np.degrees(cmd_seg), 'b-', label='cmd', linewidth=1)
    ax.plot(time_seg, np.degrees(fb_seg), 'r-', label='fb', linewidth=1, alpha=0.8)
    ax.set_ylabel('Position (deg)')
    ax.set_xlabel('Time (s)')
    ax.legend(loc='upper right')
    ax.set_title(f'delay={result.phase_delay_ms:.1f}ms, '
                 f'attenuation={(1-result.amplitude_ratio)*100:.1f}%, '
                 f'corr={result.correlation:.3f}')
    ax.grid(True, alpha=0.3)

    # ---- 子图2: 跟踪误差 ----
    ax = axes[1]
    ax.plot(time_seg, np.degrees(error), 'g-', linewidth=0.8)
    ax.axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    ax.set_ylabel('Error (deg)')
    ax.set_xlabel('Time (s)')
    ax.set_title(f'Tracking Error: mean={np.degrees(np.abs(error).mean()):.2f}°, '
                 f'max={np.degrees(np.abs(error).max()):.2f}°')
    ax.grid(True, alpha=0.3)

    # ---- 子图3: Bode 图 (增益 + 相位) ----
    freqs, gain_db, phase_deg, *_ = frequency_analysis(cmd_seg, fb_seg, dt_s)
    if freqs is not None:
        valid = ~np.isnan(gain_db) & (freqs > 0.05) & (freqs < 50)
        ax2 = axes[2]
        color1 = 'tab:blue'
        ax2.semilogx(freqs[valid], gain_db[valid], color=color1, linewidth=1)
        ax2.axhline(y=-3, color='gray', linestyle='--', linewidth=0.5, label='-3dB')
        ax2.set_ylabel('Gain (dB)', color=color1)
        ax2.set_xlabel('Frequency (Hz)')
        ax2.tick_params(axis='y', labelcolor=color1)
        ax2.set_ylim(-30, 10)
        ax2.grid(True, alpha=0.3, which='both')
        ax2.set_title('Bode Plot (Transfer Function)')

        ax3 = ax2.twinx()
        color2 = 'tab:red'
        valid_p = ~np.isnan(phase_deg) & (freqs > 0.05) & (freqs < 50)
        ax3.semilogx(freqs[valid_p], phase_deg[valid_p], color=color2, linewidth=1, alpha=0.7)
        ax3.set_ylabel('Phase (deg)', color=color2)
        ax3.tick_params(axis='y', labelcolor=color2)

    plt.tight_layout()
    os.makedirs(output_dir, exist_ok=True)
    path = os.path.join(output_dir, f'tracking_{joint_name}.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"  图表已保存: {path}")


def get_all_joint_names(df):
    """从列名中提取所有关节名 (兼容多种格式)"""
    joints = []

    # 格式1: cmd_p/<name>
    prefix = "cmd_p/"
    for col in df.columns:
        if col.startswith(prefix):
            name = col[len(prefix):]
            if f"fb_p/{name}" in df.columns:
                joints.append(name)
    if joints:
        return joints

    # 格式2 (旧): cmd_<name>
    for col in df.columns:
        if col.startswith("cmd_"):
            name = col[4:]
            if f"fb_{name}" in df.columns:
                joints.append(name)
    return joints


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    csv_path = sys.argv[1]
    joint_arg = sys.argv[2] if len(sys.argv) > 2 else "--all"

    df = pd.read_csv(csv_path)
    print(f"加载: {csv_path}")
    print(f"  行数: {len(df)}, 时长: {df['time_ms'].iloc[-1]/1000:.1f}s, "
          f"平均dt: {df['dt_ms'].mean():.2f}ms")

    output_dir = os.path.splitext(csv_path)[0] + "_analysis"

    if joint_arg == "--all":
        joint_names = get_all_joint_names(df)
    else:
        joint_names = [joint_arg]

    results = []
    for name in joint_names:
        r = analyze_joint(df, name)
        if r is not None:
            results.append(r)
            print_result(r)
            try_plot(df, name, r, output_dir)
        else:
            if joint_arg != "--all":
                print(f"\n  关节 '{name}' 无有效数据 (未找到列或信号无变化)")

    if len(results) > 1:
        print_summary_table(results)

    if not results:
        print("\n未找到有活跃信号的关节。请确认 joint_name 或使用 --all 查看所有关节。")


if __name__ == "__main__":
    main()
