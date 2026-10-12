#!/usr/bin/env python3
"""Sim2Real Gap 分析脚本 — 分析 cmd 与 fb 的位置跟踪偏差"""

import argparse
import sys
import numpy as np
import pandas as pd
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class GapResult:
    joint_name: str
    rms_error_rad: float
    rms_error_deg: float
    max_abs_error_rad: float
    max_abs_error_deg: float
    mean_abs_error_rad: float
    mean_abs_error_deg: float
    phase_delay_ms: float
    amplitude_ratio: float     # fb_amplitude / cmd_amplitude
    correlation: float         # normalized cross-correlation peak
    cmd_range_deg: float
    fb_range_deg: float
    duration_s: float


def find_pos_columns(df, joint_name):
    """查找 cmd_p 和 fb_p 列名"""
    for prefix_cmd, prefix_fb in [("cmd_p/", "fb_p/"), ("cmd_p_", "fb_p_")]:
        cmd_col = f"{prefix_cmd}{joint_name}"
        fb_col = f"{prefix_fb}{joint_name}"
        if cmd_col in df.columns and fb_col in df.columns:
            return cmd_col, fb_col
    return None, None


def get_all_pos_joints(df):
    """从列名中提取所有有 cmd_p/fb_p 对的关节名"""
    joints = []
    for prefix in ["cmd_p/", "cmd_p_"]:
        fb_prefix = prefix.replace("cmd_", "fb_")
        for col in df.columns:
            if col.startswith(prefix):
                name = col[len(prefix):]
                if f"{fb_prefix}{name}" in df.columns:
                    joints.append(name)
        if joints:
            break
    return joints


def cross_correlation_delay(cmd, fb, dt_s, max_lag_ms=500):
    """互相关法计算相位延迟"""
    cmd_d = cmd - np.mean(cmd)
    fb_d = fb - np.mean(fb)
    norm = np.linalg.norm(cmd_d) * np.linalg.norm(fb_d)
    if norm < 1e-12:
        return 0.0, 0.0

    corr = np.correlate(cmd_d, fb_d, mode="full")
    corr /= norm
    lags = np.arange(-len(cmd_d) + 1, len(cmd_d))

    max_lag_samples = int(max_lag_ms / 1000.0 / dt_s)
    center = len(cmd_d) - 1
    lo = max(0, center - max_lag_samples)
    hi = min(len(corr), center + max_lag_samples)

    valid_corr = corr[lo:hi]
    valid_lags = lags[lo:hi]
    best_idx = np.argmax(valid_corr)
    best_lag = float(valid_lags[best_idx])
    best_corr = float(valid_corr[best_idx])

    # 抛物线插值
    if 0 < best_idx < len(valid_corr) - 1:
        y0, y1, y2 = valid_corr[best_idx - 1], valid_corr[best_idx], valid_corr[best_idx + 1]
        denom = 2.0 * (2 * y1 - y0 - y2)
        if abs(denom) > 1e-12:
            best_lag += (y0 - y2) / denom

    return best_lag * dt_s * 1000.0, best_corr


def analyze_joint(df, joint_name) -> Optional[GapResult]:
    """分析单个关节的 Sim2Real gap"""
    cmd_col, fb_col = find_pos_columns(df, joint_name)
    if cmd_col is None:
        return None

    time_s = df["time_ms"].values / 1000.0
    cmd = df[cmd_col].values.astype(float)
    fb = df[fb_col].values.astype(float)

    cmd_range = cmd.max() - cmd.min()
    if cmd_range < 1e-4:  # 几乎没动
        return None

    dt_s = np.median(np.diff(time_s))
    if dt_s <= 0:
        return None

    error = fb - cmd
    rms_err = np.sqrt(np.mean(error ** 2))
    max_err = np.max(np.abs(error))
    mean_err = np.mean(np.abs(error))

    delay_ms, corr = cross_correlation_delay(cmd, fb, dt_s)

    fb_range = fb.max() - fb.min()
    amp_ratio = fb_range / cmd_range if cmd_range > 1e-9 else 1.0
    duration = time_s[-1] - time_s[0]

    return GapResult(
        joint_name=joint_name,
        rms_error_rad=rms_err,
        rms_error_deg=np.degrees(rms_err),
        max_abs_error_rad=max_err,
        max_abs_error_deg=np.degrees(max_err),
        mean_abs_error_rad=mean_err,
        mean_abs_error_deg=np.degrees(mean_err),
        phase_delay_ms=delay_ms,
        amplitude_ratio=amp_ratio,
        correlation=corr,
        cmd_range_deg=np.degrees(cmd_range),
        fb_range_deg=np.degrees(fb_range),
        duration_s=duration,
    )


def print_markdown_table(results: List[GapResult]):
    """输出 Markdown 表格"""
    print("\n## Sim2Real Gap Analysis\n")
    print("| Joint | RMS(deg) | RMS(rad) | MaxErr(deg) | MeanErr(deg) | "
          "Delay(ms) | Amp Ratio | Corr | CmdRange(deg) |")
    print("|-------|---------|---------|------------|-------------|"
          "----------|-----------|------|---------------|")
    for r in results:
        print(f"| {r.joint_name} | {r.rms_error_deg:.3f} | "
              f"{r.rms_error_rad:.5f} | {r.max_abs_error_deg:.3f} | "
              f"{r.mean_abs_error_deg:.3f} | {r.phase_delay_ms:.1f} | "
              f"{r.amplitude_ratio:.3f} | {r.correlation:.4f} | "
              f"{r.cmd_range_deg:.1f} |")

    # 汇总
    print("\n### Summary\n")
    avg_rms = np.mean([r.rms_error_deg for r in results])
    avg_delay = np.mean([r.phase_delay_ms for r in results])
    worst = max(results, key=lambda r: r.rms_error_deg)
    print(f"- Avg RMS error: {avg_rms:.3f} deg")
    print(f"- Avg phase delay: {avg_delay:.1f} ms")
    print(f"- Worst joint: **{worst.joint_name}** "
          f"(RMS={worst.rms_error_deg:.3f} deg, MaxErr={worst.max_abs_error_deg:.3f} deg)")


def plot_joints(df, joint_names):
    """绘制 cmd vs fb 对比图"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    n = len(joint_names)
    cols = min(n, 3)
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(6 * cols, 4 * rows), squeeze=False)
    fig.suptitle("Sim2Real Position Tracking", fontsize=14)

    time_s = df["time_ms"].values / 1000.0

    for i, jn in enumerate(joint_names):
        cmd_col, fb_col = find_pos_columns(df, jn)
        if cmd_col is None:
            continue
        ax = axes[i // cols][i % cols]
        cmd = np.degrees(df[cmd_col].values.astype(float))
        fb = np.degrees(df[fb_col].values.astype(float))
        ax.plot(time_s, cmd, "b-", label="cmd", linewidth=1)
        ax.plot(time_s, fb, "r-", label="fb", linewidth=0.8, alpha=0.8)
        ax.set_title(jn, fontsize=10)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Position (deg)")
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    for i in range(n, rows * cols):
        axes[i // cols][i % cols].set_visible(False)

    plt.tight_layout()
    out_path = "sim2real_gap.png"
    plt.savefig(out_path, dpi=150)
    plt.close()
    print(f"\n图表已保存: {out_path}")


def main():
    parser = argparse.ArgumentParser(description="Sim2Real Gap 分析 (位置跟踪)")
    parser.add_argument("csv_file", help="keyboard_ctrl CSV 文件路径")
    parser.add_argument("--joint", default=None, help="关节名 (默认分析所有)")
    parser.add_argument("--plot", action="store_true", help="生成 matplotlib 图表")
    args = parser.parse_args()

    df = pd.read_csv(args.csv_file)
    print(f"加载: {args.csv_file}  ({len(df)} 行)")

    if args.joint:
        joint_names = [args.joint]
    else:
        joint_names = get_all_pos_joints(df)
        if not joint_names:
            print("未找到 cmd_p/fb_p 列对, 请检查 CSV 格式")
            sys.exit(1)

    results: List[GapResult] = []
    for jn in joint_names:
        r = analyze_joint(df, jn)
        if r is not None:
            results.append(r)
        else:
            print(f"  {jn}: 无有效数据 (信号无变化或未找到列)")

    if results:
        print_markdown_table(results)
    else:
        print("\n未找到有活跃信号的关节")

    if args.plot and results:
        active_joints = [r.joint_name for r in results]
        plot_joints(df, active_joints)


if __name__ == "__main__":
    main()
