#!/usr/bin/env python3
"""阶跃响应分析脚本 — 分析 keyboard_ctrl CSV 中的力矩阶跃跟踪性能"""

import argparse
import sys
import numpy as np
import pandas as pd
from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class StepResult:
    joint_name: str
    step_index: int
    step_amplitude_nm: float   # 阶跃幅度 (Nm)
    rise_time_ms: float        # 10%->90% 上升时间
    overshoot_pct: float       # 超调量 %
    settling_time_ms: float    # 2% 带稳定时间
    steady_state_error_pct: float  # 稳态误差 %
    rms_error_nm: float        # RMS 误差 (Nm)
    rated_torque_nm: float     # 额定力矩 (用于 pass/fail)
    passed: bool = True
    fail_reason: str = ""


def find_tau_columns(df, joint_name):
    """查找 cmd_tau 和 fb_tau 列名"""
    for prefix_cmd, prefix_fb in [("cmd_tau/", "fb_tau/"), ("cmd_tau_", "fb_tau_")]:
        cmd_col = f"{prefix_cmd}{joint_name}"
        fb_col = f"{prefix_fb}{joint_name}"
        if cmd_col in df.columns and fb_col in df.columns:
            return cmd_col, fb_col
    return None, None


def get_all_tau_joints(df):
    """从列名中提取所有有 cmd_tau/fb_tau 对的关节名"""
    joints = []
    for prefix in ["cmd_tau/", "cmd_tau_"]:
        fb_prefix = prefix.replace("cmd_", "fb_")
        for col in df.columns:
            if col.startswith(prefix):
                name = col[len(prefix):]
                if f"{fb_prefix}{name}" in df.columns:
                    joints.append(name)
        if joints:
            break
    return joints


def detect_steps(cmd, time_ms, threshold):
    """检测阶跃边沿, 返回 [(edge_idx, hold_end_idx), ...]"""
    diff = np.diff(cmd)
    edges = np.where(np.abs(diff) > threshold)[0]
    if len(edges) == 0:
        return []

    # 合并连续边沿 (抖动抑制)
    groups = []
    group_start = edges[0]
    for i in range(1, len(edges)):
        if edges[i] - edges[i - 1] > 10:  # 间隔 >10 样本视为新阶跃
            groups.append(group_start)
            group_start = edges[i]
    groups.append(group_start)

    # 每个阶跃的保持段: 从 edge+1 到下一个 edge (或末尾)
    steps = []
    for i, g in enumerate(groups):
        hold_end = groups[i + 1] if i + 1 < len(groups) else len(cmd) - 1
        if hold_end - g > 20:  # 至少 20 样本的保持段
            steps.append((g, hold_end))
    return steps


def analyze_single_step(cmd, fb, time_ms, edge_idx, hold_end_idx, step_index,
                        joint_name, rated_torque):
    """分析单个阶跃响应"""
    # 阶跃前值和目标值
    pre_val = cmd[max(0, edge_idx - 5):edge_idx + 1].mean()
    target_val = cmd[edge_idx + 5:min(edge_idx + 20, hold_end_idx)].mean()
    amplitude = target_val - pre_val
    if abs(amplitude) < 1e-6:
        return None

    # 保持段切片
    seg_cmd = cmd[edge_idx:hold_end_idx]
    seg_fb = fb[edge_idx:hold_end_idx]
    seg_t = time_ms[edge_idx:hold_end_idx] - time_ms[edge_idx]
    n = len(seg_cmd)

    # --- 稳态窗口: 最后 2s 或最后 20% ---
    hold_dur_ms = seg_t[-1]
    if hold_dur_ms > 2000:
        ss_start = np.searchsorted(seg_t, hold_dur_ms - 2000)
    else:
        ss_start = int(n * 0.8)
    ss_fb = seg_fb[ss_start:]
    ss_cmd = seg_cmd[ss_start:]
    ss_mean = ss_fb.mean() if len(ss_fb) > 0 else target_val

    # --- 稳态误差 ---
    ss_error_pct = abs(ss_mean - target_val) / abs(amplitude) * 100 if abs(amplitude) > 1e-9 else 0.0

    # --- Rise time (10% -> 90%) ---
    level_10 = pre_val + 0.1 * amplitude
    level_90 = pre_val + 0.9 * amplitude
    rise_time_ms = float('nan')
    if amplitude > 0:
        idx_10 = np.where(seg_fb >= level_10)[0]
        idx_90 = np.where(seg_fb >= level_90)[0]
    else:
        idx_10 = np.where(seg_fb <= level_10)[0]
        idx_90 = np.where(seg_fb <= level_90)[0]
    if len(idx_10) > 0 and len(idx_90) > 0:
        rise_time_ms = seg_t[idx_90[0]] - seg_t[idx_10[0]]

    # --- Overshoot ---
    if amplitude > 0:
        peak = seg_fb.max()
        overshoot = max(0, (peak - target_val) / abs(amplitude) * 100)
    else:
        peak = seg_fb.min()
        overshoot = max(0, (target_val - peak) / abs(amplitude) * 100)

    # --- Settling time (2% band) ---
    band = abs(amplitude) * 0.02
    settled = np.abs(seg_fb - target_val) <= band
    settling_ms = float('nan')
    # 从末尾向前找第一个不在 band 内的点
    for k in range(n - 1, -1, -1):
        if not settled[k]:
            if k + 1 < n:
                settling_ms = seg_t[k + 1]
            break

    # --- RMS error ---
    rms = np.sqrt(np.mean((seg_fb - seg_cmd) ** 2))

    # --- Pass/fail ---
    load_ratio = abs(amplitude) / rated_torque if rated_torque > 0 else 1.0
    if load_ratio <= 0.2:
        max_rms_pct = 3.0
    elif load_ratio <= 0.5:
        max_rms_pct = 5.0
    elif load_ratio <= 0.8:
        max_rms_pct = 8.0
    else:
        max_rms_pct = 10.0

    rms_pct = rms / abs(amplitude) * 100 if abs(amplitude) > 1e-9 else 0.0
    passed = rms_pct <= max_rms_pct and overshoot <= 90
    reasons = []
    if rms_pct > max_rms_pct:
        reasons.append(f"RMS {rms_pct:.1f}%>{max_rms_pct:.0f}%")
    if overshoot > 90:
        reasons.append(f"overshoot {overshoot:.0f}%")

    return StepResult(
        joint_name=joint_name,
        step_index=step_index,
        step_amplitude_nm=amplitude,
        rise_time_ms=rise_time_ms,
        overshoot_pct=overshoot,
        settling_time_ms=settling_ms,
        steady_state_error_pct=ss_error_pct,
        rms_error_nm=rms,
        rated_torque_nm=rated_torque,
        passed=passed,
        fail_reason="; ".join(reasons),
    )


def analyze_joint_steps(df, joint_name, threshold):
    """分析一个关节的所有阶跃"""
    cmd_col, fb_col = find_tau_columns(df, joint_name)
    if cmd_col is None:
        return []

    time_ms = df["time_ms"].values.astype(float)
    cmd = df[cmd_col].values.astype(float)
    fb = df[fb_col].values.astype(float)

    steps = detect_steps(cmd, time_ms, threshold)
    if not steps:
        return []

    # 额定力矩近似: 取 cmd 最大绝对值
    rated = np.abs(cmd).max()
    if rated < 0.01:
        rated = 1.0  # fallback

    results = []
    for i, (edge, hold_end) in enumerate(steps):
        r = analyze_single_step(cmd, fb, time_ms, edge, hold_end, i,
                                joint_name, rated)
        if r is not None:
            results.append(r)
    return results


def print_markdown_table(results: List[StepResult]):
    """输出 Markdown 表格"""
    print("\n## Step Response Analysis Results\n")
    print("| Joint | Step# | Amplitude(Nm) | Rise(ms) | Overshoot(%) | "
          "Settling(ms) | SS_Err(%) | RMS(Nm) | Result |")
    print("|-------|-------|--------------|----------|-------------|"
          "-------------|----------|---------|--------|")
    for r in results:
        rise_s = f"{r.rise_time_ms:.1f}" if not np.isnan(r.rise_time_ms) else "N/A"
        sett_s = f"{r.settling_time_ms:.1f}" if not np.isnan(r.settling_time_ms) else "N/A"
        status = "PASS" if r.passed else f"FAIL({r.fail_reason})"
        print(f"| {r.joint_name} | {r.step_index} | {r.step_amplitude_nm:+.3f} | "
              f"{rise_s} | {r.overshoot_pct:.1f} | {sett_s} | "
              f"{r.steady_state_error_pct:.2f} | {r.rms_error_nm:.4f} | {status} |")


def plot_steps(df, joint_name, threshold):
    """绘制阶跃响应图"""
    import matplotlib; matplotlib.use("Agg")  # noqa: E702
    import matplotlib.pyplot as plt
    cmd_col, fb_col = find_tau_columns(df, joint_name)
    if cmd_col is None:
        return
    time_ms = df["time_ms"].values.astype(float)
    cmd, fb = df[cmd_col].values.astype(float), df[fb_col].values.astype(float)
    steps = detect_steps(cmd, time_ms, threshold)
    if not steps:
        return
    nc = min(len(steps), 3)
    nr = (len(steps) + nc - 1) // nc
    fig, axes = plt.subplots(nr, nc, figsize=(6 * nc, 4 * nr), squeeze=False)
    fig.suptitle(f"{joint_name} Step Response", fontsize=14)
    for i, (edge, he) in enumerate(steps):
        ax = axes[i // nc][i % nc]
        t = time_ms[edge:he] - time_ms[edge]
        ax.plot(t, cmd[edge:he], "b-", label="cmd_tau", linewidth=1.2)
        ax.plot(t, fb[edge:he], "r-", label="fb_tau", linewidth=1, alpha=0.8)
        ax.set_title(f"Step {i}"); ax.set_xlabel("Time (ms)"); ax.set_ylabel("Torque (Nm)")
        ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    for i in range(len(steps), nr * nc):
        axes[i // nc][i % nc].set_visible(False)
    plt.tight_layout()
    out_path = f"step_response_{joint_name}.png"
    plt.savefig(out_path, dpi=150); plt.close()
    print(f"  图表已保存: {out_path}")


def main():
    parser = argparse.ArgumentParser(description="阶跃响应分析 (力矩跟踪)")
    parser.add_argument("csv_file", help="keyboard_ctrl CSV 文件路径")
    parser.add_argument("--joint", default=None, help="关节名 (默认分析所有)")
    parser.add_argument("--threshold", type=float, default=0.1, help="阶跃检测阈值 (Nm)")
    parser.add_argument("--plot", action="store_true", help="生成 matplotlib 图表")
    args = parser.parse_args()

    df = pd.read_csv(args.csv_file)
    print(f"加载: {args.csv_file}  ({len(df)} 行)")

    if args.joint:
        joint_names = [args.joint]
    else:
        joint_names = get_all_tau_joints(df)
        if not joint_names:
            print("未找到 cmd_tau/fb_tau 列对, 请检查 CSV 格式")
            sys.exit(1)

    all_results: List[StepResult] = []
    for jn in joint_names:
        results = analyze_joint_steps(df, jn, args.threshold)
        if results:
            all_results.extend(results)
        else:
            print(f"  {jn}: 无阶跃检出 (阈值={args.threshold} Nm)")

    if all_results:
        print_markdown_table(all_results)
        n_pass = sum(1 for r in all_results if r.passed)
        print(f"\n总计: {len(all_results)} 个阶跃, "
              f"{n_pass} PASS, {len(all_results) - n_pass} FAIL")
    else:
        print("\n未检出任何阶跃, 请降低 --threshold 或确认 CSV 包含阶跃数据")

    if args.plot:
        for jn in joint_names:
            plot_steps(df, jn, args.threshold)


if __name__ == "__main__":
    main()
