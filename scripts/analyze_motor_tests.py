#!/usr/bin/env python3
"""统一电机测试分析脚本 — 整合阶跃响应、正弦跟踪、KPKD 分析"""

import argparse
import os
import sys
import time
import numpy as np
import pandas as pd
from dataclasses import dataclass
from typing import List, Optional


# ─── Step Response (inline from analyze_step_response.py) ───────────────

def _find_tau_columns(df, joint_name):
    for pc, pf in [("cmd_tau/", "fb_tau/"), ("cmd_tau_", "fb_tau_")]:
        c, f = f"{pc}{joint_name}", f"{pf}{joint_name}"
        if c in df.columns and f in df.columns:
            return c, f
    return None, None


def _get_tau_joints(df):
    joints = []
    for p in ["cmd_tau/", "cmd_tau_"]:
        fp = p.replace("cmd_", "fb_")
        for col in df.columns:
            if col.startswith(p):
                name = col[len(p):]
                if f"{fp}{name}" in df.columns:
                    joints.append(name)
        if joints:
            break
    return joints


def _detect_steps(cmd, threshold):
    diff = np.diff(cmd)
    edges = np.where(np.abs(diff) > threshold)[0]
    if len(edges) == 0:
        return []
    groups, gs = [], edges[0]
    for i in range(1, len(edges)):
        if edges[i] - edges[i - 1] > 10:
            groups.append(gs)
            gs = edges[i]
    groups.append(gs)
    steps = []
    for i, g in enumerate(groups):
        he = groups[i + 1] if i + 1 < len(groups) else len(cmd) - 1
        if he - g > 20:
            steps.append((g, he))
    return steps


def run_step_analysis(df, joint_names, threshold=0.1):
    """返回 Markdown 字符串"""
    lines = ["## Step Response Analysis\n"]
    lines.append("| Joint | Step# | Amplitude(Nm) | Rise(ms) | Overshoot(%) | "
                 "SS_Err(%) | RMS(Nm) |")
    lines.append("|-------|-------|--------------|----------|-------------|"
                 "----------|---------|")
    count = 0
    for jn in joint_names:
        cc, fc = _find_tau_columns(df, jn)
        if cc is None:
            continue
        time_ms = df["time_ms"].values.astype(float)
        cmd = df[cc].values.astype(float)
        fb = df[fc].values.astype(float)
        rated = max(np.abs(cmd).max(), 1.0)
        steps = _detect_steps(cmd, threshold)
        for si, (edge, he) in enumerate(steps):
            seg_cmd, seg_fb = cmd[edge:he], fb[edge:he]
            seg_t = time_ms[edge:he] - time_ms[edge]
            pre = cmd[max(0, edge - 5):edge + 1].mean()
            tgt = cmd[edge + 5:min(edge + 20, he)].mean()
            amp = tgt - pre
            if abs(amp) < 1e-6:
                continue
            # Rise time
            l10, l90 = pre + 0.1 * amp, pre + 0.9 * amp
            cmp = (lambda x, v: x >= v) if amp > 0 else (lambda x, v: x <= v)
            i10 = np.where(np.array([cmp(v, l10) for v in seg_fb]))[0]
            i90 = np.where(np.array([cmp(v, l90) for v in seg_fb]))[0]
            rt = (seg_t[i90[0]] - seg_t[i10[0]]) if (len(i10) and len(i90)) else float('nan')
            # Overshoot
            peak = seg_fb.max() if amp > 0 else seg_fb.min()
            ov = max(0, (peak - tgt) / abs(amp) * 100) if amp > 0 else max(0, (tgt - peak) / abs(amp) * 100)
            # SS error
            n = len(seg_cmd)
            ss_start = np.searchsorted(seg_t, seg_t[-1] - 2000) if seg_t[-1] > 2000 else int(n * 0.8)
            ss_err = abs(seg_fb[ss_start:].mean() - tgt) / abs(amp) * 100
            rms = np.sqrt(np.mean((seg_fb - seg_cmd) ** 2))
            rt_s = f"{rt:.1f}" if not np.isnan(rt) else "N/A"
            lines.append(f"| {jn} | {si} | {amp:+.3f} | {rt_s} | "
                         f"{ov:.1f} | {ss_err:.2f} | {rms:.4f} |")
            count += 1

    if count == 0:
        lines.append("| - | - | No steps detected | - | - | - | - |")
    return "\n".join(lines)


# ─── Sim2Real Gap (inline from analyze_sim2real_gap.py) ─────────────

def _find_pos_columns(df, joint_name):
    for pc, pf in [("cmd_p/", "fb_p/"), ("cmd_p_", "fb_p_")]:
        c, f = f"{pc}{joint_name}", f"{pf}{joint_name}"
        if c in df.columns and f in df.columns:
            return c, f
    return None, None


def _get_pos_joints(df):
    joints = []
    for p in ["cmd_p/", "cmd_p_"]:
        fp = p.replace("cmd_", "fb_")
        for col in df.columns:
            if col.startswith(p):
                name = col[len(p):]
                if f"{fp}{name}" in df.columns:
                    joints.append(name)
        if joints:
            break
    return joints


def _xcorr_delay(cmd, fb, dt_s, max_lag_ms=500):
    cmd_d, fb_d = cmd - cmd.mean(), fb - fb.mean()
    norm = np.linalg.norm(cmd_d) * np.linalg.norm(fb_d)
    if norm < 1e-12:
        return 0.0, 0.0
    corr = np.correlate(cmd_d, fb_d, mode="full")
    corr /= norm
    lags = np.arange(-len(cmd_d) + 1, len(cmd_d))
    ml = int(max_lag_ms / 1000.0 / dt_s)
    ctr = len(cmd_d) - 1
    lo, hi = max(0, ctr - ml), min(len(corr), ctr + ml)
    vc, vl = corr[lo:hi], lags[lo:hi]
    bi = np.argmax(vc)
    bl, bc = float(vl[bi]), float(vc[bi])
    if 0 < bi < len(vc) - 1:
        y0, y1, y2 = vc[bi - 1], vc[bi], vc[bi + 1]
        d = 2.0 * (2 * y1 - y0 - y2)
        if abs(d) > 1e-12:
            bl += (y0 - y2) / d
    return bl * dt_s * 1000.0, bc


def run_sim2real_analysis(df, joint_names):
    """返回 Markdown 字符串"""
    lines = ["## Sim2Real Gap Analysis\n"]
    lines.append("| Joint | RMS(deg) | MaxErr(deg) | MeanErr(deg) | "
                 "Delay(ms) | AmpRatio | Corr |")
    lines.append("|-------|---------|------------|-------------|"
                 "----------|----------|------|")
    count = 0
    time_s = df["time_ms"].values / 1000.0
    dt_s = np.median(np.diff(time_s))
    if dt_s <= 0:
        dt_s = 0.002

    for jn in joint_names:
        cc, fc = _find_pos_columns(df, jn)
        if cc is None:
            continue
        cmd = df[cc].values.astype(float)
        fb = df[fc].values.astype(float)
        cr = cmd.max() - cmd.min()
        if cr < 1e-4:
            continue
        err = fb - cmd
        rms = np.degrees(np.sqrt(np.mean(err ** 2)))
        maxe = np.degrees(np.max(np.abs(err)))
        meane = np.degrees(np.mean(np.abs(err)))
        delay, corr = _xcorr_delay(cmd, fb, dt_s)
        fr = fb.max() - fb.min()
        ar = fr / cr if cr > 1e-9 else 1.0
        lines.append(f"| {jn} | {rms:.3f} | {maxe:.3f} | {meane:.3f} | "
                     f"{delay:.1f} | {ar:.3f} | {corr:.4f} |")
        count += 1

    if count == 0:
        lines.append("| - | No active joints found | - | - | - | - | - |")
    return "\n".join(lines)


# ─── Main ───────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="统一电机测试分析")
    parser.add_argument("csv_file", help="keyboard_ctrl CSV 文件路径")
    parser.add_argument("--mode", choices=["step", "sine", "all"], default="all",
                        help="分析模式: step=阶跃, sine=正弦跟踪, all=全部")
    parser.add_argument("--output-dir", default=None,
                        help="报告输出目录 (默认: CSV 同目录)")
    parser.add_argument("--joint", default=None, help="关节名 (默认分析所有)")
    parser.add_argument("--threshold", type=float, default=0.1,
                        help="阶跃检测阈值 (Nm, 仅 step 模式)")
    args = parser.parse_args()

    df = pd.read_csv(args.csv_file)
    print(f"加载: {args.csv_file}  ({len(df)} 行)")

    # Resolve output dir
    if args.output_dir:
        out_dir = args.output_dir
    else:
        out_dir = os.path.dirname(os.path.abspath(args.csv_file))
    os.makedirs(out_dir, exist_ok=True)

    sections = []
    sections.append(f"# Motor Test Report\n")
    sections.append(f"- CSV: `{os.path.basename(args.csv_file)}`")
    sections.append(f"- Rows: {len(df)}")
    if "time_ms" in df.columns:
        dur = df["time_ms"].iloc[-1] / 1000.0
        sections.append(f"- Duration: {dur:.1f} s")
    sections.append(f"- Mode: {args.mode}")
    sections.append("")

    if args.mode in ("step", "all"):
        if args.joint:
            tau_joints = [args.joint]
        else:
            tau_joints = _get_tau_joints(df)
        if tau_joints:
            sections.append(run_step_analysis(df, tau_joints, args.threshold))
            sections.append("")
        else:
            sections.append("## Step Response Analysis\n\nNo cmd_tau/fb_tau columns found.\n")

    if args.mode in ("sine", "all"):
        if args.joint:
            pos_joints = [args.joint]
        else:
            pos_joints = _get_pos_joints(df)
        if pos_joints:
            sections.append(run_sim2real_analysis(df, pos_joints))
            sections.append("")
        else:
            sections.append("## Sim2Real Gap Analysis\n\nNo cmd_p/fb_p columns found.\n")

    report = "\n".join(sections)

    # Print to stdout
    print("\n" + report)

    # Save to file
    ts = time.strftime("%Y%m%d_%H%M%S")
    report_path = os.path.join(out_dir, f"report_{ts}.md")
    with open(report_path, "w") as f:
        f.write(report)
    print(f"\n报告已保存: {report_path}")


if __name__ == "__main__":
    main()
