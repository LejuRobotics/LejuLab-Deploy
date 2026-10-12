#!/usr/bin/env python3
"""CST 扫频跟踪分析 — 评估力矩内环带宽与机械导纳

输入: rosbag 录制的 merged CSV，必须包含
    timestamp
    /rt/motor_cmd/modes.{i}, /rt/motor_cmd/tau.{i}
    /rt/motor_state/tau.{i}, /rt/motor_state/q.{i}, /rt/motor_state/v.{i}

输出:
    - 控制台: 每个 CST 关节的标准评估指标 (NRMSE, -3dB BW, 相位延迟, 谐振峰, 等)
    - 图: out_dir/joint_{i}_bode.png (force/admittance Bode + 时域)
    - report: out_dir/summary.md 表格
"""
from __future__ import annotations

import argparse
import os
import sys
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy import signal


CST_MODE = 0
FS_TARGET = 1000.0  # 重采样到 1 kHz


@dataclass
class JointMetrics:
    joint: int
    n_samples: int
    duration_s: float
    sweep_f_min_hz: float
    sweep_f_max_hz: float
    tau_cmd_rms_nm: float
    tau_cmd_peak_nm: float
    # ---- 力矩跟踪 (限定在 sweep band 内) ----
    tau_nrmse_pct: float
    tau_inband_gain_dev_db: float    # 带内 |H|-1 的最大偏差 (dB)
    tau_inband_phase_max_deg: float  # 带内最大相位滞后 (deg)
    tau_xcorr_delay_us: float        # 时域互相关 lag (μs)
    tau_bw_assertion: str            # 例如 ">=2.4Hz 内带宽充足"
    tau_coherence_inband: float      # 带内平均 coherence
    # ---- 机械导纳 v/τ ----
    adm_peak_freq_hz: float          # |Y| 带内峰值频率
    adm_peak_above_trend_db: float   # 峰值高出 +20dB/dec 趋势线多少
    inertia_est_kgm2: float          # 由 |Y|≈ω/J 拟合的转动惯量
    q_amplitude_rad: float           # 实测 q 摆幅 (peak-to-peak/2)
    # ---- 评估 ----
    grade_torque: str
    grade_admittance: str
    notes: str


# ----------------------------- 数据提取 -----------------------------

def detect_cst_joints(df: pd.DataFrame) -> List[int]:
    js = []
    for i in range(64):
        c = f"/rt/motor_cmd/modes.{i}"
        if c in df.columns and (df[c] == CST_MODE).sum() > 100:
            js.append(i)
    return js


def extract_joint_window(df: pd.DataFrame, j: int) -> Optional[dict]:
    """提取关节 j 在 CST 模式下的时间窗，状态列前向填充"""
    mode_col = f"/rt/motor_cmd/modes.{j}"
    cst_idx = df.index[df[mode_col] == CST_MODE]
    if len(cst_idx) < 100:
        return None
    t_start = df["timestamp"].iloc[cst_idx[0]]
    t_end = df["timestamp"].iloc[cst_idx[-1]]
    t_all = df["timestamp"].values
    win = (t_all >= t_start) & (t_all <= t_end)
    sub = df[win].copy()

    state_cols = [
        f"/rt/motor_state/tau.{j}",
        f"/rt/motor_state/q.{j}",
        f"/rt/motor_state/v.{j}",
        f"/rt/joint_state/tau.{j}",
        f"/rt/joint_state/q.{j}",
        f"/rt/joint_state/v.{j}",
    ]
    avail = [c for c in state_cols if c in sub.columns]
    sub[avail] = sub[avail].ffill()
    # motor_cmd/tau 只在 cmd 行有值，ffill 一下
    cmd_col = f"/rt/motor_cmd/tau.{j}"
    sub[cmd_col] = sub[cmd_col].ffill()

    tt = sub["timestamp"].values
    tau_cmd = sub[cmd_col].values
    tau_meas = sub[f"/rt/motor_state/tau.{j}"].values
    q = sub[f"/rt/motor_state/q.{j}"].values
    v = sub[f"/rt/motor_state/v.{j}"].values

    valid = ~(np.isnan(tau_cmd) | np.isnan(tau_meas) | np.isnan(q) | np.isnan(v))
    if valid.sum() < 100:
        return None
    return {
        "t": tt[valid],
        "tau_cmd": tau_cmd[valid],
        "tau_meas": tau_meas[valid],
        "q": q[valid],
        "v": v[valid],
    }


def resample_uniform(data: dict, fs: float = FS_TARGET) -> dict:
    t = data["t"]
    n = int((t[-1] - t[0]) * fs)
    if n < 100:
        return data
    t_u = np.linspace(t[0], t[-1], n)
    return {
        "t": t_u,
        "fs": fs,
        "tau_cmd": np.interp(t_u, t, data["tau_cmd"]),
        "tau_meas": np.interp(t_u, t, data["tau_meas"]),
        "q": np.interp(t_u, t, data["q"]),
        "v": np.interp(t_u, t, data["v"]),
    }


# ----------------------------- 评估算法 -----------------------------

def estimate_sweep_range(tau_cmd: np.ndarray, fs: float) -> tuple:
    """用 Hilbert 瞬时频率估扫频上下限"""
    sig = tau_cmd - np.mean(tau_cmd)
    analytic = signal.hilbert(sig)
    phase = np.unwrap(np.angle(analytic))
    inst_f = np.diff(phase) / (2 * np.pi) * fs
    # 截掉边缘瞬态
    trim = max(int(0.5 * fs), 100)
    inst_f = inst_f[trim:-trim]
    # smooth
    w = max(int(0.2 * fs), 10)
    if len(inst_f) > w:
        inst_f = np.convolve(inst_f, np.ones(w) / w, mode="valid")
    inst_f = np.clip(inst_f, 0, fs / 2)
    return float(np.nanmin(inst_f)), float(np.nanmax(inst_f))


def tfestimate(u: np.ndarray, y: np.ndarray, fs: float, nperseg: int = 2048):
    """H1 估计 + 平均 coherence"""
    f, Pxy = signal.csd(u, y, fs=fs, nperseg=nperseg, noverlap=nperseg // 2)
    _, Pxx = signal.welch(u, fs=fs, nperseg=nperseg, noverlap=nperseg // 2)
    _, Pyy = signal.welch(y, fs=fs, nperseg=nperseg, noverlap=nperseg // 2)
    H = Pxy / np.maximum(Pxx, 1e-20)
    coh = np.abs(Pxy) ** 2 / np.maximum(Pxx * Pyy, 1e-20)
    return f, H, coh


def xcorr_delay(u: np.ndarray, y: np.ndarray, fs: float, max_lag_s: float = 0.02) -> float:
    """互相关求 y 相对 u 的最佳 lag (秒)。正值=y 滞后 u"""
    u = u - np.mean(u); y = y - np.mean(y)
    max_lag = int(max_lag_s * fs)
    n = len(u)
    # full xcorr 太慢，只算 ±max_lag
    lags = np.arange(-max_lag, max_lag + 1)
    corr = np.array([
        np.dot(u[max(0, -k):n - max(0, k)], y[max(0, k):n - max(0, -k)])
        for k in lags
    ])
    best = lags[np.argmax(corr)]
    return float(best / fs)


def grade_torque(nrmse_pct: float, gain_dev_db: float, phase_max_deg: float, delay_us: float) -> str:
    """带内指标判级。延迟可能含录制偏移，作为次要指标"""
    if nrmse_pct < 1.0 and abs(gain_dev_db) < 0.5 and abs(phase_max_deg) < 5:
        return "OK 优秀"
    if nrmse_pct < 3.0 and abs(gain_dev_db) < 1.0 and abs(phase_max_deg) < 15:
        return "OK 良好"
    if nrmse_pct < 8.0 and abs(gain_dev_db) < 2.0:
        return "OK 一般"
    return "WARN 偏差大"


def grade_admittance(peak_above_trend_db: float, q_amp: float) -> str:
    if q_amp < 0.005:
        return "N/A (关节锁住)"
    if peak_above_trend_db > 10.0:
        return "WARN 谐振明显"
    if peak_above_trend_db > 6.0:
        return "OK 阻尼一般"
    return "OK 阻尼充分"


# ----------------------------- 主流程 -----------------------------

def analyze_joint(j: int, raw: dict, out_dir: Path) -> JointMetrics:
    d = resample_uniform(raw)
    fs = d["fs"]
    t = d["t"]
    tau_cmd = d["tau_cmd"]
    tau_meas = d["tau_meas"]
    q = d["q"]
    v = d["v"]
    duration = t[-1] - t[0]

    # 扫频范围
    f_min, f_max = estimate_sweep_range(tau_cmd, fs)

    # 时域 NRMSE (力矩跟踪)
    err = tau_meas - tau_cmd
    rms_sig = float(np.sqrt(np.mean(tau_cmd ** 2)))
    nrmse_pct = float(np.sqrt(np.mean(err ** 2)) / max(rms_sig, 1e-9) * 100)
    tau_cmd_rms = rms_sig
    tau_cmd_peak = float(np.max(np.abs(tau_cmd)))

    # 频域: 力矩跟踪 H = tau_meas / tau_cmd
    nperseg = min(8192, len(tau_cmd) // 4)
    f, H_tau, coh_tau = tfestimate(tau_cmd, tau_meas, fs, nperseg)
    mag_tau_db = 20 * np.log10(np.maximum(np.abs(H_tau), 1e-12))
    phase_tau_deg = np.rad2deg(np.unwrap(np.angle(H_tau)))

    # 带内 mask: 用 coherence + sweep band 双重约束
    band = (f >= max(f_min * 1.05, 0.15)) & (f <= f_max * 0.95) & (coh_tau > 0.9)
    if band.sum() < 3:
        band = (f >= max(f_min, 0.1)) & (f <= f_max)

    inband_gain_dev_db = float(np.max(np.abs(mag_tau_db[band]))) if band.sum() else float("nan")
    inband_phase_max = float(np.min(phase_tau_deg[band])) if band.sum() else float("nan")  # 最负 = 最大滞后
    coh_inband = float(np.mean(coh_tau[band])) if band.sum() else float("nan")

    # 互相关求延迟 (时域更鲁棒)
    delay_s = xcorr_delay(tau_cmd, tau_meas, fs)
    delay_us = delay_s * 1e6

    # 带宽断言：如果带内幅值偏差 <1dB 且相位 >−10° 则带宽 ≥ f_max
    if inband_gain_dev_db < 1.0 and inband_phase_max > -10.0:
        bw_assert = f">={f_max:.1f} Hz (带内 |H|偏差<1dB, ∠<{abs(inband_phase_max):.0f}°)"
    else:
        bw_assert = f"~{f_max:.1f} Hz 处已有衰减"

    # 机械导纳 Y(jω) = V(jω)/T(jω)
    _, H_y, coh_y = tfestimate(tau_cmd, v, fs, nperseg)
    mag_y_db = 20 * np.log10(np.maximum(np.abs(H_y), 1e-12))
    omega = 2 * np.pi * f
    # |v/τ| ≈ ω/J  ⇒ J ≈ ω / |Y|
    with np.errstate(divide="ignore", invalid="ignore"):
        J_est = omega / np.maximum(np.abs(H_y), 1e-12)
    J_band = (f >= max(f_min * 1.2, 0.3)) & (f <= min(f_max * 0.7, 3.0)) & (coh_y > 0.5)
    inertia_kgm2 = float(np.nanmedian(J_est[J_band])) if J_band.sum() else float("nan")
    # 拟合 +20dB/dec 趋势线
    y_band = (f >= max(f_min, 0.15)) & (f <= f_max) & (coh_y > 0.5)
    if y_band.sum() > 5:
        trend = mag_y_db[y_band][0] + 20 * np.log10(f[y_band] / f[y_band][0])
        residual = mag_y_db[y_band] - trend
        peak_above_trend = float(np.max(residual))
        peak_f = float(f[y_band][np.argmax(residual)])
    else:
        peak_above_trend = float("nan")
        peak_f = float("nan")

    q_amp = float((q.max() - q.min()) / 2)
    grade_t = grade_torque(nrmse_pct, inband_gain_dev_db, inband_phase_max, delay_us)
    grade_a = grade_admittance(peak_above_trend, q_amp)

    # ---- 出图 ----
    fig, axes = plt.subplots(3, 2, figsize=(14, 10), constrained_layout=True)
    # (0,0) 时域 tau
    ax = axes[0, 0]
    show = slice(0, min(len(t), int(5 * fs)))
    ax.plot(t[show] - t[0], tau_cmd[show], "b", lw=0.7, label="cmd")
    ax.plot(t[show] - t[0], tau_meas[show], "r", lw=0.7, label="meas")
    ax.set_title(f"Joint {j} — τ time domain (first 5s)")
    ax.set_xlabel("t [s]"); ax.set_ylabel("Nm"); ax.legend(); ax.grid(True)
    # (0,1) 时域全段 q
    ax = axes[0, 1]
    ax.plot(t - t[0], q, "g", lw=0.6)
    ax.set_title(f"q response (std={q.std():.4f}rad, range=±{(q.max()-q.min())/2:.4f})")
    ax.set_xlabel("t [s]"); ax.set_ylabel("rad"); ax.grid(True)
    # (1,0) tau bode magnitude (限定带内)
    ax = axes[1, 0]
    pos = f > 0
    ax.semilogx(f[pos], mag_tau_db[pos], "b", alpha=0.3, label="all")
    ax.semilogx(f[band], mag_tau_db[band], "b", lw=2, label="in-band")
    ax.axhline(0, color="k", lw=0.5)
    ax.axhline(-1, color="r", ls="--", lw=0.5)
    ax.axvspan(f_min, f_max, alpha=0.1, color="gray", label=f"sweep {f_min:.2f}-{f_max:.2f}Hz")
    ax.set_title(f"Torque tracking |H| — 带内偏差 {inband_gain_dev_db:.2f}dB")
    ax.set_ylim(-20, 10)
    ax.set_xlabel("Hz"); ax.set_ylabel("dB"); ax.legend(); ax.grid(True, which="both")
    # (1,1) tau bode phase
    ax = axes[1, 1]
    ax.semilogx(f[pos], phase_tau_deg[pos], "b", alpha=0.3)
    ax.semilogx(f[band], phase_tau_deg[band], "b", lw=2, label="in-band")
    ax.axhline(0, color="k", lw=0.5)
    ax.axhline(-45, color="r", ls="--", lw=0.5)
    ax.axvspan(f_min, f_max, alpha=0.1, color="gray")
    ax.set_title(f"∠H — 带内最大滞后 {abs(inband_phase_max):.1f}° | xcorr延迟 {delay_us:.0f}μs")
    ax.set_ylim(-180, 60)
    ax.set_xlabel("Hz"); ax.set_ylabel("deg"); ax.legend(); ax.grid(True, which="both")
    # (2,0) admittance |Y| + trend
    ax = axes[2, 0]
    ax.semilogx(f[pos], mag_y_db[pos], "g", alpha=0.3)
    y_band_plot = (f >= max(f_min, 0.15)) & (f <= f_max)
    if y_band_plot.sum() > 5:
        trend = mag_y_db[y_band_plot][0] + 20 * np.log10(f[y_band_plot] / f[y_band_plot][0])
        ax.semilogx(f[y_band_plot], trend, "k--", lw=0.8, label="+20dB/dec (惯量)")
        ax.semilogx(f[y_band_plot], mag_y_db[y_band_plot], "g", lw=2)
    if np.isfinite(peak_f):
        ax.axvline(peak_f, color="r", ls=":", lw=0.8, label=f"peak {peak_f:.2f}Hz (+{peak_above_trend:.1f}dB)")
    ax.axvspan(f_min, f_max, alpha=0.1, color="gray")
    ax.set_title(f"Admittance |V/τ| — J≈{inertia_kgm2:.3f} kg·m²")
    ax.set_xlabel("Hz"); ax.set_ylabel("dB"); ax.legend(); ax.grid(True, which="both")
    # (2,1) coherence
    ax = axes[2, 1]
    ax.semilogx(f[pos], coh_tau[pos], "b", label="τ")
    ax.semilogx(f[pos], coh_y[pos], "g", label="Y")
    ax.axhline(0.9, color="r", ls="--", lw=0.8, label="0.9")
    ax.axvspan(f_min, f_max, alpha=0.1, color="gray")
    ax.set_title(f"Coherence  (带内 τ: {coh_inband:.3f})")
    ax.set_xlabel("Hz"); ax.set_ylabel("γ²")
    ax.set_ylim(0, 1.05); ax.legend(); ax.grid(True, which="both")
    fig.suptitle(f"Joint {j} CST sweep evaluation  |  {grade_t}", fontsize=13)
    fig.savefig(out_dir / f"joint_{j:02d}_bode.png", dpi=110)
    plt.close(fig)

    notes = []
    if q.std() < 0.005:
        notes.append("q 摆动 <0.3°，关节几乎被锁住，导纳估计仅供参考")
    if f_max < 5.0:
        notes.append(f"扫频上限只到 {f_max:.1f} Hz，无法评估更高频带宽")
    if coh_inband < 0.8:
        notes.append("coherence 偏低，可能噪声/非线性")

    return JointMetrics(
        joint=j,
        n_samples=len(t),
        duration_s=float(duration),
        sweep_f_min_hz=f_min,
        sweep_f_max_hz=f_max,
        tau_cmd_rms_nm=tau_cmd_rms,
        tau_cmd_peak_nm=tau_cmd_peak,
        tau_nrmse_pct=nrmse_pct,
        tau_inband_gain_dev_db=inband_gain_dev_db,
        tau_inband_phase_max_deg=inband_phase_max,
        tau_xcorr_delay_us=delay_us,
        tau_bw_assertion=bw_assert,
        tau_coherence_inband=coh_inband,
        adm_peak_freq_hz=peak_f,
        adm_peak_above_trend_db=peak_above_trend,
        inertia_est_kgm2=inertia_kgm2,
        q_amplitude_rad=q_amp,
        grade_torque=grade_t,
        grade_admittance=grade_a,
        notes="; ".join(notes),
    )


def write_summary(metrics: List[JointMetrics], out_md: Path, csv_path: str) -> None:
    lines = []
    lines.append(f"# CST 扫频跟踪评估报告\n")
    lines.append(f"数据源: `{csv_path}`\n")
    lines.append("\n## 评估口径\n")
    lines.append("- **力矩跟踪 (内环)**: H(jω)=τ_meas/τ_cmd，限定在 sweep band 内评估 (带外无激励)")
    lines.append("- **机械导纳**: Y(jω)=v/τ，低频段 +20dB/dec 趋势 → 转动惯量 J≈ω/|Y|，偏离趋势的峰即谐振")
    lines.append("- **延迟**: 时域互相关 lag，含 rosbag 录制偏移，跨关节差值更有意义")
    lines.append("- **优秀阈值**:  NRMSE<1%, 带内|H|偏差<0.5dB, 带内相位滞后<5°")
    lines.append("- **良好阈值**:  NRMSE<3%, 带内|H|偏差<1dB, 带内相位滞后<15°\n")
    lines.append("## 汇总表\n")
    header = ("| J | sweep[Hz] | τpk[Nm] | NRMSE% | 带内|H|偏差[dB] | 带内∠最大[°] | "
              "延迟[μs] | coh | q摆幅[°] | Y峰频[Hz] | Y峰高[dB] | Ĵ[kg·m²] | 力矩 | 导纳 |")
    sep = "|" + "|".join(["---"] * 14) + "|"
    lines.append(header); lines.append(sep)
    for m in metrics:
        lines.append(
            f"| {m.joint} | {m.sweep_f_min_hz:.2f}-{m.sweep_f_max_hz:.2f} | {m.tau_cmd_peak_nm:.2f} "
            f"| {m.tau_nrmse_pct:.2f} | {m.tau_inband_gain_dev_db:.2f} | {m.tau_inband_phase_max_deg:.1f} "
            f"| {m.tau_xcorr_delay_us:.0f} | {m.tau_coherence_inband:.3f} "
            f"| {np.rad2deg(m.q_amplitude_rad):.2f} | {m.adm_peak_freq_hz:.2f} | {m.adm_peak_above_trend_db:.2f} "
            f"| {m.inertia_est_kgm2:.3f} | {m.grade_torque} | {m.grade_admittance} |"
        )
    lines.append("\n## 每关节备注\n")
    for m in metrics:
        if m.notes:
            lines.append(f"- joint {m.joint}: {m.notes}")
    lines.append("\n## 图\n")
    for m in metrics:
        lines.append(f"![joint {m.joint}](figures/joint_{m.joint:02d}_bode.png)")
    out_md.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", help="merged rosbag CSV")
    ap.add_argument("--out", default="docs/reports/cst_sweep", help="output dir")
    ap.add_argument("--joints", type=int, nargs="*", help="只分析指定关节 id")
    args = ap.parse_args()

    out_dir = Path(args.out)
    fig_dir = out_dir / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)

    print(f"loading {args.csv} ...")
    df = pd.read_csv(args.csv, low_memory=False)
    print(f"  rows={len(df)} cols={len(df.columns)}")

    cst_joints = detect_cst_joints(df)
    if args.joints:
        cst_joints = [j for j in cst_joints if j in args.joints]
    if not cst_joints:
        print("no CST joint found", file=sys.stderr)
        sys.exit(1)
    print(f"CST joints: {cst_joints}")

    metrics = []
    for j in cst_joints:
        print(f"\n--- joint {j} ---")
        raw = extract_joint_window(df, j)
        if raw is None:
            print("  skip (insufficient data)")
            continue
        m = analyze_joint(j, raw, fig_dir)
        metrics.append(m)
        for k, v in asdict(m).items():
            print(f"  {k}: {v}")

    write_summary(metrics, out_dir / "summary.md", args.csv)
    print(f"\nreport: {out_dir/'summary.md'}")
    print(f"figures: {fig_dir}/")


if __name__ == "__main__":
    main()
