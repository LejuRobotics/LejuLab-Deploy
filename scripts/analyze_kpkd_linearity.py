#!/usr/bin/env python3
"""
TC-KPKD-001/002 后处理分析脚本

从 motor_test_runner 生成的 CSV 中提取稳态力矩数据，
计算 I_real vs q_des (或 dq_des) 线性回归，输出 PASS/FAIL 判定。

用法:
  python3 analyze_kpkd_linearity.py <csv_file> [选项]

选项:
  --joint <name>     关节名 (从CSV列名自动检测，或指定)
  --c2t <value>      c2t 系数 (默认 4.36)
  --settle <秒>      稳态提取丢弃前 N 秒 (默认 2.0)
  --plot             生成 PNG 图
  --output-dir <dir> 输出目录
"""

import argparse
import sys
import os
import re
import numpy as np

def find_joint_columns(header, joint_name=None):
    """从 CSV header 中找到目标关节的列名"""
    cols = header.strip().split(',')

    if joint_name:
        cmd_p = [c for c in cols if c.startswith('cmd_p/') and joint_name in c]
        cmd_v = [c for c in cols if c.startswith('cmd_v/') and joint_name in c]
        cmd_kp = [c for c in cols if c.startswith('cmd_kp/') and joint_name in c]
        cmd_tau = [c for c in cols if c.startswith('cmd_tau/') and joint_name in c]
        fb_tau = [c for c in cols if c.startswith('fb_tau/') and joint_name in c]
        fb_p = [c for c in cols if c.startswith('fb_p/') and joint_name in c]
    else:
        # 自动选第一个关节
        cmd_p = [c for c in cols if c.startswith('cmd_p/')]
        cmd_v = [c for c in cols if c.startswith('cmd_v/')]
        cmd_kp = [c for c in cols if c.startswith('cmd_kp/')]
        cmd_tau = [c for c in cols if c.startswith('cmd_tau/')]
        fb_tau = [c for c in cols if c.startswith('fb_tau/')]
        fb_p = [c for c in cols if c.startswith('fb_p/')]

    return {
        'cmd_p': cmd_p[0] if cmd_p else None,
        'cmd_v': cmd_v[0] if cmd_v else None,
        'cmd_kp': cmd_kp[0] if cmd_kp else None,
        'cmd_tau': cmd_tau[0] if cmd_tau else None,
        'fb_tau': fb_tau[0] if fb_tau else None,
        'fb_p': fb_p[0] if fb_p else None,
    }

def detect_setpoints(data, col, threshold=0.005):
    """检测阶跃设定点: 找到数据中的稳定段"""
    values = data[col].values
    setpoints = []
    i = 0
    while i < len(values):
        val = values[i]
        # 找到同一值的连续段
        j = i
        while j < len(values) and abs(values[j] - val) < threshold:
            j += 1
        if j - i > 50:  # 至少 50 个采样点 (0.2s @ 250Hz)
            setpoints.append({
                'value': val,
                'start': i,
                'end': j,
                'duration_samples': j - i
            })
        i = j if j > i else i + 1
    return setpoints

def extract_steady_state(data, col, start, end, settle_samples):
    """提取稳态段均值"""
    actual_start = start + settle_samples
    if actual_start >= end:
        actual_start = start + int((end - start) * 0.5)
    segment = data[col].values[actual_start:end]
    if len(segment) == 0:
        return 0, 0
    return np.mean(segment), np.std(segment)

def main():
    parser = argparse.ArgumentParser(description='KPKD 线性度分析')
    parser.add_argument('csv_file', help='CSV 文件路径')
    parser.add_argument('--joint', help='关节名')
    parser.add_argument('--c2t', type=float, default=4.36, help='c2t 系数')
    parser.add_argument('--settle', type=float, default=2.0, help='稳态丢弃时间 (秒)')
    parser.add_argument('--freq', type=int, default=250, help='采样频率')
    parser.add_argument('--plot', action='store_true', help='生成图')
    parser.add_argument('--output-dir', default='.', help='输出目录')
    args = parser.parse_args()

    try:
        import pandas as pd
    except ImportError:
        print("需要 pandas: pip install pandas")
        return 1

    # 读取 CSV (跳过 # 注释行)
    df = pd.read_csv(args.csv_file, comment='#')
    header = open(args.csv_file).readline()

    cols = find_joint_columns(header, args.joint)
    if not cols['fb_tau']:
        print(f"未找到关节力矩反馈列，请检查 CSV 或指定 --joint")
        return 1

    joint_name = cols['fb_tau'].split('/')[-1] if cols['fb_tau'] else 'unknown'
    print(f"关节: {joint_name}")
    print(f"c2t: {args.c2t}")

    settle_samples = int(args.settle * args.freq)

    # 检测是 Kp 测试 (cmd_p 变化) 还是 Kd 测试 (cmd_v 变化)
    cmd_p_col = cols['cmd_p']
    cmd_v_col = cols['cmd_v']

    # 检测 cmd_p 的设定点变化
    p_setpoints = detect_setpoints(df, cmd_p_col, threshold=0.01) if cmd_p_col else []
    v_setpoints = detect_setpoints(df, cmd_v_col, threshold=0.1) if cmd_v_col else []

    # 判断测试类型
    # Kp 测试: cmd_p 有多个不同的设定点 (且 cmd_v ≈ 0)
    # Kd 测试: cmd_v 有多个不同的设定点
    unique_p = set(round(sp['value'], 3) for sp in p_setpoints)
    unique_v = set(round(sp['value'], 2) for sp in v_setpoints)

    is_kp_test = len(unique_p) > 2
    is_kd_test = len(unique_v) > 2 and not is_kp_test

    if is_kp_test:
        print(f"检测到 Kp 测试 ({len(unique_p)} 个位置设定点)")
        input_col = cmd_p_col
        input_label = 'q_des (rad)'
        test_label = 'Kp'
    elif is_kd_test:
        print(f"检测到 Kd 测试 ({len(unique_v)} 个速度设定点)")
        input_col = cmd_v_col
        input_label = 'dq_des (rad/s)'
        test_label = 'Kd'
    else:
        print("无法自动检测测试类型 (设定点不足)")
        return 1

    # 提取每个设定点的稳态力矩
    setpoints = detect_setpoints(df, input_col, threshold=0.005)
    # 过滤掉零值设定点 (回零阶段)
    init_val = df[input_col].values[0]
    setpoints = [sp for sp in setpoints if abs(sp['value'] - init_val) > 0.01]

    if len(setpoints) < 2:
        print(f"有效设定点不足 ({len(setpoints)}), 需要至少 2 个")
        return 1

    x_data = []
    y_data = []
    print(f"\n{'设定值':>10}  {'tau_fb 均值':>12}  {'I_real':>10}  {'std':>10}")
    print('-' * 50)

    for sp in setpoints:
        input_val = sp['value'] - init_val  # 相对于初始位置的偏移
        tau_mean, tau_std = extract_steady_state(df, cols['fb_tau'], sp['start'], sp['end'], settle_samples)
        I_real = tau_mean / args.c2t

        x_data.append(input_val)
        y_data.append(I_real)
        print(f"{input_val:>10.4f}  {tau_mean:>12.4f} Nm  {I_real:>10.4f} A  {tau_std:>10.4f}")

    x = np.array(x_data)
    y = np.array(y_data)

    # 线性回归
    if len(x) < 2:
        print("数据不足")
        return 1

    slope, intercept = np.polyfit(x, y, 1)
    y_pred = slope * x + intercept
    ss_res = np.sum((y - y_pred) ** 2)
    ss_tot = np.sum((y - np.mean(y)) ** 2)
    r_squared = 1 - ss_res / ss_tot if ss_tot > 1e-15 else 0

    # 读取 Kp/Kd 值 (从 cmd_kp 列)
    if cols['cmd_kp'] and is_kp_test:
        kp_values = df[cols['cmd_kp']].values
        kp_val = np.median(kp_values[kp_values > 0]) if np.any(kp_values > 0) else 0
        expected_slope = kp_val / args.c2t
    else:
        expected_slope = slope  # 无法推算
        kp_val = 0

    slope_err = abs(slope - expected_slope) / abs(expected_slope) * 100 if abs(expected_slope) > 1e-6 else 0
    passed = r_squared >= 0.98 and slope_err <= 10

    print(f"\n--- 线性回归结果 ---")
    print(f"  斜率:     {slope:.4f}")
    print(f"  截距:     {intercept:.4f}")
    print(f"  R²:       {r_squared:.4f}")
    if kp_val > 0:
        print(f"  {test_label}:       {kp_val:.1f}")
        print(f"  期望斜率: {expected_slope:.4f}")
        print(f"  斜率误差: {slope_err:.1f}%")
    print(f"  判定:     {'PASS' if passed else 'FAIL'}")

    # 绘图
    if args.plot:
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt

            fig, ax = plt.subplots(figsize=(8, 5))
            ax.scatter(x, y, c='blue', label='实测数据', zorder=5)

            x_fit = np.linspace(min(x), max(x), 100)
            ax.plot(x_fit, slope * x_fit + intercept, 'r--',
                    label=f'拟合: y={slope:.3f}x+{intercept:.4f}, R²={r_squared:.4f}')

            if kp_val > 0:
                ax.plot(x_fit, expected_slope * x_fit, 'g:',
                        label=f'理论: slope={expected_slope:.3f}', alpha=0.7)

            ax.set_xlabel(input_label)
            ax.set_ylabel('I_real (A)')
            ax.set_title(f'{test_label} 线性验证 — {joint_name} ({"PASS" if passed else "FAIL"})')
            ax.legend()
            ax.grid(True, alpha=0.3)

            os.makedirs(args.output_dir, exist_ok=True)
            plot_path = os.path.join(args.output_dir,
                                     f'kpkd_{test_label.lower()}_{joint_name}.png')
            fig.savefig(plot_path, dpi=150, bbox_inches='tight')
            print(f"  图: {plot_path}")
            plt.close()
        except ImportError:
            print("  (matplotlib 未安装，跳过绘图)")

    return 0 if passed else 1

if __name__ == '__main__':
    sys.exit(main())
