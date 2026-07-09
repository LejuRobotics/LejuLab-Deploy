#!/usr/bin/env python3
"""
VelocityLimitedGenerator 可视化脚本
对比 kPerJoint 和 kSynchronized 两种限速模式
展示目标动态变化响应
"""
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

# ============================================================================
# 图1: 静态目标对比 (PerJoint vs Synchronized)
# ============================================================================

data_file = '/tmp/trajectory_generator_data.csv'
if os.path.exists(data_file):
    df = pd.read_csv(data_file)

    # 自动检测关节数
    num_joints = 0
    while f'j{num_joints}_pos_pj' in df.columns:
        num_joints += 1

    print(f"静态目标数据: {num_joints}-DOF")

    targets = [df[f'j{i}_pos_pj'].iloc[-1] for i in range(num_joints)]
    colors = plt.cm.tab10(np.linspace(0, 1, num_joints))

    # 2x2 布局
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # 左上: 位置对比
    ax = axes[0, 0]
    for i in range(num_joints):
        ax.plot(df['time'], df[f'j{i}_pos_pj'], '--', color='gray', linewidth=1, alpha=0.6)
        ax.plot(df['time'], df[f'j{i}_pos_sync'], '-', color=colors[i], linewidth=1.5, label=f'J{i}')
    ax.set_ylabel('Position (rad)')
    ax.set_xlabel('Time (s)')
    ax.set_title('Position (solid=Synchronized, dashed=PerJoint)')
    ax.legend(loc='best', ncol=2, fontsize=8)
    ax.grid(True, alpha=0.3)

    # 右上: 速度对比
    ax = axes[0, 1]
    for i in range(num_joints):
        ax.plot(df['time'], df[f'j{i}_vel_pj'], '--', color='gray', linewidth=1, alpha=0.6)
        ax.plot(df['time'], df[f'j{i}_vel_sync'], '-', color=colors[i], linewidth=1.5, label=f'J{i}')
    ax.set_ylabel('Velocity (rad/s)')
    ax.set_xlabel('Time (s)')
    ax.set_title('Velocity (solid=Synchronized, dashed=PerJoint)')
    ax.legend(loc='best', ncol=2, fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color='black', linewidth=0.5)
    ax.axhline(y=0.5, color='red', linestyle=':', alpha=0.5, linewidth=1)
    ax.axhline(y=-0.5, color='red', linestyle=':', alpha=0.5, linewidth=1)

    # 左下: 误差曲线
    ax = axes[1, 0]
    for i in range(num_joints):
        error_pj = np.abs(df[f'j{i}_pos_pj'] - targets[i])
        error_sync = np.abs(df[f'j{i}_pos_sync'] - targets[i])
        ax.plot(df['time'], error_pj, '--', color='gray', linewidth=1, alpha=0.6)
        ax.plot(df['time'], error_sync, '-', color=colors[i], linewidth=1.5, label=f'J{i}')
    ax.set_ylabel('Position Error (rad)')
    ax.set_xlabel('Time (s)')
    ax.set_title('Position Error (solid=Synchronized, dashed=PerJoint)')
    ax.legend(loc='best', ncol=2, fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_yscale('log')
    ax.set_ylim(bottom=1e-6)

    # 右下: 到达时间
    ax = axes[1, 1]
    threshold = 0.01
    arrival_times_pj = []
    arrival_times_sync = []
    for j in range(num_joints):
        for idx, row in df.iterrows():
            if abs(row[f'j{j}_pos_pj'] - targets[j]) < threshold:
                arrival_times_pj.append(row['time'])
                break
        else:
            arrival_times_pj.append(df['time'].max())
        for idx, row in df.iterrows():
            if abs(row[f'j{j}_pos_sync'] - targets[j]) < threshold:
                arrival_times_sync.append(row['time'])
                break
        else:
            arrival_times_sync.append(df['time'].max())

    x = np.arange(num_joints)
    width = 0.35
    ax.bar(x - width/2, arrival_times_pj, width, label='PerJoint', color='gray', alpha=0.7)
    ax.bar(x + width/2, arrival_times_sync, width, label='Synchronized', color='steelblue', alpha=0.8)
    ax.set_ylabel('Arrival Time (s)')
    ax.set_xlabel('Joint')
    ax.set_title(f'Time to Reach Target (threshold={threshold} rad)')
    ax.set_xticks(x)
    ax.set_xticklabels([f'J{i}' for i in range(num_joints)])
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')

    plt.suptitle(f'VelocityLimitedGenerator: {num_joints}-DOF Static Target Comparison',
                 fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('/tmp/trajectory_generator.png', dpi=150, bbox_inches='tight')
    print("静态目标图表已保存到 /tmp/trajectory_generator.png")
else:
    print(f"静态目标数据文件 {data_file} 不存在")

# ============================================================================
# 图2: 动态目标响应
# ============================================================================

dynamic_file = '/tmp/trajectory_generator_dynamic.csv'
if os.path.exists(dynamic_file):
    df_dyn = pd.read_csv(dynamic_file)

    # 自动检测关节数
    num_joints = 0
    while f'j{num_joints}_pos' in df_dyn.columns:
        num_joints += 1

    print(f"动态目标数据: {num_joints}-DOF")

    colors = plt.cm.tab10(np.linspace(0, 1, num_joints))

    # 2x1 布局: 位置和速度
    fig2, axes2 = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    # 上: 位置 + 目标
    ax = axes2[0]
    for i in range(num_joints):
        ax.plot(df_dyn['time'], df_dyn[f'j{i}_pos'], '-',
                color=colors[i], linewidth=2, label=f'J{i} pos')
        ax.plot(df_dyn['time'], df_dyn[f'j{i}_target'], '--',
                color=colors[i], linewidth=1.5, alpha=0.7, label=f'J{i} target')
    ax.set_ylabel('Position (rad)')
    ax.set_title('Dynamic Target Tracking (solid=position, dashed=target)')
    ax.legend(loc='best', ncol=3, fontsize=8)
    ax.grid(True, alpha=0.3)

    # 标记目标切换时间 (在未到达时切换)
    target_changes = [1.5, 3.5]
    for tc in target_changes:
        ax.axvline(x=tc, color='red', linestyle=':', alpha=0.7, linewidth=1.5)
        ax.text(tc + 0.1, ax.get_ylim()[1] * 0.9, f't={tc}s\ntarget\nchange',
                fontsize=8, color='red', va='top')

    # 下: 速度
    ax = axes2[1]
    for i in range(num_joints):
        ax.plot(df_dyn['time'], df_dyn[f'j{i}_vel'], '-',
                color=colors[i], linewidth=1.5, label=f'J{i}')
    ax.set_ylabel('Velocity (rad/s)')
    ax.set_xlabel('Time (s)')
    ax.set_title('Velocity Response to Target Changes (mid-motion switching)')
    ax.legend(loc='best', ncol=3, fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color='black', linewidth=0.5)
    ax.axhline(y=0.5, color='red', linestyle=':', alpha=0.5, linewidth=1)
    ax.axhline(y=-0.5, color='red', linestyle=':', alpha=0.5, linewidth=1)

    # 标记目标切换时间
    for tc in target_changes:
        ax.axvline(x=tc, color='red', linestyle=':', alpha=0.7, linewidth=1.5)

    plt.suptitle('VelocityLimitedGenerator: Dynamic Target Response',
                 fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('/tmp/trajectory_generator_dynamic.png', dpi=150, bbox_inches='tight')
    print("动态目标图表已保存到 /tmp/trajectory_generator_dynamic.png")
else:
    print(f"动态目标数据文件 {dynamic_file} 不存在")
    print("请运行: ./devel/lib/leju-rl-controller/test_velocity_limited_generator --gtest_filter=*Dynamic*")

plt.show()
