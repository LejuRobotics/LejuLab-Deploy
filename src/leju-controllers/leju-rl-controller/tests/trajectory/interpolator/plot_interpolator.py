#!/usr/bin/env python3
"""
MinimumJerkInterpolator 可视化脚本
对比线性插值与最小急动度插值的 7-DOF 机械臂轨迹
"""
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np

df = pd.read_csv('/tmp/interpolator_data.csv')

num_joints = 7
colors = plt.cm.tab10(np.linspace(0, 1, num_joints))

# 单窗口布局使用 GridSpec
fig = plt.figure(figsize=(16, 12))
gs = gridspec.GridSpec(3, 2, height_ratios=[1, 0.8, 0.8], hspace=0.35, wspace=0.25)

# ============================================================================
# 第1行: 全部关节总览
# ============================================================================

# 位置总览
ax_pos_all = fig.add_subplot(gs[0, 0])
for i in range(num_joints):
    ax_pos_all.plot(df['time'], df[f'j{i}_linear'], '--',
                    color=colors[i], linewidth=1, alpha=0.4)
    ax_pos_all.plot(df['time'], df[f'j{i}_mjerk'], '-',
                    label=f'J{i}', color=colors[i], linewidth=1.5)
ax_pos_all.set_ylabel('Position (rad)')
ax_pos_all.set_xlabel('Time (s)')
ax_pos_all.set_title('All Joints Position (solid=MinJerk, dashed=Linear)')
ax_pos_all.legend(loc='upper left', ncol=4, fontsize=8)
ax_pos_all.grid(True, alpha=0.3)

# 速度总览
ax_vel_all = fig.add_subplot(gs[0, 1])
for i in range(num_joints):
    ax_vel_all.plot(df['time'], df[f'j{i}_vel_linear'], '--',
                    color=colors[i], linewidth=1, alpha=0.4)
    ax_vel_all.plot(df['time'], df[f'j{i}_vel_mjerk'], '-',
                    label=f'J{i}', color=colors[i], linewidth=1.5)
ax_vel_all.set_ylabel('Velocity (rad/s)')
ax_vel_all.set_xlabel('Time (s)')
ax_vel_all.set_title('All Joints Velocity (solid=MinJerk, dashed=Linear)')
ax_vel_all.legend(loc='upper left', ncol=4, fontsize=8)
ax_vel_all.grid(True, alpha=0.3)
ax_vel_all.axhline(y=0, color='black', linewidth=0.5)

# ============================================================================
# 第2-3行: 分关节显示 (使用嵌套 GridSpec)
# ============================================================================

# 位置分关节 (第2行)
gs_pos = gridspec.GridSpecFromSubplotSpec(1, num_joints, subplot_spec=gs[1, :], wspace=0.3)
for i in range(num_joints):
    ax = fig.add_subplot(gs_pos[0, i])
    ax.plot(df['time'], df[f'j{i}_linear'], '--', color='gray', linewidth=1)
    ax.plot(df['time'], df[f'j{i}_mjerk'], '-', color=colors[i], linewidth=1.5)
    ax.set_title(f'J{i} Pos', fontsize=9)
    ax.tick_params(axis='both', labelsize=7)
    ax.grid(True, alpha=0.3)
    if i == 0:
        ax.set_ylabel('Pos (rad)', fontsize=8)

# 速度分关节 (第3行)
gs_vel = gridspec.GridSpecFromSubplotSpec(1, num_joints, subplot_spec=gs[2, :], wspace=0.3)
for i in range(num_joints):
    ax = fig.add_subplot(gs_vel[0, i])
    ax.plot(df['time'], df[f'j{i}_vel_linear'], '--', color='gray', linewidth=1)
    ax.plot(df['time'], df[f'j{i}_vel_mjerk'], '-', color=colors[i], linewidth=1.5)
    ax.set_title(f'J{i} Vel', fontsize=9)
    ax.set_xlabel('t(s)', fontsize=8)
    ax.tick_params(axis='both', labelsize=7)
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color='black', linewidth=0.3)
    if i == 0:
        ax.set_ylabel('Vel (rad/s)', fontsize=8)

plt.suptitle('7-DOF Arm: Linear vs Minimum Jerk Interpolation', fontsize=14, fontweight='bold', y=0.98)
plt.savefig('/tmp/interpolator_7dof.png', dpi=150, bbox_inches='tight')
print("图表已保存到 /tmp/interpolator_7dof.png")
plt.show()
