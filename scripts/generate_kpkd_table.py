#!/usr/bin/env python3
"""KPKD 换算表生成脚本 — 从 kuavo.json 生成各关节等效 Kp/Kd 对照表"""

import argparse
import json
import os
import sys

# 默认关节名列表 (roban_v14, 23-DOF)
DEFAULT_JOINT_NAMES = [
    "leg_l1", "leg_l2", "leg_l3", "leg_l4", "leg_l5", "leg_l6",
    "leg_r1", "leg_r2", "leg_r3", "leg_r4", "leg_r5", "leg_r6",
    "waist",
    "zarm_l1", "zarm_l2", "zarm_l3", "zarm_l4",
    "zarm_r1", "zarm_r2", "zarm_r3", "zarm_r4",
    "head_yaw", "head_pitch",
]


def find_default_config():
    """自动搜索 kuavo.json"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(script_dir, "..", "src", "leju-hardware", "config",
                     "roban_v14", "kuavo.json"),
        os.path.join(script_dir, "..", "config", "roban_v14", "kuavo.json"),
    ]
    for c in candidates:
        p = os.path.normpath(c)
        if os.path.isfile(p):
            return p
    return None


def compute_c2t_avg(c2t_entry):
    """计算 c2t 系数的平均值 (Nm/A)"""
    coeffs = c2t_entry.get("c2t_coeff", [])
    if not coeffs:
        return 0.0
    return sum(coeffs) / len(coeffs)


def resolve_motor_c2t(motor_type, motor_c2t):
    """查找电机类型对应的 C2T 平均值

    尝试顺序: 精确匹配 -> 去掉 motorevo_ 前缀 -> "ruiwo" fallback
    """
    if motor_type in motor_c2t:
        return compute_c2t_avg(motor_c2t[motor_type]), motor_type
    # 去掉 motorevo_ 前缀
    stripped = motor_type.replace("motorevo_", "")
    if stripped in motor_c2t:
        return compute_c2t_avg(motor_c2t[stripped]), stripped
    # ruiwo fallback
    if "ruiwo" in motor_type.lower() and "ruiwo" in motor_c2t:
        return compute_c2t_avg(motor_c2t["ruiwo"]), "ruiwo"
    return 0.0, motor_type


def generate_table(config_path, joint_names=None):
    """生成 KPKD 换算表"""
    with open(config_path, "r") as f:
        cfg = json.load(f)

    motors_type = cfg.get("MOTORS_TYPE", [])
    ruiwo_kp = cfg.get("ruiwo_kp", [])
    ruiwo_kd = cfg.get("ruiwo_kd", [])
    motor_c2t = cfg.get("MOTOR_C2T", {})
    num_joints = cfg.get("NUM_JOINT", len(motors_type))

    if joint_names is None:
        joint_names = DEFAULT_JOINT_NAMES[:num_joints]

    # 验证长度
    n = min(len(motors_type), len(ruiwo_kp), len(ruiwo_kd), len(joint_names))
    if n == 0:
        print("错误: kuavo.json 中缺少 MOTORS_TYPE / ruiwo_kp / ruiwo_kd")
        sys.exit(1)

    rows = []
    for i in range(n):
        mt = motors_type[i]
        kp_raw = ruiwo_kp[i]
        kd_raw = ruiwo_kd[i]
        c2t_avg, c2t_key = resolve_motor_c2t(mt, motor_c2t)
        equiv_kp = kp_raw * c2t_avg
        equiv_kd = kd_raw * c2t_avg
        rows.append({
            "idx": i + 1,
            "joint": joint_names[i],
            "motor_type": mt,
            "kp_raw": kp_raw,
            "kd_raw": kd_raw,
            "c2t_key": c2t_key,
            "c2t_avg": c2t_avg,
            "equiv_kp": equiv_kp,
            "equiv_kd": equiv_kd,
        })
    return rows


def print_markdown(rows):
    """输出 Markdown 表格"""
    print("\n## KPKD Conversion Table\n")
    print("| # | Joint | Motor Type | Kp (A/rad) | Kd (A*s/rad) | "
          "C2T Key | C2T_avg (Nm/A) | Equiv Kp (Nm/rad) | Equiv Kd (Nm*s/rad) |")
    print("|---|-------|-----------|-----------|-------------|"
          "---------|---------------|-------------------|---------------------|")
    for r in rows:
        print(f"| {r['idx']} | {r['joint']} | {r['motor_type']} | "
              f"{r['kp_raw']:.3f} | {r['kd_raw']:.3f} | "
              f"{r['c2t_key']} | {r['c2t_avg']:.4f} | "
              f"{r['equiv_kp']:.2f} | {r['equiv_kd']:.3f} |")

    # 摘要
    print("\n### Summary\n")
    print(f"- Total joints: {len(rows)}")
    motor_types = set(r["motor_type"] for r in rows)
    for mt in sorted(motor_types):
        subset = [r for r in rows if r["motor_type"] == mt]
        kp_range = f"{min(r['equiv_kp'] for r in subset):.1f} ~ {max(r['equiv_kp'] for r in subset):.1f}"
        print(f"- **{mt}** ({len(subset)} joints): Equiv Kp range = {kp_range} Nm/rad")


def main():
    parser = argparse.ArgumentParser(description="KPKD 换算表生成器")
    parser.add_argument("--config", default=None,
                        help="kuavo.json 路径 (默认自动搜索 roban_v14)")
    args = parser.parse_args()

    if args.config:
        config_path = args.config
    else:
        config_path = find_default_config()
        if config_path is None:
            print("未找到 kuavo.json, 请用 --config 指定路径")
            sys.exit(1)

    print(f"配置文件: {config_path}")
    rows = generate_table(config_path)
    print_markdown(rows)


if __name__ == "__main__":
    main()
