#!/usr/bin/env python3
"""CAN FD 电机跟踪数据可视化 (位置)

用法:
  python3 scripts/plot_tracking.py tracking.csv
  python3 scripts/plot_tracking.py tracking.csv --save tracking.png
"""

import argparse
import sys
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description="电机位置跟踪可视化")
    parser.add_argument("csv_file", help="CSV 日志文件路径")
    parser.add_argument("--save", type=str, default="",
                        help="保存图片路径 (如 tracking.png)")
    args = parser.parse_args()

    df = pd.read_csv(args.csv_file)

    # 自动检测电机ID
    motor_ids = []
    for col in df.columns:
        if col.startswith("M") and col.endswith("_tgt_rad"):
            mid = int(col.split("_")[0][1:])
            motor_ids.append(mid)

    if not motor_ids:
        print("未找到电机数据列", file=sys.stderr)
        sys.exit(1)

    time = df["time_s"].values
    colors = plt.cm.tab10(np.linspace(0, 1, max(len(motor_ids), 1)))

    fig, ax = plt.subplots(figsize=(14, 5))
    fig.suptitle(f"Position Tracking ({args.csv_file})", fontsize=14)

    for i, mid in enumerate(motor_ids):
        tgt = df[f"M{mid}_tgt_rad"].values
        pos = df[f"M{mid}_pos_rad"].values
        ax.plot(time, np.degrees(tgt), "--", color=colors[i], alpha=0.6,
                label=f"M{mid} target")
        ax.plot(time, np.degrees(pos), "-", color=colors[i],
                label=f"M{mid} actual")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position (deg)")
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    if args.save:
        plt.savefig(args.save, dpi=150, bbox_inches="tight")
        print(f"图片已保存: {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
