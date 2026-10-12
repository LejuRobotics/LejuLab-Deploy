#!/usr/bin/env python3
"""
IDL-driven vs hardcoded decoder 全量对拍测试.

对 mcap 中每一条消息，分别用 pycdr2 动态解码和内置硬编码解码，逐字段对比:
  - 不一致 → 报 mismatch + 前 5 条差异详情
  - 全一致 → 打印 "✓ all matched" + 性能数据

Usage:
    python3 spike_idl_decode.py <file.mcap>

Example:
    python3 spike_idl_decode.py ~/.ros/lejulab/mcap/recording_2026-04-16-18-21-45_0.mcap
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import leju_msgs  # noqa: E402


def run(mcap_path: str) -> int:
    try:
        from mcap.reader import make_reader
    except ImportError:
        print("install mcap: pip install mcap", file=sys.stderr)
        return 2

    # 1. Register → builds dynamic decoders from IDL
    with open(mcap_path, "rb") as f:
        summary = make_reader(f).get_summary()
    status = leju_msgs.register_from_mcap_summary(summary)
    n_dyn = sum(1 for v in status.values() if v == "dynamic")
    n_hc  = sum(1 for v in status.values() if v == "hardcoded")
    n_err = sum(1 for v in status.values() if v.startswith("error"))
    print(f"schemas: {n_dyn} dynamic, {n_hc} hardcoded-only, {n_err} errors")
    if n_dyn == 0:
        print("pycdr2 not installed or IDL parse failed — nothing to cross-check", file=sys.stderr)
        return 2

    # 2. Decode every message via both paths, cross-check
    per_schema: dict[str, dict] = {}
    mismatches: list[tuple] = []
    total = 0
    t0 = time.time()

    with open(mcap_path, "rb") as f:
        for sch, ch, m in make_reader(f).iter_messages():
            total += 1
            ps = per_schema.setdefault(sch.name, {"n": 0, "ok": 0})
            ps["n"] += 1

            # Both paths must be in _HARDCODED to cross-check
            if sch.name not in leju_msgs._HARDCODED or sch.name not in leju_msgs._DYNAMIC:
                continue

            dyn = leju_msgs.decode(sch.name, m.data)                       # dynamic
            hc  = leju_msgs._HARDCODED[sch.name](m.data)                   # hardcoded

            if dyn == hc:
                ps["ok"] += 1
            elif len(mismatches) < 5:
                diff = {k for k in set(dyn) | set(hc) if dyn.get(k) != hc.get(k)}
                mismatches.append((ch.topic, sch.name, diff))

    dt = time.time() - t0

    # 3. Report
    print(f"\nprocessed {total} msgs in {dt:.2f}s ({total/dt:.0f} msg/s)\n")
    all_ok = True
    for name, ps in sorted(per_schema.items(), key=lambda x: -x[1]["n"]):
        flag = "✓" if ps["ok"] == ps["n"] else "✗"
        if ps["ok"] != ps["n"]:
            all_ok = False
        print(f"  {flag} {name:<32} {ps['n']:>7} msgs  {ps['ok']:>7} matched")

    if mismatches:
        print(f"\nfirst {len(mismatches)} mismatch(es):")
        for topic, schema, diff_keys in mismatches:
            print(f"  {topic} ({schema}): differing keys = {diff_keys}")

    if all_ok and not mismatches:
        print("\n✓ all messages matched — dynamic and hardcoded decoders are consistent")
        return 0
    return 1


def main() -> int:
    ap = argparse.ArgumentParser(description="Cross-check dynamic vs hardcoded decoders on an mcap.")
    ap.add_argument("mcap_file")
    args = ap.parse_args()
    if not Path(args.mcap_file).exists():
        print(f"not found: {args.mcap_file}", file=sys.stderr)
        return 1
    return run(args.mcap_file)


if __name__ == "__main__":
    sys.exit(main())
