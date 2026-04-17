#!/usr/bin/env python3
"""
每个 topic 的时间统计 / Per-topic timing stats (Hz, jitter, drops).

对每个 channel 计算:
  - count, first/last log_time, 持续时长
  - mean Hz, p50/p99 inter-arrival dt (ms)
  - max gap (ms), 粗略丢帧估计 (dt > 3 × median)

用途: 定位丢帧/发布者掉线、对比两次录制的时序差异.

Usage:
    mcap_topic_stats.py <file.mcap> [--topic <t>]... [--json]

Example:
    mcap_topic_stats.py recording.mcap
    mcap_topic_stats.py recording.mcap --topic /rt/joint_cmd --topic /rt/joint_state
    mcap_topic_stats.py recording.mcap --json | jq '.[] | select(.drops > 0)'
"""

from __future__ import annotations

import argparse
import bisect
import json
import sys
from pathlib import Path


def _percentile(sorted_vals: list[float], q: float) -> float:
    if not sorted_vals:
        return 0.0
    # linear interpolation, like numpy.percentile default
    k = (len(sorted_vals) - 1) * q
    lo = int(k)
    hi = min(lo + 1, len(sorted_vals) - 1)
    return sorted_vals[lo] + (sorted_vals[hi] - sorted_vals[lo]) * (k - lo)


def compute_stats(mcap_path: str, topic_filter: list[str] | None) -> list[dict]:
    try:
        from mcap.reader import make_reader
    except ImportError:
        print("error: mcap library not installed. Run: pip install mcap", file=sys.stderr)
        sys.exit(2)

    # Accumulate log_times per channel.
    times_by_topic: dict[str, list[int]] = {}
    schema_by_topic: dict[str, str] = {}

    with open(mcap_path, "rb") as f:
        r = make_reader(f)
        summary = r.get_summary()
        # pre-populate so topics with 0 messages still appear (rare)
        for ch in summary.channels.values():
            if topic_filter and ch.topic not in topic_filter:
                continue
            times_by_topic.setdefault(ch.topic, [])
            schema_by_topic[ch.topic] = summary.schemas[ch.schema_id].name

        for _sch, ch, m in r.iter_messages(topics=topic_filter if topic_filter else None):
            times_by_topic[ch.topic].append(m.log_time)

    out: list[dict] = []
    for topic, ts in sorted(times_by_topic.items()):
        ts.sort()
        n = len(ts)
        if n == 0:
            out.append({
                "topic": topic, "schema": schema_by_topic.get(topic, ""),
                "messages": 0, "hz": 0.0,
            })
            continue
        first_ns, last_ns = ts[0], ts[-1]
        duration_s = (last_ns - first_ns) / 1e9
        if n < 2 or duration_s <= 0:
            out.append({
                "topic": topic, "schema": schema_by_topic.get(topic, ""),
                "messages": n,
                "first_ns": first_ns, "last_ns": last_ns, "duration_s": duration_s,
                "hz": 0.0,
            })
            continue

        dts_ms = [(b - a) / 1e6 for a, b in zip(ts, ts[1:])]
        dts_ms.sort()
        p50 = _percentile(dts_ms, 0.50)
        p99 = _percentile(dts_ms, 0.99)
        max_dt = dts_ms[-1]
        # "drop" = dt > max(3*p50, 2ms). Conservative; avoids fp noise at very high rates.
        threshold = max(3.0 * p50, 2.0)
        drops = sum(1 for d in dts_ms if d > threshold)

        out.append({
            "topic": topic,
            "schema": schema_by_topic.get(topic, ""),
            "messages": n,
            "first_ns": first_ns,
            "last_ns": last_ns,
            "duration_s": duration_s,
            "hz": (n - 1) / duration_s,
            "dt_ms_p50": p50,
            "dt_ms_p99": p99,
            "dt_ms_max": max_dt,
            "drops": drops,
            "drop_threshold_ms": threshold,
        })

    out.sort(key=lambda r: -r["messages"])
    return out


def print_human(rows: list[dict]) -> None:
    if not rows:
        print("(no channels match filter)")
        return
    # header
    cols = ["topic", "schema", "msgs", "Hz", "p50dt(ms)", "p99dt(ms)", "maxdt(ms)", "drops"]
    widths = [
        max(len("topic"),    max(len(r["topic"]) for r in rows)),
        max(len("schema"),   max(len(r["schema"]) for r in rows)),
        9, 8, 10, 10, 10, 6,
    ]
    fmt = "  ".join(f"{{:<{w}}}" if i < 2 else f"{{:>{w}}}" for i, w in enumerate(widths))
    print(fmt.format(*cols))
    print("  ".join("-" * w for w in widths))
    for r in rows:
        if r["messages"] < 2:
            print(fmt.format(
                r["topic"], r["schema"], r["messages"],
                f"{r.get('hz', 0):.2f}", "-", "-", "-", "-",
            ))
            continue
        print(fmt.format(
            r["topic"], r["schema"], r["messages"],
            f"{r['hz']:.2f}",
            f"{r['dt_ms_p50']:.3f}",
            f"{r['dt_ms_p99']:.3f}",
            f"{r['dt_ms_max']:.3f}",
            r["drops"],
        ))


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Per-topic timing stats (Hz, jitter p50/p99, gaps, drop estimate).",
    )
    ap.add_argument("mcap_file")
    ap.add_argument("--topic", action="append", default=[],
                    help="filter to topic(s); pass multiple times to select several. Default: all")
    ap.add_argument("--json", action="store_true", help="emit JSON to stdout")
    args = ap.parse_args()

    if not Path(args.mcap_file).exists():
        print(f"error: file not found: {args.mcap_file}", file=sys.stderr)
        return 1

    rows = compute_stats(args.mcap_file, args.topic or None)

    if args.json:
        json.dump(rows, sys.stdout, indent=2)
        sys.stdout.write("\n")
    else:
        print_human(rows)
    return 0


if __name__ == "__main__":
    sys.exit(main())
