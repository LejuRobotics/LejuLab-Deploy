#!/usr/bin/env python3
"""
导出单个 topic 的消息为 JSON Lines / Dump one MCAP topic as JSONL.

每行一条消息:
    {"t": <sec_from_bag_start>, "log_time_ns": <int>, "topic": <str>, "data": <decoded>}

适合:
  - 给其他 agent 读取 (结构化, 每行独立);
  - `jq` 过滤与统计;
  - `--raw` 导出 hex, 便于写/调试新解码器.

Usage:
    mcap_dump_topic.py <file.mcap> --topic <name>
                       [--start <s>] [--end <s>]
                       [--limit N] [--head N | --tail N]
                       [--raw]                     # hex bytes instead of decoding
                       [--pretty]                  # indent JSON (not strictly JSONL)

Example:
    mcap_dump_topic.py rec.mcap --topic /rt/imu_state --head 3
    mcap_dump_topic.py rec.mcap --topic /amp/q_target --start 10 --end 15 | jq '.data[0]'
    mcap_dump_topic.py rec.mcap --topic /some/unknown --head 1 --raw   # no decoder yet
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import leju_msgs  # noqa: E402


def iterate(
    mcap_path: str,
    topic: str,
    start_s: float | None,
    end_s: float | None,
    raw: bool,
):
    """Yield decoded message records in log_time order."""
    try:
        from mcap.reader import make_reader
    except ImportError:
        print("error: mcap library not installed. Run: pip install mcap", file=sys.stderr)
        sys.exit(2)

    with open(mcap_path, "rb") as f:
        r = make_reader(f)
        summary = r.get_summary()
        bag_start_ns = summary.statistics.message_start_time
        # Dynamically build pycdr2 decoders from the mcap's own IDL, so new schemas
        # (or extra fields like JointCmd.modes) surface automatically.
        leju_msgs.register_from_mcap_summary(summary)

        # validate topic exists
        topics = {ch.topic for ch in summary.channels.values()}
        if topic not in topics:
            print(f"error: topic {topic!r} not in mcap. Available ({len(topics)}):", file=sys.stderr)
            for t in sorted(topics):
                print(f"  {t}", file=sys.stderr)
            sys.exit(1)

        # convert relative [start_s, end_s] to absolute ns bounds
        start_ns = bag_start_ns + int(start_s * 1e9) if start_s is not None else None
        end_ns   = bag_start_ns + int(end_s * 1e9)   if end_s   is not None else None

        for sch, ch, m in r.iter_messages(
            topics=[topic],
            start_time=start_ns,
            end_time=end_ns,
            log_time_order=True,
        ):
            rec = {
                "t": (m.log_time - bag_start_ns) / 1e9,
                "log_time_ns": m.log_time,
                "topic": ch.topic,
            }
            if raw:
                rec["schema"] = sch.name
                rec["data_hex"] = m.data.hex()
                rec["data_len"] = len(m.data)
            else:
                try:
                    rec["data"] = leju_msgs.decode(sch.name, m.data)
                except leju_msgs.UnsupportedSchema as e:
                    # degrade gracefully: flag + hex so the agent can extend leju_msgs
                    rec["schema"] = sch.name
                    rec["decode_error"] = str(e)
                    rec["data_hex"] = m.data.hex()
                    rec["data_len"] = len(m.data)
            yield rec


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Dump one topic's messages as JSON Lines.",
    )
    ap.add_argument("mcap_file")
    ap.add_argument("--topic", required=True, help="topic to dump (exact match)")
    ap.add_argument("--start", type=float, help="start time (seconds from bag start)")
    ap.add_argument("--end",   type=float, help="end time (seconds from bag start)")
    ap.add_argument("--limit", type=int,   help="stop after N messages (applied after filters)")
    sel = ap.add_mutually_exclusive_group()
    sel.add_argument("--head", type=int, help="keep only first N messages after filters (same as --limit)")
    sel.add_argument("--tail", type=int, help="keep only last N messages after filters")
    ap.add_argument("--raw", action="store_true", help="hex-dump bytes instead of decoding")
    ap.add_argument("--pretty", action="store_true", help="indent each JSON record (output is no longer JSONL)")
    args = ap.parse_args()

    if not Path(args.mcap_file).exists():
        print(f"error: file not found: {args.mcap_file}", file=sys.stderr)
        return 1

    it = iterate(args.mcap_file, args.topic, args.start, args.end, args.raw)

    # Apply limit / head / tail.
    limit = args.head if args.head is not None else args.limit
    if args.tail is not None:
        # tail needs buffering
        buf: deque[dict] = deque(maxlen=args.tail)
        for rec in it:
            buf.append(rec)
        records = iter(buf)
    elif limit is not None:
        def _bounded():
            for i, rec in enumerate(it):
                if i >= limit:
                    break
                yield rec
        records = _bounded()
    else:
        records = it

    indent = 2 if args.pretty else None
    for rec in records:
        sys.stdout.write(json.dumps(rec, separators=(",", ":") if not args.pretty else (",", ": "),
                                    indent=indent, ensure_ascii=False))
        sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
