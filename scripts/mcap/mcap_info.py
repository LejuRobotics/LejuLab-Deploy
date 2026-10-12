#!/usr/bin/env python3
"""
MCAP 文件概览 / Summarise an MCAP file (header + statistics + schemas + channels).

和官方 `mcap info` 类似, 但:
  1) 为 Leju `cyclone-dds-recorder` profile 保留完整字段展示;
  2) 额外打印每个 channel 的平均频率;
  3) 在 schema 前面标记 [?] 表示 leju_msgs 还没实现该 schema 的解码器 —
     这是给下游 agent 快速发现 "哪条 topic 无法被 dump/export" 的信号;
  4) 支持 --json 输出, 便于喂给其他脚本或 LLM.

Usage:
    mcap_info.py <file.mcap>                    # human-readable (color TTY)
    mcap_info.py <file.mcap> --json             # JSON to stdout

Example:
    mcap_info.py ~/.ros/lejulab/mcap/recording_2026-04-16-18-21-45_0.mcap
    mcap_info.py recording.mcap --json | jq '.channels[] | select(.hz > 100) | .topic'
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

# Allow running from any cwd: put this file's dir on sys.path so we can import leju_msgs.
sys.path.insert(0, str(Path(__file__).resolve().parent))
import leju_msgs  # noqa: E402


# ---------- ANSI color helpers (disabled when stdout isn't a TTY or --json) ----------

class C:
    _enabled = True

    @classmethod
    def off(cls):
        cls._enabled = False

    @classmethod
    def _wrap(cls, code: str, s: str) -> str:
        return f"\033[{code}m{s}\033[0m" if cls._enabled else s

    @classmethod
    def bold(cls, s):    return cls._wrap("1", s)
    @classmethod
    def dim(cls, s):     return cls._wrap("2", s)
    @classmethod
    def red(cls, s):     return cls._wrap("91", s)
    @classmethod
    def green(cls, s):   return cls._wrap("92", s)
    @classmethod
    def yellow(cls, s):  return cls._wrap("93", s)
    @classmethod
    def cyan(cls, s):    return cls._wrap("96", s)


# ---------- data gathering ----------

def gather(mcap_path: str) -> dict:
    try:
        from mcap.reader import make_reader
    except ImportError:
        print(C.red("error: mcap library not installed. Run: pip install mcap"), file=sys.stderr)
        sys.exit(2)

    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        header = reader.get_header()
        summary = reader.get_summary()

    # Register every IDL schema so downstream iter_messages() gets the richest fields.
    # Value is "dynamic" | "hardcoded" | "error: ..." per schema.
    decoder_status = leju_msgs.register_from_mcap_summary(summary)

    s = summary.statistics
    start_ns = s.message_start_time
    end_ns = s.message_end_time
    duration_s = (end_ns - start_ns) / 1e9 if end_ns > start_ns else 0.0

    schemas = []
    for sid, sch in sorted(summary.schemas.items()):
        schemas.append({
            "id": sid,
            "name": sch.name,
            "encoding": sch.encoding,
            "data_bytes": len(sch.data),
            "decoder": leju_msgs.is_supported(sch.name),
            "decoder_path": decoder_status.get(sch.name, "unknown"),
        })

    counts = s.channel_message_counts
    channels = []
    for cid, ch in sorted(summary.channels.items()):
        sch = summary.schemas[ch.schema_id]
        n = counts.get(cid, 0)
        channels.append({
            "id": cid,
            "topic": ch.topic,
            "schema": sch.name,
            "encoding": ch.message_encoding,
            "messages": n,
            "hz": (n / duration_s) if duration_s > 0 else 0.0,
            "decoder": leju_msgs.is_supported(sch.name),
        })
    channels.sort(key=lambda c: -c["messages"])

    return {
        "file": str(Path(mcap_path).resolve()),
        "header": {"profile": header.profile, "library": header.library},
        "statistics": {
            "message_count": s.message_count,
            "schema_count": s.schema_count,
            "channel_count": s.channel_count,
            "chunk_count": s.chunk_count,
            "attachment_count": s.attachment_count,
            "metadata_count": s.metadata_count,
            "message_start_time_ns": start_ns,
            "message_end_time_ns": end_ns,
            "duration_s": duration_s,
        },
        "schemas": schemas,
        "channels": channels,
    }


# ---------- pretty printing ----------

def print_human(info: dict) -> None:
    print(C.bold(C.cyan("=" * 78)))
    print(C.bold(C.cyan("  MCAP Info")))
    print(C.bold(C.cyan("=" * 78)))

    print(f"\n{C.bold('File')}: {info['file']}")
    h = info["header"]
    print(f"{C.bold('Header')}: profile={h['profile']!r}  library={h['library']!r}")

    s = info["statistics"]
    print(f"\n{C.bold('Statistics')}")
    print(f"  messages={s['message_count']}  channels={s['channel_count']}  "
          f"schemas={s['schema_count']}  chunks={s['chunk_count']}")
    print(f"  attachments={s['attachment_count']}  metadata={s['metadata_count']}")
    print(f"  duration={s['duration_s']:.3f}s  "
          f"start_ns={s['message_start_time_ns']}  end_ns={s['message_end_time_ns']}")

    print(f"\n{C.bold('Schemas')} "
          f"(path: {C.green('dyn')}=IDL-driven via pycdr2, "
          f"{C.cyan('hc')}=hardcoded fallback, "
          f"{C.yellow('?')}=no decoder)")
    for sch in info["schemas"]:
        path = sch["decoder_path"]
        if path == "dynamic":
            tag = C.green("dyn")
        elif path == "hardcoded":
            tag = C.cyan("hc ")
        else:
            tag = C.yellow("?  ")
        print(f"  [{tag}] id={sch['id']:<3} {sch['name']:<32}  "
              f"enc={sch['encoding']:<8}  data={sch['data_bytes']}B")

    print(f"\n{C.bold('Channels')}  (topic / schema / msgs / Hz)")
    if not info["channels"]:
        print(C.dim("  (none)"))
        return
    max_topic = max(len(c["topic"]) for c in info["channels"])
    max_schema = max(len(c["schema"]) for c in info["channels"])
    for c in info["channels"]:
        tag = C.green("✓") if c["decoder"] else C.yellow("?")
        print(f"  {tag} {c['topic']:<{max_topic}}  "
              f"{c['schema']:<{max_schema}}  "
              f"msgs={c['messages']:>7}  "
              f"{c['hz']:>7.2f} Hz")


# ---------- cli ----------

def main() -> int:
    ap = argparse.ArgumentParser(
        description="Summarise an MCAP file (header, statistics, schemas, channels).",
        epilog="See scripts/mcap/README.md for the full script set.",
    )
    ap.add_argument("mcap_file", help="path to .mcap file")
    ap.add_argument("--json", action="store_true", help="emit JSON instead of human-readable output")
    args = ap.parse_args()

    if not Path(args.mcap_file).exists():
        print(f"error: file not found: {args.mcap_file}", file=sys.stderr)
        return 1

    if args.json or not sys.stdout.isatty():
        C.off()

    info = gather(args.mcap_file)

    if args.json:
        json.dump(info, sys.stdout, indent=2)
        sys.stdout.write("\n")
    else:
        print_human(info)
    return 0


if __name__ == "__main__":
    sys.exit(main())
