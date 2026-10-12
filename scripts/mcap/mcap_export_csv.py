#!/usr/bin/env python3
"""
导出数值 topic 为 CSV / Export a numeric topic from an MCAP file to CSV.

每条消息一行, 第一列始终是 `t_s` (距 bag 起点的秒数).
`--field` 可以选 IDL 里任何数值字段 (scalar / array / sequence of numbers);
不传时按 schema 选默认: Float64/Float64Array -> data, JointState/JointCmd -> q,
ImuData -> gyro, Joy -> axes. 列命名规则: scalar 用字段名本身, list 用 `<field>0`..`<field>N`.

Usage:
    mcap_export_csv.py <file.mcap> --topic <name> [--field <k>] [-o <path|->]

Example:
    mcap_export_csv.py rec.mcap --topic /amp/q_target -o qt.csv
    mcap_export_csv.py rec.mcap --topic /rt/joint_state --field q -o js_q.csv
    mcap_export_csv.py rec.mcap --topic /rt/joint_cmd --field modes -o modes.csv   # 23 列控制模式
    mcap_export_csv.py rec.mcap --topic /rt/imu_state --field header_sec -o ts.csv
    mcap_export_csv.py rec.mcap --topic /amp/arm_mode -o - | head
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import leju_msgs  # noqa: E402


# Per-schema default field (used if --field is omitted).
DEFAULT_FIELD = {
    "leju::msgs::Float64":      "data",
    "leju::msgs::Float64Array": "data",
    "leju::msgs::JointState":   "q",
    "leju::msgs::JointCmd":     "q",
    "leju::msgs::ImuData":      "gyro",
    "leju::msgs::Joy":          "axes",
}


def _extract(schema: str, decoded: dict, field: str | None) -> tuple[str, list[float]]:
    """Return (column_prefix, list_of_floats) for one decoded message.

    `decoded` is always a dict (IDL field name -> value). We pick the field to
    export, validate it's numeric, and flatten scalars to a 1-element list.
    """
    key = field or DEFAULT_FIELD.get(schema)
    if key is None:
        raise SystemExit(f"error: no default field for {schema!r}; pass --field <name>. "
                         f"Available fields: {sorted(decoded)}")
    if key not in decoded:
        raise SystemExit(f"error: field {key!r} not in decoded {schema} "
                         f"(available: {sorted(decoded)})")
    v = decoded[key]
    if isinstance(v, bool):   # bool is subclass of int — guard before int branch
        return key, [1.0 if v else 0.0]
    if isinstance(v, (int, float)):
        return key, [float(v)]
    if isinstance(v, list) and all(isinstance(x, (int, float)) for x in v):
        return key, [float(x) for x in v]
    raise SystemExit(f"error: field {schema}.{key} is not numeric "
                     f"(got {type(v).__name__}: {v!r:.80})")


def export(
    mcap_path: str,
    topic: str,
    out_path: str,
    field: str | None,
) -> int:
    try:
        from mcap.reader import make_reader
    except ImportError:
        print("error: mcap library not installed. Run: pip install mcap", file=sys.stderr)
        return 2

    with open(mcap_path, "rb") as f:
        r = make_reader(f)
        summary = r.get_summary()
        bag_start_ns = summary.statistics.message_start_time
        # Register IDL-driven decoders so JointCmd.modes etc. become available.
        leju_msgs.register_from_mcap_summary(summary)

        # locate topic + its schema
        matches = [(cid, ch) for cid, ch in summary.channels.items() if ch.topic == topic]
        if not matches:
            print(f"error: topic {topic!r} not in mcap.", file=sys.stderr)
            print("available topics:", file=sys.stderr)
            for ch in sorted(summary.channels.values(), key=lambda c: c.topic):
                print(f"  {ch.topic}", file=sys.stderr)
            return 1
        _, ch = matches[0]
        schema_name = summary.schemas[ch.schema_id].name

        if not leju_msgs.is_supported(schema_name):
            print(f"error: no decoder for schema {schema_name!r}. "
                  f"Use mcap_dump_topic.py --raw to inspect bytes.", file=sys.stderr)
            return 1

        # open output
        if out_path == "-":
            out_f = sys.stdout
            close = lambda: None
        else:
            out_f = open(out_path, "w", newline="")
            close = out_f.close

        try:
            w = csv.writer(out_f)
            header_written = False
            rows_written = 0

            for sch, chan, m in r.iter_messages(topics=[topic], log_time_order=True):
                decoded = leju_msgs.decode(sch.name, m.data)
                prefix, values = _extract(sch.name, decoded, field)

                if not header_written:
                    if len(values) == 1:
                        header = ["t_s", prefix]
                    else:
                        header = ["t_s"] + [f"{prefix}{i}" for i in range(len(values))]
                    w.writerow(header)
                    header_written = True

                t_s = (m.log_time - bag_start_ns) / 1e9
                w.writerow([f"{t_s:.9f}"] + [f"{v:.9g}" for v in values])
                rows_written += 1
        finally:
            close()

    if out_path != "-":
        print(f"wrote {rows_written} rows to {out_path}", file=sys.stderr)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Export a numeric MCAP topic to CSV (first column = t_s).",
    )
    ap.add_argument("mcap_file")
    ap.add_argument("--topic", required=True)
    ap.add_argument("--field", help="sub-field for dict-shaped schemas "
                                    "(JointState: q/v/vd/tau; JointCmd: q/v/tau/kp/kd; "
                                    "ImuData: gyro/acc/free_acc/quat)")
    ap.add_argument("-o", "--output", default=None,
                    help="output path; '-' for stdout. Default: <topic>.csv in cwd")
    args = ap.parse_args()

    if not Path(args.mcap_file).exists():
        print(f"error: file not found: {args.mcap_file}", file=sys.stderr)
        return 1

    out_path = args.output
    if out_path is None:
        safe = args.topic.strip("/").replace("/", "_") or "topic"
        if args.field:
            safe = f"{safe}_{args.field}"
        out_path = f"{safe}.csv"

    return export(args.mcap_file, args.topic, out_path, args.field)


if __name__ == "__main__":
    sys.exit(main())
