#!/usr/bin/env python3
"""Print average publish rate for a CycloneDDS topic (rostopic hz equivalent)."""

from __future__ import annotations

import argparse
import ctypes
import os
import sys
import time
from pathlib import Path

from cyclonedds.domain import DomainParticipant
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic

from dds_leju_types import Float64Array, UInt16Array


def _topic_type_for(topic: str, requested: str = "auto"):
    """Select the DDS payload type, keeping raw-camera compatibility."""
    if requested == "float64" or (
        requested == "auto"
        and topic in {
            "rt/depth_camera/frame_normalized_36x64",
            "rt/depth_camera/frame_meters_36x64",
        }
    ):
        return Float64Array
    return UInt16Array


def _preload_cyclonedds() -> None:
    if os.environ.get("CYCLONEDDS_PRELOADED") == "1":
        return
    lib_dir = os.environ.get("CYCLONEDDS_LIB_DIR")
    if not lib_dir:
        return
    lib_path = Path(lib_dir) / "libddsc.so"
    if lib_path.is_file():
        ctypes.CDLL(str(lib_path))
        os.environ["CYCLONEDDS_PRELOADED"] = "1"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Measure DDS topic publish rate (similar to rostopic hz)."
    )
    parser.add_argument(
        "topic",
        nargs="?",
        default="rt/depth_camera/frame_normalized_36x64",
        help="DDS topic name (default: rt/depth_camera/frame_normalized_36x64)",
    )
    parser.add_argument(
        "--expected-len",
        type=int,
        default=None,
        help="Expected data length (0 = skip check; inferred from --type)",
    )
    parser.add_argument(
        "--type",
        choices=("auto", "float64", "uint16"),
        default="auto",
        help="DDS payload type (default: infer from topic)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Stop after N seconds (0 = run until Ctrl+C)",
    )
    parser.add_argument(
        "--report-interval",
        type=float,
        default=1.0,
        help="Print rate every N seconds (default: 1.0)",
    )
    args = parser.parse_args()
    if args.expected_len is None:
        args.expected_len = (
            64 * 36
            if _topic_type_for(args.topic, args.type) is Float64Array
            else 480 * 270
        )
    return args


def main() -> int:
    args = _parse_args()
    _preload_cyclonedds()

    topic_name = args.topic
    participant = DomainParticipant(0)
    topic_type = _topic_type_for(topic_name, args.type)
    reader = DataReader(participant, Topic(participant, topic_name, topic_type))

    print(f"subscribed to {topic_name}")
    print(f"DDS payload type: {topic_type.__name__}")
    if args.expected_len > 0:
        print(f"expected {topic_type.__name__}.data length: {args.expected_len}")
    print("Ctrl+C to stop")

    count = 0
    last_len = 0
    start = time.time()
    last_report = start

    try:
        while True:
            samples = reader.take(N=100)
            for sample in samples:
                if sample is None:
                    continue
                count += 1
                last_len = len(sample.data)
                if args.expected_len > 0 and last_len != args.expected_len:
                    print(
                        f"warn: sample len={last_len}, expected {args.expected_len}",
                        file=sys.stderr,
                    )

            now = time.time()
            if now - last_report >= args.report_interval:
                elapsed = now - start
                hz = count / elapsed if elapsed > 0 else 0.0
                print(
                    f"average rate: {hz:.3f} Hz  "
                    f"(total={count}, elapsed={elapsed:.1f}s, last_len={last_len})"
                )
                last_report = now

            if args.duration > 0 and now - start >= args.duration:
                break
            if not samples:
                time.sleep(0.005)
    except KeyboardInterrupt:
        pass

    elapsed = time.time() - start
    hz = count / elapsed if elapsed > 0 else 0.0
    print(
        f"summary: average rate: {hz:.3f} Hz  "
        f"(total={count}, elapsed={elapsed:.1f}s, last_len={last_len})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
