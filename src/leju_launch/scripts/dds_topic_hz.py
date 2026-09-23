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

from dds_leju_types import Float64Array


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
        default="rt/depth_camera/frame_mm_240x424",
        help="DDS topic name (default: rt/depth_camera/frame_mm_240x424)",
    )
    parser.add_argument(
        "--expected-len",
        type=int,
        default=424 * 240,
        help="Expected Float64Array.data length (0 = skip check, default: 101760)",
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
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    _preload_cyclonedds()

    topic_name = args.topic
    participant = DomainParticipant(0)
    reader = DataReader(participant, Topic(participant, topic_name, Float64Array))

    print(f"subscribed to {topic_name}")
    if args.expected_len > 0:
        print(f"expected Float64Array.data length: {args.expected_len}")
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
