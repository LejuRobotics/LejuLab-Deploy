#!/usr/bin/env python3
"""Depth DDS jet-colormap side-by-side compare (raw 480x270 vs policy 64x36)."""

from __future__ import annotations

import argparse
import ctypes
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from cyclonedds.domain import DomainParticipant
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic

from dds_leju_types import Float64Array, UInt16Array
from depth_processor import process_depth_mm_frame
from depth_dds_source import NORMALIZED_TOPIC, default_shape, resolve_type
from depth_viz import compose_jet_compare_frame

DEFAULT_TOPIC = NORMALIZED_TOPIC


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
        description=(
            "Subscribe depth DDS, render raw vs policy 64x36 with jet (classic "
            "rainbow) colormap for visual comparison."
        )
    )
    parser.add_argument(
        "topic",
        nargs="?",
        default=DEFAULT_TOPIC,
        help=f"DDS topic (default: {DEFAULT_TOPIC})",
    )
    parser.add_argument(
        "--input-width", type=int, default=0, help="Raw width (0 = infer from topic/type)"
    )
    parser.add_argument(
        "--input-height", type=int, default=0, help="Raw height (0 = infer from topic/type)"
    )
    parser.add_argument(
        "--max-depth-m",
        type=float,
        default=2.5,
        help="Depth clip/normalize in meters (default: 2.5)",
    )
    parser.add_argument(
        "--colormap",
        choices=("gray", "jet", "turbo", "inferno"),
        default="jet",
        help="Color map (default: jet; use gray for grayscale)",
    )
    parser.add_argument(
        "--no-blur",
        action="store_true",
        help="Disable Gaussian blur on policy path",
    )
    parser.add_argument(
        "-o", "--output", type=str, default="", help="Save comparison MP4"
    )
    parser.add_argument(
        "--duration", type=float, default=0.0, help="Stop after N seconds"
    )
    parser.add_argument("--fps", type=float, default=30.0, help="Output FPS")
    parser.add_argument(
        "--panel-height",
        type=int,
        default=360,
        help="Panel height in pixels (default: 360)",
    )
    parser.add_argument(
        "--policy-scale",
        type=int,
        default=10,
        help="Upscale 64x36 before fitting panel height (default: 10)",
    )
    parser.add_argument(
        "--no-colorbar",
        action="store_true",
        help="Hide depth colorbar on the right",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not open live window",
    )
    parser.add_argument("--type", choices=("auto", "float64", "uint16"), default="auto", help="DDS payload type (default: auto)")
    return parser.parse_args()


def _valid_ratio_mm(depth_mm: np.ndarray) -> float:
    valid = np.isfinite(depth_mm) & (depth_mm > 0.0)
    return float(valid.mean()) if depth_mm.size else 0.0


def main() -> int:
    args = _parse_args()
    _preload_cyclonedds()

    args.payload_type = resolve_type(args.topic, args.type)
    if args.input_width <= 0 or args.input_height <= 0: args.input_width, args.input_height = default_shape(args.topic, args.payload_type)
    expected_len = args.input_width * args.input_height
    participant = DomainParticipant(0)
    dds_type = Float64Array if args.payload_type == "float64" else UInt16Array
    reader = DataReader(participant, Topic(participant, args.topic, dds_type))

    show = not args.no_show
    print(f"subscribed to {args.topic}")
    print(
        f"jet compare: raw {args.input_width}x{args.input_height} mm | "
        f"policy 64x36 | colormap={args.colormap} | max_depth={args.max_depth_m}m"
    )
    if args.output:
        print(f"recording: {args.output}")
    if show:
        print("live preview: press 'q' or Ctrl+C to quit")

    writer: cv2.VideoWriter | None = None
    frame_idx = 0
    start = time.time()
    last_frame_time = start

    try:
        while True:
            samples = reader.take(N=20)
            got_frame = False
            for sample in samples:
                if sample is None:
                    continue
                data = np.asarray(sample.data, dtype=np.float64)
                if data.size != expected_len:
                    print(
                        f"warn: len={data.size}, expected {expected_len}",
                        file=sys.stderr,
                    )
                    continue

                if args.payload_type == "float64":
                    policy = np.clip(data.reshape(36, 64), 0.0, 1.0).astype(np.float32)
                    valid_policy = float(np.isfinite(policy).mean())
                    depth_mm = cv2.resize(policy * args.max_depth_m * 1000.0, (args.input_width, args.input_height), interpolation=cv2.INTER_NEAREST)
                else:
                    depth_mm = data.reshape(args.input_height, args.input_width)
                    policy, valid_policy= process_depth_mm_frame(
                    depth_mm,
                    input_width=args.input_width,
                    input_height=args.input_height,
                    max_depth_m=args.max_depth_m,
                    gaussian_blur=not args.no_blur,
                    )
                if policy is None:
                    print("warn: dropped depth frame with no valid pixels", file=sys.stderr)
                    continue
                valid_raw = _valid_ratio_mm(depth_mm)

                now = time.time()
                hz = 1.0 / max(now - last_frame_time, 1e-6)
                last_frame_time = now
                frame_idx += 1
                got_frame = True

                image = compose_jet_compare_frame(
                    depth_mm,
                    policy,
                    max_depth_m=args.max_depth_m,
                    colormap=args.colormap,
                    policy_scale=args.policy_scale,
                    panel_height=args.panel_height,
                    show_colorbar=not args.no_colorbar,
                    frame_idx=frame_idx,
                    valid_raw=valid_raw,
                    valid_policy=valid_policy,
                    hz=hz,
                )

                if writer is None and args.output:
                    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                    writer = cv2.VideoWriter(
                        args.output,
                        fourcc,
                        args.fps,
                        (image.shape[1], image.shape[0]),
                    )
                    if not writer.isOpened():
                        print(
                            f"error: cannot open video writer: {args.output}",
                            file=sys.stderr,
                        )
                        return 1

                if writer is not None:
                    writer.write(image)

                if show:
                    cv2.imshow("depth_jet_compare", image)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        return 0

            if args.duration > 0 and time.time() - start >= args.duration:
                break
            if not got_frame:
                time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        if writer is not None:
            writer.release()
        if show:
            cv2.destroyAllWindows()

    elapsed = time.time() - start
    avg_hz = frame_idx / elapsed if elapsed > 0 else 0.0
    print(f"done: frames={frame_idx}, elapsed={elapsed:.1f}s, avg={avg_hz:.1f} Hz")
    if args.output:
        print(f"saved: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
