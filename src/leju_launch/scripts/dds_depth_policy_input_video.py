#!/usr/bin/env python3
"""Subscribe to depth DDS topic and record policy-network 64x36 input as video."""

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

from dds_leju_types import Float64Array
from depth_processor import process_depth_mm_frame

DEFAULT_TOPIC = "rt/depth_camera/frame_mm_240x424"
POLICY_W = 64
POLICY_H = 36


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
            "Subscribe rt/depth_camera/frame_mm_240x424, run the same "
            "DepthImageProcessor path as depth_walk, and preview/record 64x36 "
            "normalized depth (policy single-frame input)."
        )
    )
    parser.add_argument(
        "topic",
        nargs="?",
        default=DEFAULT_TOPIC,
        help=f"DDS topic (default: {DEFAULT_TOPIC})",
    )
    parser.add_argument(
        "--input-width",
        type=int,
        default=424,
        help="Raw DDS frame width (default: 424)",
    )
    parser.add_argument(
        "--input-height",
        type=int,
        default=240,
        help="Raw DDS frame height (default: 240)",
    )
    parser.add_argument(
        "--max-depth-m",
        type=float,
        default=2.5,
        help="Clip/normalize depth in meters (default: 2.5, matches controller)",
    )
    parser.add_argument(
        "--no-blur",
        action="store_true",
        help="Disable 3x3 Gaussian blur (real-camera path enables it by default)",
    )
    parser.add_argument(
        "--output",
        "-o",
        type=str,
        default="",
        help="Save MP4 path (records upscaled preview frames)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Stop after N seconds (0 = until Ctrl+C)",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=30.0,
        help="Output video FPS (default: 30)",
    )
    parser.add_argument(
        "--display-scale",
        type=int,
        default=10,
        help="Upscale 64x36 for display/recording (default: 10 -> 640x360)",
    )
    parser.add_argument(
        "--colormap",
        choices=("gray", "turbo", "inferno"),
        default="turbo",
        help="Visualization colormap (default: turbo)",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not open live preview window",
    )
    parser.add_argument(
        "--side-by-side",
        action="store_true",
        help="Show raw 424x240 (left) and policy 64x36 (right) in one frame",
    )
    return parser.parse_args()


def _policy_to_bgr(policy: np.ndarray, colormap: str) -> np.ndarray:
    """policy: float32 (36, 64) in [0, 1] -> BGR uint8."""
    gray = np.clip(policy * 255.0, 0.0, 255.0).astype(np.uint8)
    if colormap == "gray":
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    cmap = cv2.COLORMAP_TURBO if colormap == "turbo" else cv2.COLORMAP_INFERNO
    return cv2.applyColorMap(gray, cmap)


def _raw_mm_to_bgr(depth_mm: np.ndarray, max_depth_mm: float, colormap: str) -> np.ndarray:
    valid = np.isfinite(depth_mm) & (depth_mm > 0.0)
    gray = np.zeros(depth_mm.shape, dtype=np.uint8)
    if np.any(valid):
        norm = np.clip(depth_mm, 0.0, max_depth_mm) / max_depth_mm * 255.0
        gray[valid] = norm[valid].astype(np.uint8)
    return _policy_to_bgr(gray.astype(np.float32) / 255.0, colormap)


def _compose_display(
    depth_mm: np.ndarray,
    policy: np.ndarray,
    valid_ratio: float,
    frame_idx: int,
    hz: float,
    args: argparse.Namespace,
) -> np.ndarray:
    policy_bgr = _policy_to_bgr(policy, args.colormap)
    scale = max(1, args.display_scale)
    policy_vis = cv2.resize(
        policy_bgr,
        (POLICY_W * scale, POLICY_H * scale),
        interpolation=cv2.INTER_NEAREST,
    )

    if args.side_by_side:
        raw_bgr = _raw_mm_to_bgr(depth_mm, args.max_depth_m * 1000.0, args.colormap)
        raw_h = policy_vis.shape[0]
        raw_w = int(raw_bgr.shape[1] * (raw_h / raw_bgr.shape[0]))
        raw_vis = cv2.resize(raw_bgr, (raw_w, raw_h), interpolation=cv2.INTER_NEAREST)
        gap = 8
        canvas = np.zeros((raw_h, raw_w + gap + policy_vis.shape[1], 3), dtype=np.uint8)
        canvas[:, :raw_w] = raw_vis
        canvas[:, raw_w + gap :] = policy_vis
        cv2.putText(
            canvas,
            "raw 424x240 mm",
            (8, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            f"policy 64x36  valid={valid_ratio * 100:.1f}%",
            (raw_w + gap + 8, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        image = canvas
    else:
        image = policy_vis

    label = (
        f"#{frame_idx}  net_in={POLICY_W}x{POLICY_H}  "
        f"valid={valid_ratio * 100:.1f}%  "
        f"min={policy.min():.3f} max={policy.max():.3f} mean={policy.mean():.3f}  "
        f"~{hz:.1f}Hz"
    )
    cv2.putText(
        image,
        label,
        (8, image.shape[0] - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        image,
        label,
        (8, image.shape[0] - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 0, 0),
        1,
        cv2.LINE_AA,
    )
    return image


def main() -> int:
    args = _parse_args()
    _preload_cyclonedds()

    expected_len = args.input_width * args.input_height
    participant = DomainParticipant(0)
    reader = DataReader(participant, Topic(participant, args.topic, Float64Array))

    show = not args.no_show
    print(f"subscribed to {args.topic}")
    print(
        f"pipeline: {args.input_width}x{args.input_height} mm -> "
        f"{POLICY_W}x{POLICY_H} normalized [0,1] "
        f"(max_depth={args.max_depth_m}m, blur={'off' if args.no_blur else 'on'})"
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

                depth_mm = data.reshape(args.input_height, args.input_width)
                policy, valid_ratio = process_depth_mm_frame(
                    depth_mm,
                    input_width=args.input_width,
                    input_height=args.input_height,
                    output_width=POLICY_W,
                    output_height=POLICY_H,
                    max_depth_m=args.max_depth_m,
                    gaussian_blur=not args.no_blur,
                )

                now = time.time()
                hz = 1.0 / max(now - last_frame_time, 1e-6)
                last_frame_time = now
                frame_idx += 1
                got_frame = True

                image = _compose_display(
                    depth_mm, policy, valid_ratio, frame_idx, hz, args
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
                    cv2.imshow("depth_policy_input_64x36", image)
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
