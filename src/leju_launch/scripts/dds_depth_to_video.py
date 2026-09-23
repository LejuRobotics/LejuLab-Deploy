#!/usr/bin/env python3
"""Subscribe to depth DDS topic and preview / record as video."""

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

DEFAULT_TOPIC = "rt/depth_camera/frame_mm_240x424"
DEFAULT_WIDTH = 424
DEFAULT_HEIGHT = 240


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
            "Visualize rt/depth_camera/frame_mm_240x424 as live window and/or MP4."
        )
    )
    parser.add_argument(
        "topic",
        nargs="?",
        default=DEFAULT_TOPIC,
        help=f"DDS topic (default: {DEFAULT_TOPIC})",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=DEFAULT_WIDTH,
        help=f"Frame width in pixels (default: {DEFAULT_WIDTH})",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=DEFAULT_HEIGHT,
        help=f"Frame height in pixels (default: {DEFAULT_HEIGHT})",
    )
    parser.add_argument(
        "--max-depth-mm",
        type=float,
        default=2500.0,
        help="Depth values above this clip to max (default: 2500, same as controller)",
    )
    parser.add_argument(
        "--output",
        "-o",
        type=str,
        default="",
        help="Save MP4 to this path (e.g. depth_preview.mp4)",
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
        "--colormap",
        choices=("gray", "turbo", "inferno"),
        default="turbo",
        help="Colorization mode (default: turbo)",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not open live preview window (useful with --output on headless SSH)",
    )
    parser.add_argument(
        "--match-controller",
        action="store_true",
        help="Append 64x36 nearest-neighbor preview (what depth_walk policy sees)",
    )
    parser.add_argument(
        "--scale",
        type=int,
        default=3,
        help="Upscale factor for 64x36 inset when --match-controller (default: 3)",
    )
    return parser.parse_args()


def _depth_mm_to_u8(
    depth_mm: np.ndarray, max_depth_mm: float
) -> tuple[np.ndarray, float, float, float]:
    """Return 8-bit visualization and min/max/mean of valid (positive) depth."""
    valid = np.isfinite(depth_mm) & (depth_mm > 0.0)
    valid_ratio = float(valid.mean()) if depth_mm.size else 0.0

    vis = np.zeros(depth_mm.shape, dtype=np.uint8)
    if not np.any(valid):
        return vis, 0.0, 0.0, valid_ratio

    clipped = np.clip(depth_mm, 0.0, max_depth_mm)
    norm = (clipped / max_depth_mm * 255.0).astype(np.uint8)
    vis[valid] = norm[valid]

    vals = depth_mm[valid]
    return vis, float(vals.min()), float(vals.max()), valid_ratio


def _apply_colormap(gray_u8: np.ndarray, mode: str) -> np.ndarray:
    if mode == "gray":
        return cv2.cvtColor(gray_u8, cv2.COLOR_GRAY2BGR)
    cmap = cv2.COLORMAP_TURBO if mode == "turbo" else cv2.COLORMAP_INFERNO
    return cv2.applyColorMap(gray_u8, cmap)


def _controller_preview(depth_mm: np.ndarray, max_depth_mm: float, scale: int) -> np.ndarray:
    """Nearest resize 424x240 -> 64x36, same semantics as DepthImageProcessor."""
    h, w = depth_mm.shape
    out_h, out_w = 36, 64
    ys = np.minimum(out_h - 1, (np.arange(out_h) * h // out_h))
    xs = np.minimum(out_w - 1, (np.arange(out_w) * w // out_w))
    grid_y, grid_x = np.meshgrid(ys, xs, indexing="ij")
    sampled_mm = depth_mm[grid_y, grid_x]
    gray, _, _, _ = _depth_mm_to_u8(sampled_mm, max_depth_mm)
    color = _apply_colormap(gray, "turbo")
    if scale > 1:
        color = cv2.resize(
            color,
            (out_w * scale, out_h * scale),
            interpolation=cv2.INTER_NEAREST,
        )
    return color


def _compose_frame(
    depth_mm: np.ndarray,
    args: argparse.Namespace,
    frame_idx: int,
    hz: float,
) -> np.ndarray:
    gray, d_min, d_max, valid_ratio = _depth_mm_to_u8(depth_mm, args.max_depth_mm)
    main = _apply_colormap(gray, args.colormap)

    label = (
        f"#{frame_idx}  {args.width}x{args.height} mm  "
        f"valid={valid_ratio * 100:.1f}%  "
        f"depth=[{d_min:.0f},{d_max:.0f}]mm  ~{hz:.1f}Hz"
    )
    cv2.putText(
        main,
        label,
        (8, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        main,
        label,
        (8, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 0, 0),
        1,
        cv2.LINE_AA,
    )

    if not args.match_controller:
        return main

    inset = _controller_preview(depth_mm, args.max_depth_mm, args.scale)
    ih, iw = inset.shape[:2]
    margin = 8
    x0 = main.shape[1] - iw - margin
    y0 = main.shape[0] - ih - margin
    cv2.rectangle(
        main,
        (x0 - 2, y0 - 18),
        (x0 + iw + 2, y0 + ih + 2),
        (0, 0, 0),
        -1,
    )
    cv2.putText(
        main,
        "policy 64x36",
        (x0, y0 - 4),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    main[y0 : y0 + ih, x0 : x0 + iw] = inset
    return main


def main() -> int:
    args = _parse_args()
    _preload_cyclonedds()

    expected_len = args.width * args.height
    participant = DomainParticipant(0)
    reader = DataReader(participant, Topic(participant, args.topic, Float64Array))

    show = not args.no_show
    if show and not args.output:
        print("Live preview: press 'q' or Ctrl+C to quit")
    if args.output:
        print(f"Recording to: {args.output}")
    print(
        f"subscribed to {args.topic}  "
        f"({args.width}x{args.height}, max_depth={args.max_depth_mm:.0f} mm)"
    )

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

                depth_mm = data.reshape(args.height, args.width)
                now = time.time()
                hz = 1.0 / max(now - last_frame_time, 1e-6)
                last_frame_time = now
                frame_idx += 1
                got_frame = True

                image = _compose_frame(depth_mm, args, frame_idx, hz)

                if writer is None and args.output:
                    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                    writer = cv2.VideoWriter(
                        args.output,
                        fourcc,
                        args.fps,
                        (image.shape[1], image.shape[0]),
                    )
                    if not writer.isOpened():
                        print(f"error: cannot open video writer: {args.output}", file=sys.stderr)
                        return 1

                if writer is not None:
                    writer.write(image)

                if show:
                    cv2.imshow("depth_dds", image)
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
