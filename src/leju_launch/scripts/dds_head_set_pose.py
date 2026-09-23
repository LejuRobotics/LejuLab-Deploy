#!/usr/bin/env python3
"""Publish head yaw/pitch targets on /rt/head_trajectory via CycloneDDS.

Roban S17 convention (matches controller_manager + Foxglove layout):
  q[0] = head_yaw   (zhead_1_joint, rad)
  q[1] = head_pitch (zhead_2_joint, rad; positive = look down on S17)

Typical use before / while depth_walk is active:
  bash src/leju_launch/scripts/dds_head_set_pose.sh --depth-walk
"""

from __future__ import annotations

import argparse
import ctypes
import math
import os
import sys
import time
from pathlib import Path

from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import DataWriter
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.util import duration

from dds_leju_types import JointState, JointTrajectoryPoint

HEAD_TRAJECTORY_TOPIC = "/rt/head_trajectory"
JOINT_STATE_TOPIC = "/rt/joint_state"

# Roban S17 motor order: legs(12) + waist(1) + arms(8) + head(2)
DEFAULT_HEAD_YAW_INDEX = 21
DEFAULT_HEAD_PITCH_INDEX = 22


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
        description="Set Roban head pose via DDS /rt/head_trajectory."
    )
    parser.add_argument(
        "--depth-walk",
        action="store_true",
        help="Preset for depth_walk: pitch down 45 deg, keep current yaw, hold until Ctrl+C",
    )
    parser.add_argument(
        "--pitch-deg",
        type=float,
        default=None,
        help="Target head pitch in degrees (positive = look down on S17, default: 45 with --depth-walk else 0)",
    )
    parser.add_argument(
        "--yaw-deg",
        type=float,
        default=None,
        help="Target head yaw in degrees (default: keep current yaw from /rt/joint_state)",
    )
    parser.add_argument(
        "--move-duration",
        type=float,
        default=1.0,
        help="Seconds to linearly interpolate from current pose to target (default: 1.0)",
    )
    parser.add_argument(
        "--hold-duration",
        type=float,
        default=0.0,
        help="Seconds to keep publishing after move (0 = until Ctrl+C, default: 0)",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=50.0,
        help="Publish rate in Hz (default: 50)",
    )
    parser.add_argument(
        "--head-yaw-index",
        type=int,
        default=DEFAULT_HEAD_YAW_INDEX,
        help=f"Index of head yaw in /rt/joint_state.q (default: {DEFAULT_HEAD_YAW_INDEX})",
    )
    parser.add_argument(
        "--head-pitch-index",
        type=int,
        default=DEFAULT_HEAD_PITCH_INDEX,
        help=f"Index of head pitch in /rt/joint_state.q (default: {DEFAULT_HEAD_PITCH_INDEX})",
    )
    parser.add_argument(
        "--state-timeout",
        type=float,
        default=3.0,
        help="Seconds to wait for /rt/joint_state when yaw is not specified (default: 3.0)",
    )
    return parser.parse_args()


def _read_current_head(
    reader: DataReader,
    yaw_index: int,
    pitch_index: int,
    timeout_sec: float,
) -> tuple[float, float]:
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        sample = reader.take_one(timeout=duration(seconds=0.2))
        if sample is None:
            continue
        q = sample.q
        if len(q) <= max(yaw_index, pitch_index):
            continue
        return float(q[yaw_index]), float(q[pitch_index])
    raise TimeoutError(
        f"no /rt/joint_state sample with head indices "
        f"yaw={yaw_index}, pitch={pitch_index} within {timeout_sec:.1f}s"
    )


def _make_msg(yaw_rad: float, pitch_rad: float) -> JointTrajectoryPoint:
    now = time.time()
    sec = int(now)
    nsec = int((now - sec) * 1e9)
    return JointTrajectoryPoint(
        header_sec=sec,
        header_nanosec=nsec,
        q=[yaw_rad, pitch_rad],
        v=[0.0, 0.0],
        acc=[0.0, 0.0],
    )


def main() -> int:
    args = _parse_args()
    _preload_cyclonedds()

    if args.depth_walk:
        pitch_deg = 45.0 if args.pitch_deg is None else args.pitch_deg
        move_duration = args.move_duration if args.move_duration != 1.0 else 1.0
        hold_duration = args.hold_duration
    else:
        pitch_deg = 0.0 if args.pitch_deg is None else args.pitch_deg
        move_duration = args.move_duration
        hold_duration = args.hold_duration

    target_pitch = math.radians(pitch_deg)
    target_yaw = None if args.yaw_deg is None else math.radians(args.yaw_deg)

    participant = DomainParticipant(0)
    writer = DataWriter(
        participant,
        Topic(participant, HEAD_TRAJECTORY_TOPIC, JointTrajectoryPoint),
    )
    state_reader = DataReader(
        participant,
        Topic(participant, JOINT_STATE_TOPIC, JointState),
    )

    print(f"publisher: {HEAD_TRAJECTORY_TOPIC}")
    print(f"reading current pose from: {JOINT_STATE_TOPIC}")

    try:
        current_yaw, current_pitch = _read_current_head(
            state_reader,
            args.head_yaw_index,
            args.head_pitch_index,
            args.state_timeout,
        )
    except TimeoutError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    if target_yaw is None:
        target_yaw = current_yaw

    print(
        "current head: "
        f"yaw={math.degrees(current_yaw):.1f} deg, "
        f"pitch={math.degrees(current_pitch):.1f} deg"
    )
    print(
        "target head: "
        f"yaw={math.degrees(target_yaw):.1f} deg, "
        f"pitch={math.degrees(target_pitch):.1f} deg "
        f"(positive pitch = look down on S17)"
    )
    print(
        f"move={move_duration:.1f}s, hold="
        f"{'until Ctrl+C' if hold_duration <= 0 else f'{hold_duration:.1f}s'}, "
        f"rate={args.rate:.0f} Hz"
    )

    dt = 1.0 / max(args.rate, 1.0)
    move_duration = max(move_duration, 0.0)
    t0 = time.monotonic()

    try:
        while True:
            elapsed = time.monotonic() - t0
            if move_duration > 0.0 and elapsed < move_duration:
                phase = elapsed / move_duration
            else:
                phase = 1.0

            yaw = current_yaw + phase * (target_yaw - current_yaw)
            pitch = current_pitch + phase * (target_pitch - current_pitch)
            writer.write(_make_msg(yaw, pitch))

            if phase >= 1.0 and hold_duration > 0.0 and elapsed >= move_duration + hold_duration:
                break
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\nstopped by user")

    print(
        "done: final command "
        f"yaw={math.degrees(target_yaw):.1f} deg, "
        f"pitch={math.degrees(target_pitch):.1f} deg"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
