#!/usr/bin/env python3
"""Echo one or more registered Leju DDS topics without compiling a helper."""

import argparse
import dataclasses
import os
import platform
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional, Type

# cyclonedds 绑定是源码编译版，import 时就要能找到 libddsc.so。
# 必须在任何 from cyclonedds... import 之前设置 CYCLONEDDS_HOME。
def _auto_set_cyclonedds_home() -> None:
    if os.environ.get("CYCLONEDDS_HOME"):
        return
    machine = platform.machine().lower()
    arch_dir = "aarch64" if machine in ("aarch64", "arm64") else (
        "x86_64" if machine in ("x86_64", "amd64") else None)
    if arch_dir is None:
        return
    repo_root = Path(__file__).resolve().parent.parent
    candidates = sorted(
        (repo_root / "src/lejusdk/3rd_party" / arch_dir).glob("cyclonedds-*"),
        key=lambda p: p.name)
    c_core = [p for p in candidates if not p.name.startswith("cyclonedds-cxx")]
    if not c_core:
        return
    os.environ["CYCLONEDDS_HOME"] = str(c_core[-1])


_auto_set_cyclonedds_home()

from cyclonedds.domain import DomainParticipant
from cyclonedds.idl import IdlStruct
from cyclonedds.idl.types import (
    array,
    float32,
    float64,
    int32,
    int64,
    sequence,
    uint8,
    uint32,
)
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic


@dataclasses.dataclass
class Float64(IdlStruct, typename="leju::msgs::Float64"):
    header_sec: int32
    header_nanosec: uint32
    data: float64


@dataclasses.dataclass
class Float64Array(IdlStruct, typename="leju::msgs::Float64Array"):
    header_sec: int32
    header_nanosec: uint32
    data: sequence[float64]


@dataclasses.dataclass
class StringData(IdlStruct, typename="leju::msgs::StringData"):
    header_sec: int32
    header_nanosec: uint32
    data: str


@dataclasses.dataclass
class ImuData(IdlStruct, typename="leju::msgs::ImuData"):
    header_sec: int32
    header_nanosec: uint32
    gyro: array[float64, 3]
    acc: array[float64, 3]
    free_acc: array[float64, 3]
    quat: array[float64, 4]


@dataclasses.dataclass
class JointState(IdlStruct, typename="leju::msgs::JointState"):
    header_sec: int32
    header_nanosec: uint32
    q: sequence[float64]
    v: sequence[float64]
    vd: sequence[float64]
    tau: sequence[float64]


@dataclasses.dataclass
class JointCmd(IdlStruct, typename="leju::msgs::JointCmd"):
    header_sec: int32
    header_nanosec: uint32
    q: sequence[float64]
    v: sequence[float64]
    tau: sequence[float64]
    kp: sequence[float64]
    kd: sequence[float64]
    modes: sequence[uint8]


@dataclasses.dataclass
class HandState(IdlStruct, typename="leju::msgs::HandState"):
    header_sec: int32
    header_nanosec: uint32
    left_valid: bool
    right_valid: bool
    left_sample_age_ms: uint32
    right_sample_age_ms: uint32
    position: sequence[float64]
    velocity: sequence[float64]
    current: sequence[float64]
    state: sequence[uint8]


@dataclasses.dataclass
class Joy(IdlStruct, typename="leju::msgs::Joy"):
    header_sec: int32
    header_nanosec: uint32
    axes: sequence[float32]
    buttons: sequence[int32]


@dataclasses.dataclass
class VelocityCmd(IdlStruct, typename="leju::msgs::VelocityCmd"):
    header_sec: int32
    header_nanosec: uint32
    linear_x: float64
    linear_y: float64
    angular_z: float64


@dataclasses.dataclass
class JointTrajectoryPoint(IdlStruct, typename="leju::msgs::JointTrajectoryPoint"):
    header_sec: int32
    header_nanosec: uint32
    q: sequence[float64]
    v: sequence[float64]
    acc: sequence[float64]


@dataclasses.dataclass
class TactState(IdlStruct, typename="leju::msgs::TactState"):
    header_sec: int32
    header_nanosec: uint32
    name: str
    state: int32
    error_code: int32
    error_message: str


@dataclasses.dataclass
class QuestJoysticks(IdlStruct, typename="leju::msgs::QuestJoysticks"):
    header_sec: int32
    header_nanosec: uint32
    left_x: float32
    left_y: float32
    left_trigger: float32
    left_grip: float32
    left_first_button_pressed: bool
    left_second_button_pressed: bool
    left_first_button_touched: bool
    left_second_button_touched: bool
    right_x: float32
    right_y: float32
    right_trigger: float32
    right_grip: float32
    right_first_button_pressed: bool
    right_second_button_pressed: bool
    right_first_button_touched: bool
    right_second_button_touched: bool


@dataclasses.dataclass
class Pose(IdlStruct, typename="leju::msgs::Pose"):
    x: float32
    y: float32
    z: float32
    qx: float32
    qy: float32
    qz: float32
    qw: float32


@dataclasses.dataclass
class QuestBonePoses(IdlStruct, typename="leju::msgs::QuestBonePoses"):
    header_sec: int32
    header_nanosec: uint32
    timestamp_ms: int64
    is_high_confidence: bool
    is_hand_tracking: bool
    poses: sequence[Pose]


@dataclasses.dataclass
class AudioReceiverData(IdlStruct, typename="leju::msgs::AudioReceiverData"):
    data: sequence[uint8]


TOPIC_REGISTRY: Dict[str, Type[IdlStruct]] = {
    "/rt/joint_cmd": JointCmd,
    "/rt/hand_cmd": Float64Array,
    "/rt/hand_state": HandState,
    "/rt/motor_cmd": JointCmd,
    "/rt/joint_state": JointState,
    "/rt/motor_state": JointState,
    "/rt/imu_state": ImuData,
    "/rt/joy": Joy,
    "/rt/hardware/stop": StringData,
    "/rt/hardware/state": StringData,
    "/rt/cmd_vel": VelocityCmd,
    "/rt/vr/cmd_vel": VelocityCmd,
    "/rt/posture_height_cmd": Float64,
    "/rt/arm_trajectory": JointTrajectoryPoint,
    "/rt/head_trajectory": JointTrajectoryPoint,
    "/rt/waist_trajectory": JointTrajectoryPoint,
    "/rt/quest/bone_poses": QuestBonePoses,
    "/rt/quest/joysticks": QuestJoysticks,
    "/rt/teleop/reload_config": StringData,
    "/rt/teleop/reload_controllers": StringData,
    "/rt/micphone_data": AudioReceiverData,
    "/rt/audio_play": AudioReceiverData,
    "/rt/audio_stop": StringData,
    "/rt/audio_play_file": StringData,
    "/rt/tact_command": StringData,
    "/rt/tact_state": TactState,
    "/rt/transport_mode_command": StringData,
    "/rt/transport_mode_state": Float64,
    "/rt/fall_stand_command": StringData,
    "/rt/fall_stand_state": Float64,
    "/monitor/system_info/cpu_usage": Float64Array,
    "/monitor/system_info/cpu_temperature": Float64Array,
    "/monitor/system_info/cpu_frequency": Float64Array,
    "/monitor/system_info/memory": Float64Array,
    "/monitor/system_info/rl_cpu_core": Float64Array,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Echo registered Leju DDS topics (similar to rostopic echo)."
    )
    parser.add_argument("topics", nargs="*", help="one or more DDS topic names")
    parser.add_argument("--list", action="store_true", help="list registered topics")
    parser.add_argument(
        "--max-array", type=int, default=24,
        help="maximum displayed sequence/array elements; 0 means unlimited (default: 24)",
    )
    parser.add_argument(
        "--rate", type=float, default=0.0,
        help="maximum display rate per topic in Hz; 0 prints every sample",
    )
    parser.add_argument(
        "--once", action="store_true",
        help="exit after receiving at least one sample from every selected topic",
    )
    parser.add_argument(
        "--field", help="print only this top-level field (for example data or q)",
    )
    parser.add_argument("--domain", type=int, default=0, help="DDS domain ID (default: 0)")
    parser.add_argument(
        "--dds-config",
        help="CycloneDDS XML path; defaults to the repository non-SHM listener config",
    )
    return parser.parse_args()


def configure_dds(config_arg: Optional[str]) -> str:
    repo_root = Path(__file__).resolve().parent.parent
    config = Path(config_arg).expanduser() if config_arg else (
        repo_root / "src/leju_launch/config/cyclonedds.xml"
    )
    if not config.is_file():
        raise FileNotFoundError(f"CycloneDDS config not found: {config}")
    uri = f"file://{config.resolve()}"
    # The PyPI CycloneDDS wheel does not support this project's iceoryx
    # <SharedMemory> extension, so the echo process deliberately uses UDP.
    os.environ["CYCLONEDDS_URI"] = uri
    if os.environ.get("CYCLONEDDS_HOME"):
        print(f"CYCLONEDDS_HOME={os.environ['CYCLONEDDS_HOME']}", flush=True)
    return uri


def trim_value(value: Any, max_array: int) -> Any:
    if dataclasses.is_dataclass(value):
        return {
            field.name: trim_value(getattr(value, field.name), max_array)
            for field in dataclasses.fields(value)
        }
    if isinstance(value, (list, tuple, bytes, bytearray)):
        items = list(value)
        trimmed = items if max_array == 0 else items[:max_array]
        result = [trim_value(item, max_array) for item in trimmed]
        if max_array > 0 and len(items) > max_array:
            result.append(f"... ({len(items) - max_array} more)")
        return result
    return value


def print_sample(topic_name: str, sample: Any, args: argparse.Namespace) -> None:
    timestamp = time.strftime("%H:%M:%S")
    if args.field:
        if not hasattr(sample, args.field):
            print(f"[{timestamp}] {topic_name}: field '{args.field}' not found", flush=True)
            return
        value = trim_value(getattr(sample, args.field), args.max_array)
    else:
        value = trim_value(sample, args.max_array)
    print(f"[{timestamp}] {topic_name} {value}", flush=True)


def main() -> int:
    args = parse_args()

    if args.list:
        for name, message_type in sorted(TOPIC_REGISTRY.items()):
            print(f"{name:<45} {message_type.__name__}")
        return 0

    if not args.topics:
        print("error: provide at least one topic, or use --list", file=sys.stderr)
        return 2

    unknown = [name for name in args.topics if name not in TOPIC_REGISTRY]
    if unknown:
        for name in unknown:
            print(f"error: unregistered topic: {name}", file=sys.stderr)
        print("Use --list to show supported topics.", file=sys.stderr)
        return 2

    if args.max_array < 0 or args.rate < 0:
        print("error: --max-array and --rate must be non-negative", file=sys.stderr)
        return 2

    try:
        dds_uri = configure_dds(args.dds_config)
    except FileNotFoundError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    participant = DomainParticipant(args.domain)
    readers: Dict[str, DataReader] = {}
    for name in args.topics:
        message_type = TOPIC_REGISTRY[name]
        readers[name] = DataReader(participant, Topic(participant, name, message_type))

    print(f"DDS config: {dds_uri}")
    for name in args.topics:
        print(f"subscribed: {name} ({TOPIC_REGISTRY[name].__name__})")
    print("Ctrl+C to stop", flush=True)

    received = {name: False for name in args.topics}
    last_print = {name: 0.0 for name in args.topics}
    min_interval = 1.0 / args.rate if args.rate > 0 else 0.0
    quiet_until = time.monotonic() + 3.0  # 3s 内无数据则提示一次
    warned_quiet = False

    try:
        while True:
            had_samples = False
            for name, reader in readers.items():
                samples = reader.take(N=100)
                if samples:
                    had_samples = True
                for sample in samples:
                    if sample is None:
                        continue
                    received[name] = True
                    now = time.monotonic()
                    if min_interval == 0.0 or now - last_print[name] >= min_interval:
                        print_sample(name, sample, args)
                        last_print[name] = now
            if args.once and all(received.values()):
                break
            if not had_samples:
                now = time.monotonic()
                if now >= quiet_until and not warned_quiet:
                    warned_quiet = True
                    print(
                        "warning: no samples received within 3s - check that the "
                        "publisher is running and that --dds-config matches its "
                        "domain (e.g. /etc/cyclonedds/cyclonedds_shm.xml)",
                        file=sys.stderr, flush=True)
                time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
