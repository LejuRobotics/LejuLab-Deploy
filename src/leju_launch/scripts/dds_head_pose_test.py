#!/usr/bin/env python3
"""头部指令精度测试：下发 yaw/pitch 目标，比对 /rt/joint_state 实测角度误差。

默认用例（S17，全部在 URDF 行程内）：
    yaw +60deg, yaw -60deg, pitch +30deg, pitch -18deg

S17 / Roban 关节排布：腿(12) + 腰(1) + 臂(8) + 头(2)
    q[21] = head_yaw   (zhead_1_joint, rad)
    q[22] = head_pitch (zhead_2_joint, rad；S17 上正值 = 低头)

用法：
    bash src/leju_launch/scripts/dds_head_pose_test.sh
    bash src/leju_launch/scripts/dds_head_pose_test.sh --cases 'yaw:+60,pitch:-18'

注意：整个测试期间必须以 >2Hz 持续发布。/rt/head_trajectory 超过 0.5s 无新消息，
控制器会判定 VR 掉线并把头部插值回手柄态，实测值将不再反映指令。
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from pathlib import Path

HEAD_TRAJECTORY_TOPIC = "/rt/head_trajectory"
JOINT_STATE_TOPIC = "/rt/joint_state"

# Roban S17 motor order: legs(12) + waist(1) + arms(8) + head(2)
DEFAULT_HEAD_YAW_INDEX = 21
DEFAULT_HEAD_PITCH_INDEX = 22

AXIS_NAMES = ("yaw", "pitch")


def _preload_cyclonedds() -> None:
    """按 .sh 包装脚本导出的 CYCLONEDDS_LIB_DIR 预加载 libddsc。"""
    import ctypes
    import os

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
        description="Measure head yaw/pitch tracking error via DDS.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--cases",
        default="yaw:+45,yaw:-45,pitch:+30,pitch:-5",
        help="逗号分隔的用例，格式 axis:deg（axis 为 yaw 或 pitch）",
    )
    parser.add_argument("--move-duration", type=float, default=1.5,
                        help="每个用例从起点线性插值到目标的时间 [s]")
    parser.add_argument("--settle-duration", type=float, default=3.0,
                        help="到位后继续保持目标的时长 [s]")
    parser.add_argument("--avg-window", type=float, default=1.0,
                        help="取保持阶段最后这段时间的实测均值做判定 [s]")
    parser.add_argument("--rate", type=float, default=100.0,
                        help="发布频率 [Hz]（必须 >2，推荐 >=50）")
    parser.add_argument("--tolerance-deg", type=float, default=5.0,
                        help="判定阈值：|误差| 超过即 FAIL [deg]")
    parser.set_defaults(reset_between=True)
    parser.add_argument("--reset-between", dest="reset_between", action="store_true",
                        help="用例之间先回中位再测下一条（默认开启）")
    parser.add_argument("--no-reset-between", dest="reset_between",
                        action="store_false", default=argparse.SUPPRESS,
                        help="用例之间不回中位（连续扫）")
    parser.add_argument("--reset-duration", type=float, default=1.0,
                        help="回中位插值时长 [s]")
    parser.add_argument("--reset-settle", type=float, default=1.0,
                        help="回中位后等待稳定时长 [s]")
    parser.add_argument("--head-yaw-index", type=int, default=DEFAULT_HEAD_YAW_INDEX,
                        help="head yaw 在 /rt/joint_state.q 中的下标")
    parser.add_argument("--head-pitch-index", type=int, default=DEFAULT_HEAD_PITCH_INDEX,
                        help="head pitch 在 /rt/joint_state.q 中的下标")
    parser.add_argument("--state-timeout", type=float, default=5.0,
                        help="等待 /rt/joint_state 首个样本的超时 [s]")
    parser.add_argument("--json", type=Path, default=None,
                        help="将结果写入 JSON 文件")
    return parser.parse_args()


def _parse_cases(spec: str) -> list[dict]:
    cases = []
    for raw in spec.split(","):
        raw = raw.strip()
        if not raw:
            continue
        if ":" not in raw:
            raise ValueError(f"用例格式应为 axis:deg，收到 {raw!r}")
        axis, _, value = raw.partition(":")
        axis = axis.strip().lower()
        if axis not in AXIS_NAMES:
            raise ValueError(f"未知轴 {axis!r}（应为 yaw 或 pitch）")
        deg = float(value)
        target = [0.0, 0.0]
        target[AXIS_NAMES.index(axis)] = math.radians(deg)
        cases.append({
            "name": f"{axis}{deg:+.0f}",
            "axis": axis,
            "axis_index": AXIS_NAMES.index(axis),
            "target_deg": deg,
            "target_rad": target,
        })
    if not cases:
        raise ValueError("用例列表为空")
    return cases


class HeadIo:
    """/rt/head_trajectory 发布 + /rt/joint_state 订阅。"""

    def __init__(self, yaw_index: int, pitch_index: int):
        from cyclonedds.domain import DomainParticipant
        from cyclonedds.pub import DataWriter
        from cyclonedds.sub import DataReader
        from cyclonedds.topic import Topic

        from dds_leju_types import JointState, JointTrajectoryPoint

        self._JointTrajectoryPoint = JointTrajectoryPoint
        self.yaw_index = yaw_index
        self.pitch_index = pitch_index
        self._last_state: tuple[float, float] | None = None

        self.participant = DomainParticipant(0)
        self.writer = DataWriter(
            self.participant,
            Topic(self.participant, HEAD_TRAJECTORY_TOPIC, JointTrajectoryPoint),
        )
        self.reader = DataReader(
            self.participant,
            Topic(self.participant, JOINT_STATE_TOPIC, JointState),
        )

    def publish(self, yaw: float, pitch: float) -> bool:
        now = time.time()
        sec = int(now)
        nsec = int((now - sec) * 1e9)
        msg = self._JointTrajectoryPoint(
            header_sec=sec,
            header_nanosec=nsec,
            q=[yaw, pitch],
            v=[0.0, 0.0],
            acc=[0.0, 0.0],
        )
        return self.writer.write(msg)

    def drain_state(self) -> tuple[float, float] | None:
        """取走所有待处理样本，返回最新一个的 (yaw, pitch)。"""
        for sample in self.reader.take(N=200):
            if sample is None:
                continue
            q = sample.q
            if len(q) <= max(self.yaw_index, self.pitch_index):
                continue
            self._last_state = (float(q[self.yaw_index]), float(q[self.pitch_index]))
        return self._last_state

    def wait_for_state(self, timeout: float) -> tuple[float, float]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state = self.drain_state()
            if state is not None:
                return state
            time.sleep(0.02)
        raise TimeoutError(
            f"{timeout:.1f}s 内未收到含下标 "
            f"yaw={self.yaw_index}/pitch={self.pitch_index} 的 {JOINT_STATE_TOPIC} 样本"
        )


def _drive(head_io: HeadIo, start: list[float], target: list[float],
           duration: float, rate: float, samples: list, sample_state: bool) -> None:
    """从 start 线性插值到 target 并持续发布；可顺带采集实测值。"""
    dt = 1.0 / max(rate, 1.0)
    t0 = time.monotonic()
    n_ticks = max(int(duration / dt), 0)
    for tick in range(n_ticks + 1):
        phase = 1.0 if n_ticks == 0 else min(tick / n_ticks, 1.0)
        yaw = start[0] + phase * (target[0] - start[0])
        pitch = start[1] + phase * (target[1] - start[1])
        head_io.publish(yaw, pitch)
        # 即使不采集也要清空队列，否则保持阶段会读到带旧时间戳的积压样本
        state = head_io.drain_state()
        if sample_state and state is not None:
            samples.append((time.monotonic(), state[0], state[1]))
        _sleep_until(t0 + (tick + 1) * dt)


def _hold(head_io: HeadIo, target: list[float], duration: float, rate: float,
          samples: list) -> None:
    """持续发布固定目标并采集实测值。"""
    dt = 1.0 / max(rate, 1.0)
    t0 = time.monotonic()
    n_ticks = max(int(duration / dt), 0)
    for tick in range(n_ticks + 1):
        head_io.publish(target[0], target[1])
        state = head_io.drain_state()
        if state is not None:
            samples.append((time.monotonic(), state[0], state[1]))
        _sleep_until(t0 + (tick + 1) * dt)


def _sleep_until(deadline: float) -> None:
    remaining = deadline - time.monotonic()
    if remaining > 0:
        time.sleep(remaining)


def _window_stats(samples: list, window: float, t_end: float) -> dict:
    """对 [t_end-window, t_end] 内的样本求均值/标准差。"""
    cutoff = t_end - window
    sel = [s for s in samples if s[0] >= cutoff]
    if not sel:
        sel = samples[-10:]
    if not sel:
        return {"yaw_mean": float("nan"), "pitch_mean": float("nan"),
                "yaw_std": float("nan"), "pitch_std": float("nan"), "n": 0}
    yaws = [s[1] for s in sel]
    pitches = [s[2] for s in sel]
    return {
        "yaw_mean": statistics.fmean(yaws),
        "pitch_mean": statistics.fmean(pitches),
        "yaw_std": statistics.pstdev(yaws) if len(yaws) > 1 else 0.0,
        "pitch_std": statistics.pstdev(pitches) if len(pitches) > 1 else 0.0,
        "n": len(sel),
    }


def _run_case(head_io: HeadIo, case: dict, args: argparse.Namespace) -> dict:
    target = case["target_rad"]
    start_state = head_io.wait_for_state(args.state_timeout)
    start = [start_state[0], start_state[1]]

    samples: list = []
    _drive(head_io, start, target, args.move_duration, args.rate, samples,
           sample_state=False)
    samples.clear()
    _hold(head_io, target, args.settle_duration, args.rate, samples)
    stats = _window_stats(samples, args.avg_window, time.monotonic())

    actual = [stats["yaw_mean"], stats["pitch_mean"]]
    std = [stats["yaw_std"], stats["pitch_std"]]
    err = [actual[0] - target[0], actual[1] - target[1]]

    # 无响应判定：驱动轴几乎没动
    idx = case["axis_index"]
    moved = actual[idx] - start[idx]
    commanded = target[idx] - start[idx]
    no_response = abs(commanded) > math.radians(5.0) and abs(moved) < 0.05 * abs(commanded)
    # 符号反向判定：动了，但方向与指令相反且幅度可观（多为 yaw/pitch 约定写反）
    sign_suspect = (abs(commanded) > math.radians(5.0)
                    and commanded * moved < 0
                    and abs(moved) > 0.5 * abs(commanded))

    worst = max(abs(e) for e in err)
    result = {
        "name": case["name"],
        "axis": case["axis"],
        "axis_index": idx,
        "start_deg": [math.degrees(v) for v in start],
        "target_deg": [math.degrees(v) for v in target],
        "actual_deg": [math.degrees(v) for v in actual],
        "error_deg": [math.degrees(v) for v in err],
        "std_deg": [math.degrees(v) for v in std],
        "samples": stats["n"],
        "no_response": no_response,
        "sign_suspect": sign_suspect,
        "pass": worst <= args.tolerance_deg and not no_response and not sign_suspect,
    }
    return result


def _print_table(results: list, tolerance_deg: float) -> None:
    header = (f"{'case':<10} {'axis':<6} {'expect':>9} {'actual':>9} "
              f"{'error':>9} {'|err|':>8} {'std':>7}  verdict")
    print()
    print(header)
    print("-" * len(header))
    for res in results:
        for i, axis in enumerate(AXIS_NAMES):
            err = res["error_deg"][i]
            verdict = "PASS" if abs(err) <= tolerance_deg else "FAIL"
            if i == res["axis_index"]:
                if res["no_response"]:
                    verdict = "NO_RESP"
                elif res["sign_suspect"]:
                    verdict = "SIGN!"
            print(f"{res['name']:<10} {axis:<6} "
                  f"{res['target_deg'][i]:>8.2f}° {res['actual_deg'][i]:>8.2f}° "
                  f"{err:>+8.2f}° {abs(err):>7.2f}° {res['std_deg'][i]:>6.3f}°  {verdict}")
        print("-" * len(header))

    failed = [r["name"] for r in results if not r["pass"]]
    print()
    if failed:
        print(f"结果：{len(results) - len(failed)}/{len(results)} 通过，"
              f"阈值 ±{tolerance_deg:.1f}°；失败: {', '.join(failed)}")
        for res in results:
            if res["pass"]:
                continue
            if res["no_response"]:
                print(f"  - {res['name']}: 头部未跟随指令（检查控制器是否在 Running、"
                      f"指令是否持续 >2Hz、话题是否为 /rt/head_trajectory）")
            elif res["sign_suspect"]:
                print(f"  - {res['name']}: 实测方向与指令相反，疑似 yaw/pitch 约定写反 "
                      f"（S17 上 q[0]=yaw 绕 Z、q[1]=pitch 绕 Y）")
    else:
        print(f"结果：全部 {len(results)} 条通过（阈值 ±{tolerance_deg:.1f}°）")


def main() -> int:
    args = _parse_args()
    try:
        cases = _parse_cases(args.cases)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    if args.rate < 2.5:
        print("error: --rate 必须 >2.5Hz，否则头部指令会因 0.5s 超时被判为掉线",
              file=sys.stderr)
        return 2

    try:
        _preload_cyclonedds()
        from dds_leju_types import JointTrajectoryPoint  # noqa: F401  仅用于探活
    except ImportError as exc:
        print(f"error: 依赖缺失 ({exc})。请先 pip3 install cyclonedds==0.10.2",
              file=sys.stderr)
        return 2

    head_io = HeadIo(args.head_yaw_index, args.head_pitch_index)
    print(f"publish → {HEAD_TRAJECTORY_TOPIC}   read ← {JOINT_STATE_TOPIC}"
          f" (yaw#{args.head_yaw_index}, pitch#{args.head_pitch_index})")
    print(f"用例 {len(cases)} 条，{args.rate:.0f}Hz，"
          f"move={args.move_duration:.1f}s settle={args.settle_duration:.1f}s "
          f"avg-window={args.avg_window:.1f}s")

    try:
        head_io.wait_for_state(args.state_timeout)
    except TimeoutError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    neutral = [0.0, 0.0]
    results: list = []
    try:
        for idx, case in enumerate(cases):
            print(f"\n[{idx + 1}/{len(cases)}] {case['name']} ...", flush=True)
            results.append(_run_case(head_io, case, args))
            if args.reset_between and idx + 1 < len(cases):
                current = [results[-1]["actual_deg"][0] * math.pi / 180.0,
                           results[-1]["actual_deg"][1] * math.pi / 180.0]
                _drive(head_io, current, neutral, args.reset_duration, args.rate,
                       [], sample_state=False)
                _hold(head_io, neutral, args.reset_settle, args.rate, [])
    except KeyboardInterrupt:
        print("\n中断：已完成的用例仍会汇总", file=sys.stderr)

    if not results:
        return 1

    _print_table(results, args.tolerance_deg)

    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "timestamp": time.time(),
            "topic": HEAD_TRAJECTORY_TOPIC,
            "state_topic": JOINT_STATE_TOPIC,
            "yaw_index": args.head_yaw_index,
            "pitch_index": args.head_pitch_index,
            "tolerance_deg": args.tolerance_deg,
            "results": results,
        }
        args.json.write_text(json.dumps(payload, indent=2, ensure_ascii=False))
        print(f"\nJSON 已写入 {args.json}")

    return 0 if all(r["pass"] for r in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
