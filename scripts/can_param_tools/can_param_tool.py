#!/usr/bin/env python3
"""CAN/CANFD 电机参数调试工具（含 OTA 升级）。

用法: python3 scripts/can_param_tool.py [--scan-timeout MS] [--scan-id-max N]
                                         [--io-timeout MS] [--debug] [--no-log]

主菜单 [8] OTA 升级:
  - 仅接受 .bin 固件；.hex 请先用 'arm-none-eabi-objcopy -I ihex -O binary IN.hex OUT.bin' 转换
  - 失败的电机会留在 boot loader，可在 [8] 中用 'm <id>' 跳过扫描重试
设计文档:
  - 参数读写: docs/plans/2026-04-15-can-param-tool-design.md
  - OTA: docs/plans/2026-05-07-can-ota-design.md
"""
from __future__ import annotations

import argparse
import json
import re
import struct
import subprocess
import sys
import tempfile
import time
import traceback
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Optional


class Tier(str, Enum):
    RO = "ro"          # 只读
    SAFE = "safe"      # 普通模式可写
    EXPERT = "expert"  # 需专家模式解锁


@dataclass(frozen=True)
class ParamEntry:
    name: str
    dtype: str       # "float" | "uint32" | "hex32" | "int32"
    unit: str        # "/" 表示无单位
    tier: Tier
    desc: str = ""


# 参数表 (index 10..67)，来源:《CAN/CAN FD 修改参数说明》PDF
# RO=只读, SAFE=普通模式可写, EXPERT=专家模式才解锁
PARAM_TABLE: dict[int, ParamEntry] = {
    10: ParamEntry("Firmware Version", "hex32", "/", Tier.RO, "Firmware version number"),
    11: ParamEntry("Control Mode", "uint32", "/", Tier.SAFE, "Control Mode"),
    12: ParamEntry("Id Controller Kp", "float", "/", Tier.SAFE, "Current controller parameters"),
    13: ParamEntry("Id Controller Ki", "float", "/", Tier.SAFE, "Current controller parameters"),
    14: ParamEntry("Iq Controller Kp", "float", "/", Tier.SAFE, "Current controller parameters"),
    15: ParamEntry("Iq Controller Ki", "float", "/", Tier.SAFE, "Current controller parameters"),
    16: ParamEntry("Current Dead Zone", "float", "A", Tier.SAFE, "Current loop dead zone"),
    17: ParamEntry("Velocity Dead Zone", "float", "rad/s", Tier.SAFE, "Velocity loop dead zone"),
    18: ParamEntry("Position Dead Zone", "float", "rad", Tier.SAFE, "Position loop dead zone"),
    19: ParamEntry("Current Integral Limit", "float", "/", Tier.SAFE, "Current loop integral limit"),
    20: ParamEntry("Velocity Integral Limit", "float", "/", Tier.SAFE, "Speed loop integral limit"),
    21: ParamEntry("Torque Limit", "float", "Nm", Tier.SAFE, "Torque limit protection"),
    22: ParamEntry("Electric Angle Offset", "float", "rad", Tier.EXPERT, "严禁修改"),
    23: ParamEntry("Machine Angle Offset", "float", "rad", Tier.EXPERT, "不推荐修改"),
    24: ParamEntry("CAN COM Theta MIN", "float", "rad", Tier.SAFE, "CAN COM Theta MIN"),
    25: ParamEntry("CAN COM Theta MAX", "float", "rad", Tier.SAFE, "CAN COM Theta MAX"),
    26: ParamEntry("CAN COM Velocity MIN", "float", "rad/s", Tier.SAFE, "CAN COM Velocity MIN"),
    27: ParamEntry("CAN COM Velocity MAX", "float", "rad/s", Tier.SAFE, "CAN COM Velocity MAX"),
    28: ParamEntry("CAN COM Kp MIN", "float", "/", Tier.SAFE, "CAN COM Kp MIN"),
    29: ParamEntry("CAN COM Kp MAX", "float", "/", Tier.SAFE, "CAN COM Kp MAX"),
    30: ParamEntry("CAN COM Kd MIN", "float", "/", Tier.SAFE, "CAN COM Kd MIN"),
    31: ParamEntry("CAN COM Kd MAX", "float", "/", Tier.SAFE, "CAN COM Kd MAX"),
    32: ParamEntry("CAN COM Ki MIN", "float", "/", Tier.SAFE, "CAN COM Ki MIN"),
    33: ParamEntry("CAN COM Ki MAX", "float", "/", Tier.SAFE, "CAN COM Ki MAX"),
    34: ParamEntry("CAN COM Torque MIN", "float", "Nm", Tier.SAFE, "CAN COM Torque MIN"),
    35: ParamEntry("CAN COM Torque MAX", "float", "Nm", Tier.SAFE, "CAN COM Torque MAX"),
    36: ParamEntry("Motor ID", "uint32", "/", Tier.EXPERT, "改后工具需重新扫描"),
    37: ParamEntry("CAN COM TimeOut", "uint32", "ms", Tier.SAFE, "CAN interrupt timeout"),
    38: ParamEntry("Default Position Kp", "float", "/", Tier.SAFE, "Default Position Loop Kp"),
    39: ParamEntry("Default Position Kd", "float", "/", Tier.SAFE, "Default Position Loop Kd"),
    40: ParamEntry("Default Velocity Kp", "float", "/", Tier.SAFE, "Default Velocity Loop Kp"),
    41: ParamEntry("Default Velocity Ki", "float", "/", Tier.SAFE, "Default Velocity Loop Ki"),
    42: ParamEntry("Acceleration", "float", "rad/s^2", Tier.SAFE, "Acceleration"),
    43: ParamEntry("Torque Slop", "float", "/", Tier.SAFE, "Torque rise rate limit"),
    44: ParamEntry("NPP", "uint32", "/", Tier.EXPERT, "Number of Pole Pairs"),
    45: ParamEntry("Gear Ratio", "float", "/", Tier.EXPERT, "Gear Ratio of Reducer"),
    46: ParamEntry("Torque Constant", "float", "Nm/A", Tier.EXPERT, "Torque Constant"),
    47: ParamEntry("Rotate Dir", "uint32", "/", Tier.EXPERT, "Rotate Direction"),
    48: ParamEntry("Encoder Config", "uint32", "/", Tier.EXPERT, "Encoder Config"),
    49: ParamEntry("BUS_OV_LOCK", "float", "V", Tier.EXPERT, "Bus overvoltage lock threshold"),
    50: ParamEntry("BUS_UV_LOCK", "float", "V", Tier.EXPERT, "Bus undervoltage lock threshold"),
    51: ParamEntry("PHASE_OC_LOCK", "float", "A", Tier.EXPERT, "Over phase current lock threshold"),
    52: ParamEntry("PCB_OT_LOCK", "float", "degC", Tier.EXPERT, "Driver over-temperature lock"),
    53: ParamEntry("Stuck Current", "float", "A", Tier.EXPERT, "Stuck protect current"),
    54: ParamEntry("Stuck Velocity", "float", "rad/s", Tier.EXPERT, "Stuck protect velocity"),
    55: ParamEntry("Stuck Time", "float", "s", Tier.EXPERT, "Stuck protect time"),
    56: ParamEntry("Protect Switchs", "hex32", "/", Tier.EXPERT, "Bit0:pos bit1:vel bit2:phase OC bit3:PCB OT bit4:bus UV/OV bit5:stuck bit6:encoder bit7:CAN COM TimeOut"),
    57: ParamEntry("Encoder Gain", "uint32", "/", Tier.EXPERT, "Encoder Gain"),
    58: ParamEntry("Break Action DTC", "uint32", "/", Tier.SAFE, "Break action PWM duty cycle"),
    59: ParamEntry("Break Action Time", "uint32", "1/200s", Tier.SAFE, "Break action time"),
    60: ParamEntry("Break Hold DTC", "uint32", "/", Tier.SAFE, "Break hold PWM duty cycle"),
    61: ParamEntry("Current Filter Bandwidth", "uint32", "Hz", Tier.SAFE, "Current filter BW"),
    62: ParamEntry("Velocity Filter Bandwidth", "uint32", "Hz", Tier.SAFE, "Velocity filter BW"),
    63: ParamEntry("Position Filter Bandwidth", "uint32", "Hz", Tier.SAFE, "Position filter BW"),
    64: ParamEntry("V_calibration", "float", "/", Tier.EXPERT, "Voltage scaler of electric calibration"),
    65: ParamEntry("Dual Encoder Config", "float", "/", Tier.EXPERT, "Dual Encoder Config"),
    66: ParamEntry("MOTOR_OT_LOCK", "float", "degC", Tier.EXPERT, "Motor over-temperature lock"),
    67: ParamEntry("Can Master", "int32", "/", Tier.EXPERT, "CAN master configuration"),
}


# =========================================================================
# 协议帧编码（来源: CAN/CAN FD 参数读取/修改指令 PDF）
# =========================================================================

SAVE_FRAME = bytes([0x67, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x76])


def build_read_frame(idx: int) -> bytes:
    """读参数帧: 67 <idx> 00 00 00 00 04 76"""
    if not (0 <= idx <= 0xFF):
        raise ValueError(f"idx out of range: {idx}")
    return bytes([0x67, idx, 0x00, 0x00, 0x00, 0x00, 0x04, 0x76])


def build_write_frame(idx: int, value: float | int, dtype: str) -> bytes:
    """写参数帧: 67 <idx> <payload 4 bytes little-endian> 15 76"""
    if not (0 <= idx <= 0xFF):
        raise ValueError(f"idx out of range: {idx}")
    if dtype == "float":
        payload = struct.pack("<f", float(value))
    elif dtype in ("uint32", "hex32"):
        ival = int(value, 0) if isinstance(value, str) else int(value)
        if not (0 <= ival <= 0xFFFFFFFF):
            raise ValueError(f"uint32 out of range: {ival}")
        payload = struct.pack("<I", ival)
    elif dtype == "int32":
        ival = int(value, 0) if isinstance(value, str) else int(value)
        if not (-0x80000000 <= ival <= 0x7FFFFFFF):
            raise ValueError(f"int32 out of range: {ival}")
        payload = struct.pack("<i", ival)
    else:
        raise ValueError(f"unsupported dtype: {dtype}")
    return bytes([0x67, idx]) + payload + bytes([0x15, 0x76])


def decode_response(data: bytes, dtype: str) -> float | int:
    """从电机响应帧 (8 字节) 提取参数值。

    响应帧结构: <motor_id> <idx> <payload 4 bytes LE> <tail 2 bytes>
    """
    if len(data) != 8:
        raise ValueError(f"response must be 8 bytes, got {len(data)}")
    payload = data[2:6]
    if dtype == "float":
        return struct.unpack("<f", payload)[0]
    if dtype in ("uint32", "hex32"):
        return struct.unpack("<I", payload)[0]
    if dtype == "int32":
        return struct.unpack("<i", payload)[0]
    raise ValueError(f"unsupported dtype: {dtype}")


_CANDUMP_RE = re.compile(
    r"""
    ^\s*                              # 前导空白
    (?:\(\d+\.\d+\)\s+)?              # 可选时间戳 (timestamp)
    (?P<iface>\w+)\s+                 # 接口名
    (?P<id>[0-9A-Fa-f]+)\s+           # CAN ID
    \[(?P<dlc>\d+)\]\s+               # DLC 长度
    (?P<data>(?:[0-9A-Fa-f]{2}\s*)+)  # 数据字节
    \s*$
    """,
    re.VERBOSE,
)


def parse_candump_line(line: str) -> Optional[tuple[int, bytes]]:
    """解析 candump 一行输出，返回 (can_id, data_bytes)，解析失败返回 None。"""
    m = _CANDUMP_RE.match(line)
    if not m:
        return None
    try:
        can_id = int(m.group("id"), 16)
        data = bytes.fromhex(m.group("data").replace(" ", ""))
    except ValueError:
        return None
    return can_id, data


# =========================================================================
# ip link 输出解析
# =========================================================================

_IFACE_LINE_RE = re.compile(
    r"^\d+:\s+(?P<name>bcan\d+):\s+<(?P<flags>[^>]+)>",
    re.MULTILINE,
)


def parse_ip_link_brief(output: str) -> list[str]:
    """从 `ip link show` 输出中提取所有 UP 的 bcan* 接口名，按字母序返回。"""
    up = []
    for m in _IFACE_LINE_RE.finditer(output):
        flags = set(m.group("flags").split(","))
        if "UP" in flags and "LOWER_UP" in flags:
            up.append(m.group("name"))
    return sorted(up)


def parse_ip_link_details(output: str) -> bool:
    """判断 `ip -details link show <bus>` 输出是否说明接口是 CAN FD (有 dbitrate)。"""
    return "dbitrate" in output


# =========================================================================
# 总线发现 + 前置检查
# =========================================================================

@dataclass(frozen=True)
class BusInfo:
    name: str       # e.g. "bcan0"
    is_fd: bool     # 是否 CAN FD


def list_up_buses() -> list[BusInfo]:
    """枚举所有 UP 的 bcan* 接口，并识别 FD。"""
    try:
        out = subprocess.run(
            ["ip", "link", "show"],
            check=True, capture_output=True, text=True,
        ).stdout
    except subprocess.CalledProcessError as e:
        print(f"[error] ip link show failed: {e.stderr}")
        return []
    names = parse_ip_link_brief(out)
    result = []
    for name in names:
        try:
            d = subprocess.run(
                ["ip", "-details", "link", "show", name],
                check=True, capture_output=True, text=True,
            ).stdout
            is_fd = parse_ip_link_details(d)
        except subprocess.CalledProcessError:
            is_fd = False
        result.append(BusInfo(name=name, is_fd=is_fd))
    return result


# 可能占用 CAN 总线的进程名关键字（pgrep -f 匹配命令行）
HW_PROCESS_KEYWORDS = [
    "h12_hw_node",
    "hardware_node",
    "motorevo_test",
    "keyboard_ctrl",
]


def check_hw_processes() -> list[tuple[str, int]]:
    """检查可能占用 CAN 总线的硬件进程，返回 [(关键字, pid), ...]。"""
    hits: list[tuple[str, int]] = []
    for kw in HW_PROCESS_KEYWORDS:
        try:
            r = subprocess.run(
                ["pgrep", "-f", kw],
                capture_output=True, text=True,
            )
            if r.returncode == 0:
                for pid_str in r.stdout.strip().splitlines():
                    try:
                        hits.append((kw, int(pid_str)))
                    except ValueError:
                        pass
        except FileNotFoundError:
            break  # pgrep 不存在 → 后续 keyword 也不会有响应，直接退出
    return hits


# =========================================================================
# CAN I/O
# =========================================================================

class MotorIOError(RuntimeError):
    """CAN I/O 错误（超时、发送失败、响应异常）"""


def _format_can_frame(can_id: int, is_fd: bool, data: bytes) -> str:
    """构造 cansend 的帧字符串: '<ID>#<data>' 或 '<ID>##1<data>'"""
    sep = "##1" if is_fd else "#"
    return f"{can_id:03X}{sep}{data.hex().upper()}"


def send_and_wait(
    bus: str,
    is_fd: bool,
    motor_id: int,
    data: bytes,
    timeout_ms: int = 200,
    debug: bool = False,
) -> Optional[bytes]:
    """
    向指定总线发送一帧，等待电机响应帧（同 CAN ID，首字节 != 0x67）。

    返回响应帧的 8 字节数据；超时返回 None。
    """
    can_id = 0x600 + motor_id
    frame_str = _format_can_frame(can_id, is_fd, data)

    # 启动 candump（只抓 2 帧：自发回环 + 响应），写入临时文件
    with tempfile.NamedTemporaryFile(mode="w+", suffix=".candump", delete=False) as tmp:
        tmp_path = Path(tmp.name)

    try:
        with open(tmp_path, "w") as out:
            dump = subprocess.Popen(
                ["candump", bus, "-n", "2"],
                stdout=out, stderr=subprocess.DEVNULL,
            )
            try:
                time.sleep(0.05)  # 让 candump 先就绪
                r = subprocess.run(
                    ["cansend", bus, frame_str],
                    capture_output=True, text=True,
                )
                if r.returncode != 0:
                    raise MotorIOError(f"cansend 发送失败: {r.stderr.strip()}")

                # 等待响应
                deadline = time.monotonic() + timeout_ms / 1000.0
                response = None
                while time.monotonic() < deadline:
                    time.sleep(0.01)
                    try:
                        lines = tmp_path.read_text().splitlines()
                    except OSError:
                        lines = []
                    for line in lines:
                        parsed = parse_candump_line(line)
                        if parsed is None:
                            continue
                        rx_id, rx_data = parsed
                        if rx_id != can_id:
                            continue
                        if len(rx_data) >= 1 and rx_data[0] == 0x67:
                            continue  # 自发回环
                        response = rx_data[:8]
                        break
                    if response is not None:
                        break
                return response
            finally:
                dump.terminate()
                try:
                    dump.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    dump.kill()
                    dump.wait()  # 关键：kill 后必须 wait，避免 zombie
        # with open(...) closed the file
    finally:
        if debug:
            print(f"  [debug] tx: {frame_str}")
            try:
                print(f"  [debug] candump:\n{tmp_path.read_text()}")
            except OSError:
                pass
        tmp_path.unlink(missing_ok=True)


def scan_motors(
    buses: list[BusInfo],
    id_range: range,
    timeout_ms: int = 50,
    debug: bool = False,
) -> dict[str, list[int]]:
    """对每条总线扫描在线电机，返回 {bus_name: [motor_id, ...]}。"""
    probe = build_read_frame(12)  # 读 Id_Kp 作为探测帧
    found: dict[str, list[int]] = {}
    for bus in buses:
        online = []
        for mid in id_range:
            resp = send_and_wait(
                bus.name, bus.is_fd, mid, probe,
                timeout_ms=timeout_ms, debug=debug,
            )
            if resp is not None:
                online.append(mid)
        found[bus.name] = online
    return found


# =========================================================================
# MotorSession: 单电机会话，封装读/写/保存
# =========================================================================

# float32 roundtrip (Python float64 → pack "<f" → unpack "<f") 最多 ~1e-7 误差，
# 取 1e-5 作为回读校验容差，足够区分真实写入失败与浮点精度损失。
WRITE_READBACK_FLOAT_TOL = 1e-5


@dataclass
class MotorSession:
    bus: str
    motor_id: int
    is_fd: bool
    timeout_ms: int = 200
    debug: bool = False

    def _send(self, data: bytes) -> Optional[bytes]:
        return send_and_wait(
            self.bus, self.is_fd, self.motor_id, data,
            timeout_ms=self.timeout_ms, debug=self.debug,
        )

    def read(self, idx: int) -> float | int:
        """读指定参数，返回解码后的值。超时/异常抛 MotorIOError。"""
        if idx not in PARAM_TABLE:
            raise ValueError(f"unknown param index: {idx}")
        entry = PARAM_TABLE[idx]
        resp = self._send(build_read_frame(idx))
        if resp is None:
            raise MotorIOError(f"M{self.motor_id} 读取 idx={idx} 超时无响应")
        if len(resp) != 8:
            raise MotorIOError(f"响应帧长度异常: {resp.hex()}")
        return decode_response(resp, entry.dtype)

    def write(self, idx: int, value: float | int) -> tuple[bool, float | int]:
        """
        写 + 回读校验。返回 (ok, readback_value)。
        float 容差 1e-5，uint32 按位相等。
        """
        if idx not in PARAM_TABLE:
            raise ValueError(f"unknown param index: {idx}")
        entry = PARAM_TABLE[idx]
        if entry.tier == Tier.RO:
            raise ValueError(f"param idx={idx} is read-only")
        # 发写（部分电机不回响应帧，但若 cansend 本身失败需立即抛错，
        # 否则后续回读会拿到旧值，误报成"校验失败"）。
        # send_and_wait 在 cansend 失败时已改为抛 MotorIOError，无需重复检查。
        self._send(build_write_frame(idx, value, entry.dtype))
        # 显式读一次做回读校验
        readback = self.read(idx)
        if entry.dtype == "float":
            ok = abs(float(readback) - float(value)) < WRITE_READBACK_FLOAT_TOL
        else:
            expected = int(value, 0) if isinstance(value, str) else int(value)
            ok = int(readback) == expected
        return ok, readback

    def save(self) -> None:
        """发 Flash 保存帧，协议无响应。"""
        self._send(SAVE_FRAME)


# =========================================================================
# 操作日志
# =========================================================================

class OpLogger:
    def __init__(self, log_dir: Path, enabled: bool = True):
        self.enabled = enabled
        self._fp = None
        if enabled:
            log_dir.mkdir(parents=True, exist_ok=True)
            ts = time.strftime("%Y%m%d_%H%M%S")
            path = log_dir / f"can_param_{ts}.log"
            self._fp = open(path, "w", buffering=1)  # line buffered
            self._write_line(f"# CAN param tool log, started at {time.strftime('%Y-%m-%d %H:%M:%S')}")

    def _write_line(self, line: str) -> None:
        if self._fp is not None:
            self._fp.write(line + "\n")

    def _ts(self) -> str:
        return time.strftime("%H:%M:%S")

    def log_read(self, *, bus: str, motor_id: int, idx: int, name: str, value) -> None:
        if not self.enabled:
            return
        self._write_line(
            f"[{self._ts()}] read  bus={bus} id={motor_id} idx={idx} name={name!r} value={value}"
        )

    def log_write(self, *, bus: str, motor_id: int, idx: int, name: str,
                  old, new, readback, ok: bool) -> None:
        if not self.enabled:
            return
        result = "ok" if ok else "fail"
        self._write_line(
            f"[{self._ts()}] write bus={bus} id={motor_id} idx={idx} name={name!r} "
            f"old={old} new={new} readback={readback} result={result}"
        )

    def log_save(self, *, bus: str, motor_id: int) -> None:
        if not self.enabled:
            return
        self._write_line(
            f"[{self._ts()}] save  bus={bus} id={motor_id}"
        )

    def log_ota_begin(self, *, bus: str, motor_id: int, fw_path: str,
                      fw_size: int, sum32: int) -> None:
        if not self.enabled:
            return
        self._write_line(
            f"[{self._ts()}] ota-begin bus={bus} id={motor_id} "
            f"fw={fw_path!r} size={fw_size} sum32=0x{sum32:08X}"
        )

    def log_ota_phase(self, *, bus: str, motor_id: int, phase: str,
                      ok: bool, detail: str = "") -> None:
        if not self.enabled:
            return
        result = "ok" if ok else "fail"
        self._write_line(
            f"[{self._ts()}] ota-phase bus={bus} id={motor_id} "
            f"phase={phase} result={result} detail={detail!r}"
        )

    def log_ota_end(self, *, bus: str, motor_id: int, ok: bool,
                    fw_version: int | None = None, error: str = "") -> None:
        if not self.enabled:
            return
        result = "ok" if ok else "fail"
        ver = f"0x{fw_version:08X}" if fw_version is not None else "n/a"
        self._write_line(
            f"[{self._ts()}] ota-end   bus={bus} id={motor_id} "
            f"result={result} fw_version={ver} error={error!r}"
        )

    def close(self) -> None:
        if self._fp is not None:
            self._fp.close()
            self._fp = None


# =========================================================================
# TUI
# =========================================================================

from rich.console import Console
from rich.table import Table
from rich.prompt import Prompt, Confirm, IntPrompt
from rich.progress import Progress, BarColumn, TextColumn, TimeElapsedColumn

import can_ota

_console = Console()
_ARGS: Optional[argparse.Namespace] = None  # main() 设置，供 OTA 子流程读 io_timeout/debug


def tui_preflight() -> bool:
    """前置检查：硬件进程。返回 True 表示用户确认继续。"""
    hits = check_hw_processes()
    if not hits:
        return True
    _console.print("[yellow][warn][/yellow] 检测到以下可能占用 CAN 总线的进程:")
    for name, pid in hits:
        _console.print(f"  - {name} (pid {pid})")
    _console.print("[yellow]建议先停掉这些进程再继续，否则读参数可能看不到响应。[/yellow]")
    return Confirm.ask("[bold]仍然继续?[/bold]", default=False)


def tui_read_all_motors_all_params(
    scan_result: dict[str, list[int]],
    buses: list[BusInfo],
) -> None:
    """一次性读取所有在线电机的全部参数，导出为 JSON 并显示摘要表。

    不需要选电机——直接遍历所有 bus 上的所有在线电机，
    逐台读取全部 PARAM_TABLE 参数，结果保存为 JSON 文件。
    """
    # 构建 (bus_info, motor_id) 工作列表
    bus_map = {b.name: b for b in buses}
    work: list[tuple[BusInfo, int]] = []
    for bus_name, motor_ids in scan_result.items():
        if bus_name not in bus_map:
            continue
        for mid in sorted(motor_ids):
            work.append((bus_map[bus_name], mid))

    if not work:
        _console.print("[yellow]没有在线电机，无需导出。[/yellow]")
        return

    _console.print(f"[cyan]读取 {len(work)} 台电机全部参数 (idx 10-67)...[/cyan]")

    all_data: dict[str, dict] = {}  # key: "bcan0_M3"

    with Progress(
        TextColumn("[bold]{task.description}[/bold]"),
        BarColumn(),
        TextColumn("{task.completed}/{task.total}"),
        TimeElapsedColumn(),
        console=_console,
    ) as progress:
        task = progress.add_task("读取中", total=len(work))
        for bus, motor_id in work:
            progress.update(task, description=f"{bus.name} M{motor_id}")
            session = MotorSession(
                bus=bus.name, motor_id=motor_id, is_fd=bus.is_fd,
                timeout_ms=_ARGS.io_timeout if _ARGS else 200,
            )
            motor_data = {}
            for idx in sorted(PARAM_TABLE.keys()):
                entry = PARAM_TABLE[idx]
                try:
                    val = session.read(idx)
                    if entry.dtype == "float":
                        motor_data[_json_key(idx)] = round(float(val), 6)
                    elif entry.dtype == "hex32":
                        # Firmware Version 等：保留 0x 前缀的 hex 字符串
                        motor_data[_json_key(idx)] = f"0x{int(val):08X}"
                    else:
                        motor_data[_json_key(idx)] = int(val)
                except MotorIOError:
                    motor_data[_json_key(idx)] = None
            key = f"{bus.name}_M{motor_id}"
            all_data[key] = motor_data
            progress.advance(task)

    # 保存 JSON
    ts = time.strftime("%Y%m%d_%H%M%S")
    out_path = Path.cwd() / f"all_motors_params_{ts}.json"
    out_path.write_text(json.dumps(all_data, indent=4, ensure_ascii=False) + "\n")
    _console.print(f"[green]已导出到 {out_path}[/green] ({len(work)} 台电机, 每台 {len(PARAM_TABLE)} 个参数)")

    # 打印关键参数摘要表（只显示几项常用参数，方便快速对比）
    _print_params_summary(all_data)


def _print_params_summary(all_data: dict[str, dict]) -> None:
    """打印关键参数摘要表，方便多电机横向对比。"""
    key_indices = [10, 11, 36, 38, 39, 21, 44, 45, 46, 47]
    summary = Table(title="关键参数摘要")
    summary.add_column("Motor", justify="right")
    for idx in key_indices:
        summary.add_column(PARAM_TABLE[idx].name, justify="right")
    for motor_key, motor_data in all_data.items():
        row = [motor_key]
        for idx in key_indices:
            json_key = _json_key(idx)
            val = motor_data.get(json_key)
            if val is None:
                row.append("[dim]--[/dim]")
            else:
                row.append(_format_value(val, PARAM_TABLE[idx].dtype))
        summary.add_row(*row)
    _console.print(summary)


def tui_scan_and_select(scan_id_max: int, scan_timeout_ms: int, debug: bool
                        ) -> Optional[tuple[BusInfo, int]]:
    """扫描 + 两级菜单（先选 bus，再选 motor）。返回 (bus_info, motor_id) 或 None 退出。"""
    buses = list_up_buses()
    if not buses:
        _console.print("[red]未发现任何 UP 的 bcan* 接口。[/red]")
        return None

    _console.print(f"[cyan]扫描在线电机 (id 1-{scan_id_max}, 超时 {scan_timeout_ms}ms)...[/cyan]")
    scan_result = scan_motors(buses, range(1, scan_id_max + 1),
                              timeout_ms=scan_timeout_ms, debug=debug)

    # 打印扫描总览
    overview = Table(title="扫描结果")
    overview.add_column("序号", justify="right")
    overview.add_column("总线")
    overview.add_column("类型")
    overview.add_column("在线电机 ID")
    for i, bus in enumerate(buses, 1):
        ids = scan_result.get(bus.name, [])
        overview.add_row(
            str(i),
            bus.name,
            "CANFD" if bus.is_fd else "CAN",
            ", ".join(f"M{x}" for x in ids) if ids else "[dim](无)[/dim]",
        )
    _console.print(overview)

    total_online = sum(len(ids) for ids in scan_result.values())
    has_motors = total_online > 0

    # 第一级：选总线
    while True:
        bus_choices = [str(i) for i in range(1, len(buses) + 1)]
        if has_motors:
            bus_choices.append("a")
        bus_choices.append("q")
        choice = Prompt.ask(
            "选择总线 (序号, a 全部电机导出, 或 q 退出)" if has_motors
            else "选择总线 (序号, 或 q 退出)",
            choices=bus_choices,
        )
        if choice == "q":
            return None
        if choice == "a":
            tui_read_all_motors_all_params(scan_result, buses)
            continue
        bus = buses[int(choice) - 1]
        ids = scan_result.get(bus.name, [])
        if not ids:
            _console.print(f"[yellow]{bus.name} 上没有在线电机，请选其他总线。[/yellow]")
            continue

        # 第二级：选电机（展示 Motor ID + CAN ID，直接输入 Motor ID）
        motor_table = Table(title=f"{bus.name} 上的在线电机")
        motor_table.add_column("Motor ID", justify="right")
        motor_table.add_column("CAN ID")
        for mid in ids:
            motor_table.add_row(str(mid), f"0x{0x600 + mid:03X}")
        _console.print(motor_table)

        mid_choices = [str(m) for m in ids] + ["b"]
        picked = Prompt.ask(
            "输入 Motor ID (或 b 返回)",
            choices=mid_choices,
        )
        if picked == "b":
            continue
        return bus, int(picked)


TIER_STYLE = {
    Tier.RO:     "dim",
    Tier.SAFE:   "green",
    Tier.EXPERT: "red",
}


def _print_header(session: MotorSession, expert: bool) -> None:
    mode = "[red]专家[/red]" if expert else "[green]普通[/green]"
    _console.print(
        f"\n[cyan]===================================[/cyan]\n"
        f"  当前: [bold]{session.bus}[/bold] / [bold]M{session.motor_id}[/bold] "
        f"({'CANFD' if session.is_fd else 'CAN'})  模式: {mode}\n"
        f"[cyan]===================================[/cyan]"
    )


def _format_value(value, dtype: str) -> str:
    if dtype == "float":
        return f"{value:.6g}"
    if dtype == "hex32":
        if isinstance(value, str):
            # 来自 bulk JSON 缓存，已经是 "0xXXXXXXXX" 格式
            return value
        return f"0x{int(value):08X}"
    return str(int(value))


def tui_read_single(session: MotorSession, logger: OpLogger) -> None:
    idx = IntPrompt.ask("输入参数 index (10-67)")
    if idx not in PARAM_TABLE:
        _console.print(f"[red]未知 index: {idx}[/red]")
        return
    entry = PARAM_TABLE[idx]
    try:
        val = session.read(idx)
    except MotorIOError as e:
        _console.print(f"[red]{e}[/red]")
        return
    _console.print(
        f"  [{idx}] [{TIER_STYLE[entry.tier]}]{entry.name}[/] "
        f"= {_format_value(val, entry.dtype)} {entry.unit}"
    )
    logger.log_read(bus=session.bus, motor_id=session.motor_id,
                    idx=idx, name=entry.name, value=val)


def tui_read_all(session: MotorSession, logger: OpLogger) -> None:
    table = Table(title=f"M{session.motor_id} 全部参数")
    table.add_column("idx", justify="right")
    table.add_column("名称")
    table.add_column("值")
    table.add_column("单位")
    table.add_column("分级")
    for idx in sorted(PARAM_TABLE.keys()):
        entry = PARAM_TABLE[idx]
        try:
            val = session.read(idx)
            val_str = _format_value(val, entry.dtype)
            logger.log_read(bus=session.bus, motor_id=session.motor_id,
                            idx=idx, name=entry.name, value=val)
        except MotorIOError:
            val_str = "[dim]--[/dim]"
        table.add_row(
            str(idx),
            f"[{TIER_STYLE[entry.tier]}]{entry.name}[/]",
            val_str,
            entry.unit,
            entry.tier.value,
        )
    _console.print(table)


def _writable_indices(expert: bool) -> list[int]:
    """当前模式下可写的 index 列表。"""
    allowed_tiers = {Tier.SAFE}
    if expert:
        allowed_tiers.add(Tier.EXPERT)
    return [i for i, e in sorted(PARAM_TABLE.items()) if e.tier in allowed_tiers]


def _prompt_new_value(entry: ParamEntry) -> Optional[float | int]:
    """提示用户输入新值，支持 float 科学计数法和 uint32 十进制。"""
    raw = Prompt.ask(f"输入新值 ({entry.dtype})").strip()
    try:
        if entry.dtype == "float":
            return float(raw)
        if entry.dtype == "hex32":
            return int(raw, 0)
        return int(raw)
    except ValueError:
        _console.print(f"[red]无法解析 {raw!r} 为 {entry.dtype}[/red]")
        return None


def tui_write_param(session: MotorSession, expert: bool, logger: OpLogger) -> None:
    # 列出可写参数
    indices = _writable_indices(expert)
    table = Table(title=f"可写参数（{'专家' if expert else '普通'}模式）")
    table.add_column("idx", justify="right")
    table.add_column("名称")
    table.add_column("类型")
    table.add_column("分级")
    for idx in indices:
        e = PARAM_TABLE[idx]
        table.add_row(str(idx), f"[{TIER_STYLE[e.tier]}]{e.name}[/]", e.dtype, e.tier.value)
    _console.print(table)

    idx = IntPrompt.ask("选择要修改的 index")
    if idx not in indices:
        _console.print(f"[red]idx {idx} 在当前模式下不可写或不存在。[/red]")
        return
    entry = PARAM_TABLE[idx]

    # 读当前值
    try:
        old = session.read(idx)
    except MotorIOError as e:
        _console.print(f"[red]读取当前值失败: {e}[/red]")
        return
    _console.print(f"当前值: {_format_value(old, entry.dtype)}")

    new = _prompt_new_value(entry)
    if new is None:
        return

    _console.print(
        f"  [[{idx}] {entry.name}]  "
        f"[yellow]{_format_value(old, entry.dtype)} → {_format_value(new, entry.dtype)}[/yellow]"
    )
    if not Confirm.ask("[bold]确认写入?[/bold]", default=False):
        _console.print("已取消。")
        return

    try:
        ok, readback = session.write(idx, new)
    except MotorIOError as e:
        _console.print(f"[red]{e}[/red]")
        logger.log_write(bus=session.bus, motor_id=session.motor_id,
                         idx=idx, name=entry.name, old=old, new=new,
                         readback=None, ok=False)
        return

    logger.log_write(bus=session.bus, motor_id=session.motor_id,
                     idx=idx, name=entry.name, old=old, new=new,
                     readback=readback, ok=ok)

    if ok:
        _console.print(
            f"[green]✓ 写入成功[/green]  回读={_format_value(readback, entry.dtype)} "
            f"[dim](仅 RAM，保存到 Flash 请回主菜单选 [4])[/dim]"
        )
    else:
        _console.print(
            f"[red]✗ 回读校验失败[/red]: 期望 {_format_value(new, entry.dtype)}, "
            f"实际 {_format_value(readback, entry.dtype)}"
        )


def _parse_json_key(key: str) -> Optional[int]:
    """从 JSON key 提取参数 index。支持: "12", "12 (Id Controller Kp)", "12_Id_Kp" 等。"""
    m = re.match(r"^\s*(\d+)", key)
    if m:
        return int(m.group(1))
    return None


def _json_key(idx: int) -> str:
    """生成带参数名的 JSON key，如 '12 (Id Controller Kp)'。"""
    entry = PARAM_TABLE[idx]
    return f"{idx} ({entry.name})"


def tui_export_json(session: MotorSession, expert: bool) -> None:
    """导出当前电机参数为 JSON 文件，方便编辑后批量写回。"""
    indices = _writable_indices(expert)
    _console.print(f"[cyan]读取 {len(indices)} 个参数...[/cyan]")

    data = {}
    for idx in indices:
        entry = PARAM_TABLE[idx]
        try:
            val = session.read(idx)
            if entry.dtype == "float":
                data[_json_key(idx)] = round(float(val), 6)
            elif entry.dtype == "hex32":
                data[_json_key(idx)] = f"0x{int(val):08X}"
            else:
                data[_json_key(idx)] = int(val)
        except MotorIOError:
            data[_json_key(idx)] = None

    default_path = f"M{session.motor_id}_params.json"
    out_path = Prompt.ask("保存路径", default=default_path).strip()
    out = Path(out_path).expanduser()
    if out.is_dir():
        out = out / default_path
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(data, indent=4, ensure_ascii=False) + "\n")
    out_path = str(out)
    _console.print(f"[green]已导出到 {out_path}[/green] ({len(data)} 个参数)")
    _console.print("[dim]编辑后可用 [6] 批量写回。[/dim]")


def tui_batch_write_json(session: MotorSession, expert: bool, logger: OpLogger) -> None:
    """从 JSON 文件批量写入参数。key 支持 "12" 或 "12 (Id Controller Kp)" 格式。"""
    path_str = Prompt.ask("输入 JSON 文件路径").strip()
    json_path = Path(path_str).expanduser()
    if not json_path.is_file():
        _console.print(f"[red]文件不存在: {json_path}[/red]")
        return

    try:
        raw = json.loads(json_path.read_text())
    except (json.JSONDecodeError, OSError) as e:
        _console.print(f"[red]读取 JSON 失败: {e}[/red]")
        return

    if not isinstance(raw, dict):
        _console.print("[red]JSON 顶层必须是 object，如 {\"12 (Id Controller Kp)\": 0.8}[/red]")
        return

    allowed = set(_writable_indices(expert))
    items: list[tuple[int, float | int]] = []
    for key, val in raw.items():
        if val is None:
            continue
        idx = _parse_json_key(key)
        if idx is None:
            _console.print(f"[red]无法从 key 提取 index: {key!r}（key 须以数字开头）[/red]")
            return
        if idx not in PARAM_TABLE:
            _console.print(f"[red]未知 index: {idx}[/red]")
            return
        if idx not in allowed:
            entry = PARAM_TABLE[idx]
            _console.print(
                f"[red]idx {idx} ({entry.name}) 在当前{'普通' if not expert else '专家'}模式下不可写。[/red]"
            )
            if entry.tier == Tier.EXPERT and not expert:
                _console.print("[yellow]提示: 切换专家模式后可写此参数。[/yellow]")
            return
        items.append((idx, val))

    if not items:
        _console.print("[yellow]JSON 为空，无参数可写。[/yellow]")
        return

    # 批量读取当前值，显示差异表
    _console.print(f"\n[cyan]批量写入预览 ({len(items)} 个参数):[/cyan]")
    diff_table = Table(title="写入计划")
    diff_table.add_column("idx", justify="right")
    diff_table.add_column("名称")
    diff_table.add_column("当前值", justify="right")
    diff_table.add_column("→", justify="center")
    diff_table.add_column("新值", justify="right")
    diff_table.add_column("类型")

    current_values: dict[int, float | int] = {}
    for idx, new_val in items:
        entry = PARAM_TABLE[idx]
        try:
            cur = session.read(idx)
            cur_str = _format_value(cur, entry.dtype)
            current_values[idx] = cur
        except MotorIOError:
            cur_str = "[red]读取失败[/red]"
        new_str = _format_value(new_val, entry.dtype)
        diff_table.add_row(
            str(idx),
            f"[{TIER_STYLE[entry.tier]}]{entry.name}[/]",
            cur_str,
            "→",
            f"[yellow]{new_str}[/yellow]",
            entry.dtype,
        )
    _console.print(diff_table)

    if not Confirm.ask(f"[bold]确认写入以上 {len(items)} 个参数?[/bold]", default=False):
        _console.print("已取消。")
        return

    # 逐个写入 + 回读校验
    ok_count = 0
    fail_count = 0
    for idx, new_val in items:
        entry = PARAM_TABLE[idx]
        old = current_values.get(idx)
        try:
            ok, readback = session.write(idx, new_val)
        except MotorIOError as e:
            _console.print(f"  [red]✗ [{idx}] {entry.name}: {e}[/red]")
            logger.log_write(bus=session.bus, motor_id=session.motor_id,
                             idx=idx, name=entry.name, old=old, new=new_val,
                             readback=None, ok=False)
            fail_count += 1
            continue

        logger.log_write(bus=session.bus, motor_id=session.motor_id,
                         idx=idx, name=entry.name, old=old, new=new_val,
                         readback=readback, ok=ok)
        if ok:
            _console.print(
                f"  [green]✓[/green] [{idx}] {entry.name}: "
                f"{_format_value(new_val, entry.dtype)}  回读={_format_value(readback, entry.dtype)}"
            )
            ok_count += 1
        else:
            _console.print(
                f"  [red]✗[/red] [{idx}] {entry.name}: 回读校验失败 "
                f"(期望 {_format_value(new_val, entry.dtype)}, "
                f"实际 {_format_value(readback, entry.dtype)})"
            )
            fail_count += 1

    # 汇总
    _console.print(
        f"\n[cyan]批量写入完成:[/cyan] "
        f"[green]{ok_count} 成功[/green], "
        f"{'[red]' + str(fail_count) + ' 失败[/red]' if fail_count else '0 失败'}"
    )
    if ok_count > 0 and fail_count == 0:
        _console.print("[dim]仅写入 RAM，保存到 Flash 请回主菜单选 [4]。[/dim]")
        if Confirm.ask("[bold]现在保存到 Flash?[/bold]", default=False):
            tui_save_flash(session, logger)


UNLOCK_PHRASE = "UNLOCK"


def tui_save_flash(session: MotorSession, logger: OpLogger) -> None:
    _console.print(
        "[yellow]将把当前 RAM 参数保存到 Flash（永久生效）。[/yellow]\n"
        "[yellow]此操作不可撤销（除非下次再覆盖写）。[/yellow]"
    )
    if not Confirm.ask("[bold]确认保存?[/bold]", default=False):
        _console.print("已取消。")
        return
    try:
        session.save()
    except MotorIOError as e:
        _console.print(f"[red]{e}[/red]")
        return
    logger.log_save(bus=session.bus, motor_id=session.motor_id)
    _console.print(
        "[green]保存帧已发送。[/green]\n"
        "[dim]协议无响应，建议断电重启后读回关键参数确认持久化。[/dim]"
    )


def _select_motor_ids(bus: BusInfo, online_ids: list[int]) -> tuple[list[int], bool]:
    """让用户选择要升级的电机 ID 列表，支持 'm <id>' 跳过扫描。

    输入示例:
      "1,3,5"        → ([1, 3, 5], False)（必须都在 online_ids 里）
      "m 7"          → ([7], True)（手动模式，不验证在线，用于 boot 中重试）
      "m 7,9"        → ([7, 9], True)（手动模式可多个）
    返回空列表和 manual 标志，返回空列表表示用户取消。
    """
    _console.print(
        f"[cyan]在 {bus.name} 上选择要升级的电机 ID（逗号分隔）。[/cyan]\n"
        f"  在线: {', '.join(str(i) for i in online_ids) if online_ids else '(无)'}\n"
        f"  手动模式: 输入 'm <id1,id2,...>' 跳过在线检查（用于 boot 中重试）"
    )
    raw = Prompt.ask("电机 ID", default="").strip()
    if not raw:
        return [], False

    manual = raw.startswith("m ") or raw.startswith("M ")
    payload = raw[2:].strip() if manual else raw

    ids: list[int] = []
    for tok in payload.split(","):
        tok = tok.strip()
        if not tok:
            continue
        try:
            mid = int(tok)
        except ValueError:
            _console.print(f"[red]非法 ID: {tok!r}[/red]")
            return [], False
        if mid < 1 or mid > 0xFF:
            _console.print(f"[red]ID 越界: {mid}[/red]")
            return [], False
        if not manual and mid not in online_ids:
            _console.print(f"[red]ID {mid} 不在在线列表，请用 'm {mid}' 强制升级。[/red]")
            return [], False
        ids.append(mid)
    return ids, manual


def tui_ota_upgrade(buses: list[BusInfo], scan_result: dict[str, list[int]],
                    logger: OpLogger, args) -> None:
    """OTA 升级 TUI 入口。

    流程: 选 bus → 选电机 → 选 .bin → 概览确认 → 串行升级 → 汇总。
    """
    if not buses:
        _console.print("[red]未发现任何 UP 的 bcan* 接口。[/red]")
        return

    # 1) 选 bus
    overview = Table(title="可用总线")
    overview.add_column("序号", justify="right")
    overview.add_column("总线")
    overview.add_column("类型")
    overview.add_column("在线电机")
    for i, bus in enumerate(buses, 1):
        ids = scan_result.get(bus.name, [])
        overview.add_row(
            str(i), bus.name,
            "CANFD" if bus.is_fd else "CAN",
            ", ".join(f"M{x}" for x in ids) if ids else "[dim](无)[/dim]",
        )
    _console.print(overview)

    choice = Prompt.ask(
        "选择总线 (序号, q 退出)",
        choices=[str(i) for i in range(1, len(buses) + 1)] + ["q"],
    )
    if choice == "q":
        return
    bus = buses[int(choice) - 1]
    online = scan_result.get(bus.name, [])

    # 2) 选电机
    motor_ids, manual = _select_motor_ids(bus, online)
    if not motor_ids:
        _console.print("[yellow]已取消。[/yellow]")
        return

    # 3) 选 .bin
    fw_path_str = Prompt.ask("固件 .bin 路径").strip()
    fw_path = Path(fw_path_str).expanduser()
    try:
        firmware = can_ota.load_firmware_bin(fw_path)
    except (FileNotFoundError, ValueError) as e:
        _console.print(f"[red]{e}[/red]")
        return
    raw_sum32 = can_ota.compute_sum32(firmware)
    chunk_size = 64 if bus.is_fd else 8
    wire_sum32 = can_ota.compute_wire_sum32(firmware, chunk_size)

    # 4) 概览
    pad = (chunk_size - len(firmware) % chunk_size) % chunk_size
    info = Table(title="升级概览")
    info.add_column("项")
    info.add_column("值")
    info.add_row("总线", f"{bus.name} ({'CANFD' if bus.is_fd else 'CAN'})")
    info.add_row("电机", ", ".join(f"M{m}" for m in motor_ids))
    info.add_row("固件", str(fw_path))
    info.add_row("大小", f"{len(firmware)} bytes")
    info.add_row("raw sum32 (对拍 hexed.it)", f"0x{raw_sum32:08X}")
    info.add_row("wire sum32 (含 padding，实际下发)", f"0x{wire_sum32:08X}")
    info.add_row("末帧 padding", f"{pad} 字节 (0xFF)")
    _console.print(info)
    if args.debug:
        _console.print(f"[dim][debug] raw={raw_sum32:#x} wire={wire_sum32:#x}[/dim]")

    _console.print(
        "[red]⚠ 升级中请勿断电、勿拔 CAN 线。失败的电机会留在 boot 中，可用 'm <id>' 重试。[/red]"
    )
    if not Confirm.ask(f"[bold]确认升级 {len(motor_ids)} 台电机?[/bold]", default=False):
        _console.print("已取消。")
        return

    # 5) 串行升级
    summary: list[tuple[int, str, str, str]] = []  # (motor_id, status, version, error)
    for motor_id in motor_ids:
        _ota_one_motor(bus, motor_id, firmware, wire_sum32, fw_path, logger, args, summary,
                       skip_enter_boot=manual)

    # 6) 汇总
    sumtab = Table(title="升级汇总")
    sumtab.add_column("Motor ID", justify="right")
    sumtab.add_column("状态")
    sumtab.add_column("固件版本")
    sumtab.add_column("错误")
    for mid, status, ver, err in summary:
        sumtab.add_row(str(mid), status, ver, err)
    _console.print(sumtab)


def _ota_one_motor(bus: BusInfo, motor_id: int, firmware: bytes, sum32: int,
                   fw_path: Path, logger: OpLogger, args,
                   summary: list[tuple[int, str, str, str]],
                   *, skip_enter_boot: bool = False) -> None:
    """升级单台电机，结果追加到 summary。"""
    logger.log_ota_begin(
        bus=bus.name, motor_id=motor_id, fw_path=str(fw_path),
        fw_size=len(firmware), sum32=sum32,
    )

    progress = Progress(
        TextColumn(f"[bold]M{motor_id}[/bold] {{task.fields[phase]}}"),
        BarColumn(),
        TextColumn("{task.completed}/{task.total} B"),
        TimeElapsedColumn(),
        console=_console,
    )

    task_id = None

    def on_progress(p) -> None:
        nonlocal task_id
        if task_id is None:
            return
        progress.update(
            task_id,
            completed=p.sent,
            total=max(p.total, len(firmware)),
            phase=p.phase,
        )

    try:
        with progress:
            task_id = progress.add_task("升级", total=len(firmware), phase="enter_boot")
            ok, err = can_ota.upgrade_motor(
                bus=bus.name, is_fd=bus.is_fd, motor_id=motor_id,
                firmware=firmware, sum32=sum32,
                skip_enter_boot=skip_enter_boot,
                progress_cb=on_progress, debug=args.debug,
            )
    except KeyboardInterrupt:
        _console.print(f"[yellow]M{motor_id} 已中断。[/yellow]")
        logger.log_ota_end(bus=bus.name, motor_id=motor_id, ok=False, error="aborted")
        summary.append((motor_id, "[yellow]aborted[/yellow]", "-", "Ctrl+C"))
        return

    if not ok:
        _console.print(f"[red]M{motor_id} 升级失败: {err}[/red]")
        logger.log_ota_end(bus=bus.name, motor_id=motor_id, ok=False, error=err)
        summary.append((motor_id, "[red]fail[/red]", "-", err))
        return

    # 升级成功 → 重试 3 次读 idx=10 固件版本
    session = MotorSession(
        bus=bus.name, motor_id=motor_id, is_fd=bus.is_fd,
        timeout_ms=args.io_timeout, debug=args.debug,
    )
    fw_version: Optional[int] = None
    for attempt in range(3):
        time.sleep(0.2)
        try:
            fw_version = int(session.read(10))
            break
        except MotorIOError:
            continue

    if fw_version is None:
        _console.print(
            f"[yellow]M{motor_id} 升级帧已收到 A3，但 idx=10 固件版本回读失败 (3 次)。[/yellow]"
        )
        logger.log_ota_end(bus=bus.name, motor_id=motor_id, ok=True,
                           fw_version=None, error="version readback failed")
        summary.append((motor_id, "[yellow]ok (warn)[/yellow]", "-", "version readback failed"))
    else:
        _console.print(f"[green]M{motor_id} 升级成功，新版本 0x{fw_version:08X}。[/green]")
        logger.log_ota_end(bus=bus.name, motor_id=motor_id, ok=True, fw_version=fw_version)
        summary.append((motor_id, "[green]ok[/green]", f"0x{fw_version:08X}", ""))


def tui_ota_upgrade_from_menu(logger: OpLogger) -> None:
    """主菜单 [8] 入口：重新扫描总线并启动 OTA 流程。"""
    if _ARGS is None:
        _console.print("[red][bug] _ARGS 未初始化[/red]")
        return
    buses = list_up_buses()
    scan_result = scan_motors(
        buses, range(1, _ARGS.scan_id_max + 1),
        timeout_ms=_ARGS.scan_timeout, debug=_ARGS.debug,
    )
    tui_ota_upgrade(buses, scan_result, logger, _ARGS)


def tui_toggle_expert(current: bool) -> bool:
    """返回切换后的 expert 标志。"""
    if current:
        _console.print("[green]已退出专家模式。[/green]")
        return False
    _console.print(
        "[red]⚠ 专家模式会解锁机械零位、物理参数、保护阈值等危险寄存器。[/red]\n"
        "[red]误改可能导致电机无法启动或损坏。[/red]"
    )
    phrase = Prompt.ask(f"输入 [bold]{UNLOCK_PHRASE}[/bold] 解锁 (其他任意键取消)")
    if phrase == UNLOCK_PHRASE:
        _console.print("[red]已进入专家模式。[/red]")
        return True
    _console.print("未解锁。")
    return False


def _main_menu_loop(session: MotorSession, logger: OpLogger) -> str:
    """
    循环处理主菜单动作，直到用户选择切换电机或退出。
    返回: 'switch' 切换电机；'quit' 退出。
    """
    expert = False
    while True:
        _print_header(session, expert)
        _console.print(
            "  [1] 读取单个参数\n"
            "  [2] 读取全部参数\n"
            "  [3] 修改参数\n"
            "  [4] 保存到 Flash\n"
            "  [5] 切换电机\n"
            "  [6] 从 JSON 批量写入\n"
            "  [7] 导出参数为 JSON\n"
            "  [8] OTA 升级（重新扫描 + 选 bin）\n"
            "  [e] 切换专家模式\n"
            "  [q] 退出"
        )
        action = Prompt.ask("> ", choices=["1", "2", "3", "4", "5", "6", "7", "8", "e", "q"])
        try:
            if action == "1":
                tui_read_single(session, logger)
            elif action == "2":
                tui_read_all(session, logger)
            elif action == "3":
                tui_write_param(session, expert, logger)
            elif action == "4":
                tui_save_flash(session, logger)
            elif action == "5":
                return "switch"
            elif action == "6":
                tui_batch_write_json(session, expert, logger)
            elif action == "7":
                tui_export_json(session, expert)
            elif action == "8":
                tui_ota_upgrade_from_menu(logger)
            elif action == "e":
                expert = tui_toggle_expert(expert)
            elif action == "q":
                return "quit"
        except KeyboardInterrupt:
            _console.print("\n[dim](已取消当前操作)[/dim]")
        except Exception as e:  # 一次操作出错不应拖崩工具
            _console.print(f"[red][bug] 操作异常: {e}[/red]")
            traceback.print_exc()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scan-timeout", type=int, default=50, help="单帧扫描超时 (ms)")
    parser.add_argument("--scan-id-max", type=int, default=16, help="扫描电机 ID 上限 (默认 16)")
    parser.add_argument("--io-timeout", type=int, default=200, help="I/O 读写超时 (ms)")
    parser.add_argument("--debug", action="store_true", help="显示原始 candump 输出")
    parser.add_argument("--no-log", action="store_true", help="关闭日志记录")
    args = parser.parse_args()
    global _ARGS
    _ARGS = args

    # 前置检查
    try:
        if not tui_preflight():
            return 0
    except KeyboardInterrupt:
        return 0

    # 日志
    log_dir = Path.cwd() / "logs"
    logger = OpLogger(log_dir, enabled=not args.no_log)

    try:
        while True:
            picked = tui_scan_and_select(
                scan_id_max=args.scan_id_max,
                scan_timeout_ms=args.scan_timeout,
                debug=args.debug,
            )
            if picked is None:
                break
            bus, motor_id = picked
            session = MotorSession(
                bus=bus.name,
                motor_id=motor_id,
                is_fd=bus.is_fd,
                timeout_ms=args.io_timeout,
                debug=args.debug,
            )
            result = _main_menu_loop(session, logger)
            if result == "quit":
                break
            # 'switch' 回到扫描+选择
    except KeyboardInterrupt:
        _console.print("\n退出。")
    finally:
        logger.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
