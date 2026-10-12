#!/usr/bin/env python3
"""机器人电机参数/固件一致性校验工具。

读取当前机器人的全部电机参数和固件版本，与一份"标准 JSON"逐项对比，输出差异报告。

标准 JSON 由 can_param_tool.py 导出（主菜单扫描后选 'a' 全电机导出），本脚本
不负责生成标准，只做"读取当前 + 对比标准"。底层 CAN I/O、参数表（PARAM_TABLE，
idx 10-67，含 idx=10 固件版本）、总线发现、电机扫描均复用 can_param_tool.py。

用法:
  # 用 can_param_tool.py 在标杆机器人上导出标准（菜单选 'a'），得到 all_motors_params_*.json
  # 然后在待测机器人上校验:
  python3 can_param_check.py all_motors_params_xxx.json

  # 同时把当前机器人读到的参数另存一份，方便归档:
  python3 can_param_check.py baseline.json --dump current.json

标准 JSON 格式（与 can_param_tool.py "全部电机导出" 一致）:
  {
    "bcan0_M3": {
      "10 (Firmware Version)": "0x00010002",
      "11 (Control Mode)": 2,
      "12 (Id Controller Kp)": 0.3,
      ...
    },
    ...
  }

退出码: 0=全部一致; 1=存在差异; 2=运行错误（无总线/无电机/标准文件问题等）。
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Optional

from rich.console import Console
from rich.table import Table
from rich.prompt import Confirm

# 复用 can_param_tool.py 的底层能力
from can_param_tool import (
    PARAM_TABLE,
    BusInfo,
    MotorIOError,
    MotorSession,
    Tier,
    list_up_buses,
    scan_motors,
)

import can_ota

_console = Console()

# 浮点对比容差。导出端用 round(val, 6) 量化到 6 位小数，最小可分辨差就是 1e-6，
# 故容差取半个量化步长 5e-7：真正相同的值（量化后位级相等）判等，差到一个步长
# 才算不一致。容差用开区间 <，避免恰好等于步长时的边界误报。
FLOAT_TOL = 5e-7

# 固件版本参数 index（hex32），对比时单独高亮。
FW_VERSION_IDX = 10

# 电机个体参数：22/23 电角度/机械角度偏移是单台电机出厂标定，36 Motor ID 已由电机 key
# 匹配。标准来自单台机器快照，与其它机器必然不同，对比时跳过。
SKIP_COMPARE_IDXS: frozenset[int] = frozenset({22, 23, 36})

# 电角度偏移 index。跳过对比但仍读取；实际值为 0 极少见，多半是标定被擦除，需告警。
ELEC_ANGLE_IDX = 22

# 核心参数 index（写入时只写这些，来自标准参数模板定义）
# 10(FW/RO)·11-15(电流环)·21(TorqueLimit)·24-35(CANCOM)·43(TorqueSlop)
# ·44-46(NPP/GR/Kt)·49(BUS_OV)·51-53(保护)·55(StuckTime)·56(Protect)·66(OT_LOCK)·67(CanMaster)
CORE_IDXS: frozenset[int] = frozenset({
    10, 11, 12, 13, 14, 15,
    21,
    24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35,
    43, 44, 45, 46,
    49, 51, 52, 53, 55, 56,
    66, 67,
})


def _json_key(idx: int) -> str:
    """生成带参数名的 JSON key，如 '12 (Id Controller Kp)'。与 can_param_tool 保持一致。"""
    return f"{idx} ({PARAM_TABLE[idx].name})"


def _read_motor_params(session: MotorSession) -> dict[str, object]:
    """读取单台电机的全部参数（idx 10-67），返回 JSON-friendly dict。

    读取失败的参数值为 None（与 can_param_tool 全量导出行为一致）。
    """
    data: dict[str, object] = {}
    for idx in sorted(PARAM_TABLE.keys()):
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
    return data


def read_all_motors(
    scan_id_max: int,
    scan_timeout_ms: int,
    io_timeout_ms: int,
    debug: bool,
) -> Optional[dict[str, dict]]:
    """扫描所有 UP 总线上的在线电机，逐台读全部参数。

    返回 {"bcan0_M3": {...}, ...}；无总线/无电机返回 None。
    """
    buses = list_up_buses()
    if not buses:
        print("[error] 未发现任何 UP 的 bcan* 接口。", file=sys.stderr)
        return None

    print(f"[info] 扫描在线电机 (id 1-{scan_id_max}, 超时 {scan_timeout_ms}ms)...")
    scan_result = scan_motors(
        buses, range(1, scan_id_max + 1),
        timeout_ms=scan_timeout_ms, debug=debug,
    )

    bus_map = {b.name: b for b in buses}
    work: list[tuple[BusInfo, int]] = []
    for bus_name, motor_ids in scan_result.items():
        bus = bus_map.get(bus_name)
        if bus is None:
            continue
        for mid in sorted(motor_ids):
            work.append((bus, mid))

    if not work:
        print("[error] 没有扫描到在线电机。", file=sys.stderr)
        return None

    print(f"[info] 读取 {len(work)} 台电机全部参数 (idx 10-67)...")
    all_data: dict[str, dict] = {}
    for bus, motor_id in work:
        print(f"  - {bus.name} M{motor_id} ...")
        session = MotorSession(
            bus=bus.name, motor_id=motor_id, is_fd=bus.is_fd,
            timeout_ms=io_timeout_ms, debug=debug,
        )
        all_data[f"{bus.name}_M{motor_id}"] = _read_motor_params(session)
    return all_data


def _to_int(v) -> Optional[int]:
    """把 JSON 里的整型值（int 或 '0x...' 字符串）转成 int；失败返回 None。"""
    if isinstance(v, bool):  # 防御：JSON true/false 不应进来
        return int(v)
    if isinstance(v, str):
        try:
            return int(v, 0)
        except ValueError:
            return None
    if isinstance(v, (int, float)):
        return int(v)
    return None


# 参数对比结果分类
MATCH = "match"            # 一致
MISMATCH = "mismatch"      # 真·不一致（两者都有值但不同）—— 计入"不通过"
UNREADABLE = "unreadable"  # 一方为 null（固件版本差异/读取失败，某寄存器不存在）—— 不计入"不通过"
SKIPPED = "skipped"        # 电机个体参数（SKIP_COMPARE_IDXS），只展示不对比 —— 不计入"不通过"


def _float_error(expected, actual) -> Optional[float]:
    """浮点绝对误差 |expected-actual|；无法计算返回 None。"""
    try:
        return abs(float(expected) - float(actual))
    except (TypeError, ValueError):
        return None


def classify(idx: int, expected, actual) -> tuple[str, Optional[float]]:
    """对单个参数分类，返回 (kind, error)。

    error 仅对浮点 MISMATCH/MATCH 有意义（绝对误差），其余为 None。
    expected 可以是列表（如多个可用固件版本），实际值命中任一项即一致；空列表按 null 处理。
    """
    if isinstance(expected, list):
        if not expected:
            expected = None
        else:
            for item in expected:
                kind, error = classify(idx, item, actual)
                if kind != MISMATCH:
                    return kind, error
            return MISMATCH, None

    # 一方 null：固件差异/不可读，不算真·不一致
    if expected is None or actual is None:
        if expected is None and actual is None:
            return MATCH, None
        return UNREADABLE, None

    entry = PARAM_TABLE[idx]
    if entry.dtype == "float":
        err = _float_error(expected, actual)
        if err is None:
            return MISMATCH, None
        return (MATCH if err < FLOAT_TOL else MISMATCH), err
    # uint32 / hex32：hex32 在 JSON 里是 "0x..." 字符串，统一转 int 比较
    return (MATCH if _to_int(expected) == _to_int(actual) else MISMATCH), None


class ParamResult:
    """单个参数的对比结果。"""

    def __init__(self, idx: int, expected, actual, kind: str, error: Optional[float]):
        self.idx = idx
        self.expected = expected
        self.actual = actual
        self.kind = kind
        self.error = error

    @property
    def is_fw(self) -> bool:
        return self.idx == FW_VERSION_IDX

    @property
    def is_core(self) -> bool:
        return self.idx in CORE_IDXS


class MotorResult:
    """单台电机的对比结果。"""

    def __init__(self, motor: str):
        self.motor = motor
        self.params: list[ParamResult] = []
        self.elec_angle_zero = False  # 当前电角度偏移读数为 0（疑似标定被擦除）
        self.elec_angle_unreadable = False  # 当前电角度偏移读不到

    @property
    def mismatches(self) -> list[ParamResult]:
        return [p for p in self.params if p.kind == MISMATCH]

    @property
    def core_mismatches(self) -> list[ParamResult]:
        """核心参数不一致（会被写入修复）。"""
        return [p for p in self.mismatches if p.is_core]

    @property
    def noncore_mismatches(self) -> list[ParamResult]:
        """非核心参数不一致（仅供参考,不会写入）。"""
        return [p for p in self.mismatches if not p.is_core]

    @property
    def unreadables(self) -> list[ParamResult]:
        return [p for p in self.params if p.kind == UNREADABLE]

    @property
    def fw(self) -> Optional[ParamResult]:
        """固件版本参数结果（idx=10），标准里没记录则为 None。"""
        for p in self.params:
            if p.is_fw:
                return p
        return None

    @property
    def max_error(self) -> Optional[ParamResult]:
        """浮点误差最大的参数（含一致项），无浮点参数则 None。"""
        floats = [p for p in self.params if p.error is not None]
        if not floats:
            return None
        return max(floats, key=lambda p: p.error)

    @property
    def passed(self) -> bool:
        """无真·不一致即通过（固件差异/不可读不算未通过）。"""
        return not self.mismatches


def compare(
    baseline: dict[str, dict],
    current: dict[str, dict],
) -> tuple[list[MotorResult], list[str], list[str]]:
    """对比标准与当前数据，按 bus+电机ID（即顶层 key）匹配。

    返回 (逐台电机结果, 标准有但当前缺失的电机, 当前多出的电机)。
    """
    results: list[MotorResult] = []
    baseline_motors = set(baseline.keys())
    current_motors = set(current.keys())

    missing = sorted(baseline_motors - current_motors)
    extra = sorted(current_motors - baseline_motors)

    for motor in sorted(baseline_motors & current_motors):
        base_params = baseline[motor]
        cur_params = current[motor]
        mr = MotorResult(motor)
        elec_angle = cur_params.get(_json_key(ELEC_ANGLE_IDX))
        mr.elec_angle_zero = elec_angle is not None and float(elec_angle) == 0.0
        mr.elec_angle_unreadable = elec_angle is None
        for idx in sorted(PARAM_TABLE.keys()):
            key = _json_key(idx)
            if idx in SKIP_COMPARE_IDXS:
                # 仍记录实际值供 -v 展示，但不参与判定
                mr.params.append(ParamResult(
                    idx, base_params.get(key), cur_params.get(key), SKIPPED, None,
                ))
                continue
            # 标准里没记录该参数则跳过（兼容只含部分参数的旧标准文件）
            if key not in base_params:
                continue
            expected = base_params.get(key)
            actual = cur_params.get(key)
            kind, error = classify(idx, expected, actual)
            mr.params.append(ParamResult(idx, expected, actual, kind, error))
        results.append(mr)
    return results, missing, extra


_ELEC_ANGLE_ZERO_WARNING = (
    "⚠ 电角度偏移(idx 22)为 0，疑似电角度标定被擦除，请重新校准电机！"
)
_ELEC_ANGLE_UNREADABLE_WARNING = (
    "⚠ 电角度偏移(idx 22)读取失败，无法确认电角度标定是否正常，请检查电机！"
)


def _fmt(v) -> str:
    if isinstance(v, list):
        return " / ".join(str(item) for item in v)
    return "[dim]--[/dim]" if v is None else str(v)


def _err_str(p: ParamResult) -> str:
    """误差列文本：浮点显示绝对误差，不可读区分标准 null 还是当前 null。"""
    if p.kind == UNREADABLE:
        if p.expected is None:
            return "[yellow]标准固件此参数为 null[/yellow]"
        return "[yellow]当前固件未响应/不可读[/yellow]"
    if p.error is not None:  # 浮点
        return f"误差={p.error:.6g}"
    return ""  # 整型/hex 无误差概念


def print_report(
    results: list[MotorResult],
    missing: list[str],
    extra: list[str],
    verbose: bool,
) -> None:
    """逐台电机打印彩色对比报告。"""
    _console.print("\n[bold cyan]========== 参数一致性校验报告 ==========[/bold cyan]")

    if missing:
        _console.print(
            f"\n[yellow][警告] 标准中存在但当前机器人未扫到的电机 "
            f"({len(missing)}):[/yellow]"
        )
        for m in missing:
            _console.print(f"  [yellow]- {m}[/yellow]")
    if extra:
        _console.print(
            f"\n[blue][提示] 当前机器人多出（标准中没有）的电机 "
            f"({len(extra)}):[/blue]"
        )
        for m in extra:
            _console.print(f"  [blue]- {m}[/blue]")

    # 逐台电机
    for mr in results:
        _print_one_motor(mr, verbose)

    zero_angle = [r.motor for r in results if r.elec_angle_zero]
    if zero_angle:
        _console.print(
            f"\n[bold red][警告] {len(zero_angle)} 台电机电角度偏移为 0: "
            f"{', '.join(zero_angle)}[/bold red]",
            highlight=False,
        )
        _console.print(f"[bold red]{_ELEC_ANGLE_ZERO_WARNING}[/bold red]", highlight=False)
    unreadable_angle = [r.motor for r in results if r.elec_angle_unreadable]
    if unreadable_angle:
        _console.print(
            f"\n[bold red][警告] {len(unreadable_angle)} 台电机电角度偏移读取失败: "
            f"{', '.join(unreadable_angle)}[/bold red]",
            highlight=False,
        )
        _console.print(f"[bold red]{_ELEC_ANGLE_UNREADABLE_WARNING}[/bold red]", highlight=False)

    # 总汇总
    n_pass = sum(1 for r in results if r.passed)
    n_fail = len(results) - n_pass
    all_ok = n_fail == 0 and not missing
    _console.print("\n[bold]=======================================[/bold]")
    status = "[bold green]通过[/bold green]" if all_ok else "[bold red]未通过[/bold red]"
    _console.print(
        f"总结论: {status}   "
        f"[green]{n_pass} 台通过[/green] / "
        f"{('[red]' + str(n_fail) + ' 台未通过[/red]') if n_fail else '0 台未通过'}"
        f"   (缺失 {len(missing)} 台, 多出 {len(extra)} 台)"
    )
    _console.print("[bold]=======================================[/bold]")


def _parse_motor_key(key: str) -> tuple[str, int]:
    """从 'bcan0_M3' 解析出 (bus_name, motor_id)。"""
    idx = key.rindex("_M")
    return key[:idx], int(key[idx + 2:])


def ota_firmware(
    baseline: dict[str, dict],
    results: list[MotorResult],
    io_timeout_ms: int,
    debug: bool,
    auto_yes: bool,
) -> bool:
    """对固件版本不一致的电机执行 OTA 升级。成功返回 True。"""
    # 收集固件不一致的电机
    fw_mismatches = [r for r in results if r.fw is not None and r.fw.kind == MISMATCH]
    if not fw_mismatches:
        return True

    _console.print(
        f"\n[bold yellow]发现 {len(fw_mismatches)} 台电机固件版本与标准不一致:[/bold yellow]"
    )
    for r in fw_mismatches:
        fw_key = _json_key(FW_VERSION_IDX)
        _console.print(
            f"  [yellow]{r.motor}: 当前 {r.fw.actual}  →  标准 {_fmt(r.fw.expected)}[/yellow]"
        )

    if not auto_yes:
        if not Confirm.ask(
            f"\n[bold]是否通过 OTA 将这 {len(fw_mismatches)} 台电机升级到标准固件?[/bold]",
            default=False,
        ):
            _console.print("跳过 OTA。")
            return False

    # 选 .bin 固件
    from rich.prompt import Prompt
    fw_path_str = Prompt.ask("固件 .bin 路径").strip()
    fw_path = Path(fw_path_str).expanduser()
    try:
        firmware = can_ota.load_firmware_bin(fw_path)
    except (FileNotFoundError, ValueError) as e:
        _console.print(f"[red]{e}[/red]")
        return False

    # 解析目标电机所属总线
    buses = list_up_buses()
    bus_map = {b.name: b for b in buses}

    chunk_size = 64  # 假设 CANFD（暂不支持 classic CAN 自动检测）
    wire_sum32 = can_ota.compute_wire_sum32(firmware, chunk_size)
    pad = (chunk_size - len(firmware) % chunk_size) % chunk_size
    raw_sum32 = can_ota.compute_sum32(firmware)

    info = Table(title="OTA 升级概览")
    info.add_column("项")
    info.add_column("值")
    info.add_row("固件", str(fw_path))
    info.add_row("大小", f"{len(firmware)} bytes")
    info.add_row("目标电机", ", ".join(r.motor for r in fw_mismatches))
    info.add_row("padding", f"{pad} B (0xFF)")
    _console.print(info)

    _console.print("[red]⚠ 升级中请勿断电。[/red]")
    if not auto_yes:
        if not Confirm.ask(
            f"[bold]确认升级 {len(fw_mismatches)} 台电机?[/bold]", default=False,
        ):
            _console.print("已取消。")
            return False

    # 串行升级
    ok_count = 0
    fail_count = 0
    for r in fw_mismatches:
        bus_name, mid = _parse_motor_key(r.motor)
        bus = bus_map.get(bus_name)
        if bus is None:
            _console.print(f"  [red]✗ {r.motor}: 总线离线[/red]")
            fail_count += 1
            continue

        _console.print(f"\n[cyan]升级 {r.motor} ...[/cyan]")
        try:
            ok, err = can_ota.upgrade_motor(
                bus=bus.name, is_fd=bus.is_fd, motor_id=mid,
                firmware=firmware, sum32=wire_sum32, debug=debug,
            )
        except KeyboardInterrupt:
            _console.print(f"[yellow]✗ {r.motor}: 已中断[/yellow]")
            fail_count += 1
            continue
        except Exception as e:
            _console.print(f"[red]✗ {r.motor}: {e}[/red]")
            fail_count += 1
            continue

        if not ok:
            _console.print(f"[red]✗ {r.motor}: {err}[/red]")
            fail_count += 1
        else:
            _console.print(f"[green]✓ {r.motor} OTA 完成[/green]")
            ok_count += 1

    _console.print(
        f"\n[bold]OTA 完成:[/bold] "
        f"[green]{ok_count} 成功[/green]"
        f"{', [red]' + str(fail_count) + ' 失败[/red]' if fail_count else ''}"
    )
    return fail_count == 0


def batch_write(
    baseline: dict[str, dict],
    results: list[MotorResult],
    io_timeout_ms: int,
    debug: bool,
    auto_yes: bool,
) -> bool:
    """将标准参数批量写入当前匹配的电机。成功返回 True，失败/取消返回 False。"""
    # 匹配的电机
    matched = [r.motor for r in results]
    if not matched:
        _console.print("\n[yellow]没有匹配的电机，跳过写入。[/yellow]")
        return False

    # 构建写入计划（只写核心参数，排除 RO、标准为 null、标准未收录的）
    writable: list[tuple[str, int]] = []  # (motor_key, idx)
    for motor_key in matched:
        base = baseline[motor_key]
        for idx in sorted(PARAM_TABLE.keys()):
            if idx not in CORE_IDXS:
                continue
            entry = PARAM_TABLE[idx]
            if entry.tier == Tier.RO:
                continue
            key = _json_key(idx)
            if key not in base:
                continue
            if base[key] is None:
                continue
            writable.append((motor_key, idx))

    if not writable:
        _console.print("\n[yellow]没有可写的参数（全部为 RO 或 null）。[/yellow]")
        return False

    # 解析总线信息
    buses = list_up_buses()
    bus_map = {b.name: b for b in buses}

    # 写入预览表
    _console.print(
        f"\n[bold cyan]写入计划:[/bold cyan] {len(matched)} 台电机, "
        f"{len(writable)} 个参数"
    )
    preview = Table(title="写入参数预览", show_header=True, header_style="bold")
    preview.add_column("电机")
    preview.add_column("idx", justify="right")
    preview.add_column("参数名")
    preview.add_column("写入值")
    preview.add_column("分级")
    preview.add_column("总线")
    expert_count = 0
    for motor_key, idx in writable:
        entry = PARAM_TABLE[idx]
        bus_name, mid = _parse_motor_key(motor_key)
        tier_style = {"ro": "dim", "safe": "green", "expert": "red"}[entry.tier.value]
        if entry.tier == Tier.EXPERT:
            expert_count += 1
        preview.add_row(
            motor_key, str(idx),
            f"[{tier_style}]{entry.name}[/{tier_style}]",
            _fmt(baseline[motor_key][_json_key(idx)]),
            f"[{tier_style}]{entry.tier.value}[/{tier_style}]",
            bus_name if bus_name in bus_map else f"[red]{bus_name} (离线)[/red]",
        )
    _console.print(preview)

    if expert_count:
        _console.print(
            f"\n[red]⚠ 含 {expert_count} 个专家参数（红色标注），误改可能导致电机无法启动或损坏。[/red]"
        )

    if not auto_yes:
        if not Confirm.ask(
            f"\n[bold]确认将标准参数写入以上 {len(matched)} 台电机?[/bold]",
            default=False,
        ):
            _console.print("已取消。")
            return False

    # 逐台逐参数写入
    _console.print(f"\n[cyan]开始写入...[/cyan]")
    ok_count = 0
    fail_count = 0
    failures: list[str] = []

    for motor_key, idx in writable:
        bus_name, mid = _parse_motor_key(motor_key)
        bus = bus_map.get(bus_name)
        if bus is None:
            fail_count += 1
            failures.append(f"{motor_key} idx={idx}: 总线离线")
            continue

        entry = PARAM_TABLE[idx]
        val = baseline[motor_key][_json_key(idx)]
        session = MotorSession(
            bus=bus_name, motor_id=mid, is_fd=bus.is_fd,
            timeout_ms=io_timeout_ms, debug=debug,
        )
        try:
            ok, readback = session.write(idx, val)
        except (MotorIOError, ValueError) as e:
            _console.print(f"  [red]✗ {motor_key} idx={idx} {entry.name}: {e}[/red]")
            fail_count += 1
            failures.append(f"{motor_key} idx={idx} ({entry.name}): {e}")
            continue

        if ok:
            _console.print(
                f"  [green]✓ {motor_key} idx={idx} {entry.name}: "
                f"{_fmt(val)}[/green]"
            )
            ok_count += 1
        else:
            _console.print(
                f"  [red]✗ {motor_key} idx={idx} {entry.name}: "
                f"回读校验失败 写入={_fmt(val)} 回读={_fmt(readback)}[/red]"
            )
            fail_count += 1
            failures.append(
                f"{motor_key} idx={idx} ({entry.name}): "
                f"回读校验失败 写入={_fmt(val)} 回读={_fmt(readback)}"
            )

    # 写入汇总
    _console.print(
        f"\n[bold]写入完成:[/bold] "
        f"[green]{ok_count} 成功[/green]"
        f"{', [red]' + str(fail_count) + ' 失败[/red]' if fail_count else ''}"
    )

    # 全部成功 → 直接保存到 Flash（不再单独确认）
    if ok_count > 0 and fail_count == 0:
        _console.print("\n[cyan]保存到 Flash...[/cyan]")
        saved = set()
        save_fails = 0
        for motor_key, idx in writable:
            if motor_key in saved:
                continue
            saved.add(motor_key)
            bus_name, mid = _parse_motor_key(motor_key)
            bus = bus_map.get(bus_name)
            if bus is None:
                save_fails += 1
                continue
            session = MotorSession(
                bus=bus_name, motor_id=mid, is_fd=bus.is_fd,
                timeout_ms=io_timeout_ms, debug=debug,
            )
            try:
                session.save()
                _console.print(f"  [green]✓ {motor_key} Flash 保存帧已发送[/green]")
            except MotorIOError as e:
                _console.print(f"  [red]✗ {motor_key}: {e}[/red]")
                save_fails += 1
        if save_fails:
            _console.print(f"[yellow]⚠ {save_fails} 台电机 Flash 保存失败。[/yellow]")
        else:
            _console.print(
                "[green]Flash 保存完成。[/green]\n"
                "[dim]建议断电重启后读回关键参数确认持久化。[/dim]"
            )

    if fail_count > 0:
        _console.print(f"\n[red]失败明细:[/red]")
        for f in failures:
            _console.print(f"  [red]{f}[/red]")
        return False
    return True


def _fw_label(fw: Optional[ParamResult]) -> str:
    """固件版本标签（始终显示，高亮）。一致青色，不一致红色并附标准值。"""
    if fw is None:
        return "[dim]固件版本: (标准未记录)[/dim]"
    actual = _fmt(fw.actual)
    if fw.kind == MISMATCH:
        return (
            f"[bold red]固件版本: {actual}[/bold red] "
            f"[red](标准 {_fmt(fw.expected)})[/red]"
        )
    if fw.kind == UNREADABLE:
        return f"[bold yellow]固件版本: {actual}[/bold yellow]"
    return f"[bold cyan]固件版本: {actual}[/bold cyan]"


def _print_one_motor(mr: MotorResult, verbose: bool) -> None:
    """打印单台电机的对比结果。核心不一致红色(会被写入),非核心黄色(仅供参考)。"""
    core_mm = mr.core_mismatches
    noncore_mm = mr.noncore_mismatches
    unreadables = mr.unreadables

    if mr.passed:
        head = f"[green]✓ {mr.motor}  通过[/green]"
        if unreadables:
            head += f" [yellow](含 {len(unreadables)} 项固件差异/不可读)[/yellow]"
    else:
        parts = []
        if core_mm:
            parts.append(f"[red]{len(core_mm)} 项核心不一致(会被写入)[/red]")
        if noncore_mm:
            parts.append(f"[yellow]{len(noncore_mm)} 项非核心差异(仅供参考)[/yellow]")
        head = f"[red]✗ {mr.motor}  未通过[/red]  " + " / ".join(parts)
        if unreadables:
            head += f" [yellow](另 {len(unreadables)} 项不可读)[/yellow]"

    head += "  " + _fw_label(mr.fw)
    me = mr.max_error
    if me is not None:
        head += f"  [dim]最大误差={me.error:.3g} (id={me.idx})[/dim]"
    _console.print(f"\n{head}")
    if mr.elec_angle_zero:
        _console.print(f"  [bold red]{_ELEC_ANGLE_ZERO_WARNING}[/bold red]", highlight=False)
    if mr.elec_angle_unreadable:
        _console.print(f"  [bold red]{_ELEC_ANGLE_UNREADABLE_WARNING}[/bold red]", highlight=False)

    # 选要显示的行：verbose 全显，否则只显 MISMATCH + UNREADABLE
    rows = mr.params if verbose else [p for p in mr.params if p.kind not in (MATCH, SKIPPED)]
    if not rows:
        return

    table = Table(show_header=True, header_style="bold", box=None, pad_edge=False)
    table.add_column("idx", justify="right")
    table.add_column("参数名")
    table.add_column("标准值", justify="right")
    table.add_column("实际值", justify="right")
    table.add_column("误差/说明")
    for p in rows:
        entry = PARAM_TABLE[p.idx]
        if p.kind == MATCH:
            style, mark, note = "green", "✓", ""
        elif p.kind == SKIPPED:
            style, mark, note = "dim", "-", "[dim]不参与对比[/dim]"
        elif p.kind == MISMATCH:
            if p.is_core:
                style, mark, note = "red", "✗", "[red]核心，会被写入修复[/red]"
            else:
                style, mark, note = "yellow", "⚠", "[yellow]非核心，仅供参考[/yellow]"
        else:  # UNREADABLE
            style, mark, note = "yellow", "⚠", _err_str(p)
        name = entry.name + (" [bold](固件版本)[/bold]" if p.is_fw else "")
        err_col = note if note else _err_str(p)
        table.add_row(
            f"[{style}]{mark}{p.idx}[/{style}]",
            f"[{style}]{name}[/{style}]",
            _fmt(p.expected),
            _fmt(p.actual),
            err_col,
        )
    _console.print(table)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "baseline",
        help="标准 JSON 文件路径（由 can_param_tool.py 全电机导出）",
    )
    parser.add_argument(
        "--dump", default=None,
        help="把当前机器人读到的参数另存为 JSON（可选，仅归档用）",
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true",
        help="逐台电机列出全部参数（默认只列差异/不可读项）",
    )
    parser.add_argument("--scan-timeout", type=int, default=50, help="单帧扫描超时 (ms)")
    parser.add_argument("--scan-id-max", type=int, default=16, help="扫描电机 ID 上限 (默认 16)")
    parser.add_argument("--io-timeout", type=int, default=200, help="I/O 读取超时 (ms)")
    parser.add_argument("--debug", action="store_true", help="显示原始 candump 输出")
    parser.add_argument(
        "-y", "--yes", action="store_true",
        help="跳过确认提示（写入 + 保存到 Flash 均直接执行）",
    )
    args = parser.parse_args()

    try:
        return _run(args)
    except KeyboardInterrupt:
        print("\n[中断]", file=sys.stderr)
        return 2


def _run(args: argparse.Namespace) -> int:
    # 1) 载入标准 JSON
    baseline_path = Path(args.baseline).expanduser()
    if not baseline_path.is_file():
        print(f"[error] 标准 JSON 文件不存在: {baseline_path}", file=sys.stderr)
        return 2
    try:
        baseline = json.loads(baseline_path.read_text())
    except (json.JSONDecodeError, OSError) as e:
        print(f"[error] 读取标准 JSON 失败: {e}", file=sys.stderr)
        return 2
    if not isinstance(baseline, dict) or not all(
        isinstance(v, dict) for v in baseline.values()
    ):
        print(
            "[error] 标准 JSON 顶层必须是 {电机key: {参数: 值}} 结构。",
            file=sys.stderr,
        )
        return 2

    # 2) 读取当前机器人
    current = read_all_motors(
        scan_id_max=args.scan_id_max,
        scan_timeout_ms=args.scan_timeout,
        io_timeout_ms=args.io_timeout,
        debug=args.debug,
    )
    if current is None:
        return 2

    # 3) 可选归档
    if args.dump:
        dump_path = Path(args.dump).expanduser()
        dump_path.parent.mkdir(parents=True, exist_ok=True)
        dump_path.write_text(json.dumps(current, indent=4, ensure_ascii=False) + "\n")
        print(f"[info] 当前机器人参数已另存到 {dump_path}")

    # 4) 对比 + 报告
    results, missing, extra = compare(baseline, current)
    print_report(results, missing, extra, verbose=args.verbose)

    # 多出电机（extra）只提示不算失败；任一电机有真·不一致、或有缺失电机算未通过。
    auto_yes = getattr(args, "yes", False)

    # 4.5) 如果有固件版本不一致的电机，询问是否 OTA 升级
    fw_mismatch_motors = [r for r in results if r.fw is not None and r.fw.kind == MISMATCH]
    if fw_mismatch_motors:
        ota_done = ota_firmware(
            baseline, results,
            io_timeout_ms=args.io_timeout, debug=args.debug, auto_yes=auto_yes,
        )
        if ota_done:
            # OTA 成功后重新读取 + 重新对比
            _console.print("\n[cyan]OTA 完成，重新读取参数并校验...[/cyan]")
            current = read_all_motors(
                scan_id_max=args.scan_id_max,
                scan_timeout_ms=args.scan_timeout,
                io_timeout_ms=args.io_timeout,
                debug=args.debug,
            )
            if current is not None:
                results, missing, extra = compare(baseline, current)
                _console.print("\n[bold cyan]--- OTA 后重新校验 ---[/bold cyan]")
                print_report(results, missing, extra, verbose=args.verbose)

    any_mismatch = any(not r.passed for r in results)
    check_failed = any_mismatch or missing

    # 5) 无论校验成功与否，询问是否将标准参数写入当前电机
    if auto_yes or Confirm.ask(
        "\n[bold]是否将标准参数写入当前电机?[/bold]", default=False,
    ):
        written_ok = batch_write(baseline, results,
                                 io_timeout_ms=args.io_timeout,
                                 debug=args.debug,
                                 auto_yes=auto_yes)
        if written_ok:
            # 写入全部成功：参数已对齐，返回 0
            return 0
        return 1  # 写入有失败

    return 1 if check_failed else 0


if __name__ == "__main__":
    sys.exit(main())
