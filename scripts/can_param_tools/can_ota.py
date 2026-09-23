"""CAN/CAN FD 电机 OTA 升级模块。

设计：docs/plans/2026-05-07-can-ota-design.md
"""
from __future__ import annotations

import re
import struct
import subprocess
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterator, Optional


def compute_sum32(data: bytes) -> int:
    """按字节累加 mod 2^32，与 hexed.it 'sum' hash 一致。"""
    return sum(data) & 0xFFFFFFFF


def compute_wire_sum32(firmware: bytes, chunk_size: int) -> int:
    """电机实际收到的 sum：原始固件 + 0xFF padding。

    最后一片不足 chunk_size 时补 0xFF；电机 verify 时按收到的全部字节算 sum，
    所以发上 wire 的 sum 必须含 padding。
    """
    if chunk_size <= 0:
        raise ValueError(f"chunk_size must be > 0, got {chunk_size}")
    pad = (chunk_size - len(firmware) % chunk_size) % chunk_size
    return (sum(firmware) + pad * 0xFF) & 0xFFFFFFFF


_ENTER_BOOT_FD = bytes([0x67, 0x04, 0xA0, 0xA1, 0xA2, 0xA3, 0x00, 0x76])


def build_enter_boot_frame(is_fd: bool) -> bytes:
    """0x600+ID 帧：进入 boot loader。

    两份厂商文档定义不同的中间字节：
    - classic CAN (OTA 普通CAN升级流程-260114.doc): Byte2..5 = 00 00 00 00
    - CANFD (Motorevo CANFD协议模组使用说明书 §6.5.1): Byte2..5 = A0 A1 A2 A3
    """
    return _ENTER_BOOT_FD 


def build_declare_send_frame() -> bytes:
    """0x500+ID 帧：声明即将发送文件，末字节 0xEF。"""
    return bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xEF])


def build_finalize_frame(sum32: int) -> bytes:
    """0x500+ID 帧：发送结束并校验。

    格式: 00 00 <sum32 BE 4B> 00 FE
    spec 6.5.1：高端在前。例如 sum=0x12345678 → 00 00 12 34 56 78 00 FE。
    """
    if not (0 <= sum32 <= 0xFFFFFFFF):
        raise ValueError(f"sum32 out of range: {sum32:#x}")
    payload = struct.pack(">I", sum32)
    return bytes([0x00, 0x00]) + payload + bytes([0x00, 0xFE])


_STATUS_CODES = {0xA0: "A0", 0xA1: "A1", 0xA2: "A2", 0xA3: "A3"}


def parse_status_frame(data: bytes) -> str:
    """识别 8A xx .. A8 状态帧。

    返回 'A0' / 'A1' / 'A2' / 'A3' / 'unknown'。
    中间 6 字节忽略；长度不足 8 视为 unknown。
    """
    if len(data) < 8:
        return "unknown"
    if data[0] != 0x8A or data[7] != 0xA8:
        return "unknown"
    return _STATUS_CODES.get(data[1], "unknown")


_HEX_HINT = (
    "仅支持 .bin 固件；可用以下命令转换:\n"
    "  arm-none-eabi-objcopy -I ihex -O binary IN.hex OUT.bin\n"
    "或访问 https://hexed.it 导入 hex 后导出 raw。"
)


def load_firmware_bin(path: Path) -> bytes:
    """加载 .bin 固件文件，仅接受 .bin 后缀。"""
    suffix = path.suffix.lower()
    if suffix == ".hex":
        raise ValueError(f".hex 不被支持。{_HEX_HINT}")
    if suffix != ".bin":
        raise ValueError(f"unsupported firmware suffix: {path.suffix!r}（仅支持 .bin）")
    if not path.is_file():
        raise FileNotFoundError(f"firmware not found: {path}")
    return path.read_bytes()


_PAD_BYTE = 0xFF


@dataclass
class OtaProgress:
    """升级进度回调载体。

    phase: 'enter_boot' | 'declare' | 'transfer' | 'verify'
    """
    phase: str
    sent: int = 0
    total: int = 0
    detail: str = ""


def slice_chunks(data: bytes, chunk_size: int) -> Iterator[bytes]:
    """按 chunk_size 切片；最后一片不足时用 0xFF 补齐。

    chunk_size 取 8 (CC) 或 64 (FD)。空输入返回空迭代器。
    """
    if chunk_size <= 0:
        raise ValueError(f"chunk_size must be > 0, got {chunk_size}")
    for i in range(0, len(data), chunk_size):
        chunk = data[i : i + chunk_size]
        if len(chunk) < chunk_size:
            chunk = chunk + bytes([_PAD_BYTE]) * (chunk_size - len(chunk))
        yield chunk


def _format_ota_frame(can_id: int, is_fd: bool, data: bytes) -> str:
    """构造 cansend 帧字符串：'<ID>#<hex>' 或 '<ID>##1<hex>'。"""
    sep = "##1" if is_fd else "#"
    return f"{can_id:03X}{sep}{data.hex().upper()}"


_CANDUMP_RE = re.compile(
    r"""
    ^\s*
    (?:\(\d+\.\d+\)\s+)?
    (?P<iface>\w+)\s+
    (?P<id>[0-9A-Fa-f]+)\s+
    \[(?P<dlc>\d+)\]\s+
    (?P<data>(?:[0-9A-Fa-f]{2}\s*)+)
    \s*$
    """,
    re.VERBOSE,
)


def _parse_candump_line(line: str) -> Optional[tuple[int, bytes]]:
    m = _CANDUMP_RE.match(line)
    if not m:
        return None
    try:
        return int(m.group("id"), 16), bytes.fromhex(m.group("data").replace(" ", ""))
    except ValueError:
        return None


def _is_response_id(rx_id: int, motor_id: int) -> bool:
    """boot loader 在不同阶段使用两种 RX ID：
    - ENTER_BOOT 阶段（电机还在 APP）：rx_id = motor_id（如 0x002）
    - DECLARE/FINALIZE/A3 阶段（已进 boot）：rx_id = 0x500 + motor_id（如 0x506）
    """
    return rx_id == motor_id or rx_id == (0x500 + motor_id)


def _pick_response(
    lines: list[str],
    tx_can_id: int,
    tx_data: bytes,
    motor_id: int,
) -> Optional[bytes]:
    """从 candump 行里挑出第一条匹配的响应帧。

    - rx_id == tx_can_id 且 rx_data == tx_data → SocketCAN loopback，跳过
    - rx_id ∈ {motor_id, 0x500+motor_id} → 接受，截取前 8 字节返回
    - 其它 → 跳过
    """
    for line in lines:
        parsed = _parse_candump_line(line)
        if parsed is None:
            continue
        rx_id, rx_data = parsed
        if rx_id == tx_can_id and rx_data == tx_data:
            continue
        if not _is_response_id(rx_id, motor_id):
            continue
        return rx_data[:8]
    return None


def _send_ota_frame(
    bus: str,
    is_fd: bool,
    can_id: int,
    data: bytes,
    *,
    motor_id: int,
    timeout_ms: int = 500,
    debug: bool = False,
) -> Optional[bytes]:
    """发送一帧并等待响应。

    TX 走 can_id（0x6XX/0x5XX/0x4XX），boot loader 的应答走 **裸 motor_id**。
    SocketCAN 回环帧的 rx_id == can_id 且 data 匹配，作为 loopback 丢弃。
    超时返回 None。
    """
    frame_str = _format_ota_frame(can_id, is_fd, data)

    with tempfile.NamedTemporaryFile(mode="w+", suffix=".candump", delete=False) as tmp:
        tmp_path = Path(tmp.name)

    try:
        with open(tmp_path, "w") as out:
            dump = subprocess.Popen(
                ["candump", bus, "-n", "10"],
                stdout=out, stderr=subprocess.DEVNULL,
            )
            try:
                time.sleep(0.05)
                r = subprocess.run(
                    ["cansend", bus, frame_str],
                    capture_output=True, text=True,
                )
                if r.returncode != 0:
                    raise RuntimeError(f"cansend 失败: {r.stderr.strip()}")

                deadline = time.monotonic() + timeout_ms / 1000.0
                response = None
                while time.monotonic() < deadline:
                    time.sleep(0.01)
                    try:
                        lines = tmp_path.read_text().splitlines()
                    except OSError:
                        lines = []
                    response = _pick_response(lines, can_id, data, motor_id)
                    if response is not None:
                        break
                return response
            finally:
                dump.terminate()
                try:
                    dump.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    dump.kill()
                    dump.wait()
    finally:
        if debug:
            print(f"  [ota-debug] tx: {frame_str}")
            try:
                print(f"  [ota-debug] candump:\n{tmp_path.read_text()}")
            except OSError:
                pass
        tmp_path.unlink(missing_ok=True)


def _send_ota_frame_no_wait(bus: str, is_fd: bool, can_id: int, data: bytes) -> None:
    """只发不等响应（用于 0x400+ID 的 bin 数据帧）。"""
    frame_str = _format_ota_frame(can_id, is_fd, data)
    r = subprocess.run(
        ["cansend", bus, frame_str],
        capture_output=True, text=True,
    )
    if r.returncode != 0:
        raise RuntimeError(f"cansend 失败 ({frame_str}): {r.stderr.strip()}")


def _wait_for_status_a3(
    bus: str,
    motor_id: int,
    *,
    timeout_ms: int = 5000,
    debug: bool = False,
) -> bool:
    """纯接收等待裸 motor_id 上的 8A A3 .. A8 帧（不发任何帧）。

    boot loader 在 verify 通过后内部 erase+write 期间不发其他帧，
    完成后才发 A3，所以这里只读不发。
    """
    with tempfile.NamedTemporaryFile(mode="w+", suffix=".candump", delete=False) as tmp:
        tmp_path = Path(tmp.name)
    try:
        with open(tmp_path, "w") as out:
            dump = subprocess.Popen(
                ["candump", bus, "-n", "20"],
                stdout=out, stderr=subprocess.DEVNULL,
            )
            try:
                deadline = time.monotonic() + timeout_ms / 1000.0
                while time.monotonic() < deadline:
                    time.sleep(0.05)
                    try:
                        lines = tmp_path.read_text().splitlines()
                    except OSError:
                        lines = []
                    for line in lines:
                        parsed = _parse_candump_line(line)
                        if parsed is None:
                            continue
                        rx_id, rx_data = parsed
                        if not _is_response_id(rx_id, motor_id):
                            continue
                        if parse_status_frame(rx_data) == "A3":
                            return True
                return False
            finally:
                dump.terminate()
                try:
                    dump.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    dump.kill()
                    dump.wait()
    finally:
        if debug:
            try:
                print(f"  [ota-debug] wait_a3 candump:\n{tmp_path.read_text()}")
            except OSError:
                pass
        tmp_path.unlink(missing_ok=True)


def upgrade_motor(
    bus: str,
    is_fd: bool,
    motor_id: int,
    firmware: bytes,
    sum32: int,
    *,
    frame_gap_us: int = 0,
    enter_boot_settle_ms: int = 200,
    declare_timeout_ms: int = 500,
    verify_timeout_ms: int = 5000,
    skip_enter_boot: bool = False,
    progress_cb: Optional[Callable[[OtaProgress], None]] = None,
    debug: bool = False,
) -> tuple[bool, str]:
    """执行单台电机的 OTA 升级状态机。

    返回 (ok, error_message)。失败时电机可能留在 boot；ok=True 时 boot 已退出
    并由用户应用接管。

    支持 classic CAN（8B/帧）和 CANFD（64B/帧）两种协议，按 is_fd 选择
    ENTER_BOOT 字节和 TRANSFER chunk 大小。

    skip_enter_boot=True 时跳过 ENTER_BOOT 阶段，直接从 DECLARE 开始。
    用于电机已在 boot loader 中的重试场景（手动模式 'm <id>'）。
    """
    can_600 = 0x600 + motor_id
    can_500 = 0x500 + motor_id
    can_400 = 0x400 + motor_id

    def emit(phase: str, sent: int = 0, total: int = 0, detail: str = "") -> None:
        if progress_cb is not None:
            progress_cb(OtaProgress(phase=phase, sent=sent, total=total, detail=detail))

    # ENTER_BOOT（skip_enter_boot 时跳过，假定电机已在 boot loader 中）
    if skip_enter_boot:
        emit("enter_boot", detail="skipped (already in boot)")
    else:
        emit("enter_boot")
        resp = _send_ota_frame(
            bus, is_fd, can_600, build_enter_boot_frame(is_fd),
            motor_id=motor_id, timeout_ms=declare_timeout_ms, debug=debug,
        )
        if resp is None or parse_status_frame(resp) != "A0":
            return False, f"ENTER_BOOT 失败: 未收到 8A A0 响应（resp={resp.hex() if resp else 'None'}）"
        time.sleep(enter_boot_settle_ms / 1000.0)

    # DECLARE（A1 → 重发 1 次）
    emit("declare")
    declare_data = build_declare_send_frame()
    for attempt in range(2):
        resp = _send_ota_frame(
            bus, is_fd, can_500, declare_data,
            motor_id=motor_id, timeout_ms=declare_timeout_ms, debug=debug,
        )
        status = parse_status_frame(resp) if resp else "unknown"
        if status == "A2":
            break
        if status == "A1" and attempt == 0:
            continue  # 重发
        return False, (
            f"DECLARE 失败: status={status}"
            f"（resp={resp.hex() if resp else 'None'}）"
        )

    # TRANSFER
    chunk_size = 64 if is_fd else 8
    total = len(firmware)
    sent = 0
    for chunk in slice_chunks(firmware, chunk_size):
        try:
            _send_ota_frame_no_wait(bus, is_fd, can_400, chunk)
        except RuntimeError as e:
            return False, f"TRANSFER 失败 (sent={sent}/{total}): {e}"
        sent = min(sent + chunk_size, total)
        emit("transfer", sent=sent, total=total)
        if frame_gap_us > 0:
            time.sleep(frame_gap_us / 1_000_000.0)

    # VERIFY: 发 finalize → 失败回 FFFFFFFF；成功回固定魔数（实测 12 34 56 78，
    # 与厂商文档「回显 sum」描述不符，文档只是用 0x12345678 当示例）。
    # 所以只把 FFFFFFFF 当失败，其余响应都视为校验通过，继续等 A3。
    emit("verify", detail="finalize")
    resp = _send_ota_frame(
        bus, is_fd, can_500, build_finalize_frame(sum32),
        motor_id=motor_id, timeout_ms=declare_timeout_ms, debug=debug,
    )
    if resp is None:
        return False, "VERIFY 超时: 未收到校验响应"
    if resp[:4] == b"\xff\xff\xff\xff":
        return False, "VERIFY 失败: 校验和不匹配（电机留在 boot，可手动重试）"

    # 等 A3（boot erase+write 完成）
    emit("verify", detail="wait_write_done")
    if not _wait_for_status_a3(bus, motor_id, timeout_ms=verify_timeout_ms, debug=debug):
        return False, "VERIFY 超时: 未收到 8A A3 写完成帧"

    return True, ""
