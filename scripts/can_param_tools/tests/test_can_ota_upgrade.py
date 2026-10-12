from __future__ import annotations

from typing import Optional

import pytest

import can_ota
from can_ota import OtaProgress, upgrade_motor


def make_send_stub(responses: list[Optional[bytes]]):
    """按调用顺序返回预设响应；用尽后抛错（露出多余调用）。"""
    calls: list[tuple[int, bytes]] = []
    queue = list(responses)

    def stub(bus, is_fd, can_id, data, *, motor_id, timeout_ms=500, debug=False):
        calls.append((can_id, data, motor_id))
        if not queue:
            raise AssertionError(f"未预期的 _send_ota_frame 调用: id={can_id:#x} data={data.hex()}")
        return queue.pop(0)

    return stub, calls


def make_no_wait_stub():
    calls: list[tuple[int, bytes]] = []

    def stub(bus, is_fd, can_id, data):
        calls.append((can_id, data))

    return stub, calls


@pytest.fixture
def patch_io(monkeypatch):
    """安装 stub 并返回收集到的调用记录。"""
    state: dict = {}

    def setup(send_responses, wait_a3=True, motor_id=1):
        send_stub, send_calls = make_send_stub(send_responses)
        nowait_stub, nowait_calls = make_no_wait_stub()
        monkeypatch.setattr(can_ota, "_send_ota_frame", send_stub)
        monkeypatch.setattr(can_ota, "_send_ota_frame_no_wait", nowait_stub)
        monkeypatch.setattr(can_ota, "_wait_for_status_a3",
                            lambda bus, motor_id, *, timeout_ms, debug=False: wait_a3)
        state["send"] = send_calls
        state["nowait"] = nowait_calls
        return state

    return setup


def _ack(byte1: int) -> bytes:
    """8A xx 00 00 00 00 00 A8."""
    return bytes([0x8A, byte1, 0, 0, 0, 0, 0, 0xA8])


def _be_echo(sum32: int) -> bytes:
    """4 字节 BE sum，电机校验成功的响应。"""
    return sum32.to_bytes(4, "big")


class TestEnterBootDataPerBus:
    """ENTER_BOOT 数据按 is_fd 切换：两份厂商文档定义不同。"""

    def test_cc_uses_zero_middle_bytes(self, patch_io):
        state = patch_io(send_responses=[None])
        upgrade_motor(
            bus="vcan0", is_fd=False, motor_id=1,
            firmware=b"\x00" * 8, sum32=0, enter_boot_settle_ms=0,
        )
        enter_boot_data = state["send"][0][1]
        assert enter_boot_data == bytes(
            [0x67, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x76]
        )

    def test_fd_uses_a0_a1_a2_a3(self, patch_io):
        state = patch_io(send_responses=[None])
        upgrade_motor(
            bus="vcan0", is_fd=True, motor_id=1,
            firmware=b"\x00" * 64, sum32=0, enter_boot_settle_ms=0,
        )
        enter_boot_data = state["send"][0][1]
        assert enter_boot_data == bytes(
            [0x67, 0x04, 0xA0, 0xA1, 0xA2, 0xA3, 0x00, 0x76]
        )


class TestUpgradeMotorHappyPath:
    def test_cc_bus_8byte_firmware(self, patch_io):
        """8 字节固件、classic CAN：每阶段都按预期成功。"""
        fw = bytes(range(1, 9))  # 8 字节正好一帧 CC
        sum32 = sum(fw)

        state = patch_io(
            send_responses=[
                _ack(0xA0), _ack(0xA2), _be_echo(sum32),
            ],
        )

        ok, msg = upgrade_motor(
            bus="vcan0", is_fd=False, motor_id=1,
            firmware=fw, sum32=sum32, enter_boot_settle_ms=0,
        )
        assert ok, msg

        ids = [c[0] for c in state["send"]]
        assert ids == [0x601, 0x501, 0x501]

        nowait_ids = [c[0] for c in state["nowait"]]
        assert nowait_ids == [0x401]
        assert state["nowait"][0][1] == fw

    def test_fd_bus_64byte_firmware(self, patch_io):
        """64 字节固件、CANFD：每阶段都按预期成功。"""
        fw = bytes(range(1, 65))
        sum32 = sum(fw)

        state = patch_io(
            send_responses=[
                _ack(0xA0), _ack(0xA2), _be_echo(sum32),
            ],
        )

        events: list[OtaProgress] = []
        ok, msg = upgrade_motor(
            bus="vcan0", is_fd=True, motor_id=1,
            firmware=fw, sum32=sum32,
            progress_cb=lambda p: events.append(OtaProgress(**vars(p))),
            enter_boot_settle_ms=0,
        )
        assert ok, msg

        nowait_ids = [c[0] for c in state["nowait"]]
        assert nowait_ids == [0x401]
        assert state["nowait"][0][1] == fw

        phases = [e.phase for e in events]
        assert "enter_boot" in phases
        assert "declare" in phases
        assert "transfer" in phases
        assert "verify" in phases


class TestUpgradeMotorFailures:
    def test_enter_boot_timeout(self, patch_io):
        patch_io(send_responses=[None])
        ok, msg = upgrade_motor(
            bus="vcan0", is_fd=True, motor_id=1,
            firmware=b"\x00" * 64, sum32=0, enter_boot_settle_ms=0,
        )
        assert not ok
        assert "ENTER_BOOT" in msg

    def test_enter_boot_wrong_status(self, patch_io):
        patch_io(send_responses=[_ack(0xA1)])
        ok, msg = upgrade_motor(
            bus="vcan0", is_fd=True, motor_id=1,
            firmware=b"\x00" * 64, sum32=0, enter_boot_settle_ms=0,
        )
        assert not ok
        assert "ENTER_BOOT" in msg

    def test_declare_a1_then_a2_succeeds(self, patch_io):
        fw = b"\x42" * 64
        sum32 = sum(fw)
        patch_io(send_responses=[
            _ack(0xA0), _ack(0xA1), _ack(0xA2), _be_echo(sum32),
        ])
        ok, msg = upgrade_motor(
            bus="vcan0", is_fd=True, motor_id=1,
            firmware=fw, sum32=sum32, enter_boot_settle_ms=0,
        )
        assert ok, msg

    def test_declare_a1_twice_fails(self, patch_io):
        patch_io(send_responses=[
            _ack(0xA0), _ack(0xA1), _ack(0xA1),
        ])
        ok, msg = upgrade_motor(
            bus="vcan0", is_fd=True, motor_id=1,
            firmware=b"\x00" * 64, sum32=0, enter_boot_settle_ms=0,
        )
        assert not ok
        assert "DECLARE" in msg

    def test_verify_returns_ffffffff(self, patch_io):
        fw = b"\x55" * 64
        patch_io(send_responses=[
            _ack(0xA0), _ack(0xA2), b"\xff\xff\xff\xff",
        ])
        ok, msg = upgrade_motor(
            bus="vcan0", is_fd=True, motor_id=1,
            firmware=fw, sum32=sum(fw), enter_boot_settle_ms=0,
        )
        assert not ok
        assert "VERIFY" in msg and "校验和不匹配" in msg

    def test_verify_a3_timeout(self, patch_io):
        fw = b"\x33" * 64
        sum32 = sum(fw)
        patch_io(
            send_responses=[
                _ack(0xA0), _ack(0xA2), _be_echo(sum32),
            ],
            wait_a3=False,
        )
        ok, msg = upgrade_motor(
            bus="vcan0", is_fd=True, motor_id=1,
            firmware=fw, sum32=sum32, enter_boot_settle_ms=0,
        )
        assert not ok
        assert "8A A3" in msg

    def test_verify_accepts_observed_magic_response(self, patch_io):
        """实测电机校验通过返回固定魔数 12 34 56 78，不是真的 sum 回显。
        除 FFFFFFFF 外都视为通过。
        """
        fw = b"\xAA" * 64
        sum32 = sum(fw)
        patch_io(send_responses=[
            _ack(0xA0), _ack(0xA2),
            bytes.fromhex("12345678"),  # 实测魔数
        ])
        ok, msg = upgrade_motor(
            bus="vcan0", is_fd=True, motor_id=1,
            firmware=fw, sum32=sum32, enter_boot_settle_ms=0,
        )
        assert ok, msg


class TestUpgradeMotorChunking:
    def test_cc_chunks_8_bytes(self, patch_io):
        fw = b"".join(bytes([i]) * 8 for i in range(3))  # 24 字节 → 3 帧 ×8
        sum32 = sum(fw)
        state = patch_io(send_responses=[
            _ack(0xA0), _ack(0xA2), _be_echo(sum32),
        ])
        ok, _ = upgrade_motor(
            bus="vcan0", is_fd=False, motor_id=1,
            firmware=fw, sum32=sum32, enter_boot_settle_ms=0,
        )
        assert ok
        nowait = state["nowait"]
        assert len(nowait) == 3
        for can_id, data in nowait:
            assert can_id == 0x401
            assert len(data) == 8

    def test_fd_chunks_64_bytes(self, patch_io):
        fw = b"\xAA" * 130  # 130 字节 → 3 帧 (64+64+ pad 62)
        sum32 = sum(fw)
        state = patch_io(send_responses=[
            _ack(0xA0), _ack(0xA2), _be_echo(sum32),
        ])
        ok, _ = upgrade_motor(
            bus="vcan0", is_fd=True, motor_id=2,
            firmware=fw, sum32=sum32, enter_boot_settle_ms=0,
        )
        assert ok
        nowait = state["nowait"]
        assert len(nowait) == 3
        for can_id, data in nowait:
            assert can_id == 0x402
            assert len(data) == 64
        last_data = nowait[-1][1]
        assert last_data[:2] == b"\xAA\xAA"
        assert last_data[2:] == b"\xFF" * 62
