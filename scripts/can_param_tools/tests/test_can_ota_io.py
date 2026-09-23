import subprocess
from pathlib import Path

import pytest

import can_ota


def test_format_ota_frame_cc():
    s = can_ota._format_ota_frame(0x601, is_fd=False, data=bytes([0x67, 0x04]))
    assert s == "601#6704"


def test_format_ota_frame_fd():
    s = can_ota._format_ota_frame(0x501, is_fd=True, data=bytes([0xAA, 0xBB]))
    assert s == "501##1AABB"


class TestPickResponse:
    """boot loader 应答走裸 motor_id；TX 回环按 (tx_can_id + data) 过滤。"""

    def test_picks_response_on_bare_motor_id(self):
        # 实测 bcan2 上的 ENTER_BOOT 响应：rx_id=0x002 而非 0x602
        lines = [
            "(1779347935.252829)  bcan2  602   [8]  67 04 00 00 00 00 00 76",
            "(1779347935.390529)  bcan2  002   [8]  8A A0 00 00 00 00 00 A8",
        ]
        resp = can_ota._pick_response(
            lines,
            tx_can_id=0x602,
            tx_data=bytes.fromhex("6704000000000076"),
            motor_id=0x002,
        )
        assert resp == bytes.fromhex("8AA00000000000A8")

    def test_filters_loopback(self):
        # 只有 loopback 帧，没有真响应 → None
        lines = [
            "(1.0)  bcan0  601   [8]  67 04 00 00 00 00 00 76",
        ]
        resp = can_ota._pick_response(
            lines,
            tx_can_id=0x601,
            tx_data=bytes.fromhex("6704000000000076"),
            motor_id=0x001,
        )
        assert resp is None

    def test_ignores_other_motors(self):
        # 总线上有别的 motor 在回帧 → 不要混进来
        lines = [
            "(1.0)  bcan0  005   [8]  8A A0 00 00 00 00 00 A8",
            "(1.1)  bcan0  002   [8]  8A A2 00 00 00 00 00 A8",
        ]
        resp = can_ota._pick_response(
            lines,
            tx_can_id=0x502,
            tx_data=b"\x00" * 7 + b"\xEF",
            motor_id=0x002,
        )
        assert resp == bytes.fromhex("8AA20000000000A8")

    def test_picks_response_on_0x500_plus_id(self):
        # 实测 bcan3 上的 FINALIZE 响应：rx_id=0x506（即 0x500+0x06）
        lines = [
            "(1779349203.336156)  bcan3  506   [8]  00 00 9C 62 84 00 00 FE",
            "(1779349203.336320)  bcan3  506   [4]  FF FF FF FF",
        ]
        resp = can_ota._pick_response(
            lines,
            tx_can_id=0x506,
            tx_data=bytes.fromhex("0000 9C62 8400 00FE".replace(" ", "")),
            motor_id=0x006,
        )
        assert resp == bytes.fromhex("FFFFFFFF")
