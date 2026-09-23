from pathlib import Path

import pytest

from can_ota import (
    compute_sum32,
    compute_wire_sum32,
    build_enter_boot_frame,
    build_declare_send_frame,
    build_finalize_frame,
)


class TestComputeSum32:
    def test_empty(self):
        assert compute_sum32(b"") == 0

    def test_single_byte(self):
        assert compute_sum32(b"\x01") == 1

    def test_multi_byte(self):
        assert compute_sum32(b"\xff" * 5) == 5 * 0xFF

    def test_typical_aggregate(self):
        # 16 字节累加：mask 不会被触发，仅验证常规路径
        # （实际 wrap 需要 ~4GB 数据触发，无法在单测中触发；
        # 见 docs/plans/2026-05-07-can-ota-design.md §9 的注记）
        data = b"\x80\x80\x80\x80" * 4  # sum = 0x80*16 = 0x800
        assert compute_sum32(data) == 0x800

    def test_within_uint32_bound(self):
        """对足够大的输入，结果仍落在 uint32 范围内。"""
        result = compute_sum32(b"\xff" * 1024)
        assert 0 <= result <= 0xFFFFFFFF
        assert result == 0xff * 1024

    def test_known_pattern(self):
        # 'A'=0x41, 'B'=0x42, 'C'=0x43 → 0xC6
        assert compute_sum32(b"ABC") == 0xC6


class TestComputeWireSum32:
    """wire sum = sum(firmware) + 0xFF * padding 字节数。"""

    def test_no_padding_when_aligned(self):
        fw = b"\x01" * 64  # 64 字节正好 1 帧 FD
        assert compute_wire_sum32(fw, 64) == sum(fw)

    def test_cc_padding_4_bytes(self):
        # 实测：80988 字节 firmware，CC 8B chunk → padding 4B 0xFF
        # raw=0x0084629C, wire=0x0084629C + 4*0xFF = 0x00846698
        raw = 0x0084629C
        pad_contribution = 4 * 0xFF
        # 构造一段任意长度数据让 padding=4
        fw = b"\x42" * 4  # 4 字节 → CC 8B → padding 4 字节
        assert compute_wire_sum32(fw, 8) == sum(fw) + pad_contribution
        # 形式上验证算法
        assert (sum(fw) + pad_contribution) & 0xFFFFFFFF == compute_wire_sum32(fw, 8)
        # raw 是单独的检查路径
        del raw

    def test_fd_padding_62_bytes(self):
        fw = b"\xAA" * 130  # 130 字节 → 2*64 + 2 字节 → padding 62 字节
        wire = compute_wire_sum32(fw, 64)
        assert wire == sum(fw) + 62 * 0xFF

    def test_rejects_bad_chunk_size(self):
        with pytest.raises(ValueError):
            compute_wire_sum32(b"abc", 0)


class TestFrames:
    def test_enter_boot_cc(self):
        # classic CAN spec (OTA 普通CAN升级流程): Byte2..5 = 00 00 00 00
        assert build_enter_boot_frame(is_fd=False) == bytes(
            [0x67, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x76]
        )

    def test_enter_boot_fd(self):
        # CANFD spec §6.5.1: Byte2..5 = A0 A1 A2 A3
        assert build_enter_boot_frame(is_fd=True) == bytes(
            [0x67, 0x04, 0xA0, 0xA1, 0xA2, 0xA3, 0x00, 0x76]
        )

    def test_declare_send(self):
        f = build_declare_send_frame()
        assert len(f) == 8
        assert f[7] == 0xEF

    def test_finalize_be_byte_order(self):
        # spec 6.5.1: sum=0x12345678 高端在前 → "00 00 12 34 56 78 00 FE"
        assert build_finalize_frame(0x12345678) == bytes(
            [0x00, 0x00, 0x12, 0x34, 0x56, 0x78, 0x00, 0xFE]
        )

    def test_finalize_zero(self):
        assert build_finalize_frame(0) == bytes(
            [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE]
        )

    def test_finalize_max(self):
        assert build_finalize_frame(0xFFFFFFFF) == bytes(
            [0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFE]
        )

    def test_finalize_rejects_out_of_range(self):
        with pytest.raises(ValueError):
            build_finalize_frame(0x1_0000_0000)
        with pytest.raises(ValueError):
            build_finalize_frame(-1)


from can_ota import parse_status_frame


class TestParseStatusFrame:
    def test_a0_enter_boot_ack(self):
        assert parse_status_frame(bytes([0x8A, 0xA0, 0, 0, 0, 0, 0, 0xA8])) == "A0"

    def test_a1_declare_fail(self):
        assert parse_status_frame(bytes([0x8A, 0xA1, 0, 0, 0, 0, 0, 0xA8])) == "A1"

    def test_a2_declare_ok(self):
        assert parse_status_frame(bytes([0x8A, 0xA2, 0, 0, 0, 0, 0, 0xA8])) == "A2"

    def test_a3_write_done(self):
        assert parse_status_frame(bytes([0x8A, 0xA3, 0, 0, 0, 0, 0, 0xA8])) == "A3"

    def test_unknown_head(self):
        assert parse_status_frame(bytes([0x00, 0xA0, 0, 0, 0, 0, 0, 0xA8])) == "unknown"

    def test_unknown_tail(self):
        assert parse_status_frame(bytes([0x8A, 0xA0, 0, 0, 0, 0, 0, 0x00])) == "unknown"

    def test_unknown_status_byte(self):
        assert parse_status_frame(bytes([0x8A, 0x55, 0, 0, 0, 0, 0, 0xA8])) == "unknown"

    def test_short_frame(self):
        assert parse_status_frame(bytes([0x8A, 0xA0])) == "unknown"

    def test_ignores_middle_bytes(self):
        # 中间 6 字节应被忽略
        assert parse_status_frame(
            bytes([0x8A, 0xA2, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xA8])
        ) == "A2"


from can_ota import load_firmware_bin


class TestLoadFirmwareBin:
    def test_loads_bin_file(self, tmp_path: Path):
        p = tmp_path / "fw.bin"
        p.write_bytes(b"\xde\xad\xbe\xef")
        assert load_firmware_bin(p) == b"\xde\xad\xbe\xef"

    def test_loads_bin_uppercase_suffix(self, tmp_path: Path):
        p = tmp_path / "FW.BIN"
        p.write_bytes(b"\x01\x02")
        assert load_firmware_bin(p) == b"\x01\x02"

    def test_rejects_hex_with_objcopy_hint(self, tmp_path: Path):
        p = tmp_path / "fw.hex"
        p.write_text(":00000001FF\n")
        with pytest.raises(ValueError, match="objcopy"):
            load_firmware_bin(p)

    def test_rejects_other_suffix(self, tmp_path: Path):
        p = tmp_path / "fw.txt"
        p.write_bytes(b"x")
        with pytest.raises(ValueError):
            load_firmware_bin(p)

    def test_missing_file(self, tmp_path: Path):
        p = tmp_path / "nope.bin"
        with pytest.raises(FileNotFoundError):
            load_firmware_bin(p)


from can_ota import OtaProgress, slice_chunks


class TestSliceChunks:
    def test_cc_8byte_exact(self):
        # 16 字节 → 2 帧 ×8B
        chunks = list(slice_chunks(bytes(range(16)), chunk_size=8))
        assert len(chunks) == 2
        assert chunks[0] == bytes(range(8))
        assert chunks[1] == bytes(range(8, 16))

    def test_cc_8byte_padded(self):
        # 5 字节 → 1 帧 ×8B，末尾补 3 个 0xFF
        chunks = list(slice_chunks(b"\x01\x02\x03\x04\x05", chunk_size=8))
        assert chunks == [bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0xFF, 0xFF, 0xFF])]

    def test_fd_64byte_padded(self):
        chunks = list(slice_chunks(b"\xAA" * 65, chunk_size=64))
        assert len(chunks) == 2
        assert chunks[0] == b"\xAA" * 64
        assert chunks[1] == b"\xAA" + b"\xFF" * 63

    def test_empty_input(self):
        assert list(slice_chunks(b"", chunk_size=8)) == []


class TestOtaProgress:
    def test_default_fields(self):
        p = OtaProgress(phase="enter_boot")
        assert p.phase == "enter_boot"
        assert p.sent == 0
        assert p.total == 0
        assert p.detail == ""
