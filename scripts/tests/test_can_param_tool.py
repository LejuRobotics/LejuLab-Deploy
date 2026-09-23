"""Unit tests for can_param_tool.py pure functions."""
import sys
import unittest
from pathlib import Path

# 让测试能导入 scripts/can_param_tool.py
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


class ScaffoldTest(unittest.TestCase):
    def test_import(self):
        import can_param_tool  # noqa: F401


from can_param_tool import PARAM_TABLE, Tier


class ParamTableTest(unittest.TestCase):
    def test_covers_index_10_to_67(self):
        indices = sorted(PARAM_TABLE.keys())
        self.assertEqual(indices[0], 10)
        self.assertEqual(indices[-1], 67)
        # 连续区间，允许中间无跳号
        self.assertEqual(len(indices), 67 - 10 + 1)

    def test_firmware_version_is_readonly(self):
        self.assertEqual(PARAM_TABLE[10].tier, Tier.RO)

    def test_angle_offset_is_expert(self):
        self.assertEqual(PARAM_TABLE[22].tier, Tier.EXPERT)
        self.assertEqual(PARAM_TABLE[23].tier, Tier.EXPERT)

    def test_id_kp_is_safe(self):
        self.assertEqual(PARAM_TABLE[12].name, "Id Controller Kp")
        self.assertEqual(PARAM_TABLE[12].dtype, "float")
        self.assertEqual(PARAM_TABLE[12].tier, Tier.SAFE)

    def test_motor_id_is_expert(self):
        self.assertEqual(PARAM_TABLE[36].tier, Tier.EXPERT)
        self.assertEqual(PARAM_TABLE[36].dtype, "uint32")

    def test_all_params_have_name_and_dtype(self):
        for idx, entry in PARAM_TABLE.items():
            self.assertTrue(entry.name, f"index {idx} missing name")
            self.assertIn(entry.dtype, ("float", "uint32", "hex32", "int32"), f"index {idx} bad dtype")


from can_param_tool import build_read_frame, build_write_frame, SAVE_FRAME


class FrameEncodingTest(unittest.TestCase):
    def test_read_frame_id_kp_idx_12(self):
        """PDF 示例: 读 ID=5 电机的 Id_Kp (idx=12), 数据段 67 0C 00 00 00 00 04 76"""
        self.assertEqual(
            build_read_frame(12).hex().upper(),
            "670C000000000476",
        )

    def test_write_frame_0p1_float_to_id_kp(self):
        """PDF 示例: 写 0.1 到 idx=12, 数据段 67 0C CD CC CC 3D 15 76"""
        self.assertEqual(
            build_write_frame(12, 0.1, "float").hex().upper(),
            "670CCDCCCC3D1576",
        )

    def test_write_frame_uint32(self):
        """写 uint32: idx=44 (NPP), value=14"""
        frame = build_write_frame(44, 14, "uint32")
        # 67 2C 0E 00 00 00 15 76
        self.assertEqual(frame.hex().upper(), "672C0E0000001576")

    def test_save_frame(self):
        self.assertEqual(SAVE_FRAME.hex().upper(), "6700000000000476")

    def test_write_frame_rejects_bad_dtype(self):
        with self.assertRaises(ValueError):
            build_write_frame(12, 0.1, "int8")

    def test_frames_are_always_8_bytes(self):
        self.assertEqual(len(build_read_frame(10)), 8)
        self.assertEqual(len(build_write_frame(12, 0.1, "float")), 8)
        self.assertEqual(len(SAVE_FRAME), 8)


from can_param_tool import decode_response


class DecodeResponseTest(unittest.TestCase):
    def test_decode_float_0p1(self):
        """PDF 示例的反向: CD CC CC 3D -> 0.1"""
        data = bytes.fromhex("01 0C CDCCCC3D 0476".replace(" ", ""))
        self.assertAlmostEqual(decode_response(data, "float"), 0.1, places=5)

    def test_decode_uint32(self):
        # payload = 14 -> 0E 00 00 00
        data = bytes.fromhex("01 2C 0E000000 0476".replace(" ", ""))
        self.assertEqual(decode_response(data, "uint32"), 14)

    def test_decode_rejects_wrong_length(self):
        with self.assertRaises(ValueError):
            decode_response(bytes.fromhex("01 2C 0E00"), "uint32")


from can_param_tool import parse_candump_line


class ParseCandumpTest(unittest.TestCase):
    def test_standard_can_line(self):
        """标准 CAN 8 字节: bcan2  601   [8]  67 0C 00 00 00 00 04 76"""
        line = "  bcan2  601   [8]  67 0C 00 00 00 00 04 76"
        can_id, data = parse_candump_line(line)
        self.assertEqual(can_id, 0x601)
        self.assertEqual(data, bytes.fromhex("670C000000000476"))

    def test_canfd_line_short(self):
        """CANFD 帧(8字节数据也可能出现): bcan0  601  [08]  01 0C CD CC CC 3D 04 76"""
        line = "  bcan0  601  [08]  01 0C CD CC CC 3D 04 76"
        can_id, data = parse_candump_line(line)
        self.assertEqual(can_id, 0x601)
        self.assertEqual(data, bytes.fromhex("010CCDCCCC3D0476"))

    def test_line_with_timestamp(self):
        """candump -t a: (1712345678.123456) bcan0 601 [8] 01 0C ..."""
        line = " (1712345678.123456)  bcan0  601   [8]  01 0C 00 00 00 00 00 00"
        can_id, data = parse_candump_line(line)
        self.assertEqual(can_id, 0x601)

    def test_empty_line_returns_none(self):
        self.assertIsNone(parse_candump_line(""))
        self.assertIsNone(parse_candump_line("   "))

    def test_malformed_line_returns_none(self):
        self.assertIsNone(parse_candump_line("garbage"))


from can_param_tool import parse_ip_link_brief, parse_ip_link_details


class IpLinkParseTest(unittest.TestCase):
    # 样本: `ip link show` 简短格式
    BRIEF_OUTPUT = """\
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN mode DEFAULT
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
2: eth0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc mq state UP mode DEFAULT
    link/ether aa:bb:cc:dd:ee:ff brd ff:ff:ff:ff:ff:ff
3: bcan0: <NOARP,UP,LOWER_UP,ECHO> mtu 72 qdisc pfifo_fast state UP mode DEFAULT
    link/can
4: bcan1: <NOARP,UP,LOWER_UP,ECHO> mtu 72 qdisc pfifo_fast state UP mode DEFAULT
    link/can
5: bcan2: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP mode DEFAULT
    link/can
6: bcan3: <NOARP> mtu 16 qdisc noop state DOWN mode DEFAULT
    link/can
"""

    def test_list_up_bcan_buses(self):
        up = parse_ip_link_brief(self.BRIEF_OUTPUT)
        self.assertEqual(up, ["bcan0", "bcan1", "bcan2"])

    # 样本: `ip -details link show bcan0`
    DETAILS_FD = """\
3: bcan0: <NOARP,UP,LOWER_UP,ECHO> mtu 72 qdisc pfifo_fast state UP mode DEFAULT group default qlen 1000
    link/can  promiscuity 0 allmulti 0 minmtu 0 maxmtu 0
    can state ERROR-ACTIVE restart-ms 100
      bitrate 1000000 sample-point 0.800
      tq 50 prop-seg 7 phase-seg1 12 phase-seg2 5 sjw 1 brp 1
      dbitrate 5000000 dsample-point 0.750
      dtq 20 dprop-seg 4 dphase-seg1 5 dphase-seg2 5 dsjw 1 dbrp 1
      tcan4x5x: tseg1 2..256 tseg2 2..128 sjw 1..128 brp 1..512 brp_inc 1
      tcan4x5x: dtseg1 1..32 dtseg2 1..16 dsjw 1..16 dbrp 1..32 dbrp_inc 1
      clock 40000000
      re-started bus-errors arbit-lost error-warn error-pass bus-off
      0          0          0          0          0          0
"""

    DETAILS_NON_FD = """\
5: bcan2: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP mode DEFAULT group default qlen 1000
    link/can  promiscuity 0 allmulti 0 minmtu 0 maxmtu 0
    can state ERROR-ACTIVE restart-ms 100
      bitrate 1000000 sample-point 0.750
      tq 62 prop-seg 6 phase-seg1 9 phase-seg2 0 sjw 1 brp 1
      tcan4x5x: tseg1 2..256 tseg2 2..128 sjw 1..128 brp 1..512 brp_inc 1
      clock 40000000
"""

    def test_fd_detection_true(self):
        self.assertTrue(parse_ip_link_details(self.DETAILS_FD))

    def test_fd_detection_false(self):
        self.assertFalse(parse_ip_link_details(self.DETAILS_NON_FD))


import tempfile as _tempfile
from can_param_tool import OpLogger


class OpLoggerTest(unittest.TestCase):
    def test_log_write_op(self):
        with _tempfile.TemporaryDirectory() as d:
            logger = OpLogger(Path(d))
            logger.log_write(bus="bcan2", motor_id=1, idx=12, name="Id Kp",
                             old=0.907, new=0.800, readback=0.800, ok=True)
            logger.close()
            files = list(Path(d).glob("can_param_*.log"))
            self.assertEqual(len(files), 1)
            content = files[0].read_text()
            self.assertIn("bus=bcan2", content)
            self.assertIn("idx=12", content)
            self.assertIn("old=0.907", content)
            self.assertIn("new=0.8", content)
            self.assertIn("result=ok", content)

    def test_noop_logger_writes_nothing(self):
        with _tempfile.TemporaryDirectory() as d:
            logger = OpLogger(Path(d), enabled=False)
            logger.log_write(bus="bcan2", motor_id=1, idx=12, name="x",
                             old=0, new=1, readback=1, ok=True)
            logger.close()
            self.assertEqual(list(Path(d).glob("can_param_*.log")), [])


from can_param_tool import _writable_indices


class WritableIndicesTest(unittest.TestCase):
    def test_normal_mode_excludes_ro_and_expert(self):
        normal = _writable_indices(expert=False)
        self.assertNotIn(10, normal, "RO (firmware version) should be excluded")
        self.assertNotIn(22, normal, "EXPERT (angle offset) should be excluded")
        self.assertNotIn(36, normal, "EXPERT (motor id) should be excluded")
        self.assertIn(12, normal, "SAFE (Id Kp) should be included")
        self.assertIn(38, normal, "SAFE (default pos Kp) should be included")

    def test_expert_mode_excludes_only_ro(self):
        expert = _writable_indices(expert=True)
        self.assertNotIn(10, expert, "RO (firmware version) should still be excluded")
        self.assertIn(22, expert, "EXPERT (angle offset) should be included")
        self.assertIn(56, expert, "EXPERT (protect switchs) should be included")
        self.assertIn(12, expert, "SAFE still included")

    def test_expert_is_superset_of_normal(self):
        normal = set(_writable_indices(expert=False))
        expert = set(_writable_indices(expert=True))
        self.assertTrue(normal.issubset(expert))

    def test_writable_indices_are_sorted(self):
        for expert in (False, True):
            indices = _writable_indices(expert=expert)
            self.assertEqual(indices, sorted(indices))


if __name__ == "__main__":
    unittest.main()
