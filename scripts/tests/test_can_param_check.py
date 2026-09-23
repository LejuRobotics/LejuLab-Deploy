"""Unit tests for can_param_check.py parameter selection."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "can_param_tools"))

from can_param_check import CORE_IDXS, MATCH, MISMATCH, SKIPPED, UNREADABLE, classify, compare
from can_param_tool import decode_response


class CoreParameterTest(unittest.TestCase):
    def test_can_master_is_a_core_writable_parameter(self):
        self.assertIn(67, CORE_IDXS)


class FirmwareListTest(unittest.TestCase):
    FW_LIST = ["0xFB268241", "0xFB268242"]

    def test_either_listed_firmware_matches(self):
        self.assertEqual(classify(10, self.FW_LIST, "0xFB268241")[0], MATCH)
        self.assertEqual(classify(10, self.FW_LIST, "0xFB268242")[0], MATCH)

    def test_unlisted_firmware_mismatches(self):
        self.assertEqual(classify(10, self.FW_LIST, "0xFB268243")[0], MISMATCH)

    def test_unreadable_firmware_is_not_mismatch(self):
        self.assertEqual(classify(10, self.FW_LIST, None)[0], UNREADABLE)

    def test_single_firmware_still_exact(self):
        self.assertEqual(classify(10, "0xFB268242", "0xFB268241")[0], MISMATCH)


class IndividualParameterTest(unittest.TestCase):
    BASE = {
        "bcan0_M1": {
            "11 (Control Mode)": 2,
            "22 (Electric Angle Offset)": 2.689622,
            "23 (Machine Angle Offset)": 3.969559,
            "36 (Motor ID)": 1,
        }
    }

    def _compare(self, **overrides):
        current = {"bcan0_M1": dict(self.BASE["bcan0_M1"], **overrides)}
        results, _, _ = compare(self.BASE, current)
        return results[0]

    def test_individual_calibration_and_motor_id_are_not_compared(self):
        mr = self._compare(**{
            "22 (Electric Angle Offset)": 1.0,
            "23 (Machine Angle Offset)": 2.0,
            "36 (Motor ID)": 5,
        })
        self.assertTrue(mr.passed)
        kinds = {p.idx: p.kind for p in mr.params}
        self.assertEqual(kinds[11], MATCH)
        for idx in (22, 23, 36):
            self.assertEqual(kinds[idx], SKIPPED)
        self.assertEqual(mr.unreadables, [])
        skipped = {p.idx: p.actual for p in mr.params if p.kind == SKIPPED}
        self.assertEqual(skipped, {22: 1.0, 23: 2.0, 36: 5})

    def test_zero_electric_angle_offset_is_flagged(self):
        mr = self._compare(**{"22 (Electric Angle Offset)": 0.0})
        self.assertTrue(mr.elec_angle_zero)
        self.assertTrue(mr.passed)

    def test_unreadable_electric_angle_offset_is_flagged(self):
        mr = self._compare(**{"22 (Electric Angle Offset)": None})
        self.assertTrue(mr.elec_angle_unreadable)
        self.assertFalse(mr.elec_angle_zero)
        self.assertTrue(mr.passed)
        self.assertFalse(self._compare().elec_angle_unreadable)

    def test_nonzero_or_unreadable_electric_angle_offset_is_not_flagged(self):
        self.assertFalse(self._compare().elec_angle_zero)
        self.assertFalse(
            self._compare(**{"22 (Electric Angle Offset)": None}).elec_angle_zero
        )


class Int32ResponseTest(unittest.TestCase):
    def test_decode_negative_can_master_value(self):
        data = bytes.fromhex("0143FFFFFFFF0476")
        self.assertEqual(decode_response(data, "int32"), -1)


if __name__ == "__main__":
    unittest.main()
