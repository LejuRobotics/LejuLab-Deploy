#!/usr/bin/env python3

import os
import sys
import unittest

import numpy as np


SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, SCRIPTS_DIR)

from depth_processor import process_depth_mm_frame  # noqa: E402


def _quantized_frame_hash(frame):
    value = 1469598103934665603
    for pixel in frame.ravel():
        scaled = np.float32(pixel) * np.float32(100000.0)
        quantized = int(np.floor(float(scaled) + 0.5)) & 0xFFFFFFFF
        for byte_index in range(4):
            value ^= (quantized >> (byte_index * 8)) & 0xFF
            value = (value * 1099511628211) & 0xFFFFFFFFFFFFFFFF
    return value


class DepthProcessorTest(unittest.TestCase):
    def test_matches_new_controller_real_camera_pipeline(self):
        rows, columns = np.indices((240, 424))
        source = (700 + rows * 3 + columns * 2).astype(np.float64)
        source[72:112, 128:178] = 0.0
        source[150:156, 300:306] = 4200.0

        actual, valid_ratio = process_depth_mm_frame(
            source, input_width=424, input_height=240
        )

        self.assertEqual(actual.shape, (36, 64))
        self.assertEqual(actual.dtype, np.float32)
        self.assertGreater(valid_ratio, 0.0)
        self.assertLessEqual(valid_ratio, 1.0)
        self.assertAlmostEqual(float(actual[0, 0]), 0.28657764, delta=1e-5)
        self.assertAlmostEqual(float(actual[14, 23]), 0.49755901, delta=1e-5)
        self.assertAlmostEqual(float(actual[23, 46]), 0.70598102, delta=1e-5)
        self.assertAlmostEqual(float(actual[35, 63]), 0.88462240, delta=1e-5)
        self.assertEqual(_quantized_frame_hash(actual), 0x8C121454FC55FE5C)

    def test_nonstandard_size_preserves_legacy_float64_conversion(self):
        source = np.array(
            [
                [2423.82236921, 1.0, 857.40414026, 2.0],
                [1225.41961626, 3.0, 135.82558171, 4.0],
                [703.53060501, 5.0, 1479.06905620, 6.0],
            ],
            dtype=np.float64,
        )
        expected = (
            np.clip(source[np.ix_([0, 1], [0, 2])] * 0.001, 0.0, 2.5)
            / 2.5
        ).astype(np.float32)

        actual, _ = process_depth_mm_frame(
            source,
            input_width=4,
            input_height=3,
            output_width=2,
            output_height=2,
            gaussian_blur=False,
        )

        np.testing.assert_array_equal(actual, expected)

    def test_native_480x270_uses_real_camera_cleanup(self):
        source = np.full((270, 480), 1000.0, dtype=np.float64)
        source[125:146, 230:251] = 5000.0

        actual, valid_ratio = process_depth_mm_frame(
            source, input_width=480, input_height=270
        )

        self.assertEqual(actual.shape, (36, 64))
        self.assertGreater(valid_ratio, 0.0)
        self.assertAlmostEqual(float(actual[18, 32]), 0.4, delta=0.08)

    def test_all_zero_real_frame_returns_invalid_without_raising(self):
        try:
            actual, valid_ratio = process_depth_mm_frame(
                np.zeros((240, 424), dtype=np.float64),
                input_width=424,
                input_height=240,
            )
        except ValueError as error:
            self.fail(f"an invalid camera frame must be droppable: {error}")

        self.assertIsNone(actual)
        self.assertEqual(valid_ratio, 0.0)


if __name__ == "__main__":
    unittest.main()
