#!/usr/bin/env python3
import os
import sys
import unittest

import numpy as np


sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from depth_pointcloud import (  # noqa: E402
    PinholeIntrinsics,
    backproject_policy,
    backproject_raw_mm,
)


class DepthPointcloudColormapTest(unittest.TestCase):
    def setUp(self):
        self.intrinsics = PinholeIntrinsics.from_fov(4, 3)

    def assert_gray(self, colors):
        self.assertGreater(colors.shape[0], 0)
        np.testing.assert_array_equal(colors[:, 0], colors[:, 1])
        np.testing.assert_array_equal(colors[:, 1], colors[:, 2])

    def test_raw_pointcloud_accepts_gray_colormap(self):
        _, colors = backproject_raw_mm(
            np.full((3, 4), 1000.0), self.intrinsics, step=1, colormap="gray"
        )
        self.assert_gray(colors)

    def test_policy_pointcloud_accepts_gray_colormap(self):
        _, colors = backproject_policy(
            np.full((36, 64), 0.4, dtype=np.float32),
            self.intrinsics,
            input_width=4,
            input_height=3,
            colormap="gray",
        )
        self.assert_gray(colors)


if __name__ == "__main__":
    unittest.main()
