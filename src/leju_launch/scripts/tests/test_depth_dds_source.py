import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parents[1]))

from depth_dds_source import default_shape, resolve_type


def test_normalized_auto():
    assert resolve_type("rt/depth_camera/frame_normalized_36x64") == "float64"
    assert default_shape("rt/depth_camera/frame_normalized_36x64", "float64") == (64, 36)


def test_raw_auto_and_override():
    assert resolve_type("rt/depth_camera/frame_u16_mm_270x480") == "uint16"
    assert default_shape("rt/depth_camera/frame_u16_mm_270x480", "uint16") == (480, 270)
    assert resolve_type("custom", "float64") == "float64"
