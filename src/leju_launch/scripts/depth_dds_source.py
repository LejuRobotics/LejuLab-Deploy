"""Pure helpers shared by depth DDS inspection tools."""

from __future__ import annotations

NORMALIZED_TOPIC = "rt/depth_camera/frame_normalized_36x64"
RAW_TOPIC = "rt/depth_camera/frame_u16_mm_270x480"
POLICY_WIDTH = 64
POLICY_HEIGHT = 36
RAW_WIDTH = 480
RAW_HEIGHT = 270


def resolve_type(topic: str, requested: str = "auto") -> str:
    if requested not in ("auto", "float64", "uint16"):
        raise ValueError(f"unsupported DDS type: {requested}")
    if requested != "auto":
        return requested
    return "float64" if "normalized" in topic or "meters_36x64" in topic else "uint16"


def default_shape(topic: str, payload_type: str) -> tuple[int, int]:
    if payload_type == "float64" or "36x64" in topic:
        return POLICY_WIDTH, POLICY_HEIGHT
    return RAW_WIDTH, RAW_HEIGHT
