"""Depth visualization helpers (jet / turbo colormaps, colorbar)."""

from __future__ import annotations

import cv2
import numpy as np

POLICY_W = 64
POLICY_H = 36

COLORMAPS = {
    "jet": cv2.COLORMAP_JET,
    "turbo": cv2.COLORMAP_TURBO,
    "inferno": cv2.COLORMAP_INFERNO,
    "gray": None,
}


def normalized_mm_to_bgr(
    depth_mm: np.ndarray,
    max_depth_mm: float,
    colormap: str = "jet",
) -> np.ndarray:
    """Millimeter depth -> pseudo-color BGR. Invalid/zero -> black."""
    valid = np.isfinite(depth_mm) & (depth_mm > 0.0)
    gray = np.zeros(depth_mm.shape, dtype=np.uint8)
    if np.any(valid):
        norm = np.clip(depth_mm, 0.0, max_depth_mm) / max_depth_mm * 255.0
        gray[valid] = norm[valid].astype(np.uint8)
    return normalized_u8_to_bgr(gray, colormap)


def normalized_policy_to_bgr(
    policy: np.ndarray,
    colormap: str = "jet",
) -> np.ndarray:
    """Policy float [0,1] (36,64) -> pseudo-color BGR."""
    gray = np.clip(policy * 255.0, 0.0, 255.0).astype(np.uint8)
    return normalized_u8_to_bgr(gray, colormap)


def normalized_u8_to_bgr(gray: np.ndarray, colormap: str) -> np.ndarray:
    cmap = COLORMAPS.get(colormap)
    if cmap is None:
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    return cv2.applyColorMap(gray, cmap)


def depth_u8_per_point_to_bgr(gray_u8: np.ndarray, colormap: str = "jet") -> np.ndarray:
    """Map 1D per-point uint8 depths to BGR colors (N, 3)."""
    if gray_u8.size == 0:
        return np.zeros((0, 3), dtype=np.uint8)
    colored = normalized_u8_to_bgr(gray_u8.reshape(-1, 1), colormap)
    return colored.reshape(-1, 3)


def make_colorbar(
    height: int,
    width: int = 48,
    max_depth_m: float = 2.5,
    colormap: str = "jet",
) -> np.ndarray:
    """Vertical depth colorbar with meter labels."""
    bar = np.linspace(255, 0, height, dtype=np.uint8).reshape(height, 1)
    bar = np.repeat(bar, width - 16, axis=1)
    colored = normalized_u8_to_bgr(bar, colormap)
    canvas = np.zeros((height, width, 3), dtype=np.uint8)
    canvas[:, : colored.shape[1]] = colored

    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(canvas, f"{max_depth_m:.1f}m", (2, 18), font, 0.35, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(canvas, "0", (2, height - 6), font, 0.35, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(canvas, "m", (2, height // 2 + 4), font, 0.32, (200, 200, 200), 1, cv2.LINE_AA)
    return canvas


def compose_jet_compare_frame(
    depth_mm: np.ndarray,
    policy: np.ndarray,
    *,
    max_depth_m: float = 2.5,
    colormap: str = "jet",
    policy_scale: int = 10,
    panel_height: int = 360,
    show_colorbar: bool = True,
    frame_idx: int = 0,
    valid_raw: float = 0.0,
    valid_policy: float = 0.0,
    hz: float = 0.0,
) -> np.ndarray:
    """Side-by-side: raw 424x240 | policy 64x36 (both jet), optional colorbar."""
    max_depth_mm = max_depth_m * 1000.0
    raw_bgr = normalized_mm_to_bgr(depth_mm, max_depth_mm, colormap)
    policy_bgr = normalized_policy_to_bgr(policy, colormap)

    raw_vis = _fit_height(raw_bgr, panel_height)
    scale = max(1, policy_scale)
    policy_up = cv2.resize(
        policy_bgr,
        (POLICY_W * scale, POLICY_H * scale),
        interpolation=cv2.INTER_NEAREST,
    )
    policy_vis = _fit_height(policy_up, panel_height)

    gap = 12
    gap_col = np.zeros((panel_height, gap, 3), dtype=np.uint8)
    parts: list[np.ndarray] = [raw_vis, gap_col, policy_vis]
    if show_colorbar:
        parts.extend(
            [
                gap_col.copy(),
                make_colorbar(
                    panel_height, width=56, max_depth_m=max_depth_m, colormap=colormap
                ),
            ]
        )

    canvas = np.hstack(parts)
    x = raw_vis.shape[1] + gap // 2
    cv2.line(canvas, (x, 0), (x, panel_height), (80, 80, 80), 2)

    _draw_label(canvas, "RAW 424x240 (mm DDS)", 8, 24)
    _draw_label(
        canvas,
        f"POLICY 64x36  valid={valid_policy * 100:.1f}%",
        raw_vis.shape[1] + gap + 8,
        24,
    )

    footer = (
        f"#{frame_idx}  colormap={colormap}  max={max_depth_m:.1f}m  "
        f"raw_valid={valid_raw * 100:.1f}%  "
        f"policy min={policy.min():.3f} max={policy.max():.3f}  ~{hz:.1f}Hz"
    )
    _draw_label(canvas, footer, 8, panel_height - 8, scale=0.48, bottom=True)
    return canvas


def _fit_height(image: np.ndarray, target_h: int) -> np.ndarray:
    h, w = image.shape[:2]
    if h == target_h:
        return image
    target_w = max(1, int(w * target_h / h))
    return cv2.resize(image, (target_w, target_h), interpolation=cv2.INTER_NEAREST)


def _draw_label(
    canvas: np.ndarray,
    text: str,
    x: int,
    y: int,
    scale: float = 0.55,
    bottom: bool = False,
) -> None:
    if bottom:
        y = min(canvas.shape[0] - 4, y)
    cv2.putText(canvas, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(canvas, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 1, cv2.LINE_AA)
