"""Python port of leju-rl-controller DepthImageProcessor (real-camera path).

Matches depth_image_processor.cpp + real_camera_processor_ (gaussian_blur=True).
"""

from __future__ import annotations

import numpy as np


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _inpaint_three_passes(pixels: np.ndarray) -> np.ndarray:
    """Fill zero holes when at least 3 valid 8-neighbors exist (3 passes)."""
    out = pixels.copy()
    h, w = out.shape
    for _ in range(3):
        before = out.copy()
        for y in range(h):
            for x in range(w):
                if before[y, x] > 0.0:
                    continue
                total = 0.0
                count = 0
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        sy, sx = y + dy, x + dx
                        if sy < 0 or sx < 0 or sy >= h or sx >= w:
                            continue
                        neighbor = before[sy, sx]
                        if neighbor > 0.0:
                            total += neighbor
                            count += 1
                if count >= 3:
                    out[y, x] = total / count
    return out


def _gaussian_blur_3x3(pixels: np.ndarray) -> np.ndarray:
    """Separable [1,2,1]/4 kernel, clamp-to-edge (matches controller C++)."""
    h, w = pixels.shape
    before = pixels.copy()
    out = np.zeros_like(before)
    for y in range(h):
        for x in range(w):
            value = 0.0
            for dy in (-1, 0, 1):
                sy = _clamp(y + dy, 0, h - 1)
                wy = 2 if dy == 0 else 1
                for dx in (-1, 0, 1):
                    sx = _clamp(x + dx, 0, w - 1)
                    wx = 2 if dx == 0 else 1
                    value += wy * wx * before[int(sy), int(sx)]
            out[y, x] = value / 16.0
    return out


def process_depth_mm_frame(
    depth_mm: np.ndarray,
    *,
    input_width: int = 424,
    input_height: int = 240,
    output_width: int = 64,
    output_height: int = 36,
    max_depth_m: float = 2.5,
    crop_top: int = 0,
    crop_bottom: int = 0,
    crop_left: int = 0,
    crop_right: int = 0,
    gaussian_blur: bool = True,
) -> tuple[np.ndarray, float]:
    """Process raw millimeter depth into normalized policy input [0, 1], shape (36, 64).

    Returns (policy_input, valid_ratio) where valid_ratio counts positive depth
    before inpaint/blur, same as the C++ processor.
    """
    if depth_mm.shape != (input_height, input_width):
        raise ValueError(
            f"expected depth shape ({input_height}, {input_width}), got {depth_mm.shape}"
        )

    left = _clamp(crop_left, 0, input_width - 1)
    right = _clamp(crop_right, 0, input_width - left - 1)
    top = _clamp(crop_top, 0, input_height - 1)
    bottom = _clamp(crop_bottom, 0, input_height - top - 1)
    crop_w = input_width - left - right
    crop_h = input_height - top - bottom
    if crop_w <= 0 or crop_h <= 0:
        raise ValueError("empty crop region")

    out_h, out_w = output_height, output_width
    ys = top + np.minimum(crop_h - 1, np.arange(out_h) * crop_h // out_h)
    xs = left + np.minimum(crop_w - 1, np.arange(out_w) * crop_w // out_w)
    grid_y, grid_x = np.meshgrid(ys, xs, indexing="ij")

    sampled = depth_mm[grid_y, grid_x].astype(np.float64)
    values = sampled * 0.001  # millimeters -> meters
    values[~np.isfinite(values) | (values < 0.0)] = 0.0
    values = np.clip(values, 0.0, max_depth_m)

    valid = int(np.count_nonzero(values > 0.0))
    valid_ratio = valid / float(out_h * out_w)

    pixels = (values / max_depth_m).astype(np.float32)
    pixels = _inpaint_three_passes(pixels)
    if gaussian_blur:
        pixels = _gaussian_blur_3x3(pixels)

    return pixels, valid_ratio
