"""Python mirror of leju-rl-controller DepthImageProcessor.

The 480x270 native and legacy 424x240 millimetre paths match the real-camera
processing used by depth_walk. Other dimensions retain the generic path.
"""

from __future__ import annotations

import cv2
import numpy as np


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _require_image(image: np.ndarray) -> np.ndarray:
    depth = np.asarray(image)
    if depth.ndim != 2 or depth.size == 0:
        raise ValueError("depth image must be a non-empty two-dimensional array")
    return depth


def _remove_white_spots(
    depth: np.ndarray,
    depth_diff: float,
    white_threshold: float,
    max_spot_size: int,
    surrounding_black_ratio: float,
) -> np.ndarray:
    """Remove small high-depth components surrounded by lower-depth pixels."""
    result = depth.copy()
    white_mask = depth > white_threshold
    component_count, labels, stats, _ = cv2.connectedComponentsWithStats(
        white_mask.astype(np.uint8), connectivity=8
    )
    if component_count <= 1:
        return result

    areas = stats[1:, cv2.CC_STAT_AREA]
    white_indices = np.where(white_mask)
    white_labels = labels[white_indices]
    depth_sums = np.bincount(
        white_labels, weights=depth[white_indices], minlength=component_count
    )
    pixel_counts = np.bincount(white_labels, minlength=component_count)
    component_means = depth_sums / (pixel_counts + 1e-7)

    dilated_labels = cv2.dilate(
        labels.astype(np.uint16), np.ones((3, 3), dtype=np.uint8)
    )
    border_mask = (dilated_labels > 0) & (labels == 0)
    if not np.any(border_mask):
        return result

    border_labels = dilated_labels[border_mask]
    border_is_black = depth[border_mask] < (
        component_means - depth_diff
    )[border_labels]
    black_counts = np.bincount(
        border_labels,
        weights=border_is_black.astype(np.int32),
        minlength=component_count,
    )
    border_counts = np.bincount(border_labels, minlength=component_count)
    black_ratio = np.zeros(component_count, dtype=np.float32)
    present = border_counts > 0
    black_ratio[present] = black_counts[present] / border_counts[present]

    removable = np.where(
        (areas < max_spot_size)
        & (black_ratio[1:] >= surrounding_black_ratio)
    )[0] + 1
    if removable.size:
        result[np.isin(labels, removable)] = 0.0
    return result


def _inpaint_ns(depth: np.ndarray, mask: np.ndarray) -> np.ndarray:
    valid = np.isfinite(depth) & (depth > 0.0)
    if not np.any(valid):
        raise ValueError("depth image has no valid pixels")

    minimum = np.float32(depth[valid].min())
    maximum = np.float32(depth[valid].max())
    span = np.float32(maximum - minimum)
    gray = np.zeros(depth.shape, dtype=np.float32)
    if span > 0.0:
        gray[valid] = (
            (depth[valid] - minimum) / span * np.float32(255.0)
        )
    gray = np.clip(gray, 0.0, 255.0).astype(np.uint8)
    inpainted_gray = cv2.inpaint(gray, mask, 2.0, cv2.INPAINT_NS)
    inpainted = (
        inpainted_gray.astype(np.float32)
        / np.float32(255.0)
        * span
        + minimum
    )
    inpainted[valid] = depth[valid]
    return inpainted


def _process_real_camera(
    depth_mm: np.ndarray,
    output_width: int,
    output_height: int,
    max_depth_m: float,
    gaussian_blur: bool,
) -> tuple[np.ndarray | None, float]:
    depth = depth_mm.astype(np.float32, copy=True) / np.float32(1000.0)
    depth[~np.isfinite(depth) | (depth < 0.0)] = 0.0
    depth = cv2.resize(depth, (212, 120), interpolation=cv2.INTER_NEAREST)
    depth = _remove_white_spots(depth, 0.25, 2.5, 1024, 0.5)
    depth = _remove_white_spots(depth, 0.5, 1.0, 1024, 0.5)

    mask = np.zeros(depth.shape, dtype=np.uint8)
    mask[depth == 0.0] = 255
    if np.any(mask):
        mask = cv2.dilate(mask, np.ones((2, 2), dtype=np.uint8), iterations=1)

    depth = cv2.resize(
        depth, (output_width, output_height), interpolation=cv2.INTER_NEAREST
    )
    mask = cv2.resize(
        mask, (output_width, output_height), interpolation=cv2.INTER_NEAREST
    )
    valid_count = int(np.count_nonzero(depth > 0.0))
    if valid_count == 0:
        return None, 0.0

    processed = _inpaint_ns(depth, mask)
    if gaussian_blur:
        processed = cv2.GaussianBlur(
            processed,
            (3, 3),
            sigmaX=1.0,
            sigmaY=1.0,
            borderType=cv2.BORDER_DEFAULT,
        )
    processed = (
        np.clip(processed, 0.0, max_depth_m) / np.float32(max_depth_m)
    ).astype(np.float32)
    return processed, valid_count / float(output_width * output_height)


def _inpaint_three_passes(pixels: np.ndarray) -> np.ndarray:
    out = pixels.copy()
    height, width = out.shape
    for _ in range(3):
        before = out.copy()
        for y in range(height):
            for x in range(width):
                if before[y, x] > 0.0:
                    continue
                total = 0.0
                count = 0
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        sy, sx = y + dy, x + dx
                        if sy < 0 or sx < 0 or sy >= height or sx >= width:
                            continue
                        neighbour = before[sy, sx]
                        if neighbour > 0.0:
                            total += neighbour
                            count += 1
                if count >= 3:
                    out[y, x] = total / count
    return out


def _gaussian_blur_legacy(pixels: np.ndarray) -> np.ndarray:
    height, width = pixels.shape
    before = pixels.copy()
    out = np.zeros_like(before)
    for y in range(height):
        for x in range(width):
            value = 0.0
            for dy in (-1, 0, 1):
                sy = int(_clamp(y + dy, 0, height - 1))
                weight_y = 2 if dy == 0 else 1
                for dx in (-1, 0, 1):
                    sx = int(_clamp(x + dx, 0, width - 1))
                    weight_x = 2 if dx == 0 else 1
                    value += weight_y * weight_x * before[sy, sx]
            out[y, x] = value / 16.0
    return out


def _process_legacy(
    depth_mm: np.ndarray,
    output_width: int,
    output_height: int,
    max_depth_m: float,
    crop_top: int,
    crop_bottom: int,
    crop_left: int,
    crop_right: int,
    gaussian_blur: bool,
) -> tuple[np.ndarray, float]:
    input_height, input_width = depth_mm.shape
    left = int(_clamp(crop_left, 0, input_width - 1))
    right = int(_clamp(crop_right, 0, input_width - left - 1))
    top = int(_clamp(crop_top, 0, input_height - 1))
    bottom = int(_clamp(crop_bottom, 0, input_height - top - 1))
    crop_width = input_width - left - right
    crop_height = input_height - top - bottom
    if crop_width <= 0 or crop_height <= 0:
        raise ValueError("empty crop region")

    ys = top + np.minimum(
        crop_height - 1,
        np.arange(output_height) * crop_height // output_height,
    )
    xs = left + np.minimum(
        crop_width - 1,
        np.arange(output_width) * crop_width // output_width,
    )
    sampled = depth_mm[np.ix_(ys, xs)].astype(np.float64)
    values = sampled * 0.001
    values[~np.isfinite(values) | (values < 0.0)] = 0.0
    values = np.clip(values, 0.0, max_depth_m)
    valid_ratio = np.count_nonzero(values > 0.0) / float(values.size)
    pixels = (values / max_depth_m).astype(np.float32)
    pixels = _inpaint_three_passes(pixels)
    if gaussian_blur:
        pixels = _gaussian_blur_legacy(pixels)
    return pixels, valid_ratio


def process_depth_mm_frame(
    depth_mm: np.ndarray,
    *,
    input_width: int = 480,
    input_height: int = 270,
    output_width: int = 64,
    output_height: int = 36,
    max_depth_m: float = 2.5,
    crop_top: int = 0,
    crop_bottom: int = 0,
    crop_left: int = 0,
    crop_right: int = 0,
    gaussian_blur: bool = True,
) -> tuple[np.ndarray | None, float]:
    """Convert one DDS depth frame into the controller's policy image."""
    source = _require_image(depth_mm)
    if source.shape != (input_height, input_width):
        raise ValueError(
            f"expected depth shape ({input_height}, {input_width}), got {source.shape}"
        )
    if output_width <= 0 or output_height <= 0:
        raise ValueError("output dimensions must be positive")
    if max_depth_m <= 0.0:
        raise ValueError("max_depth_m must be positive")

    if (input_width, input_height) in ((424, 240), (480, 270)):
        return _process_real_camera(
            source,
            output_width,
            output_height,
            max_depth_m,
            gaussian_blur,
        )
    return _process_legacy(
        source,
        output_width,
        output_height,
        max_depth_m,
        crop_top,
        crop_bottom,
        crop_left,
        crop_right,
        gaussian_blur,
    )
