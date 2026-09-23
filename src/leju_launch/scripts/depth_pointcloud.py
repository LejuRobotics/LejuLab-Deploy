"""Depth -> point cloud back-projection and 2D rendering for comparison."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from depth_viz import depth_u8_per_point_to_bgr


@dataclass
class PinholeIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_fov(
        cls,
        width: int,
        height: int,
        hfov_deg: float = 87.0,
        vfov_deg: float | None = None,
    ) -> PinholeIntrinsics:
        """Build intrinsics from horizontal FOV (approximate, no distortion)."""
        hfov = np.deg2rad(hfov_deg)
        cx = (width - 1) * 0.5
        cy = (height - 1) * 0.5
        fx = cx / np.tan(hfov * 0.5)
        if vfov_deg is not None:
            vfov = np.deg2rad(vfov_deg)
            fy = cy / np.tan(vfov * 0.5)
        else:
            fy = fx
        return cls(width=width, height=height, fx=fx, fy=fy, cx=cx, cy=cy)


def policy_pixel_to_source(
    policy_x: int,
    policy_y: int,
    input_width: int,
    input_height: int,
    output_width: int = 64,
    output_height: int = 36,
    crop_left: int = 0,
    crop_top: int = 0,
    crop_right: int = 0,
    crop_bottom: int = 0,
) -> tuple[int, int]:
    """Map policy bin (x,y) to source pixel — same rule as DepthImageProcessor."""
    left = max(0, min(crop_left, input_width - 1))
    right = max(0, min(crop_right, input_width - left - 1))
    top = max(0, min(crop_top, input_height - 1))
    bottom = max(0, min(crop_bottom, input_height - top - 1))
    crop_w = input_width - left - right
    crop_h = input_height - top - bottom
    sx = left + min(crop_w - 1, policy_x * crop_w // output_width)
    sy = top + min(crop_h - 1, policy_y * crop_h // output_height)
    return sx, sy


def backproject_raw_mm(
    depth_mm: np.ndarray,
    intrinsics: PinholeIntrinsics,
    *,
    step: int = 3,
    max_depth_m: float = 2.5,
) -> tuple[np.ndarray, np.ndarray]:
    """Subsampled raw depth -> Nx3 points (camera frame) and Nx3 BGR colors."""
    h, w = depth_mm.shape
    us = np.arange(0, w, max(1, step))
    vs = np.arange(0, h, max(1, step))
    uu, vv = np.meshgrid(us, vs)
    uu = uu.ravel()
    vv = vv.ravel()
    z_mm = depth_mm[vv, uu]
    valid = np.isfinite(z_mm) & (z_mm > 0.0)
    uu, vv, z_mm = uu[valid], vv[valid], z_mm[valid]
    if uu.size == 0:
        return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.uint8)

    z = np.clip(z_mm * 0.001, 0.0, max_depth_m).astype(np.float32)
    x = (uu.astype(np.float32) - intrinsics.cx) * z / intrinsics.fx
    y = (vv.astype(np.float32) - intrinsics.cy) * z / intrinsics.fy
    points = np.stack([x, y, z], axis=1)

    norm = (z / max_depth_m * 255.0).astype(np.uint8)
    colors = depth_u8_per_point_to_bgr(norm, "jet")
    return points, colors


def backproject_policy(
    policy: np.ndarray,
    intrinsics: PinholeIntrinsics,
    *,
    input_width: int = 424,
    input_height: int = 240,
    max_depth_m: float = 2.5,
) -> tuple[np.ndarray, np.ndarray]:
    """64x36 normalized policy depth -> point cloud in same camera frame."""
    out_h, out_w = policy.shape
    xs = np.arange(out_w)
    ys = np.arange(out_h)
    gx, gy = np.meshgrid(xs, ys)
    gx = gx.ravel()
    gy = gy.ravel()
    depth_norm = policy.ravel()
    valid = depth_norm > 0.0
    gx, gy, depth_norm = gx[valid], gy[valid], depth_norm[valid]
    if gx.size == 0:
        return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.uint8)

    src_u = np.empty_like(gx, dtype=np.int32)
    src_v = np.empty_like(gy, dtype=np.int32)
    for i in range(gx.size):
        src_u[i], src_v[i] = policy_pixel_to_source(
            int(gx[i]), int(gy[i]), input_width, input_height, out_w, out_h
        )

    z = np.clip(depth_norm * max_depth_m, 0.0, max_depth_m).astype(np.float32)
    x = (src_u.astype(np.float32) - intrinsics.cx) * z / intrinsics.fx
    y = (src_v.astype(np.float32) - intrinsics.cy) * z / intrinsics.fy
    points = np.stack([x, y, z], axis=1)

    norm = (z / max_depth_m * 255.0).astype(np.uint8)
    colors = depth_u8_per_point_to_bgr(norm, "jet")
    return points, colors


def render_xz_bev(
    points: np.ndarray,
    colors: np.ndarray,
    width: int,
    height: int,
    *,
    x_range: tuple[float, float] = (-1.2, 1.2),
    z_range: tuple[float, float] = (0.0, 2.8),
    point_radius: int = 2,
    bg_color: tuple[int, int, int] = (24, 24, 24),
) -> np.ndarray:
    """Bird's-eye view: X horizontal, Z vertical (forward = up)."""
    canvas = np.full((height, width, 3), bg_color, dtype=np.uint8)
    if points.size == 0:
        return canvas

    x, _y, z = points[:, 0], points[:, 1], points[:, 2]
    x_min, x_max = x_range
    z_min, z_max = z_range
    px = ((x - x_min) / max(x_max - x_min, 1e-6) * (width - 1)).astype(np.int32)
    pz = ((1.0 - (z - z_min) / max(z_max - z_min, 1e-6)) * (height - 1)).astype(
        np.int32
    )
    inside = (px >= 0) & (px < width) & (pz >= 0) & (pz < height)
    px, pz, colors = px[inside], pz[inside], colors[inside]
    z_vis = z[inside]

    order = np.argsort(-z_vis)
    for idx in order:
        cv2.circle(canvas, (int(px[idx]), int(pz[idx])), point_radius, colors[idx].tolist(), -1)

    _draw_grid(canvas, x_range, z_range)
    return canvas


def render_yz_side(
    points: np.ndarray,
    colors: np.ndarray,
    width: int,
    height: int,
    *,
    y_range: tuple[float, float] = (-0.8, 0.8),
    z_range: tuple[float, float] = (0.0, 2.8),
    point_radius: int = 2,
    bg_color: tuple[int, int, int] = (24, 24, 24),
) -> np.ndarray:
    """Side view: Y horizontal (down positive), Z vertical."""
    canvas = np.full((height, width, 3), bg_color, dtype=np.uint8)
    if points.size == 0:
        return canvas

    _x, y, z = points[:, 0], points[:, 1], points[:, 2]
    y_min, y_max = y_range
    z_min, z_max = z_range
    py = ((y - y_min) / max(y_max - y_min, 1e-6) * (width - 1)).astype(np.int32)
    pz = ((1.0 - (z - z_min) / max(z_max - z_min, 1e-6)) * (height - 1)).astype(
        np.int32
    )
    inside = (py >= 0) & (py < width) & (pz >= 0) & (pz < height)
    py, pz, colors = py[inside], pz[inside], colors[inside]
    z_vis = z[inside]

    order = np.argsort(-z_vis)
    for idx in order:
        cv2.circle(canvas, (int(py[idx]), int(pz[idx])), point_radius, colors[idx].tolist(), -1)
    return canvas


def compose_pointcloud_compare(
    raw_points: np.ndarray,
    raw_colors: np.ndarray,
    policy_points: np.ndarray,
    policy_colors: np.ndarray,
    *,
    panel_w: int = 420,
    panel_h: int = 320,
    view: str = "bev",
    point_radius: int = 2,
    frame_idx: int = 0,
    n_raw: int = 0,
    n_policy: int = 0,
    hz: float = 0.0,
) -> np.ndarray:
    """Side-by-side point cloud panels."""
    if view == "both":
        raw_top = render_xz_bev(raw_points, raw_colors, panel_w, panel_h // 2, point_radius=point_radius)
        raw_bot = render_yz_side(raw_points, raw_colors, panel_w, panel_h // 2, point_radius=point_radius)
        raw_panel = np.vstack([raw_top, raw_bot])
        pol_top = render_xz_bev(policy_points, policy_colors, panel_w, panel_h // 2, point_radius=point_radius)
        pol_bot = render_yz_side(policy_points, policy_colors, panel_w, panel_h // 2, point_radius=point_radius)
        policy_panel = np.vstack([pol_top, pol_bot])
    elif view == "side":
        raw_panel = render_yz_side(raw_points, raw_colors, panel_w, panel_h, point_radius=point_radius)
        policy_panel = render_yz_side(policy_points, policy_colors, panel_w, panel_h, point_radius=point_radius)
    else:
        raw_panel = render_xz_bev(raw_points, raw_colors, panel_w, panel_h, point_radius=point_radius)
        policy_panel = render_xz_bev(
            policy_points, policy_colors, panel_w, panel_h, point_radius=max(2, point_radius + 1)
        )

    gap = 12
    gap_col = np.zeros((panel_h, gap, 3), dtype=np.uint8)
    canvas = np.hstack([raw_panel, gap_col, policy_panel])

    view_label = {"bev": "BEV X-Z", "side": "side Y-Z", "both": "BEV+side"}[view]
    _label(canvas, f"RAW cloud ({view_label})  pts={n_raw}", 8, 22)
    _label(canvas, f"POLICY 64x36 cloud  pts={n_policy}", panel_w + gap + 8, 22)
    footer = f"#{frame_idx}  view={view}  ~{hz:.1f}Hz  (camera frame: X right, Y down, Z forward)"
    _label(canvas, footer, 8, panel_h - 8, scale=0.45)
    return canvas


def _draw_grid(
    canvas: np.ndarray,
    x_range: tuple[float, float],
    z_range: tuple[float, float],
) -> None:
    h, w = canvas.shape[:2]
    grid_color = (48, 48, 48)
    for z_m in np.arange(0.5, z_range[1], 0.5):
        t = 1.0 - (z_m - z_range[0]) / max(z_range[1] - z_range[0], 1e-6)
        y = int(t * (h - 1))
        cv2.line(canvas, (0, y), (w - 1, y), grid_color, 1)
    for x_m in (-1.0, 0.0, 1.0):
        t = (x_m - x_range[0]) / max(x_range[1] - x_range[0], 1e-6)
        x = int(t * (w - 1))
        cv2.line(canvas, (x, 0), (x, h - 1), grid_color, 1)


def _label(
    canvas: np.ndarray,
    text: str,
    x: int,
    y: int,
    scale: float = 0.55,
) -> None:
    cv2.putText(canvas, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(canvas, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 1, cv2.LINE_AA)
