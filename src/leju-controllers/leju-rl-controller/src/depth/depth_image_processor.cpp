#include "leju-rl-controller/depth/depth_image_processor.h"

#include <algorithm>
#include <cmath>

namespace leju::depth {

DepthImageProcessor::DepthImageProcessor(DepthProcessorConfig config)
    : config_(config) {}

ProcessedDepth DepthImageProcessor::process(const DepthFrame& frame) const {
  ProcessedDepth out;
  out.timestamp_sec = frame.timestamp_sec;
  out.sequence = frame.sequence;
  if (frame.width <= 0 || frame.height <= 0 ||
      frame.pixels.size() != static_cast<size_t>(frame.width * frame.height)) {
    out.error = "invalid input dimensions";
    return out;
  }
  const int left = std::clamp(config_.crop_left, 0, frame.width - 1);
  const int right = std::clamp(config_.crop_right, 0, frame.width - left - 1);
  const int top = std::clamp(config_.crop_top, 0, frame.height - 1);
  const int bottom = std::clamp(config_.crop_bottom, 0, frame.height - top - 1);
  const int crop_w = frame.width - left - right;
  const int crop_h = frame.height - top - bottom;
  if (crop_w <= 0 || crop_h <= 0) {
    out.error = "empty crop";
    return out;
  }
  out.width = config_.output_width;
  out.height = config_.output_height;
  out.pixels.assign(static_cast<size_t>(out.width * out.height), 0.0f);
  size_t valid = 0;
  for (int y = 0; y < out.height; ++y) {
    const int sy = top + std::min(crop_h - 1, y * crop_h / out.height);
    for (int x = 0; x < out.width; ++x) {
      const int sx = left + std::min(crop_w - 1, x * crop_w / out.width);
      float value = frame.pixels[static_cast<size_t>(sy * frame.width + sx)];
      if (frame.unit == DepthUnit::kMillimeters) value *= 0.001f;
      if (!std::isfinite(value) || value < 0.0f) value = 0.0f;
      value = std::clamp(value, 0.0f, config_.max_depth_m);
      if (value > 0.0f) ++valid;
      out.pixels[static_cast<size_t>(y * out.width + x)] =
          value / config_.max_depth_m;
    }
  }
  // 源端在 Gaussian 前对零值空洞做 inpaint。采用三次快照邻域传播，
  // 仅填充至少有 3 个有效邻居的空洞，边界/全空帧仍保持 0。
  for (int pass = 0; pass < 3; ++pass) {
    const auto before = out.pixels;
    for (int y = 0; y < out.height; ++y) {
      for (int x = 0; x < out.width; ++x) {
        const size_t idx = static_cast<size_t>(y * out.width + x);
        if (before[idx] > 0.0f) continue;
        float sum = 0.0f;
        int count = 0;
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) continue;
            const int sx = x + dx;
            const int sy = y + dy;
            if (sx < 0 || sy < 0 || sx >= out.width || sy >= out.height) continue;
            const float neighbor = before[static_cast<size_t>(sy * out.width + sx)];
            if (neighbor > 0.0f) {
              sum += neighbor;
              ++count;
            }
          }
        }
        if (count >= 3) out.pixels[idx] = sum / static_cast<float>(count);
      }
    }
  }
  if (config_.gaussian_blur) {
    const auto before = out.pixels;
    // cv2.GaussianBlur(..., (3,3), sigma=1) is the source setting.  The
    // separable [1,2,1]/4 kernel is numerically equivalent up to the
    // border policy; clamp-to-edge keeps depth values valid at boundaries.
    for (int y = 0; y < out.height; ++y) {
      for (int x = 0; x < out.width; ++x) {
        float value = 0.0f;
        for (int dy = -1; dy <= 1; ++dy) {
          const int sy = std::clamp(y + dy, 0, out.height - 1);
          const int wy = (dy == 0) ? 2 : 1;
          for (int dx = -1; dx <= 1; ++dx) {
            const int sx = std::clamp(x + dx, 0, out.width - 1);
            const int wx = (dx == 0) ? 2 : 1;
            value += static_cast<float>(wy * wx) *
                     before[static_cast<size_t>(sy * out.width + sx)];
          }
        }
        out.pixels[static_cast<size_t>(y * out.width + x)] = value / 16.0f;
      }
    }
  }
  out.valid_ratio = static_cast<double>(valid) / out.pixels.size();
  out.valid = true;
  return out;
}

}  // namespace leju::depth
