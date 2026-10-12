#include "leju-rl-controller/depth/depth_image_processor.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>

namespace leju::depth {
namespace {

cv::Mat RemoveWhiteSpots(const cv::Mat& depth, double depth_diff,
                         double white_threshold, int max_spot_size,
                         double surrounding_black_ratio) {
  cv::Mat result = depth.clone();
  cv::Mat white_mask;
  cv::compare(depth, white_threshold, white_mask, cv::CMP_GT);
  cv::Mat labels;
  cv::Mat stats;
  cv::Mat centroids;
  const int component_count = cv::connectedComponentsWithStats(
      white_mask, labels, stats, centroids, 8, CV_32S);
  if (component_count <= 1) return result;

  std::vector<double> depth_sums(component_count, 0.0);
  std::vector<int> pixel_counts(component_count, 0);
  for (int y = 0; y < depth.rows; ++y) {
    for (int x = 0; x < depth.cols; ++x) {
      const int label = labels.at<int>(y, x);
      if (label > 0) {
        depth_sums[label] += depth.at<float>(y, x);
        ++pixel_counts[label];
      }
    }
  }
  std::vector<double> component_means(component_count, 0.0);
  for (int label = 1; label < component_count; ++label) {
    component_means[label] =
        depth_sums[label] / (static_cast<double>(pixel_counts[label]) + 1e-7);
  }

  cv::Mat labels_u16;
  labels.convertTo(labels_u16, CV_16U);
  cv::Mat dilated_labels;
  cv::dilate(labels_u16, dilated_labels,
             cv::Mat::ones(3, 3, CV_8U));
  std::vector<int> border_counts(component_count, 0);
  std::vector<int> black_counts(component_count, 0);
  for (int y = 0; y < depth.rows; ++y) {
    for (int x = 0; x < depth.cols; ++x) {
      if (labels.at<int>(y, x) != 0) continue;
      const int label = static_cast<int>(dilated_labels.at<uint16_t>(y, x));
      if (label <= 0 || label >= component_count) continue;
      ++border_counts[label];
      if (depth.at<float>(y, x) < component_means[label] - depth_diff) {
        ++black_counts[label];
      }
    }
  }

  std::vector<uint8_t> remove(component_count, 0);
  for (int label = 1; label < component_count; ++label) {
    const double ratio = border_counts[label] == 0
                             ? 0.0
                             : static_cast<double>(black_counts[label]) /
                                   border_counts[label];
    if (stats.at<int>(label, cv::CC_STAT_AREA) < max_spot_size &&
        ratio >= surrounding_black_ratio) {
      remove[label] = 1;
    }
  }
  for (int y = 0; y < depth.rows; ++y) {
    for (int x = 0; x < depth.cols; ++x) {
      const int label = labels.at<int>(y, x);
      if (label > 0 && remove[label]) result.at<float>(y, x) = 0.0f;
    }
  }
  return result;
}

bool ProcessRealCameraFrame(const DepthFrame& frame,
                            const DepthProcessorConfig& config,
                            ProcessedDepth& out) {
  cv::Mat depth(frame.height, frame.width, CV_32F);
  for (int y = 0; y < frame.height; ++y) {
    for (int x = 0; x < frame.width; ++x) {
      float value =
          frame.pixels[static_cast<size_t>(y * frame.width + x)] / 1000.0f;
      if (!std::isfinite(value) || value < 0.0f) value = 0.0f;
      depth.at<float>(y, x) = value;
    }
  }

  cv::resize(depth, depth, cv::Size(frame.width / 2, frame.height / 2),
             0.0, 0.0, cv::INTER_NEAREST);
  depth = RemoveWhiteSpots(depth, 0.25, 2.5, 1024, 0.5);
  depth = RemoveWhiteSpots(depth, 0.5, 1.0, 1024, 0.5);

  cv::Mat mask;
  cv::compare(depth, 0.0, mask, cv::CMP_EQ);
  if (cv::countNonZero(mask) > 0) {
    cv::dilate(mask, mask, cv::Mat::ones(2, 2, CV_8U));
  }
  cv::resize(depth, depth,
             cv::Size(config.output_width, config.output_height),
             0.0, 0.0, cv::INTER_NEAREST);
  cv::resize(mask, mask,
             cv::Size(config.output_width, config.output_height),
             0.0, 0.0, cv::INTER_NEAREST);

  cv::Mat valid_mask;
  cv::compare(depth, 0.0, valid_mask, cv::CMP_GT);
  const int valid_count = cv::countNonZero(valid_mask);
  if (valid_count == 0) {
    out.error = "depth image has no valid pixels";
    return false;
  }
  double minimum_raw = 0.0;
  double maximum_raw = 0.0;
  cv::minMaxLoc(depth, &minimum_raw, &maximum_raw, nullptr, nullptr,
                valid_mask);
  const float minimum = static_cast<float>(minimum_raw);
  const float maximum = static_cast<float>(maximum_raw);
  const float span = maximum - minimum;
  cv::Mat gray(depth.size(), CV_8U, cv::Scalar(0));
  for (int y = 0; y < depth.rows; ++y) {
    for (int x = 0; x < depth.cols; ++x) {
      if (valid_mask.at<uint8_t>(y, x) == 0) continue;
      const float scaled = span > 0.0f
                               ? (depth.at<float>(y, x) - minimum) /
                                     span * 255.0f
                               : 0.0f;
      gray.at<uint8_t>(y, x) = static_cast<uint8_t>(
          std::clamp(scaled, 0.0f, 255.0f));
    }
  }

  cv::Mat inpainted_gray;
  cv::inpaint(gray, mask, inpainted_gray, 2.0, cv::INPAINT_NS);
  cv::Mat inpainted(depth.size(), CV_32F);
  for (int y = 0; y < depth.rows; ++y) {
    for (int x = 0; x < depth.cols; ++x) {
      if (valid_mask.at<uint8_t>(y, x) != 0) {
        inpainted.at<float>(y, x) = depth.at<float>(y, x);
      } else {
        inpainted.at<float>(y, x) =
            inpainted_gray.at<uint8_t>(y, x) / 255.0f * span + minimum;
      }
    }
  }

  cv::Mat filtered;
  if (config.gaussian_blur) {
    cv::GaussianBlur(inpainted, filtered, cv::Size(3, 3), 1.0, 1.0,
                     cv::BORDER_DEFAULT);
  } else {
    filtered = inpainted;
  }
  out.width = config.output_width;
  out.height = config.output_height;
  out.pixels.resize(static_cast<size_t>(out.width * out.height));
  for (int y = 0; y < out.height; ++y) {
    for (int x = 0; x < out.width; ++x) {
      const float value = std::clamp(filtered.at<float>(y, x), 0.0f,
                                     config.max_depth_m);
      out.pixels[static_cast<size_t>(y * out.width + x)] =
          value / config.max_depth_m;
    }
  }
  out.valid_ratio = static_cast<double>(valid_count) /
                    static_cast<double>(out.pixels.size());
  return true;
}

}  // namespace

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

  const bool supported_real_camera_size =
      (frame.width == 424 && frame.height == 240) ||
      (frame.width == 480 && frame.height == 270);
  const bool is_real_camera =
      supported_real_camera_size && frame.unit == DepthUnit::kMillimeters;
  if (is_real_camera) {
    if (!ProcessRealCameraFrame(frame, config_, out)) return out;
    out.valid = true;
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
