#pragma once

#include <cstdint>
#include <vector>

namespace leju::depth {

enum class DepthUnit { kMeters, kMillimeters };

struct DepthFrame {
  int width = 0;
  int height = 0;
  DepthUnit unit = DepthUnit::kMeters;
  double timestamp_sec = 0.0;
  uint64_t sequence = 0;
  std::vector<float> pixels;
};

struct ProcessedDepth {
  bool valid = false;
  int width = 0;
  int height = 0;
  std::vector<float> pixels;
  double timestamp_sec = 0.0;
  uint64_t sequence = 0;
  double valid_ratio = 0.0;
  const char* error = nullptr;
};

}  // namespace leju::depth
