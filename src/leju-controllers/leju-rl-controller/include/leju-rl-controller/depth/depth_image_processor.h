#pragma once

#include "leju-rl-controller/depth/depth_frame.h"

namespace leju::depth {

struct DepthProcessorConfig {
  int output_width = 64;
  int output_height = 36;
  float max_depth_m = 2.5f;
  int crop_top = 0;
  int crop_bottom = 0;
  int crop_left = 0;
  int crop_right = 0;
  // Source depth_inpainter/depth_stack_ffs use a 3x3 Gaussian blur after
  // inpainting.  Keep it opt-in for unit-test/general callers; the global
  // DDS depth provider enables it explicitly.
  bool gaussian_blur = false;
};

class DepthImageProcessor {
 public:
  explicit DepthImageProcessor(DepthProcessorConfig config = {});
  ProcessedDepth process(const DepthFrame& frame) const;

 private:
  DepthProcessorConfig config_;
};

}  // namespace leju::depth
