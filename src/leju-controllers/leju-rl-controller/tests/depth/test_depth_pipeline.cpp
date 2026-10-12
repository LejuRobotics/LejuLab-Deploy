#include <atomic>
#include <cmath>
#include <limits>
#include <thread>

#include <gtest/gtest.h>

#include "leju-rl-controller/controllers/depth_stance_gate.h"
#include "leju-rl-controller/depth/depth_history_buffer.h"
#include "leju-rl-controller/depth/depth_image_processor.h"
#include "leju-rl-controller/depth/depth_observation_provider.h"

namespace leju::depth {
namespace {

TEST(DepthStanceGate, RequestsStopAfterDebouncedZeroCommand) {
  DepthStanceGate gate;

  EXPECT_FALSE(gate.update(false, 5));
  for (int tick = 0; tick < 4; ++tick) {
    EXPECT_FALSE(gate.update(true, 5));
  }
  EXPECT_TRUE(gate.update(true, 5));
  EXPECT_TRUE(gate.update(true, 5));
}

TEST(DepthStanceGate, MotionCancelsPendingStopRequest) {
  DepthStanceGate gate;

  for (int tick = 0; tick < 4; ++tick) {
    EXPECT_FALSE(gate.update(true, 5));
  }
  EXPECT_FALSE(gate.update(false, 5));
  for (int tick = 0; tick < 4; ++tick) {
    EXPECT_FALSE(gate.update(true, 5));
  }
  EXPECT_TRUE(gate.update(true, 5));
}

TEST(DepthImageProcessor, ConvertsMillimetersAndNormalizes) {
  DepthFrame frame;
  frame.width = 4;
  frame.height = 4;
  frame.unit = DepthUnit::kMillimeters;
  frame.pixels.assign(16, 1000.0f);
  DepthProcessorConfig config;
  config.output_width = 2;
  config.output_height = 2;
  const auto result = DepthImageProcessor(config).process(frame);
  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.pixels.size(), 4u);
  for (float value : result.pixels) EXPECT_NEAR(value, 0.4f, 1e-6f);
}

TEST(DepthImageProcessor, SanitizesInvalidAndClipsFarDepth) {
  DepthFrame frame;
  frame.width = 2;
  frame.height = 2;
  frame.pixels = {std::numeric_limits<float>::quiet_NaN(), -1.0f, 1.25f, 5.0f};
  DepthProcessorConfig config;
  config.output_width = 2;
  config.output_height = 2;
  const auto result = DepthImageProcessor(config).process(frame);
  ASSERT_TRUE(result.valid);
  EXPECT_FLOAT_EQ(result.pixels[0], 0.0f);
  EXPECT_FLOAT_EQ(result.pixels[1], 0.0f);
  EXPECT_FLOAT_EQ(result.pixels[2], 0.5f);
  EXPECT_FLOAT_EQ(result.pixels[3], 1.0f);
}

TEST(DepthImageProcessor, MatchesKuavoRosRealCameraPreprocessing) {
  DepthFrame frame;
  frame.width = 424;
  frame.height = 240;
  frame.unit = DepthUnit::kMillimeters;
  frame.pixels.resize(static_cast<size_t>(frame.width * frame.height));
  for (int y = 0; y < frame.height; ++y) {
    for (int x = 0; x < frame.width; ++x) {
      frame.pixels[static_cast<size_t>(y * frame.width + x)] =
          static_cast<float>(700 + y * 3 + x * 2);
    }
  }
  for (int y = 72; y < 112; ++y) {
    for (int x = 128; x < 178; ++x) {
      frame.pixels[static_cast<size_t>(y * frame.width + x)] = 0.0f;
    }
  }
  for (int y = 150; y < 156; ++y) {
    for (int x = 300; x < 306; ++x) {
      frame.pixels[static_cast<size_t>(y * frame.width + x)] = 4200.0f;
    }
  }

  DepthProcessorConfig config;
  config.gaussian_blur = true;
  const auto result = DepthImageProcessor(config).process(frame);

  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.width, 64);
  ASSERT_EQ(result.height, 36);
  EXPECT_NEAR(result.pixels[0 * 64 + 0], 0.28657764f, 1e-5f);
  EXPECT_NEAR(result.pixels[14 * 64 + 23], 0.49755901f, 1e-5f);
  EXPECT_NEAR(result.pixels[23 * 64 + 46], 0.70598102f, 1e-5f);
  EXPECT_NEAR(result.pixels[35 * 64 + 63], 0.88462240f, 1e-5f);
  uint64_t hash = 1469598103934665603ULL;
  for (float value : result.pixels) {
    const int32_t quantized = static_cast<int32_t>(std::lround(value * 100000.0f));
    for (int byte = 0; byte < 4; ++byte) {
      hash ^= static_cast<uint8_t>(
          (static_cast<uint32_t>(quantized) >> (byte * 8)) & 0xffu);
      hash *= 1099511628211ULL;
    }
  }
  EXPECT_EQ(hash, 0x8c121454fc55fe5cULL);
}

TEST(DepthImageProcessor, UsesRealCameraCleanupForGemini330NativeDepth) {
  DepthFrame frame;
  frame.width = 480;
  frame.height = 270;
  frame.unit = DepthUnit::kMillimeters;
  frame.pixels.assign(static_cast<size_t>(frame.width * frame.height), 1000.0f);
  for (int y = 125; y < 146; ++y) {
    for (int x = 230; x < 251; ++x) {
      frame.pixels[static_cast<size_t>(y * frame.width + x)] = 5000.0f;
    }
  }

  DepthProcessorConfig config;
  config.gaussian_blur = true;
  const auto result = DepthImageProcessor(config).process(frame);

  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.width, 64);
  ASSERT_EQ(result.height, 36);
  EXPECT_NEAR(result.pixels[18 * 64 + 32], 0.4f, 0.08f);
}

TEST(DepthHistoryBuffer, InitializesAndExportsEightImages) {
  DepthHistoryBuffer history;
  std::vector<float> image(DepthHistoryBuffer::kImageSize, 0.25f);
  ASSERT_TRUE(history.push(image, 1.0, 1, 1.0));
  const auto snapshot = history.snapshot();
  ASSERT_TRUE(snapshot.ready);
  ASSERT_EQ(snapshot.data.size(), 8u * DepthHistoryBuffer::kImageSize);
  for (float value : snapshot.data) EXPECT_FLOAT_EQ(value, 0.25f);
}

TEST(DepthHistoryBuffer, MatchesSourceMujocoThirtyHertzHistory) {
  DepthHistoryBuffer history;
  for (uint64_t i = 0; i < 22; ++i) {
    ASSERT_TRUE(history.push(
        std::vector<float>(DepthHistoryBuffer::kImageSize, static_cast<float>(i)),
        static_cast<double>(i), i, 1.0));
  }
  const auto snapshot = history.snapshot();
  const int expected[] = {0, 3, 6, 9, 12, 15, 18, 21};
  for (size_t i = 0; i < 8; ++i) {
    EXPECT_FLOAT_EQ(snapshot.data[i * DepthHistoryBuffer::kImageSize],
                    static_cast<float>(expected[i]));
  }
}

TEST(DepthHistoryBuffer, ResetClearsReadiness) {
  DepthHistoryBuffer history;
  ASSERT_TRUE(history.push(std::vector<float>(DepthHistoryBuffer::kImageSize, 1.0f),
                           1.0, 1, 1.0));
  history.reset();
  EXPECT_FALSE(history.snapshot().ready);
}

TEST(PreprocessedDepth, PreservesPixelsAndMetadata) {
  std::vector<double> values(DepthHistoryBuffer::kImageSize, 0.25);
  values[0] = 0.0;
  values[1] = 0.125;
  values.back() = 1.0;

  const auto result = BuildPreprocessedDepth(values, 12.5, 37);

  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->valid);
  EXPECT_EQ(result->width, 64);
  EXPECT_EQ(result->height, 36);
  EXPECT_DOUBLE_EQ(result->timestamp_sec, 12.5);
  EXPECT_EQ(result->sequence, 37u);
  ASSERT_EQ(result->pixels.size(), values.size());
  EXPECT_FLOAT_EQ(result->pixels[0], 0.0f);
  EXPECT_FLOAT_EQ(result->pixels[1], 0.125f);
  EXPECT_FLOAT_EQ(result->pixels.back(), 1.0f);
}

TEST(PreprocessedDepth, RejectsInvalidSizeAndPixelValues) {
  std::vector<double> values(DepthHistoryBuffer::kImageSize, 0.5);
  values.pop_back();
  EXPECT_FALSE(BuildPreprocessedDepth(values, 1.0, 1).has_value());

  values.assign(DepthHistoryBuffer::kImageSize, 0.5);
  values[10] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(BuildPreprocessedDepth(values, 1.0, 1).has_value());
  values[10] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(BuildPreprocessedDepth(values, 1.0, 1).has_value());
  values[10] = -0.001;
  EXPECT_FALSE(BuildPreprocessedDepth(values, 1.0, 1).has_value());
  values[10] = 1.001;
  EXPECT_FALSE(BuildPreprocessedDepth(values, 1.0, 1).has_value());
}

TEST(PreprocessedDepth, ComputesPositivePixelRatio) {
  std::vector<double> values(DepthHistoryBuffer::kImageSize, 0.0);
  values[0] = 0.2;
  values[100] = 1.0;

  const auto result = BuildPreprocessedDepth(values, 2.0, 3);

  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->valid_ratio,
                   2.0 / static_cast<double>(DepthHistoryBuffer::kImageSize));
}

TEST(DepthDdsInput, PreprocessedCameraDisablesMujocoFallback) {
  DepthDdsInputConfig config;
  config.topic = "rt/camera/depth_policy_input";
  config.preprocessed = true;
  EXPECT_FALSE(ShouldSubscribeMujocoDepthFallback(config));

  config.preprocessed = false;
  EXPECT_TRUE(ShouldSubscribeMujocoDepthFallback(config));
  config.topic = "rt/depth_camera/frame_meters_36x64";
  EXPECT_FALSE(ShouldSubscribeMujocoDepthFallback(config));
}

TEST(DepthObservationProvider, UsesFortyThreeFramePreprocessedHistoryAndTimeout) {
  DepthObservationProvider provider;
  for (uint64_t i = 0; i < 43; ++i) {
    ProcessedDepth depth;
    depth.valid = true;
    depth.width = 64;
    depth.height = 36;
    depth.timestamp_sec = 100.0 + static_cast<double>(i);
    depth.sequence = i;
    depth.valid_ratio = 1.0;
    depth.pixels.assign(DepthHistoryBuffer::kImageSize,
                        static_cast<float>(i) / 42.0f);
    ASSERT_TRUE(provider.submitPreprocessed(depth));
  }

  const auto fresh = provider.snapshot(142.25, 0.5);
  ASSERT_TRUE(fresh.ready);
  const int expected[] = {0, 6, 12, 18, 24, 30, 36, 42};
  for (size_t i = 0; i < 8; ++i) {
    EXPECT_FLOAT_EQ(fresh.data[i * DepthHistoryBuffer::kImageSize],
                    static_cast<float>(expected[i]) / 42.0f);
  }
  EXPECT_EQ(fresh.sequence, 42u);
  EXPECT_FALSE(provider.snapshot(142.51, 0.5).ready);
}

TEST(DepthObservationProvider, RejectsMalformedPreprocessedDepth) {
  DepthObservationProvider provider;
  ProcessedDepth depth;
  depth.valid = true;
  depth.width = 64;
  depth.height = 36;
  depth.valid_ratio = 1.0;
  depth.pixels.assign(DepthHistoryBuffer::kImageSize, 0.5f);
  EXPECT_TRUE(provider.submitPreprocessed(depth));

  depth.valid = false;
  EXPECT_FALSE(provider.submitPreprocessed(depth));
  depth.valid = true;
  depth.width = 63;
  EXPECT_FALSE(provider.submitPreprocessed(depth));
  depth.width = 64;
  depth.pixels.pop_back();
  EXPECT_FALSE(provider.submitPreprocessed(depth));
}

TEST(DepthLatency, ComputesValidAndNegativeLatency) {
  const auto positive = ComputeDepthLatencySec(10, 250000000u, 10.75);
  ASSERT_TRUE(positive.has_value());
  EXPECT_DOUBLE_EQ(*positive, 0.5);

  const auto negative = ComputeDepthLatencySec(11, 0u, 10.75);
  ASSERT_TRUE(negative.has_value());
  EXPECT_DOUBLE_EQ(*negative, -0.25);
}

TEST(DepthLatency, RejectsInvalidTimestamps) {
  EXPECT_FALSE(ComputeDepthLatencySec(0, 0u, 1.0).has_value());
  EXPECT_FALSE(ComputeDepthLatencySec(1, 1000000000u, 2.0).has_value());
  EXPECT_FALSE(ComputeDepthLatencySec(
                   1, 0u, std::numeric_limits<double>::quiet_NaN())
                   .has_value());
  EXPECT_FALSE(ComputeDepthLatencySec(
                   1, 0u, std::numeric_limits<double>::infinity())
                   .has_value());
}

}  // namespace
}  // namespace leju::depth
