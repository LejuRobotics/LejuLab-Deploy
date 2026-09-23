#include <atomic>
#include <cmath>
#include <limits>
#include <thread>

#include <gtest/gtest.h>

#include "leju-rl-controller/depth/depth_history_buffer.h"
#include "leju-rl-controller/depth/depth_image_processor.h"

namespace leju::depth {
namespace {

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

}  // namespace
}  // namespace leju::depth
