#include <gtest/gtest.h>

#include "leju-rl-controller/utils/velocity_stop_filter.h"

using leju::VelocityStopFilter;
using leju::VelocityStopFilterConfig;

TEST(VelocityStopFilterTest, RampsDownWhenTargetIsStop) {
  VelocityStopFilterConfig config;
  config.enabled = true;
  config.deceleration = 1.0;
  config.stop_target_threshold = 0.01;

  VelocityStopFilter filter;
  filter.setConfig(config);

  EXPECT_DOUBLE_EQ(filter.update(0.8, 0.02), 0.8);
  EXPECT_NEAR(filter.update(0.0, 0.02), 0.78, 1e-9);
  EXPECT_NEAR(filter.update(0.0, 0.02), 0.76, 1e-9);
}

TEST(VelocityStopFilterTest, AccelerationPassesThrough) {
  VelocityStopFilterConfig config;
  config.enabled = true;
  config.deceleration = 1.0;
  config.stop_target_threshold = 0.01;

  VelocityStopFilter filter;
  filter.setConfig(config);

  EXPECT_DOUBLE_EQ(filter.update(0.1, 0.02), 0.1);
  EXPECT_DOUBLE_EQ(filter.update(0.8, 0.02), 0.8);
}

TEST(VelocityStopFilterTest, DoesNotFilterNonStopTargets) {
  VelocityStopFilterConfig config;
  config.enabled = true;
  config.deceleration = 1.0;
  config.stop_target_threshold = 0.01;

  VelocityStopFilter filter;
  filter.setConfig(config);

  EXPECT_DOUBLE_EQ(filter.update(0.8, 0.02), 0.8);
  EXPECT_DOUBLE_EQ(filter.update(0.3, 0.02), 0.3);
}

