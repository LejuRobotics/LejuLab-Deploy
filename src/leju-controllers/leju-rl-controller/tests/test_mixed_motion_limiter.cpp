#include <gtest/gtest.h>

#include "leju-rl-controller/utils/mixed_motion_limiter.h"

using leju::MixedMotionLimiter;
using leju::MixedMotionLimitsConfig;

namespace {

MixedMotionLimiter makeLimiter() {
  MixedMotionLimitsConfig config;
  config.enabled = true;
  config.angular_vel_threshold = 0.05;
  config.max_linear_vel_with_angular = 0.5;
  config.linear_vel_threshold = 0.05;
  config.max_angular_vel_with_linear = 0.4;

  MixedMotionLimiter limiter;
  limiter.setConfig(config);
  return limiter;
}

}  // namespace

TEST(MixedMotionLimiterTest, DisabledPassesThrough) {
  MixedMotionLimiter limiter;  // 默认 enabled=false

  double vx = 0.8, vy = 0.3, wz = 0.9;
  limiter.apply(vx, vy, wz);

  EXPECT_DOUBLE_EQ(vx, 0.8);
  EXPECT_DOUBLE_EQ(vy, 0.3);
  EXPECT_DOUBLE_EQ(wz, 0.9);
}

TEST(MixedMotionLimiterTest, LinearVelocityCapsAngular) {
  MixedMotionLimiter limiter = makeLimiter();

  double vx = 0.4, vy = 0.0, wz = 0.8;
  limiter.apply(vx, vy, wz);

  EXPECT_DOUBLE_EQ(vx, 0.4);  // 线速度未超 0.5，不受影响
  EXPECT_DOUBLE_EQ(vy, 0.0);
  EXPECT_DOUBLE_EQ(wz, 0.4);  // 角速度被压到 max_angular_vel_with_linear
}

TEST(MixedMotionLimiterTest, AngularCapPreservesSign) {
  MixedMotionLimiter limiter = makeLimiter();

  double vx = 0.4, vy = 0.0, wz = -0.8;
  limiter.apply(vx, vy, wz);

  EXPECT_DOUBLE_EQ(wz, -0.4);
}

TEST(MixedMotionLimiterTest, PureRotationUnlimited) {
  MixedMotionLimiter limiter = makeLimiter();

  double vx = 0.0, vy = 0.0, wz = 0.8;
  limiter.apply(vx, vy, wz);

  EXPECT_DOUBLE_EQ(wz, 0.8);  // 原地转向不受限
}

TEST(MixedMotionLimiterTest, AngularVelocityCapsLinear) {
  MixedMotionLimiter limiter = makeLimiter();

  double vx = 0.6, vy = 0.8, wz = 0.3;
  limiter.apply(vx, vy, wz);

  // 线速度模长 1.0 > 0.5，vx/vy 等比缩放到模长 0.5
  EXPECT_NEAR(vx, 0.3, 1e-9);
  EXPECT_NEAR(vy, 0.4, 1e-9);
  EXPECT_DOUBLE_EQ(wz, 0.3);  // 角速度未超 0.4，不受影响
}

TEST(MixedMotionLimiterTest, PureLinearUnlimited) {
  MixedMotionLimiter limiter = makeLimiter();

  double vx = 0.8, vy = 0.0, wz = 0.0;
  limiter.apply(vx, vy, wz);

  EXPECT_DOUBLE_EQ(vx, 0.8);  // 无转向时直行不受限
}

TEST(MixedMotionLimiterTest, BothCappedSimultaneously) {
  MixedMotionLimiter limiter = makeLimiter();

  double vx = 0.8, vy = 0.0, wz = 0.9;
  limiter.apply(vx, vy, wz);

  EXPECT_NEAR(vx, 0.5, 1e-9);
  EXPECT_DOUBLE_EQ(wz, 0.4);
}

TEST(MixedMotionLimiterTest, BelowThresholdsUntouched) {
  MixedMotionLimiter limiter = makeLimiter();

  // 线速度低于 linear_vel_threshold → 不触发角速度限制
  double vx = 0.02, vy = 0.0, wz = 0.8;
  limiter.apply(vx, vy, wz);
  EXPECT_DOUBLE_EQ(wz, 0.8);

  // 角速度低于 angular_vel_threshold → 不触发线速度限制
  vx = 0.8, vy = 0.0, wz = 0.02;
  limiter.apply(vx, vy, wz);
  EXPECT_DOUBLE_EQ(vx, 0.8);
}
