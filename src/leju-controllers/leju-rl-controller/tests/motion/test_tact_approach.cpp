// Copyright 2026 Leju Robotics. All rights reserved.
//
// BuildTactApproachSamples 单元测试。
//
// 背景：所有 tact 文件首帧都在 keyframe=0，parser 的 init 帧插入逻辑
// （条件 front().keyframe > 0）永不触发，init_arm_pos 被忽略。
// LB+B 冻结手臂后再播放其他 tact 时，指令从冻结位姿阶跃到动作 0 帧位姿，
// 且 tact 速度前馈绕过 External 模式限速器，手臂快速甩回原位。
// 修复：Play() 中前置一段最小急动度接入过渡，从当前位姿平滑走到轨迹首采样点。

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "leju-rl-controller/motion/tact_player.h"

namespace leju {
namespace runtime {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;

vr::tact_player::JointTrajectorySample MakeFirstSample(
    const std::vector<double>& q_rad,
    bool has_waist = false,
    double waist_q_rad = 0.0) {
  vr::tact_player::JointTrajectorySample s;
  s.q = q_rad;
  s.v.assign(q_rad.size(), 0.0);
  s.acc.assign(q_rad.size(), 0.0);
  s.has_waist = has_waist;
  s.waist_q = waist_q_rad;
  return s;
}

TEST(TactApproachTest, FarStartGeneratesSmoothApproach) {
  const double loop_dt = 0.01;
  TactApproachOptions opts;

  auto first = MakeFirstSample({1.0, -0.5});
  std::vector<double> init_arm_deg = {0.0, 0.0};

  auto samples = BuildTactApproachSamples(first, init_arm_deg, {}, loop_dt, opts);
  ASSERT_FALSE(samples.empty());

  // 起点等于当前位姿
  EXPECT_NEAR(samples.front().q[0], 0.0, 1e-6);
  EXPECT_NEAR(samples.front().q[1], 0.0, 1e-6);

  // 终点收敛到轨迹首采样点（最后一个过渡采样与 first_sample 间隔一个 tick）
  EXPECT_NEAR(samples.back().q[0], 1.0, opts.velocity * loop_dt * 1.5);
  EXPECT_NEAR(samples.back().q[1], -0.5, opts.velocity * loop_dt * 1.5);

  // 时长：T = 1.875 * max_dist / velocity = 1.875s，未触发钳制
  const double expected_duration = 1.875 * 1.0 / opts.velocity;
  EXPECT_NEAR(samples.size() * loop_dt, expected_duration, 2 * loop_dt);

  // 逐拍位置增量不超过峰值速度约束（阶跃消除的核心断言）
  double max_step = 0.0;
  for (size_t i = 1; i < samples.size(); ++i) {
    for (size_t j = 0; j < 2; ++j) {
      max_step = std::max(max_step, std::abs(samples[i].q[j] - samples[i - 1].q[j]));
    }
  }
  EXPECT_LE(max_step, opts.velocity * loop_dt * 1.05);

  // 速度曲线：两端静止、中段有速度前馈
  EXPECT_NEAR(samples.front().v[0], 0.0, 1e-6);
  EXPECT_NEAR(samples.back().v[0], 0.0, opts.velocity * 0.1);
  EXPECT_GT(samples[samples.size() / 2].v[0], 0.5 * opts.velocity);
}

TEST(TactApproachTest, NearStartReturnsEmpty) {
  auto first = MakeFirstSample({0.5, -0.2});
  // 当前位姿与首采样点几乎重合（转成度传入）
  std::vector<double> init_arm_deg = {0.5 * kRadToDeg, -0.2 * kRadToDeg};

  auto samples = BuildTactApproachSamples(first, init_arm_deg, {}, 0.01, {});
  EXPECT_TRUE(samples.empty());
}

TEST(TactApproachTest, EmptyOrShortInitReturnsEmpty) {
  auto first = MakeFirstSample({1.0, -0.5});

  EXPECT_TRUE(BuildTactApproachSamples(first, {}, {}, 0.01, {}).empty());
  // init 维度不足
  EXPECT_TRUE(BuildTactApproachSamples(first, {10.0}, {}, 0.01, {}).empty());
}

TEST(TactApproachTest, WaistInterpolated) {
  const double loop_dt = 0.01;
  TactApproachOptions opts;

  // 手臂已就位，仅腰部相差 0.6 rad
  auto first = MakeFirstSample({0.3}, /*has_waist=*/true, /*waist_q_rad=*/0.6);
  std::vector<double> init_arm_deg = {0.3 * kRadToDeg};
  std::vector<double> init_waist_deg = {0.0};

  auto samples = BuildTactApproachSamples(first, init_arm_deg, init_waist_deg,
                                          loop_dt, opts);
  ASSERT_FALSE(samples.empty());

  EXPECT_TRUE(samples.front().has_waist);
  EXPECT_NEAR(samples.front().waist_q, 0.0, 1e-6);
  EXPECT_NEAR(samples.back().waist_q, 0.6, opts.velocity * loop_dt * 1.5);

  // 腰部逐拍增量同样受峰值速度约束
  double max_step = 0.0;
  for (size_t i = 1; i < samples.size(); ++i) {
    max_step = std::max(max_step, std::abs(samples[i].waist_q - samples[i - 1].waist_q));
  }
  EXPECT_LE(max_step, opts.velocity * loop_dt * 1.05);

  // 手臂保持原位不动
  for (const auto& s : samples) {
    EXPECT_NEAR(s.q[0], 0.3, 1e-6);
  }
}

TEST(TactApproachTest, WaistHeldWhenInitWaistMissing) {
  const double loop_dt = 0.01;
  auto first = MakeFirstSample({1.0}, /*has_waist=*/true, /*waist_q_rad=*/0.6);
  std::vector<double> init_arm_deg = {0.0};

  auto samples = BuildTactApproachSamples(first, init_arm_deg, {}, loop_dt, {});
  ASSERT_FALSE(samples.empty());

  // 无腰部初始值时，过渡期间腰部锁定在首采样点，不产生阶跃后再回摆
  for (const auto& s : samples) {
    EXPECT_TRUE(s.has_waist);
    EXPECT_NEAR(s.waist_q, 0.6, 1e-9);
    EXPECT_NEAR(s.waist_v, 0.0, 1e-9);
  }
}

TEST(TactApproachTest, DurationClampedToMax) {
  const double loop_dt = 0.01;
  TactApproachOptions opts;

  // 3 rad 距离 → 理论 5.625s，应被钳制到 max_duration
  auto first = MakeFirstSample({3.0});
  std::vector<double> init_arm_deg = {0.0};

  auto samples = BuildTactApproachSamples(first, init_arm_deg, {}, loop_dt, opts);
  ASSERT_FALSE(samples.empty());
  EXPECT_NEAR(samples.size() * loop_dt, opts.max_duration, 2 * loop_dt);
}

TEST(TactApproachTest, DurationClampedToMin) {
  const double loop_dt = 0.01;
  TactApproachOptions opts;

  // 距离刚超过阈值 → 理论时长极短，应被钳制到 min_duration
  auto first = MakeFirstSample({0.01});
  std::vector<double> init_arm_deg = {0.0};

  auto samples = BuildTactApproachSamples(first, init_arm_deg, {}, loop_dt, opts);
  ASSERT_FALSE(samples.empty());
  EXPECT_NEAR(samples.size() * loop_dt, opts.min_duration, 2 * loop_dt);
}

}  // namespace
}  // namespace runtime
}  // namespace leju
