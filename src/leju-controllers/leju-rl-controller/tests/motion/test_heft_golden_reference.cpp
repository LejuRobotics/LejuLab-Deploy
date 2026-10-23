// Golden 一致性测试：同一段参考数据分别经
//   CsvHeftReferenceSource(MotionTrajectory) 与 LiveHeftReferenceProvider
// 采样，验证两条路径输出的参考窗口一致（policy 时间栅格对齐时应逐元素相等）。

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "leju-rl-controller/motion/csv_heft_reference_source.h"
#include "leju-rl-controller/motion/live_heft_reference_provider.h"
#include "leju-rl-controller/motion/motion_trajectory.h"

namespace leju {
namespace {

constexpr int kJoints = 21;
constexpr double kDt = 0.02;
const std::vector<int> kOffsets = {0, 1, 2, 3, 4, 6, 8, 10, 12, 16,
                                   -1, -2, -4, -8};

// 生成一段平滑合成运动的 deploy CSV（与 MotionTrajectory 解析格式一致）
std::string WriteSyntheticCsv(int num_frames) {
  std::string path = ::testing::TempDir() + "/heft_golden_motion.csv";
  std::ofstream f(path);
  f << "body_pos_x\tbody_pos_y\tbody_pos_z\tbody_quat_w\tbody_quat_x\t"
       "body_quat_y\tbody_quat_z";
  for (int j = 0; j < kJoints; ++j) f << "\tjoint_pos" << (j < 10 ? "0" : "") << j;
  for (int j = 0; j < kJoints; ++j) f << "\tjoint_vel_" << (j < 10 ? "0" : "") << j;
  f << "\n";
  for (int k = 0; k < num_frames; ++k) {
    const double t = k * kDt;
    const double yaw = 0.2 * std::sin(0.5 * t);
    const double w = std::cos(yaw / 2), z = std::sin(yaw / 2);
    f << 0.3 * t << "\t" << 0.05 * std::sin(t) << "\t"
      << 0.7 + 0.02 * std::cos(t) << "\t" << w << "\t0\t0\t" << z;
    for (int j = 0; j < kJoints; ++j) {
      f << "\t" << 0.1 * std::sin(t + 0.3 * j);
    }
    for (int j = 0; j < kJoints; ++j) {
      f << "\t" << 0.0;
    }
    f << "\n";
  }
  return path;
}

TEST(HeftGoldenReference, CsvAndLiveWindowsMatch) {
  const int kFrames = 200;
  const std::string csv_path = WriteSyntheticCsv(kFrames);

  // --- CSV 路径 ---
  MotionTrajectory motion;
  ASSERT_TRUE(motion.load(csv_path));
  ASSERT_EQ(motion.getNumFrames(), kFrames);
  CsvHeftReferenceSource csv_source([&motion]() { return &motion; });

  // --- Live 路径：按 20ms 栅格注入相同帧（recv_time = 帧号*0.02）---
  LiveHeftReferenceProvider::Config cfg;
  cfg.num_joints = kJoints;
  cfg.policy_dt = kDt;
  cfg.cursor_delay = 16 * kDt;  // 0.32s：游标落在 newest-16 帧
  cfg.history_margin = 8 * kDt + 0.02;
  cfg.fresh_timeout = 1e9;      // 测试用合成时间戳，关闭新鲜度门槛
  cfg.hold_timeout = 1e9;
  cfg.buffer_capacity_sec = 1e9;
  LiveHeftReferenceProvider live(cfg);

  for (int k = 0; k < kFrames; ++k) {
    live.injectFrame(k * kDt, true, true, 1, k + 1,
                     motion.getFrameBodyPos(k), motion.getFrameBodyQuat(k),
                     motion.getFrameJointPos(k));
  }

  // 游标 = newest - 0.32 = (kFrames-1-16)*kDt → CSV 帧号 kFrames-17
  const int csv_cursor = kFrames - 1 - 16;
  while (motion.getCurrentFrame() < csv_cursor) motion.next();
  ASSERT_EQ(motion.getCurrentFrame(), csv_cursor);

  std::vector<HeftRefFrame> w_csv, w_live;
  ASSERT_TRUE(csv_source.sampleWindow(kOffsets, w_csv));
  ASSERT_TRUE(live.sampleWindow(kOffsets, w_live));
  ASSERT_EQ(w_csv.size(), w_live.size());

  for (size_t i = 0; i < kOffsets.size(); ++i) {
    EXPECT_NEAR((w_csv[i].pos - w_live[i].pos).norm(), 0.0, 1e-9)
        << "pos mismatch at offset " << kOffsets[i];
    EXPECT_NEAR((w_csv[i].jpos - w_live[i].jpos).matrix().norm(), 0.0, 1e-9)
        << "jpos mismatch at offset " << kOffsets[i];
    const double quat_dot =
        std::abs(w_csv[i].quat.normalized().dot(w_live[i].quat.normalized()));
    EXPECT_NEAR(quat_dot, 1.0, 1e-9)
        << "quat mismatch at offset " << kOffsets[i];
  }

  std::remove(csv_path.c_str());
}

TEST(HeftGoldenReference, ResumeWaitsForReadyAndCorrectedFrame) {
  LiveHeftReferenceProvider::Config cfg;
  cfg.num_joints = kJoints;
  cfg.cursor_delay = 0.0;
  cfg.history_margin = 0.0;
  cfg.fresh_timeout = 1e9;
  cfg.buffer_capacity_sec = 1e9;

  auto inject = [](LiveHeftReferenceProvider& source, uint64_t seq,
                   double value) {
    source.injectFrame(
        static_cast<double>(seq), true, true, 1, seq,
        Eigen::Vector3d(value, 0.0, 0.7), Eigen::Quaterniond::Identity(),
        array_t::Constant(kJoints, value));
  };

  // Reference first, ready second.
  LiveHeftReferenceProvider frame_first(cfg);
  inject(frame_first, 1, 0.0);
  frame_first.setStreamFrozen(true);
  frame_first.requestStreamResume();
  inject(frame_first, 2, 1.0);
  EXPECT_TRUE(frame_first.streamFrozen());
  frame_first.injectResumeReady();
  EXPECT_FALSE(frame_first.streamFrozen());
  EXPECT_TRUE(frame_first.consumeReferenceDiscontinuity());

  // Ready first, reference second.
  LiveHeftReferenceProvider ready_first(cfg);
  inject(ready_first, 1, 0.0);
  ready_first.setStreamFrozen(true);
  ready_first.requestStreamResume();
  ready_first.injectResumeReady();
  EXPECT_TRUE(ready_first.streamFrozen());
  inject(ready_first, 2, 1.0);
  EXPECT_FALSE(ready_first.streamFrozen());
}

}  // namespace
}  // namespace leju

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
