#pragma once

#include <array>
#include <atomic>
#include <deque>
#include <memory>

#include "leju-rl-controller/controllers/generic_rl_controller.h"
#include "leju-rl-controller/depth/depth_observation_provider.h"
#include "kuavo_solver/ankle_solver.h"

namespace leju {

class DepthWalkController final : public GenericRLController {
 public:
  DepthWalkController(const RobotVersion& version, const std::string& name);
  bool initialize() override;
  void reset() override;
  bool depthReady() const;

 protected:
  bool loadConfig(const std::string& config_path) override;
  bool updateImpl(double time, const RobotState& state, const ImuData& imu,
                  RobotCmd& cmd) override;
  void updateRobotCmd(RobotCmd& cmd) override;
  void computeObservation() override;
  void computeActions() override;
  bool inferActions(const array_t& observation, array_t& inferred_actions) override;
  int customObservationSize() const override { return kNetworkObsSize; }

 private:
  static constexpr int kDepthSize = 8 * 36 * 64;
  static constexpr int kSingleObsSize = 18510;
  static constexpr int kFrameStack = 5;
  static constexpr int kNetworkObsSize = kSingleObsSize * kFrameStack;
  static constexpr int kActionSize = 21;
  static constexpr int kOutputSize = 25;

  array_t buildSingleObservation(const depth::DepthHistorySnapshot& depth);
  void clearDepthObservationHistory();
  bool isDepthSnapshotReady(const depth::DepthHistorySnapshot& snapshot) const;
  bool applyStartupPoseBlend(double time, RobotCmd& cmd);
  std::shared_ptr<depth::DepthObservationProvider> depth_provider_;
  std::deque<array_t> frame_stack_;
  double depth_timeout_sec_ = 0.1;
  double min_valid_ratio_ = 0.05;
  double phase_ = 0.0;
  int phase_stand_flag_ = 2;
  int phase_turns_ = 5;
  // Source DepthWalkController fills all five frame-stack entries before its
  // first inference. Keep the action cache at zero during the same warm-up.
  std::atomic<int> observation_warmup_count_{0};
  int diagnostic_inference_count_ = 0;
  std::atomic<double> gait_frequency_{1.0};
  std::atomic<double> fre_phase_raw_{0.0};
  double leg_bias_ = 0.5;
  double stance_ratio_ = 0.5;
  std::array<double, 3> velocity_estimate_{};
  mutable bool depth_ready_ = false;
  // One snapshot is prepared at the start of each control tick and consumed
  // by computeObservation(). This prevents a timeout decision from changing
  // between the readiness check and observation construction.
  depth::DepthHistorySnapshot tick_depth_snapshot_;
  bool tick_depth_snapshot_prepared_ = false;
  std::atomic<bool> observation_valid_this_tick_{false};
  // The S17 MuJoCo scene starts with zero joint angles, while the depth
  // policy is trained around its configured default posture.  Blend only the
  // depth controller into that posture before enabling inference; AMP keeps
  // its existing lifecycle unchanged.
  double startup_pose_duration_sec_ = 1.5;
  double startup_pose_max_wait_sec_ = 5.0;
  double startup_pose_position_tolerance_ = 0.015;
  double startup_pose_velocity_tolerance_ = 0.05;
  double startup_pose_start_time_ = 0.0;
  bool startup_pose_active_ = true;
  bool source_mujoco_executor_ = false;
  // Do not use the source q=0 executor until a validated policy action exists.
  std::atomic<bool> source_executor_action_ready_{false};
  std::unique_ptr<AnkleSolver> ankle_solver_s17_;
};

}  // namespace leju
