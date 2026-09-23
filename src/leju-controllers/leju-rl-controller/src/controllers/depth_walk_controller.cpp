#include "leju-rl-controller/controllers/depth_walk_controller.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cmath>
#include <sstream>

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include "leju-rl-controller/controllers/controller_registry.h"
#include "leju-rl-controller/controllers/depth_actuator_contract.h"
#include "leju-rl-controller/rl_log.h"

namespace leju {
namespace {
double SteadySeconds() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
}  // namespace

DepthWalkController::DepthWalkController(const RobotVersion& version,
                                         const std::string& name)
    : GenericRLController(version, name),
      depth_provider_(depth::GlobalDepthObservationProvider()) {
  if (version.major() == 1 && version.minor() == 7) {
    ankle_solver_s17_ = std::make_unique<AnkleSolver>();
    ankle_solver_s17_->getconfig(ANKLE_SOLVER_TYPE_S2GEN_2);
  }
}

bool DepthWalkController::initialize() {
  // loadConfig() (inside GenericRLController::initialize) must run before the
  // static DDS subscriber is created; otherwise it binds to the default sim
  // topic (rt/depth_camera/frame_meters_36x64) and never sees real-camera mm
  // frames even though ConfigureDepthDdsInput was called later.
  if (!GenericRLController::initialize()) return false;
  if (!depth::StartDepthDdsSubscription()) {
    RL_LOGE("DepthWalkController failed to start DDS depth subscription");
    return false;
  }
  if (policyJointCount() != kActionSize) {
    RL_LOGE("DepthWalkController requires 21 policy joints, got %d",
            policyJointCount());
    return false;
  }
  try {
    const auto probe = model_->forward(std::vector<float>(kNetworkObsSize, 0.0f));
    if (probe.size() != kOutputSize) {
      RL_LOGE("DepthWalkController model output mismatch: got %zu, expected %d",
              probe.size(), kOutputSize);
      return false;
    }
  } catch (const std::exception& e) {
    RL_LOGE("DepthWalkController model contract check failed: %s", e.what());
    return false;
  }
  RL_LOGI("DepthWalkController contract: float32[%d] -> float32[%d]",
          kNetworkObsSize, kOutputSize);
  return true;
}

bool DepthWalkController::loadConfig(const std::string& path) {
  if (!GenericRLController::loadConfig(path)) return false;
  try {
    const auto root = YAML::LoadFile(path);
    const auto depth_cfg = root["HumanoidRobotCfg"]["depth"];
    if (depth_cfg) {
      if (depth_cfg["timeout_sec"])
        depth_timeout_sec_ = depth_cfg["timeout_sec"].as<double>();
      if (depth_cfg["min_valid_ratio"])
        min_valid_ratio_ = depth_cfg["min_valid_ratio"].as<double>();
      depth::DepthDdsInputConfig input;
      if (depth_cfg["input_topic"])
        input.topic = depth_cfg["input_topic"].as<std::string>();
      if (depth_cfg["input_width"])
        input.width = depth_cfg["input_width"].as<int>();
      if (depth_cfg["input_height"])
        input.height = depth_cfg["input_height"].as<int>();
      if (depth_cfg["input_frame_decimation"])
        input.frame_decimation =
            depth_cfg["input_frame_decimation"].as<int>();
      if (depth_cfg["input_unit"]) {
        const std::string unit = depth_cfg["input_unit"].as<std::string>();
        if (unit == "millimeters" || unit == "mm") {
          input.unit = depth::DepthUnit::kMillimeters;
        } else if (unit == "meters" || unit == "m") {
          input.unit = depth::DepthUnit::kMeters;
        } else {
          throw std::runtime_error("depth.input_unit must be meters or millimeters");
        }
      }
      depth::ConfigureDepthDdsInput(input);
      RL_LOGI("Depth DDS input: topic=%s %dx%d unit=%s decimation=%d "
              "(legacy 64x36 meters remains accepted)",
              input.topic.c_str(), input.width, input.height,
              input.unit == depth::DepthUnit::kMillimeters ? "mm" : "m",
              input.frame_decimation);
    }
    const auto env = root["HumanoidRobotCfg"]["env"];
    if (env["leg_bias"]) leg_bias_ = env["leg_bias"].as<double>();
    if (env["stance_ratio"]) stance_ratio_ = env["stance_ratio"].as<double>();
    if (env["startup_pose_duration_sec"]) startup_pose_duration_sec_ = std::max(0.0, env["startup_pose_duration_sec"].as<double>());
    if (env["startup_pose_max_wait_sec"]) startup_pose_max_wait_sec_ = std::max(startup_pose_duration_sec_, env["startup_pose_max_wait_sec"].as<double>());
    if (env["startup_pose_position_tolerance"]) startup_pose_position_tolerance_ = std::max(0.0, env["startup_pose_position_tolerance"].as<double>());
    if (env["startup_pose_velocity_tolerance"]) startup_pose_velocity_tolerance_ = std::max(0.0, env["startup_pose_velocity_tolerance"].as<double>());
    if (env["source_mujoco_executor"])
      source_mujoco_executor_ = env["source_mujoco_executor"].as<bool>();
    if (source_mujoco_executor_) {
      const char* runtime = std::getenv("LEJU_MUJOCO_SIM");
      const bool authorized =
          runtime && (std::string(runtime) == "1" ||
                      std::string(runtime) == "true" ||
                      std::string(runtime) == "TRUE");
      if (!authorized) {
        RL_LOGE("source_mujoco_executor=true is MuJoCo-only; refuse to "
                "load without LEJU_MUJOCO_SIM=1");
        return false;
      }
    }
  } catch (const std::exception& e) {
    RL_LOGE("DepthWalkController depth config error: %s", e.what());
    return false;
  }
  return true;
}

void DepthWalkController::reset() {
  GenericRLController::reset();
  frame_stack_.assign(kFrameStack, array_t::Zero(kSingleObsSize));
  observation_warmup_count_.store(0);
  diagnostic_inference_count_ = 0;
  phase_ = 0.0;
  phase_stand_flag_ = 2;
  phase_turns_ = 5;
  fre_phase_raw_.store(0.0);
  gait_frequency_.store(1.0);
  velocity_estimate_.fill(0.0);
  depth_ready_ = false;
  tick_depth_snapshot_ = depth::DepthHistorySnapshot{};
  tick_depth_snapshot_prepared_ = false;
  observation_valid_this_tick_.store(false);
  startup_pose_start_time_ = 0.0;
  startup_pose_active_ = true;
  source_executor_action_ready_.store(false);
  depth_provider_->reset();
}

bool DepthWalkController::depthReady() const {
  const auto snapshot =
      depth_provider_->snapshot(SteadySeconds(), depth_timeout_sec_);
  return isDepthSnapshotReady(snapshot);
}

bool DepthWalkController::isDepthSnapshotReady(
    const depth::DepthHistorySnapshot& snapshot) const {
  return snapshot.ready && snapshot.data.size() == kDepthSize &&
         snapshot.valid_ratio >= min_valid_ratio_;
}

void DepthWalkController::clearDepthObservationHistory() {
  frame_stack_.assign(kFrameStack, array_t::Zero(kSingleObsSize));
  observations_.setZero();
  observation_warmup_count_.store(0);
  observation_valid_this_tick_.store(false);
}

bool DepthWalkController::updateImpl(double time, const RobotState& state,
                                     const ImuData& imu, RobotCmd& cmd) {
  (void)state;
  (void)imu;
  // Keep this guard before the depth-ready check: a delayed camera must not
  // leave the robot at the scene's zero pose until the first valid frame.
  if (applyStartupPoseBlend(time, cmd)) return true;
  tick_depth_snapshot_ =
      depth_provider_->snapshot(SteadySeconds(), depth_timeout_sec_);
  tick_depth_snapshot_prepared_ = true;
  if (!isDepthSnapshotReady(tick_depth_snapshot_)) {
    if (depth_ready_ || observation_warmup_count_.load() > 0) {
      clearDepthObservationHistory();
    }
    depth_ready_ = false;
    // 深度首帧/连续有效帧未到达时不能返回空 RobotCmd；空命令会被
    // ControlLoop 丢弃，底层执行器随后失去保持目标。GenericRLController
    // 的非策略关节保持逻辑也在这里复用，确保切入深度控制器是安全的。
    const size_t motor_count = current_state_.q.size();
    if (cmd.q.size() != motor_count) cmd.resize(motor_count);
    for (size_t i = 0; i < motor_count; ++i) {
      const double q = i < static_cast<int>(current_state_.q.size())
                           ? current_state_.q[i]
                           : 0.0;
      cmd.q[i] = q;
      cmd.v[i] = 0.0;
      cmd.tau[i] = 0.0;
      cmd.modes[i] = 2;
      cmd.kp[i] = 100.0;
      cmd.kd[i] = 10.0;
    }
    return true;
  }
  depth_ready_ = true;
  return GenericRLController::updateImpl(time, state, imu, cmd);
}

bool DepthWalkController::applyStartupPoseBlend(double time, RobotCmd& cmd) {
  if (!startup_pose_active_) return false;
  if (startup_pose_start_time_ <= 0.0) {
    startup_pose_start_time_ = time;
    RL_LOGI("DepthWalkController: blending startup posture for %.2f s",
            startup_pose_duration_sec_);
  }
  const double elapsed = time - startup_pose_start_time_;
  bool settled = elapsed >= startup_pose_duration_sec_;
  double max_position_error = 0.0;
  double max_velocity = 0.0;
  for (int policy = 0; policy < policyJointCount(); ++policy) {
    const int motor = policy_joint_ids_[policy];
    if (motor < 0 || motor >= static_cast<int>(current_state_.q.size()) ||
        policy >= default_joint_pos_.size()) continue;
    const double target = joint_direction_[policy] * default_joint_pos_[policy];
    max_position_error = std::max(max_position_error,
                                  std::abs(target - current_state_.q[motor]));
    if (motor < static_cast<int>(current_state_.v.size()))
      max_velocity = std::max(max_velocity, std::abs(current_state_.v[motor]));
  }
  settled = settled && max_position_error <= startup_pose_position_tolerance_ &&
            max_velocity <= startup_pose_velocity_tolerance_;
  const bool timed_out = startup_pose_max_wait_sec_ > 0.0 &&
                         elapsed >= startup_pose_max_wait_sec_;
  // Do not remain in the startup hold forever when the simulator publishes
  // a small residual velocity.  The source controller starts policy after
  // the transition window; the max-wait value is the safety escape hatch.
  if (startup_pose_duration_sec_ <= 0.0 || settled || timed_out) {
    startup_pose_active_ = false;
    RL_LOGI("DepthWalkController: startup posture %s, enabling policy (elapsed=%.2f pos_err=%.4f vel=%.4f)",
            timed_out && !settled ? "timeout" : "settled", elapsed,
            max_position_error, max_velocity);
    return false;
  }

  const double alpha = std::clamp(
      (time - startup_pose_start_time_) / startup_pose_duration_sec_, 0.0, 1.0);
  const size_t motor_count = current_state_.q.size();
  if (cmd.q.size() != motor_count) cmd.resize(motor_count);
  for (size_t motor = 0; motor < motor_count; ++motor) {
    const double current = current_state_.q[motor];
    cmd.q[motor] = current;
    cmd.v[motor] = 0.0;
    cmd.tau[motor] = 0.0;
    cmd.modes[motor] = 2;
    cmd.kp[motor] = 100.0;
    cmd.kd[motor] = 10.0;
  }
  for (int policy = 0; policy < policyJointCount(); ++policy) {
    const int motor = policy_joint_ids_[policy];
    if (motor < 0 || motor >= static_cast<int>(motor_count) ||
        policy >= default_joint_pos_.size()) {
      continue;
    }
    const double target = joint_direction_[policy] * default_joint_pos_[policy];
    cmd.q[motor] = current_state_.q[motor] +
                   alpha * (target - current_state_.q[motor]);
    cmd.kp[motor] = joint_kp_[policy];
    cmd.kd[motor] = joint_kd_[policy];
  }
  return true;
}

void DepthWalkController::updateRobotCmd(RobotCmd& cmd) {
  // Keep target DDS/MuJoCo command semantics: GenericRLController emits
  // the complete joint-space PD torque and holds q=current for CST.
  GenericRLController::updateRobotCmd(cmd);
  const char* runtime = std::getenv("LEJU_MUJOCO_SIM");
  const bool mujoco_runtime_authorized =
      runtime && (std::string(runtime) == "1" ||
                  std::string(runtime) == "true" ||
                  std::string(runtime) == "TRUE");
  if (leju::depth::sourceMujocoExecutorEnabled(
          source_mujoco_executor_, source_executor_action_ready_.load(),
          mujoco_runtime_authorized)) {
    // Match the ROS S17 path: ankle damping is computed in motor velocity
    // space and mapped back to joint torque. Direct -Kd*joint_velocity is not
    // equivalent for the coupled ankle transmission and causes chatter.
    if (ankle_solver_s17_ && current_state_.q.size() >= 12 &&
        current_state_.v.size() >= 12) {
      Eigen::VectorXd q_leg(12), v_leg(12);
      for (int j = 0; j < 12; ++j) {
        q_leg[j] = current_state_.q[j];
        v_leg[j] = current_state_.v[j];
      }
      const Eigen::VectorXd motor_pos =
          ankle_solver_s17_->joint_to_motor_position(q_leg);
      const Eigen::VectorXd motor_vel =
          ankle_solver_s17_->joint_to_motor_velocity(q_leg, motor_pos, v_leg);
      Eigen::VectorXd motor_damping(12);
      for (int j = 0; j < 12; ++j)
        motor_damping[j] = -joint_kd_[j + 1] * motor_vel[j];
      const Eigen::VectorXd joint_damping =
          ankle_solver_s17_->motor_to_joint_torque(q_leg, motor_pos,
                                                   motor_damping);
      for (int j = 0; j < 12; ++j) {
        const int policy = j + 1;  // policy index 0 is waist-first
        const int motor = policy_joint_ids_[policy];
        if (motor < 0 || motor >= static_cast<int>(cmd.tau.size())) continue;
        // GenericRLController already computed the proportional term. Recover
        // it before replacing only the damping component.
        const double proportional = cmd.tau[motor] +
            joint_kd_[policy] * current_state_.v[motor];
        cmd.tau[motor] = std::clamp(
            proportional + joint_damping[j],
            -joint_torque_limit_[policy], joint_torque_limit_[policy]);
      }
    }
    // Source ROS MuJoCo writes the already-computed actuation torque directly
    // to the joint actuator.  Use mode 3 as a target-side direct-torque
    // escape hatch; do not add a second PD loop here.
    for (int i = 0; i < policyJointCount(); ++i) {
      const int motor = policy_joint_ids_[i];
      if (motor < 0 || motor >= static_cast<int>(cmd.q.size())) continue;
      cmd.q[motor] = 0.0;
      cmd.v[motor] = 0.0;
      cmd.kp[motor] = 0.0;
      cmd.kd[motor] = 0.0;
      cmd.modes[motor] = 3;
    }
  }
}

array_t DepthWalkController::buildSingleObservation(
    const depth::DepthHistorySnapshot& depth_snapshot) {
  array_t single = array_t::Zero(kSingleObsSize);
  for (int i = 0; i < kDepthSize; ++i)
    single[i] = std::clamp(static_cast<double>(depth_snapshot.data[i]), 0.0, 1.0);

  // Keep depth and AMP on exactly the same DDS command path.  The depth
  // controller owns its image stack, but must still consume Generic's
  // scaled/filtered command cache rather than the raw joystick values.
  const array_t command_values = filteredVelocityCommandSnapshot();
  const double command_x = command_values.size() > 0 ? command_values[0] : 0.0;
  const double command_y = command_values.size() > 1 ? command_values[1] : 0.0;
  const double command_w = command_values.size() > 2 ? command_values[2] : 0.0;
  single.segment<3>(18432) << command_x, command_y, command_w;

  const double gait_frequency = gait_frequency_.load();
  // Exact source updatePhase() semantics. Source SensorData is waist-first,
  // so source indices 2/8 must be resolved through the policy-to-motor map.
  const double delta_phase = 0.02 * gait_frequency;
  phase_ += delta_phase;
  const double cmd_vxc = 1.0;
  const double x = std::min(command_x, cmd_vxc);
  const bool near_zero_command =
      std::abs(x) < 0.01 &&
      std::abs(command_y * cmd_vxc) < 0.01 &&
      std::abs(command_w * cmd_vxc) < 0.01;
  const double estimated_speed =
      std::sqrt(velocity_estimate_[0] * velocity_estimate_[0] +
                velocity_estimate_[1] * velocity_estimate_[1] +
                velocity_estimate_[2] * velocity_estimate_[2]);
  if (near_zero_command) {
    if (estimated_speed < 0.2 || cmd_vxc < 0.5) {
      phase_stand_flag_ = 1;
    }
    double par = 0.0;
    if (policy_joint_ids_.size() > 8) {
      const int left_motor = policy_joint_ids_[2];
      const int right_motor = policy_joint_ids_[8];
      if (left_motor >= 0 && right_motor >= 0 &&
          left_motor < static_cast<int>(current_state_.q.size()) &&
          right_motor < static_cast<int>(current_state_.q.size())) {
        par = current_state_.q[left_motor] - current_state_.q[right_motor];
      }
    }
    if (phase_stand_flag_ > 0 &&
        (phase_ > 0.99999 || phase_ < delta_phase + 1e-6)) {
      if ((par < 0.08 && par > -0.08) || phase_turns_ >= 2) {
        phase_ = 0.0;
        phase_stand_flag_ = 2;
      } else {
        ++phase_turns_;
      }
    }
    if (estimated_speed > 0.3 && cmd_vxc > 0.5 &&
        phase_stand_flag_ == 2) {
      phase_stand_flag_ = 0;
      phase_turns_ = 0;
    }
  } else {
    phase_stand_flag_ = 0;
    phase_turns_ = 0;
  }
  phase_ = std::fmod(phase_, 1.0);
  single.segment<5>(18435) << std::sin(2.0 * M_PI * phase_),
      std::cos(2.0 * M_PI * phase_), leg_bias_, stance_ratio_, gait_frequency;
  single.segment<3>(18440) << current_imu_.gyro[0], current_imu_.gyro[1],
      current_imu_.gyro[2];

  Eigen::Quaterniond q(current_imu_.quat[0], current_imu_.quat[1],
                       current_imu_.quat[2], current_imu_.quat[3]);
  if (q.norm() < 1e-9) q = Eigen::Quaterniond::Identity();
  q.normalize();
  const Eigen::Vector3d gravity =
      q.toRotationMatrix().transpose() * Eigen::Vector3d(0.0, 0.0, -1.0);
  single.segment<3>(18443) = gravity.array();

  // The ROS controller puts getCurrentAction() in the observation.  In the
  // DDS controller this is the action cache, not last_actions_: last_actions_
  // is only the value kept for bookkeeping when the cache is replaced and
  // would make the policy action-history one inference behind.
  const array_t current_action = currentActionSnapshot();
  for (int i = 0; i < kActionSize; ++i) {
    const int motor = policy_joint_ids_[i];
    if (motor < 0 || motor >= static_cast<int>(current_state_.q.size()) ||
        motor >= static_cast<int>(current_state_.v.size()))
      continue;
    single[18446 + i] =
        joint_direction_[i] * current_state_.q[motor] - default_joint_pos_[i];
    single[18467 + i] = joint_direction_[i] * current_state_.v[motor];
    single[18488 + i] =
        (i < current_action.size()) ? current_action[i] : 0.0;
  }
  single[18509] = fre_phase_raw_.load();
  return single;
}

void DepthWalkController::computeObservation() {
  // Depth uses a custom image stack, so explicitly run the common command
  // cache update that GenericRLController::computeObservation normally does.
  refreshVelocityCommandCache();
  if (!tick_depth_snapshot_prepared_ ||
      !isDepthSnapshotReady(tick_depth_snapshot_)) {
    clearDepthObservationHistory();
    return;
  }
  if (frame_stack_.size() != kFrameStack)
    frame_stack_.assign(kFrameStack, array_t::Zero(kSingleObsSize));
  frame_stack_.pop_front();
  frame_stack_.push_back(buildSingleObservation(tick_depth_snapshot_));
  const int warmup_count = std::min(
      kFrameStack, observation_warmup_count_.fetch_add(1) + 1);
  if (warmup_count < kFrameStack) {
    // Match the ROS controller's pre_load_frames_num=5: do not expose a
    // partially zero-padded frame stack to the policy.
    observations_.setZero();
    observation_valid_this_tick_.store(false);
    return;
  }
  int offset = 0;
  for (const auto& frame : frame_stack_) {
    observations_.segment(offset, kSingleObsSize) = frame;
    offset += kSingleObsSize;
  }
  observation_valid_this_tick_.store(true);
}

void DepthWalkController::computeActions() {
  array_t inferred;
  if (inferActions(observations_, inferred)) updateActionCache(inferred);
}

bool DepthWalkController::inferActions(const array_t& observation,
                                       array_t& action) {
  if (!observation_valid_this_tick_.load() ||
      observation_warmup_count_.load() < kFrameStack ||
      observation.size() != kNetworkObsSize || !model_ || !model_->isLoaded())
    return false;
  std::vector<float> input(static_cast<size_t>(kNetworkObsSize));
  for (int i = 0; i < kNetworkObsSize; ++i)
    input[static_cast<size_t>(i)] = static_cast<float>(observation[i]);
  const auto output = model_->forward(input);
  if (output.size() != kOutputSize) {
    RL_LOGE("DepthWalkController output mismatch: got %zu, expected %d",
            output.size(), kOutputSize);
    return false;
  }
  action.resize(kActionSize);
  for (int i = 0; i < kActionSize; ++i) action[i] = output[i];
  if (std::getenv("LEJU_DEPTH_FREEZE_ACTION") != nullptr) {
    static array_t frozen_action;
    if (frozen_action.size() == 0) frozen_action = action;
    action = frozen_action;
  }
  gait_frequency_.store(std::tanh(output[21] / 4.0) * 0.2 + 1.0);
  fre_phase_raw_.store(output[21]);
  for (int i = 0; i < 3; ++i) velocity_estimate_[i] = output[22 + i];
  if (diagnostic_inference_count_ < 8) {
    double depth_min = observation[0];
    double depth_max = observation[0];
    double depth_sum = 0.0;
    for (int i = 0; i < kDepthSize; ++i) {
      depth_min = std::min(depth_min, observation[i]);
      depth_max = std::max(depth_max, observation[i]);
      depth_sum += observation[i];
    }
    std::ostringstream action_text;
    action_text.setf(std::ios::fixed);
    action_text.precision(5);
    for (int i = 0; i < kActionSize; ++i) {
      if (i != 0) action_text << ',';
      action_text << action[i];
    }
    RL_LOGI("Depth diagnostic #%d: depth[min=%.5f max=%.5f mean=%.5f] "
            "cmd=[%.5f %.5f %.5f] phase=[%.5f %.5f %.5f %.5f %.5f] "
            "action=[%s]",
            diagnostic_inference_count_, depth_min, depth_max,
            depth_sum / kDepthSize, observation[18432], observation[18433],
            observation[18434], observation[18435], observation[18436],
            observation[18437], observation[18438], observation[18439],
            action_text.str().c_str());

    // Boundary diagnostic: print exactly the non-image values that enter the
    // 92550-element policy tensor.  This is intentionally limited to the
    // first few frames so it cannot perturb the control loop in steady state.
    if (diagnostic_inference_count_ == 0) {
      std::ostringstream state_text;
      state_text.setf(std::ios::fixed);
      state_text.precision(6);
      state_text << "gyro=[" << observation[18440] << ' ' << observation[18441]
                 << ' ' << observation[18442] << "] gravity=["
                 << observation[18443] << ' ' << observation[18444] << ' '
                 << observation[18445] << "] joint_pos=[";
      for (int i = 0; i < kActionSize; ++i) {
        if (i) state_text << ' ';
        state_text << observation[18446 + i];
      }
      state_text << "] joint_vel=[";
      for (int i = 0; i < kActionSize; ++i) {
        if (i) state_text << ' ';
        state_text << observation[18467 + i];
      }
      state_text << "] action_hist=[";
      for (int i = 0; i < kActionSize; ++i) {
        if (i) state_text << ' ';
        state_text << observation[18488 + i];
      }
      state_text << "] fre_raw=" << observation[18509];
      RL_LOGI("Depth observation boundary: %s", state_text.str().c_str());

      // Each depth frame is 2304 values.  Report only three pixels per frame
      // (top-left, center, bottom-right) and the frame mean to verify frame
      // order without dumping a large tensor to the log.
      std::ostringstream depth_text;
      depth_text.setf(std::ios::fixed);
      depth_text.precision(6);
      for (int frame = 0; frame < kFrameStack; ++frame) {
        const int base = frame * kSingleObsSize;
        double frame_sum = 0.0;
        for (int i = 0; i < kDepthSize; ++i) frame_sum += observation[base + i];
        if (frame) depth_text << " | ";
        depth_text << "f" << frame << " mean=" << frame_sum / kDepthSize
                   << " tl=" << observation[base]
                   << " c=" << observation[base + (36 / 2) * 64 + 64 / 2]
                   << " br=" << observation[base + kDepthSize - 1];
      }
      RL_LOGI("Depth observation frames: %s", depth_text.str().c_str());
    }
    ++diagnostic_inference_count_;
  }
  source_executor_action_ready_.store(true);
  return true;
}

REGISTER_CONTROLLER("DepthWalkController", DepthWalkController, depth_walk);

}  // namespace leju
