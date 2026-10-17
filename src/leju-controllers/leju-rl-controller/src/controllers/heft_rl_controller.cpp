#include "leju-rl-controller/controllers/heft_rl_controller.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <filesystem>

#include "leju-rl-controller/controllers/controller_registry.h"
#include "leju-rl-controller/inference/model_factory.h"
#include "leju-rl-controller/motion/csv_heft_reference_source.h"
#include "leju-rl-controller/motion/heft_csv_record_gate.h"
#include "leju-rl-controller/motion/heft_policy_csv_recorder.h"
#include "leju-rl-controller/motion/heft_stream_freeze_gate.h"
#include "leju-rl-controller/motion/live_heft_reference_provider.h"
#include "leju-rl-controller/rl_log.h"

#include <yaml-cpp/yaml.h>

namespace leju {

namespace {
constexpr double kGravityZ = -1.0;

// Align with HEFT math.py; rot6d is column-major (unlike getMotionAnchorOriB).
Eigen::Vector3d quatApplyInverse(const Eigen::Quaterniond& q,
                                 const Eigen::Vector3d& v) {
  return q.conjugate() * v;
}

Eigen::Quaterniond quatInFrame(const Eigen::Quaterniond& frame_quat,
                               const Eigen::Quaterniond& quat_w) {
  return frame_quat.conjugate() * quat_w;
}

array_t quatToRot6d(const Eigen::Quaterniond& q) {
  const Eigen::Matrix3d R = q.normalized().toRotationMatrix();
  array_t out(6);
  out << R(0, 0), R(1, 0), R(2, 0), R(0, 1), R(1, 1), R(2, 1);
  return out;
}
}  // namespace

HeftRLController::HeftRLController(const RobotVersion& version,
                                   const std::string& name)
    : GenericRLController(version, name) {}

HeftRLController::~HeftRLController() {
  if (policy_csv_recorder_ && policy_csv_recorder_->recording()) {
    // Synchronous: the process is going away, make sure the file lands.
    policy_csv_recorder_->stopAndSave("shutdown", /*async=*/false);
  }
}

void HeftRLController::pause() {
  // Controller switched away (fallback to amp, manual switch, ...): stop the
  // take now so a later re-activation cannot splice two time ranges together.
  if (policy_csv_recorder_ && policy_csv_recorder_->recording()) {
    policy_csv_recorder_->stopAndSave("controller switch");
  }
  GenericRLController::pause();
}

void HeftRLController::reset() {
  GenericRLController::reset();
  boot_counter_ = boot_indicator_max_;
  history_primed_ = false;
  live_yaw_initialized_ = false;
  resetActionFilter();
  // Drop start/stop presses latched while this controller was inactive, so
  // activation does not suddenly begin/end a take.
  HeftCsvRecordGate::instance().consumeStart();
  HeftCsvRecordGate::instance().consumeStop();
  if (reference_source_) {
    reference_source_->onActivate();
  }
}

bool HeftRLController::initialize() {
  if (!GenericRLController::initialize()) {
    return false;
  }

  // 构建参考源（loadConfig 已确定模式）
  if (reference_source_mode_ == "live") {
    if (!live_provider_ || !live_provider_->startSubscription()) {
      RL_LOG_FAILURE("HEFT live reference subscription failed");
      return false;
    }
    RL_LOG_SUCCESS("HEFT reference source: live (Pico/GMR DDS)");
  } else {
    RL_LOG_INFO("HEFT reference source: csv (offline playback)");
  }
  return true;
}

bool HeftRLController::loadConfig(const std::string& config_path) {
  if (!GenericRLController::loadConfig(config_path)) {
    return false;
  }

  try {
    YAML::Node root = YAML::LoadFile(config_path);
    YAML::Node heft = root["HumanoidRobotCfg"]["env"]["heft"];
    if (!heft) {
      RL_LOG_FAILURE("Missing 'HumanoidRobotCfg.env.heft' in: %s",
                     config_path.c_str());
      return false;
    }

    future_steps_ = heft["future_steps"].as<std::vector<int>>();
    history_taps_ = heft["history_steps"].as<std::vector<int>>();
    prev_action_steps_ = heft["prev_action_steps"].as<int>(8);
    boot_indicator_max_ = heft["boot_indicator_max"].as<double>(25.0);
    action_clip_ = heft["action_clip"].as<double>(10.0);
    action_alpha_ = heft["action_alpha"].as<double>(0.9);
    output_name_ = heft["output_name"].as<std::string>("action");
    expected_obs_dim_ = heft["expected_obs_dim"].as<int>(0);

    if (future_steps_.empty() || history_taps_.empty()) {
      RL_LOG_FAILURE("env.heft.future_steps / history_steps must be non-empty");
      return false;
    }
    for (int tap : history_taps_) {
      if (tap < 0) {
        RL_LOG_FAILURE("env.heft.history_steps must be >= 0 (got %d)", tap);
        return false;
      }
    }
    if (action_alpha_ < 0.0 || action_alpha_ > 1.0) {
      RL_LOG_FAILURE("env.heft.action_alpha must be in [0, 1] (got %f)",
                     action_alpha_);
      return false;
    }

    // ==================== 参考源配置 ====================
    if (getName() == "heft_pico") {
      reference_source_mode_ = "live";
    } else if (getName() == "heft") {
      reference_source_mode_ = "csv";
    } else {
      reference_source_mode_ =
          heft["reference_source"].as<std::string>("csv");
    }
    RL_LOG_INFO("HEFT '%s': reference_source=%s (config=%s)",
                getName().c_str(), reference_source_mode_.c_str(),
                config_path.c_str());

    if (reference_source_mode_ == "live") {
      YAML::Node live = heft["live"];
      LiveHeftReferenceProvider::Config cfg;
      if (live) {
        cfg.topic = live["topic"].as<std::string>(cfg.topic);
        cfg.expected_robot_version =
            live["robot_version"].as<std::string>(cfg.expected_robot_version);
        cfg.expected_schema_version =
            live["schema_version"].as<uint32_t>(cfg.expected_schema_version);
        cfg.cursor_delay = live["cursor_delay"].as<double>(cfg.cursor_delay);
        cfg.fresh_timeout = live["fresh_timeout"].as<double>(cfg.fresh_timeout);
        cfg.hold_timeout = live["hold_timeout"].as<double>(cfg.hold_timeout);
        cfg.buffer_capacity_sec =
            live["buffer_capacity"].as<double>(cfg.buffer_capacity_sec);
        fallback_controller_ =
            live["fallback_controller"].as<std::string>(fallback_controller_);
      }
      cfg.policy_dt = policy_dt_;
      // 覆盖需求由 future/history 偏移推导（默认值兜底）
      const int max_future =
          *std::max_element(future_steps_.begin(), future_steps_.end());
      const int min_offset =
          *std::min_element(future_steps_.begin(), future_steps_.end());
      const double derived_delay = max_future * policy_dt_;
      if (!live || !live["cursor_delay"]) {
        cfg.cursor_delay = derived_delay;
      } else if (cfg.cursor_delay + 1e-9 < derived_delay) {
        RL_LOG_WARNING(
            "HEFT live.cursor_delay %.3f < required %.3f (max future %d); "
            "future window will clamp to newest frame",
            cfg.cursor_delay, derived_delay, max_future);
      }
      cfg.history_margin =
          std::max(0.02, -static_cast<double>(std::min(0, min_offset)) *
                             policy_dt_ + 0.02);

      auto provider = std::make_unique<LiveHeftReferenceProvider>(cfg);
      live_provider_ = provider.get();
      reference_source_ = std::move(provider);

      // Record policy-sampled 50 Hz reference (sampleWindow offset 0).
      std::string record_dir = "../motion_data/pico_recording";
      if (live && live["record_csv_dir"]) {
        record_dir = live["record_csv_dir"].as<std::string>();
      }
      const std::filesystem::path cfg_dir =
          std::filesystem::path(config_path).parent_path();
      const std::filesystem::path abs_dir =
          (cfg_dir / record_dir).lexically_normal();
      policy_csv_recorder_ =
          std::make_unique<HeftPolicyCsvRecorder>(abs_dir.string());
      RL_LOG_INFO("HEFT live policy CSV recorder → %s (LT+Y start / LT+X stop)",
                  abs_dir.string().c_str());
    } else if (reference_source_mode_ == "csv") {
      reference_source_ = std::make_unique<CsvHeftReferenceSource>(
          [this]() { return getCurrentMotion(); });
      live_provider_ = nullptr;
      policy_csv_recorder_.reset();
    } else {
      RL_LOG_FAILURE("Unknown env.heft.reference_source '%s' (csv|live)",
                     reference_source_mode_.c_str());
      return false;
    }

    boot_counter_ = boot_indicator_max_;
    resetActionFilter();
    return true;
  } catch (const std::exception& e) {
    RL_LOG_FAILURE("HEFT config error in %s: %s", config_path.c_str(), e.what());
    return false;
  }
}

bool HeftRLController::loadPolicy(const std::string& policy_path) {
  // Must set output name before load(): resolve happens at load time.
  // HEFT ONNX has many heads; default index 0 is often "policy_1" (obs-sized), not "action".
  model_ = ModelFactory::create(inference_engine_);
  if (!model_) {
    RL_LOG_FAILURE("Failed to create inference model: %s",
                   inference_engine_.c_str());
    return false;
  }
  model_->setOutputName(output_name_);
  if (!model_->load(policy_path)) {
    RL_LOG_FAILURE("Failed to load HEFT policy: %s", policy_path.c_str());
    return false;
  }
  if (!model_->isLoaded()) {
    RL_LOG_FAILURE("Output '%s' not found: %s",
                   output_name_.c_str(), policy_path.c_str());
    return false;
  }
  RL_LOG_SUCCESS("HEFT policy loaded: %s (output='%s')",
                 policy_path.c_str(), output_name_.c_str());
  return true;
}

bool HeftRLController::isActivationReady(std::string& reason) const {
  if (reference_source_mode_ != "live") {
    return true;  // csv 模式保持现状（无就绪门槛）
  }
  if (!reference_source_ || !reference_source_->ready()) {
    if (live_provider_) {
      reason = "HEFT live reference not ready (calibrated? buffered? fresh? "
               "age=" + std::to_string(live_provider_->newestAgeSec()) +
               "s frames=" + std::to_string(live_provider_->bufferedFrames()) + ")";
    } else {
      reason = "HEFT live reference provider missing";
    }
    return false;
  }
  return true;
}

bool HeftRLController::requestsFallback(std::string& to) const {
  if (reference_source_mode_ != "live" || !reference_source_) {
    return false;
  }
  if (reference_source_->degraded()) {
    to = fallback_controller_;
    return true;
  }
  return false;
}

int HeftRLController::expectedObsDim() const {
  const int S = static_cast<int>(future_steps_.size());
  const int H = static_cast<int>(history_taps_.size());
  const int J = policy_joint_count_;
  return 1 + 3 * (S - 1) + 6 * S + 2 * J * S + S + 3 * S +
         2 * 3 * H + 2 * J * H + J * prev_action_steps_;
}

void HeftRLController::initObsHistory() {
  const int J = policy_joint_count_;
  if (J <= 0) {
    RL_LOG_FAILURE("initObsHistory before joint mapping (J=%d)", J);
    return;
  }

  const int cap = *std::max_element(history_taps_.begin(), history_taps_.end()) + 1;
  angvel_buf_.init(cap, 3);
  projg_buf_.init(cap, 3);
  jpos_buf_.init(cap, J);
  jvel_buf_.init(cap, J);
  action_buf_.init(std::max(prev_action_steps_, 1), J);

  const int obs_dim = expectedObsDim();
  if (expected_obs_dim_ > 0 && obs_dim != expected_obs_dim_) {
    RL_LOG_FAILURE("HEFT obs dim mismatch: computed %d, expected %d",
                   obs_dim, expected_obs_dim_);
    throw std::runtime_error("HEFT observation dimension mismatch");
  }

  observations_.resize(obs_dim);
  observations_.setZero();
}

void HeftRLController::resetObsHistory() {
  if (angvel_buf_.initialized()) {
    angvel_buf_.resetZero();
    projg_buf_.resetZero();
    jpos_buf_.resetZero();
    jvel_buf_.resetZero();
    action_buf_.resetZero();
  }
  observations_.setZero();
  boot_counter_ = boot_indicator_max_;
  history_primed_ = false;
  resetActionFilter();
}

void HeftRLController::resetActionFilter() {
  std::lock_guard<std::mutex> lock(action_filter_mutex_);
  filtered_actions_.resize(0);
  action_filter_initialized_ = false;
}

void HeftRLController::primeHistory() {
  const Eigen::Quaterniond quat(current_imu_.quat[0], current_imu_.quat[1],
                                current_imu_.quat[2], current_imu_.quat[3]);
  Eigen::Vector3d proj_g =
      quatApplyInverse(quat, Eigen::Vector3d(0., 0., kGravityZ));
  proj_g.normalize();

  angvel_buf_.reset(getBaseAngVel());
  projg_buf_.reset(array_t(proj_g.array()));
  jpos_buf_.reset(getHeftJointPos());
  jvel_buf_.reset(getPolicyJointVel());
  action_buf_.resetZero();
  history_primed_ = true;
}

void HeftRLController::pushCurrentState() {
  const Eigen::Quaterniond quat(current_imu_.quat[0], current_imu_.quat[1],
                                current_imu_.quat[2], current_imu_.quat[3]);
  Eigen::Vector3d proj_g =
      quatApplyInverse(quat, Eigen::Vector3d(0., 0., kGravityZ));
  proj_g.normalize();

  angvel_buf_.push(getBaseAngVel());
  projg_buf_.push(array_t(proj_g.array()));
  jpos_buf_.push(getHeftJointPos());
  jvel_buf_.push(getPolicyJointVel());
}

Eigen::Quaterniond HeftRLController::getAlignedBaseQuat() const {
  const Eigen::Quaterniond base_quat(current_imu_.quat[0], current_imu_.quat[1],
                                     current_imu_.quat[2], current_imu_.quat[3]);
  const Eigen::Quaterniond dummy_world(
      Eigen::AngleAxisd(dummy_world_yaw_, Eigen::Vector3d::UnitZ()));
  return dummy_world.conjugate() * base_quat;
}

Eigen::Quaterniond HeftRLController::getFreejointAlignedQuat() const {
  Eigen::Quaterniond imu(current_imu_.quat[0], current_imu_.quat[1],
                         current_imu_.quat[2], current_imu_.quat[3]);
  if (!policy_joint_ids_.empty()) {
    const int waist_motor = policy_joint_ids_[0];
    if (waist_motor >= 0 &&
        waist_motor < static_cast<int>(current_state_.q.size())) {
      const double q_waist_hw = current_state_.q[static_cast<size_t>(waist_motor)];
      imu = imu * Eigen::AngleAxisd(-q_waist_hw, Eigen::Vector3d::UnitZ());
    }
  }
  const Eigen::Quaterniond dummy_world(
      Eigen::AngleAxisd(dummy_world_yaw_, Eigen::Vector3d::UnitZ()));
  return dummy_world.conjugate() * imu;
}

array_t HeftRLController::getHeftJointPos() const {
  return getPolicyJointPos() + default_joint_pos_;
}

void HeftRLController::initializeDummyWorldYaw() {
  if (reference_source_mode_ == "live") {
    return;
  }
  GenericRLController::initializeDummyWorldYaw();
  RL_LOG_INFO("HEFT %s: dummy_world_yaw=%.3f rad (Sonic-style CSV align)",
              reference_source_mode_.c_str(), dummy_world_yaw_);
}

void HeftRLController::initializeDummyWorldYawFromLive() {
  dummy_world_yaw_ = 0.0;

  const Eigen::Quaterniond target_quat = reference_source_->currentRootQuat();
  Eigen::Quaterniond robot_root(current_imu_.quat[0], current_imu_.quat[1],
                                current_imu_.quat[2], current_imu_.quat[3]);
  if (!policy_joint_ids_.empty()) {
    const int waist_motor = policy_joint_ids_[0];
    if (waist_motor >= 0 &&
        waist_motor < static_cast<int>(current_state_.q.size())) {
      const double q_waist_hw = current_state_.q[static_cast<size_t>(waist_motor)];
      robot_root =
          robot_root * Eigen::AngleAxisd(-q_waist_hw, Eigen::Vector3d::UnitZ());
    }
  }

  Eigen::AngleAxisd R_BaseTarget(target_quat * robot_root.inverse());
  if (R_BaseTarget.axis()(2) > 0.0) {
    dummy_world_yaw_ = -R_BaseTarget.angle();
  } else {
    dummy_world_yaw_ = R_BaseTarget.angle();
  }

  if (std::abs(R_BaseTarget.axis()(2)) < 0.7) {
    RL_LOG_WARNING("HEFT live: initial base pitch/roll deviation detected");
  }

  RL_LOG_INFO("HEFT live: initialized dummy_world_yaw: %.3f rad",
              dummy_world_yaw_);
  live_yaw_initialized_ = true;
}

void HeftRLController::computeObservation() {
  // 与 mimic 一致：停播后轨迹停在当前帧（播完=末帧），策略继续跑以自稳。
  if (!reference_source_) {
    RL_LOG_WARNING("HEFT reference source missing; observation zeroed");
    observations_.setZero();
    return;
  }

  const int S = static_cast<int>(future_steps_.size());

  // RT+Y / RT+X：先更新冻结态，再采样
  if (live_provider_) {
    auto& freeze_gate = HeftStreamFreezeGate::instance();
    if (freeze_gate.consumeFreeze()) {
      live_provider_->setStreamFrozen(true);
    }
    if (freeze_gate.consumeResume()) {
      live_provider_->requestStreamResume();
    }
  }

  // live 模式：首次采样前用 live 根姿态初始化 dummy_world_yaw_
  if (reference_source_mode_ == "live" && !live_yaw_initialized_ &&
      reference_source_->ready()) {
    initializeDummyWorldYawFromLive();
  }

  std::vector<HeftRefFrame> frames;
  if (!reference_source_->sampleWindow(future_steps_, frames) ||
      static_cast<int>(frames.size()) != S) {
    if (reference_source_mode_ == "csv") {
      RL_LOG_WARNING("HEFT requires a loaded motion; observation zeroed");
      observations_.setZero();
      return;
    }
    RL_LOG_WARNING("HEFT %s window unavailable; holding last observation",
                   reference_source_mode_.c_str());
    return;
  }

  // Live: record the same 50 Hz frame the policy uses (future_steps offset 0).
  if (policy_csv_recorder_ && reference_source_mode_ == "live") {
    auto& gate = HeftCsvRecordGate::instance();
    if (gate.consumeStart()) {
      policy_csv_recorder_->start("LT+Y");
    }
    if (gate.consumeStop()) {
      policy_csv_recorder_->stopAndSave("LT+X");
    }
    if (policy_csv_recorder_->recording()) {
      for (int i = 0; i < S; ++i) {
        if (future_steps_[i] == 0) {
          policy_csv_recorder_->append(frames[i]);
          break;
        }
      }
    }
  }

  if (live_provider_ && live_provider_->consumeReferenceDiscontinuity()) {
    primeHistory();
  }

  if (!history_primed_) {
    primeHistory();
  }
  pushCurrentState();
  action_buf_.push(getActionsSnapshot());

  const array_t q_now = getHeftJointPos();
  // live 参考是 base_link，IMU 在 torso，需去腰偏，否则转腰变航向→脚动。
  const Eigen::Quaterniond rq_aligned =
      (reference_source_mode_ == "csv") ? getAlignedBaseQuat()
                                        : getFreejointAlignedQuat();

  int offset = 0;
  auto put = [&](const array_t& seg) {
    observations_.segment(offset, seg.size()) = seg;
    offset += static_cast<int>(seg.size());
  };

  observations_[offset++] = boot_counter_ / boot_indicator_max_;

  for (int i = 1; i < S; ++i) {
    const Eigen::Vector3d diff =
        quatApplyInverse(frames[0].quat, frames[i].pos - frames[0].pos);
    put(array_t(diff.array()));
  }
  for (int i = 0; i < S; ++i) {
    put(quatToRot6d(quatInFrame(rq_aligned, frames[i].quat)));
  }
  for (int i = 0; i < S; ++i) {
    put(frames[i].jpos);
  }
  for (int i = 0; i < S; ++i) {
    put(array_t(frames[i].jpos - q_now));
  }
  for (int i = 0; i < S; ++i) {
    observations_[offset++] = frames[i].pos[2];
  }
  for (int i = 0; i < S; ++i) {
    const Eigen::Vector3d g =
        quatApplyInverse(frames[i].quat, Eigen::Vector3d(0., 0., kGravityZ));
    put(array_t(g.array()));
  }

  put(angvel_buf_.takeFlattened(history_taps_));
  put(projg_buf_.takeFlattened(history_taps_));
  put(jpos_buf_.takeFlattened(history_taps_));
  put(jvel_buf_.takeFlattened(history_taps_));

  {
    std::vector<int> action_offsets(prev_action_steps_);
    for (int i = 0; i < prev_action_steps_; ++i) {
      action_offsets[i] = i;
    }
    put(action_buf_.takeFlattened(action_offsets));
  }

  if (offset != static_cast<int>(observations_.size())) {
    RL_LOG_FAILURE("HEFT obs size mismatch: %d vs %ld",
                   offset, static_cast<long>(observations_.size()));
  }

  boot_counter_ = std::max(boot_counter_ - 1.0, 0.0);

  reference_source_->onPolicyTick();

  if (logger_) {
    std::vector<double> obs_vec(observations_.data(),
                                observations_.data() + observations_.size());
    logger_->publishVector("/rl_controller/observations", obs_vec);
    logger_->publishValue("/rl_controller/heft/boot", boot_counter_);
    logger_->publishValue("/rl_controller/heft/motion_frame",
                          reference_source_->progressFrame());

    const HeftRefFrame& cur = frames[0];
    if (cur.jpos.size() > 0) {
      std::vector<double> ref_q(cur.jpos.data(),
                                cur.jpos.data() + cur.jpos.size());
      logger_->publishVector("/rl_controller/heft/ref_q", ref_q);
    }
    if (cur.jvel.size() == cur.jpos.size() && cur.jvel.size() > 0) {
      std::vector<double> ref_qd(cur.jvel.data(),
                                 cur.jvel.data() + cur.jvel.size());
      logger_->publishVector("/rl_controller/heft/ref_qd", ref_qd);
    }
  }
}

bool HeftRLController::inferActions(const array_t& observation,
                                    array_t& inferred_actions) {
  if (!GenericRLController::inferActions(observation, inferred_actions)) {
    return false;
  }
  if (inferred_actions.size() == 0) {
    return true;
  }

  inferred_actions =
      inferred_actions.min(action_clip_).max(-action_clip_);
  {
    std::lock_guard<std::mutex> lock(action_filter_mutex_);
    if (!action_filter_initialized_ ||
        filtered_actions_.size() != inferred_actions.size() ||
        action_alpha_ >= 1.0) {
      filtered_actions_ = inferred_actions;
      action_filter_initialized_ = true;
    } else {
      filtered_actions_ = (1.0 - action_alpha_) * filtered_actions_ +
                          action_alpha_ * inferred_actions;
    }
  }
  return true;
}

array_t HeftRLController::getActionsSnapshot() const {
  array_t actions = GenericRLController::getActionsSnapshot();
  if (actions.size() > 0) {
    actions = actions.min(action_clip_).max(-action_clip_);
  }
  return actions;
}

array_t HeftRLController::getMotorActionsSnapshot() const {
  {
    std::lock_guard<std::mutex> lock(action_filter_mutex_);
    if (action_filter_initialized_ && filtered_actions_.size() > 0) {
      return filtered_actions_;
    }
  }
  return getActionsSnapshot();
}

void HeftRLController::updateRobotCmd(RobotCmd& cmd) {
  const array_t saved_default = default_joint_pos_;
  default_joint_pos_.setZero();
  GenericRLController::updateRobotCmd(cmd);
  default_joint_pos_ = saved_default;
}

REGISTER_CONTROLLER("HeftRLController", HeftRLController, heft_rl);

void ForceLinkHeftRLController() {}

}  // namespace leju
