// Copyright 2024 Leju Robotics. All rights reserved.

#include "leju-rl-controller/motion/tact_player.h"

#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdlib>

#include "leju-rl-controller/motion/tact_parser.h"
#include "leju-rl-controller/runtime/input/action_trigger.h"
#include "leju-rl-controller/rl_log.h"

namespace leju {
namespace runtime {

namespace {

// 校验动作名：拒绝路径分隔符和 ..，防止路径穿越
bool IsValidTactName(const std::string& name, std::string* err) {
  if (name.empty()) {
    if (err) *err = "tact name is empty";
    return false;
  }
  // 拒绝含路径分隔符或上级目录引用的名字
  if (name.find('/') != std::string::npos ||
      name.find('\\') != std::string::npos ||
      name.find("..") != std::string::npos) {
    if (err) *err = "tact name contains invalid characters";
    return false;
  }
  return true;
}
constexpr double kDegToRad = M_PI / 180.0;

// 五次多项式 s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5，两端速度/加速度为零
double QuinticS(double tau) {
  return tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau));
}

double QuinticDsDtau(double tau) {
  return tau * tau * (30.0 + tau * (-60.0 + 30.0 * tau));
}

double QuinticD2sDtau2(double tau) {
  return tau * (60.0 + tau * (-180.0 + 120.0 * tau));
}

}  // namespace

std::vector<vr::tact_player::JointTrajectorySample> BuildTactApproachSamples(
    const vr::tact_player::JointTrajectorySample& first_sample,
    const std::vector<double>& init_arm_deg,
    const std::vector<double>& init_waist_deg,
    double loop_dt,
    const TactApproachOptions& options) {
  std::vector<vr::tact_player::JointTrajectorySample> samples;

  const std::size_t dof = first_sample.q.size();
  if (dof == 0 || init_arm_deg.size() < dof || loop_dt <= 0.0) {
    return samples;
  }

  std::vector<double> start_q(dof);
  double max_dist = 0.0;
  for (std::size_t j = 0; j < dof; ++j) {
    start_q[j] = init_arm_deg[j] * kDegToRad;
    max_dist = std::max(max_dist, std::abs(first_sample.q[j] - start_q[j]));
  }

  const bool interp_waist = first_sample.has_waist && !init_waist_deg.empty();
  double start_waist = first_sample.waist_q;
  if (interp_waist) {
    // tact 腰部数值空间与电机编码器一致（PlaybackLoop 输出时统一取反）
    start_waist = init_waist_deg[0] * kDegToRad;
    max_dist = std::max(max_dist, std::abs(first_sample.waist_q - start_waist));
  }

  if (max_dist < options.threshold) {
    return samples;
  }

  // 峰值速度不超过 options.velocity：五次多项式峰值速度 = 1.875 * dist / T
  double duration = 1.875 * max_dist / options.velocity;
  duration = std::max(options.min_duration, std::min(options.max_duration, duration));

  const std::size_t steps = static_cast<std::size_t>(duration / loop_dt);
  samples.reserve(steps);
  for (std::size_t i = 0; i < steps; ++i) {
    const double tau = static_cast<double>(i) * loop_dt / duration;
    const double s = QuinticS(tau);
    const double ds = QuinticDsDtau(tau) / duration;
    const double d2s = QuinticD2sDtau2(tau) / (duration * duration);

    vr::tact_player::JointTrajectorySample sample;
    sample.q.resize(dof);
    sample.v.resize(dof);
    sample.acc.resize(dof);
    for (std::size_t j = 0; j < dof; ++j) {
      const double delta = first_sample.q[j] - start_q[j];
      sample.q[j] = start_q[j] + delta * s;
      sample.v[j] = delta * ds;
      sample.acc[j] = delta * d2s;
    }

    sample.has_waist = first_sample.has_waist;
    if (interp_waist) {
      const double delta = first_sample.waist_q - start_waist;
      sample.waist_q = start_waist + delta * s;
      sample.waist_v = delta * ds;
      sample.waist_acc = delta * d2s;
    } else if (first_sample.has_waist) {
      sample.waist_q = first_sample.waist_q;
    }

    samples.push_back(std::move(sample));
  }

  return samples;
}

TactPlayer::TactPlayer(CommandBuffer* cmd_buffer,
                       TriggerBuffer* trigger_buffer,
                       std::size_t arm_dof,
                       int waist_index,
                       int hand_index,
                       const std::string& action_dir,
                       double loop_dt)
    : cmd_buffer_(cmd_buffer),
      trigger_buffer_(trigger_buffer),
      arm_dof_(arm_dof),
      waist_index_(waist_index),
      hand_index_(hand_index),
      action_dir_(action_dir),
      loop_dt_(loop_dt) {}

TactPlayer::~TactPlayer() {
  Stop();
}

int TactPlayer::getState() const {
  return state_.load();
}

int TactPlayer::getErrorCode() const {
  return error_code_.load();
}

std::string TactPlayer::getLastName() const {
  std::lock_guard<std::mutex> st_lock(state_mutex_);
  return last_name_;
}

std::string TactPlayer::getErrorMessage() const {
  std::lock_guard<std::mutex> st_lock(state_mutex_);
  return error_message_;
}

void TactPlayer::markFailed(int error_code, const std::string& message, const std::string& name) {
  error_code_.store(error_code);
  {
    std::lock_guard<std::mutex> st_lock(state_mutex_);
    last_name_ = name;
    error_message_ = message;
  }
  state_.store(kTactFailed);
}

void TactPlayer::SetArmMode(const std::string& mode_name) {
  if (trigger_buffer_) {
    trigger_buffer_->push(MakeSetArmModeTrigger(mode_name));
  }
}

void TactPlayer::SetWaistMode(const std::string& mode_name) {
  if (trigger_buffer_) {
    trigger_buffer_->push(MakeSetWaistModeTrigger(mode_name));
  }
}

bool TactPlayer::Play(const std::string& name,
                      const std::vector<double>& init_arm_pos,
                      const std::vector<double>& init_waist_pos,
                      std::string* message,
                      int64_t target_execution_time_ns) {
  std::lock_guard<std::mutex> lock(play_mutex_);

  // 播放中拒绝新请求
  if (playing_.load()) {
    if (message) *message = "TactPlayer busy: another motion is playing";
    error_code_.store(kTactErrBusy);
    {
      std::lock_guard<std::mutex> st_lock(state_mutex_);
      last_name_ = name;
      error_message_ = message ? *message : std::string();
    }
    state_.store(kTactFailed);
    return false;
  }

  if (!cmd_buffer_ || !trigger_buffer_) {
    if (message) *message = "TactPlayer not initialized";
    error_code_.store(kTactErrNotInitialized);
    {
      std::lock_guard<std::mutex> st_lock(state_mutex_);
      last_name_ = name;
      error_message_ = message ? *message : std::string();
    }
    state_.store(kTactFailed);
    return false;
  }

  // 路径穿越校验
  if (!IsValidTactName(name, message)) {
    error_code_.store(kTactErrInvalidName);
    {
      std::lock_guard<std::mutex> st_lock(state_mutex_);
      last_name_ = name;
      error_message_ = message ? *message : std::string();
    }
    state_.store(kTactFailed);
    return false;
  }

  const std::string tact_path = action_dir_ + "/" + name + ".tact";

  // 二次校验：realpath 确保解析后仍在 action_dir_ 内
  {
    char* resolved = realpath(tact_path.c_str(), nullptr);
    if (!resolved) {
      if (message) *message = "tact action not found: " + name;
      error_code_.store(kTactErrFileNotFound);
      {
        std::lock_guard<std::mutex> st_lock(state_mutex_);
        last_name_ = name;
        error_message_ = message ? *message : std::string();
      }
      state_.store(kTactFailed);
      return false;
    }
    std::string resolved_path(resolved);
    free(resolved);
    // 规范化 action_dir_ 后做前缀比较
    char* dir_resolved = realpath(action_dir_.c_str(), nullptr);
    if (!dir_resolved) {
      if (message) *message = "action directory not accessible";
      error_code_.store(kTactErrFileNotFound);
      {
        std::lock_guard<std::mutex> st_lock(state_mutex_);
        last_name_ = name;
        error_message_ = message ? *message : std::string();
      }
      state_.store(kTactFailed);
      return false;
    }
    std::string resolved_dir(dir_resolved);
    free(dir_resolved);
    // 确保 resolved_dir 以 / 结尾
    if (resolved_dir.back() != '/') {
      resolved_dir += '/';
    }
    if (resolved_path.compare(0, resolved_dir.size(), resolved_dir) != 0) {
      if (message) *message = "tact action not found: " + name;
      error_code_.store(kTactErrFileNotFound);
      {
        std::lock_guard<std::mutex> st_lock(state_mutex_);
        last_name_ = name;
        error_message_ = message ? *message : std::string();
      }
      state_.store(kTactFailed);
      return false;
    }
  }

  // 解析 tact
  vr::tact_player::TactAction action;
  std::string parse_error;
  vr::tact_player::ParseOptions parse_options;
  parse_options.arm_dof = arm_dof_;
  parse_options.waist_index = waist_index_;
  parse_options.hand_start_index = hand_index_;
  if (!init_arm_pos.empty()) {
    parse_options.init_arm_pos = init_arm_pos;
  }
  if (!init_waist_pos.empty()) {
    parse_options.init_waist_pos = init_waist_pos;
  }
  if (!vr::tact_player::LoadTactFile(tact_path, parse_options, &action, &parse_error)) {
    // 错误信息不泄露完整路径
    if (message) *message = "parse tact failed: " + parse_error;
    error_code_.store(kTactErrParseFailed);
    {
      std::lock_guard<std::mutex> st_lock(state_mutex_);
      last_name_ = name;
      error_message_ = message ? *message : std::string();
    }
    state_.store(kTactFailed);
    return false;
  }

  // 贝塞尔插值
  vr::tact_player::BezierInterpolator interpolator;
  vr::tact_player::InterpolateOptions interp_options;
  interp_options.arm_dof = arm_dof_;
  interp_options.speed_scale = 1.0;
  interp_options.enable_waist = (waist_index_ >= 0);
  interp_options.enable_hand = (hand_index_ >= 0);
  if (!interpolator.Build(action, interp_options, &parse_error)) {
    if (message) *message = "build interpolation failed: " + parse_error;
    error_code_.store(kTactErrInterpolationFailed);
    {
      std::lock_guard<std::mutex> st_lock(state_mutex_);
      last_name_ = name;
      error_message_ = message ? *message : std::string();
    }
    state_.store(kTactFailed);
    return false;
  }

  // 预计算：以 loop_dt_ 步长采样整条贝塞尔曲线，消除 100Hz 写→1000Hz 读的台阶效应
  const double total = interpolator.Duration();
  const bool has_waist = interpolator.HasWaist();
  precomputed_samples_.clear();
  precomputed_samples_.reserve(static_cast<size_t>(total / loop_dt_) + 2);
  for (double t = 0.0; t <= total + loop_dt_ * 0.5; t += loop_dt_) {
    precomputed_samples_.push_back(interpolator.Evaluate(t));
  }
  precomputed_has_waist_ = has_waist;

  // 前置接入过渡：从当前位姿平滑走到轨迹首采样点。
  // tact 文件首帧均在 keyframe=0，parser 的 init 帧逻辑不生效，若当前位姿
  // 偏离 0 帧（如 LB+B 冻结后再播放），无过渡会导致指令阶跃、手臂快速甩动。
  if (!precomputed_samples_.empty()) {
    auto approach = BuildTactApproachSamples(
        precomputed_samples_.front(), init_arm_pos, init_waist_pos, loop_dt_);
    if (!approach.empty()) {
      RL_LOGI("TactPlayer: prepend approach transition %.2fs (%zu samples)",
              approach.size() * loop_dt_, approach.size());
      precomputed_samples_.insert(precomputed_samples_.begin(),
                                  std::make_move_iterator(approach.begin()),
                                  std::make_move_iterator(approach.end()));
    }
  }

  // 等上一个 worker 彻底结束（正常情况下 playing_ 为 false 时已 join）
  if (worker_.joinable()) {
    worker_.join();
  }

  stop_requested_.store(false);
  frozen_.store(false);
  playing_.store(true);
  // 状态模型：成功启动播放，覆盖上次终态
  error_code_.store(kTactOk);
  {
    std::lock_guard<std::mutex> st_lock(state_mutex_);
    last_name_ = name;
    error_message_.clear();
  }
  state_.store(kTactPlaying);
  worker_ = std::thread(&TactPlayer::PlaybackLoop, this, target_execution_time_ns);

  if (message) *message = "tact playback started: " + name;
  return true;
}

void TactPlayer::Stop() {
  stop_requested_.store(true);
  if (worker_.joinable()) {
    worker_.join();
  }
}

void TactPlayer::Freeze() {
  frozen_.store(true);
  Stop();
}

void TactPlayer::PlaybackLoop(int64_t target_execution_time_ns) {
  // 切 External 模式
  SetArmMode("kExternal");
  if (precomputed_has_waist_) {
    SetWaistMode("kExternal");
  }

  const auto period = std::chrono::duration<double>(loop_dt_);
  auto start_time = std::chrono::steady_clock::now();

  // 时间守门员：起播基准对齐绝对目标（解析/插值已在 Play() 完成，仅等待到点）。
  // 目标已过时立即播并记 missed。
  if (target_execution_time_ns > 0) {
    const auto system_now = std::chrono::system_clock::now();
    const auto target_sys = std::chrono::system_clock::time_point(
        std::chrono::nanoseconds(target_execution_time_ns));
    const auto target_steady = std::chrono::steady_clock::now()
        + (target_sys - system_now);
    if (target_steady > start_time) {
      const auto lead_ms = std::chrono::duration<double, std::milli>(
          target_steady - start_time).count();
      start_time = target_steady;
      RL_LOGI("TactPlayer: aligned start to target_execution_time_ns=%lld (%.1f ms ahead)",
              static_cast<long long>(target_execution_time_ns), lead_ms);
    } else {
      const auto miss_ms = std::chrono::duration<double, std::milli>(
          start_time - target_steady).count();
      RL_LOGI("TactPlayer: missed target_execution_time by %.1f ms, playing immediately",
              miss_ms);
    }
  }

  for (size_t i = 0; i < precomputed_samples_.size(); ++i) {
    if (stop_requested_.load()) {
      break;
    }

    const auto& sample = precomputed_samples_[i];

    ExternalJointTarget arm_cmd;
    arm_cmd.q = sample.q;
    arm_cmd.v = sample.v;
    arm_cmd.acc = sample.acc;
    cmd_buffer_->writeArmTarget(arm_cmd);

    if (sample.has_waist) {
      ExternalJointTarget waist_cmd;
      waist_cmd.q = {-sample.waist_q};
      waist_cmd.v = {-sample.waist_v};
      waist_cmd.acc = {-sample.waist_acc};
      cmd_buffer_->writeWaistTarget(waist_cmd);
    }

    if (sample.has_hand && sample.hand_position.size() == 12) {
      ExternalHandTarget hand_cmd;
      hand_cmd.position = sample.hand_position;
      cmd_buffer_->writeHandTarget(hand_cmd);
    }

    // sleep_until 绝对时间，消除 sleep_for 的累积时序漂移
    std::this_thread::sleep_until(start_time + std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(period * (i + 1)));
  }

  // 恢复 Auto 模式（冻结时不恢复，保持 keep_pose 由 LB+B 设置的模式）
  if (!frozen_.load()) {
    SetArmMode("kAuto");
    if (precomputed_has_waist_) {
      SetWaistMode("kAuto");
    }
  } else {
    RL_LOGI("TactPlayer: frozen, skipping Auto restore");
  }

  // 状态模型收尾：按退出路径设终态。
  // 注意 Freeze() 会先调 Stop()，故 frozen_ 时 stop_requested_ 也为 true，需先判 frozen_。
  if (frozen_.load()) {
    state_.store(kTactFrozen);
  } else if (stop_requested_.load()) {
    state_.store(kTactStopped);
  } else {
    state_.store(kTactFinished);
  }

  RL_LOGI("TactPlayer: playback finished");
  playing_.store(false);
}

}  // namespace runtime
}  // namespace leju
