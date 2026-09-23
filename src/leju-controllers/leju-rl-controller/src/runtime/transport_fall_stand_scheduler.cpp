#include "leju-rl-controller/runtime/transport_fall_stand_scheduler.h"

#include <algorithm>
#include <cmath>
#include <string>

#include "leju-rl-controller/controllers/controller_manager.h"
#include "leju-rl-controller/runtime/data_types.hpp"
#include "leju-rl-controller/runtime/fall_stand_coordinator.h"
#include "leju-rl-controller/runtime/transport_mode_coordinator.h"
#include "leju-rl-controller/rl_log.h"
#include "lejusdk-lowlevel/data_types.h"
#include "lejusdk-vr/data_types.h"

namespace leju {
namespace runtime {

namespace {

// 调度事件名（与 teleop_bindings.yaml 的 TransportFallStand 绑定一致）
constexpr char kEventEnter[] = "transport.enter";
constexpr char kEventLock[] = "transport.lock";
constexpr char kEventExit[] = "transport.exit";
constexpr char kEventFallDown[] = "transport.fall_down";
constexpr char kEventStandUp[] = "fallstand.standup";

// 搬运状态语音（resources/music，audio_player_node 按文件名播放）
constexpr char kVoiceNotLocked[] =
    "电机未锁定请扶住背部把手禁止抬起机器人.wav";
constexpr char kVoiceTransportActive[] = "进入搬运模式可安全移动.wav";
constexpr char kVoiceCancelTransport[] = "取消搬运恢复正常行走状态.wav";
constexpr char kVoiceExitTransport[] = "退出搬运模式恢复正常.wav";

// 时间参数
constexpr double kProtectionWindowSec = 5.0;
constexpr double kProtectionMaxAccumulatedSec = 15.0;
constexpr double kDriveIntervalSec = 0.2;
// 在 CM switchDuration 之上多等一点，再查插值完成
constexpr double kPostAmpSwitchSlackSec = 0.5;
constexpr double kWaitAmpLogIntervalSec = 2.0;
constexpr double kHandResetDelaySec = 1.0;
constexpr double kFourFingerGripDelaySec = 0.5;

// IMU 自检判据
const double kUprightCos = std::cos(5.0 * M_PI / 180.0);
const double kLyingFlatCos = std::cos(15.0 * M_PI / 180.0);

// 头部低头角度（正 pitch = 低头）
constexpr double kHeadDownPitchRad = 20.0 * M_PI / 180.0;

constexpr char kAmpController[] = "amp";
constexpr char kFallStandController[] = "mimic_fall_stand";

// 搬运状态（与 TransportModeState 对齐）
constexpr int kInactive = 0;
constexpr int kInterpolating = 1;
constexpr int kReady = 2;
constexpr int kActive = 3;
constexpr int kHandingOver = 4;

// 倒地起身阶段
constexpr int kFsIdle = 0;
constexpr int kFsHolding = 2;
constexpr int kFsStandingUp = 3;
constexpr int kFsStanding = 4;

// 判据（须单位四元数）：|R(2,2)|=1-2(x²+y²)>cos5° 直立、|R(0,2)|=2(xz-wy)>cos15° 躺平（fabs 使倒立亦判直立；阈值与研杨一致）
bool isUpright(const ImuData& imu) {
  const double x = imu.quat[1], y = imu.quat[2];
  const double gz = 1.0 - 2.0 * (x * x + y * y);
  return std::fabs(gz) > kUprightCos;
}

bool isLyingFlat(const ImuData& imu) {
  const double w = imu.quat[0], x = imu.quat[1], y = imu.quat[2], z = imu.quat[3];
  const double gx = 2.0 * (x * z - w * y);
  return std::fabs(gx) > kLyingFlatCos;
}

}  // namespace

void TransportFallStandScheduler::onEvent(const std::string& name,
                                          const ImuData& imu, double now) {
  if (name.empty() || !transport_) {
    return;
  }
  const int ts = static_cast<int>(transport_->state());

  if (name == kEventStandUp) {
    const int fs = fall_stand_ ? fall_stand_->fallStandState() : -1;
    onStandUpEvent(imu, ts, fs, now);
    return;
  }

  // 保护窗内除 transport.lock 重置计时外，不响应搬运事件
  if (protection_active_) {
    if (name == kEventLock) {
      // audio_player_node 收到新的文件名会清空当前队列，实现打断并重播。
      playVoice(kVoiceNotLocked);
      const double since_start = now - protection_start_;
      const double remain_cap = kProtectionMaxAccumulatedSec - since_start;
      if (remain_cap > 0.0) {
        protection_deadline_ = now + std::min(kProtectionWindowSec, remain_cap);
        RL_LOGI("TransportFallStandScheduler: protection window reset");
      }
    }
    return;
  }

  handleTransportEvent(name, imu, ts);
}

void TransportFallStandScheduler::handleTransportEvent(const std::string& event,
                                                        const ImuData& imu, int ts) {
  if (event == kEventEnter) {
    if (ts == kInactive) {
      transport_->sendCommand(TransportModeCommand::kEnter);
    }
    return;
  }
  if (event == kEventLock) {
    if (ts == kReady) {
      transport_->sendCommand(TransportModeCommand::kLock);
    }
    return;
  }
  if (event == kEventExit) {
    if (ts == kReady) {
      transport_->sendCommand(TransportModeCommand::kHandOver);
      setHeadPitch(0.0);
      pending_exit_voice_ = kVoiceCancelTransport;
      normal_exit_pending_ = true;
      exit_from_ready_ = true;
    } else if (ts == kActive) {
      // ACTIVE 非直立拒退出：amp 按站立假设接管，中姿态直回会失稳；先躺平或人工扶正
      if (isUpright(imu)) {
        transport_->sendCommand(TransportModeCommand::kHandOver);
        setHeadPitch(0.0);
        pending_exit_voice_ = kVoiceExitTransport;
        normal_exit_pending_ = true;
        exit_from_ready_ = false;
      } else {
        RL_LOGW("TransportFallStandScheduler: exit rejected, not upright");
      }
    }
    return;
  }
  if (event == kEventFallDown) {
    if (ts == kActive) {
      transport_->sendCommand(TransportModeCommand::kFallDown);
      setHeadPitch(0.0);
    }
    return;
  }
}

void TransportFallStandScheduler::setHeadPitch(double pitch_rad) {
  if (!controller_manager_) {
    return;
  }
  vr::JointTrajectoryPoint cmd;
  cmd.q = {0.0, pitch_rad};
  controller_manager_->setHeadTarget(cmd);
}

void TransportFallStandScheduler::playVoice(const std::string& file_name) const {
  if (audio_player_) {
    audio_player_(file_name);
  } else {
    RL_LOGW("TransportFallStandScheduler: audio player unavailable for '%s'",
            file_name.c_str());
  }
}

bool TransportFallStandScheduler::areHandsAvailable() const {
  return !hands_available_checker_ || hands_available_checker_();
}

bool TransportFallStandScheduler::setHandsClosed(bool closed) const {
  if (!areHandsAvailable()) {
    return true;
  }
  if (!hand_grip_setter_) {
    RL_LOGW("TransportFallStandScheduler: hand grip setter unavailable");
    return false;
  }
  if (!hand_grip_setter_(closed)) {
    RL_LOGW("TransportFallStandScheduler: failed to %s dexterous hands",
            closed ? "close" : "open");
    return false;
  }
  return true;
}

bool TransportFallStandScheduler::setHandsPreGrip() const {
  if (!areHandsAvailable()) {
    return true;
  }
  if (!hand_pre_grip_setter_) {
    RL_LOGW("TransportFallStandScheduler: hand pre-grip setter unavailable");
    return false;
  }
  if (!hand_pre_grip_setter_()) {
    RL_LOGW("TransportFallStandScheduler: failed to prepare dexterous thumbs");
    return false;
  }
  return true;
}

bool TransportFallStandScheduler::setFourFingersClosed() const {
  if (!areHandsAvailable()) {
    return true;
  }
  if (!four_finger_grip_setter_) {
    RL_LOGW("TransportFallStandScheduler: four-finger grip setter unavailable");
    return false;
  }
  if (!four_finger_grip_setter_()) {
    RL_LOGW("TransportFallStandScheduler: failed to close four fingers");
    return false;
  }
  return true;
}

void TransportFallStandScheduler::beginHandGripSequence(double now) {
  if (!areHandsAvailable()) {
    hand_grip_stage_ = HandGripStage::kIdle;
    return;
  }
  if (setHandsPreGrip()) {
    hand_grip_stage_ = HandGripStage::kWaitFourFingers;
    hand_grip_stage_at_ = now + kHandResetDelaySec;
  } else {
    hand_grip_stage_ = HandGripStage::kIdle;
  }
}

void TransportFallStandScheduler::tickHandGripSequencer(double now) {
  if (hand_grip_stage_ == HandGripStage::kIdle || now < hand_grip_stage_at_) {
    return;
  }
  if (hand_grip_stage_ == HandGripStage::kWaitFourFingers) {
    if (!setFourFingersClosed()) {
      hand_grip_stage_ = HandGripStage::kIdle;
      return;
    }
    hand_grip_stage_ = HandGripStage::kWaitThumbs;
    hand_grip_stage_at_ = now + kFourFingerGripDelaySec;
    return;
  }
  hand_grip_stage_ = HandGripStage::kIdle;
  setHandsClosed(true);
}

void TransportFallStandScheduler::tick(const ImuData& imu, double now) {
  if (!transport_) {
    return;
  }
  const int ts = static_cast<int>(transport_->state());
  const int fs = fall_stand_ ? fall_stand_->fallStandState() : -1;

  onTransportEdge(ts, now);
  tickHandGripSequencer(now);

  // 保护窗自解
  if (protection_active_) {
    const double since_start = now - protection_start_;
    if (now >= protection_deadline_ || since_start >= kProtectionMaxAccumulatedSec) {
      protection_active_ = false;
    }
  }

  onFallStandEdge(fs, now);
  driveHardChain(imu, ts, fs, now);
  driveSoftChain(imu, ts, fs, now);
  tickExitSequencer(now);
}

void TransportFallStandScheduler::onTransportEdge(int ts, double now) {
  if (ts == prev_transport_state_) {
    return;
  }
  if (ts == kInterpolating) {
    protection_active_ = true;
    protection_start_ = now;
    protection_deadline_ = now + kProtectionWindowSec;
    playVoice(kVoiceNotLocked);
    beginHandGripSequence(now);
  } else if (ts == kActive) {
    // 进 ACTIVE 即低头 20°（正 pitch=低头）：对齐研杨，降低重心便于搬运；退出时复位
    setHeadPitch(kHeadDownPitchRad);
    playVoice(kVoiceTransportActive);
  } else if (ts == kHandingOver) {
    // 普通退出路径（FALL_DOWN 不置 normal_exit_pending_）
    if (normal_exit_pending_) {
      normal_exit_pending_ = false;
      beginNormalExit(now);
    }
  } else if (ts == kInactive) {
    // 统一在搬运退出完成点恢复张手。倒地起身路径会先等 AMP
    // 回切/插值完成再进入 INACTIVE，因此起身过程中仍保持握拳。
    setHandsClosed(false);
    hand_grip_stage_ = HandGripStage::kIdle;
    normal_exit_pending_ = false;
    exit_from_ready_ = false;
    pending_exit_voice_.clear();
    protection_active_ = false;
    resetChains();
  }
  prev_transport_state_ = ts;
}

void TransportFallStandScheduler::onFallStandEdge(int fs, double now) {
  if (fs == prev_fall_stand_state_) {
    return;
  }
  prev_fall_stand_state_ = fs;
  if (fs == kFsHolding || fs == kFsStandingUp) {
    saw_standup_progress_ = true;
  }
  const bool non_transport =
      static_cast<int>(transport_->state()) == kInactive;
  if (fs == kFsHolding && non_transport) {
    beginHandGripSequence(now);
  }
  if (fs == kFsStanding && non_transport) {
    setHandsClosed(false);
  }
  // 硬起身：首帧到位后自动 STAND_UP。
  if (fs == kFsHolding && hard_chain_active_ && !hard_chain_standup_sent_) {
    if (fall_stand_) {
      fall_stand_->sendCommand(FallStandCommand::kStandUp);
    }
    hard_chain_standup_sent_ = true;
  }
  // 手柄发起的起身完成后退出编排。
  if (fs == kFsStanding && saw_standup_progress_ &&
      (hard_chain_active_ || soft_stage_ != 0 || solo_chain_active_)) {
    beginFallPathExit(now);
    hard_chain_active_ = false;
    soft_stage_ = 0;
    solo_chain_active_ = false;
  }
}

void TransportFallStandScheduler::driveHardChain(const ImuData& imu, int ts, int fs,
                                                double now) {
  (void)imu;
  if (!hard_chain_active_ || hard_chain_prepare_done_) {
    return;
  }
  // 研杨仅在 ACTIVE 按键发起硬起身时检查躺平；此处只等待真正进入 HANDING_OVER。
  if (ts != kHandingOver) {
    return;
  }
  if (fs == kFsIdle) {
    if (now >= hard_next_drive_) {
      if (fall_stand_) {
        fall_stand_->sendCommand(FallStandCommand::kPrepare);
      }
      hard_next_drive_ = now + kDriveIntervalSec;
    }
  } else {
    hard_chain_prepare_done_ = true;
  }
}

void TransportFallStandScheduler::driveSoftChain(const ImuData& imu, int ts, int fs,
                                                double now) {
  (void)imu;
  if (soft_stage_ != 1 || ts != kHandingOver) {
    return;
  }
  if (fs == kFsIdle) {
    if (now >= soft_next_drive_) {
      if (fall_stand_) {
        fall_stand_->sendCommand(FallStandCommand::kPrepare);
      }
      soft_next_drive_ = now + kDriveIntervalSec;
    }
  } else if (fs == kFsHolding) {
    soft_stage_ = 2;
    RL_LOGI("TransportFallStandScheduler: soft stand-up holding ready");
  }
}

void TransportFallStandScheduler::onStandUpEvent(const ImuData& imu, int ts, int fs,
                                                 double now) {
  if (ts == kActive && !hard_chain_active_) {
    if (isLyingFlat(imu)) {
      transport_->sendCommand(TransportModeCommand::kFallDown);
      setHeadPitch(0.0);
      hard_chain_active_ = true;
      hard_chain_prepare_done_ = false;
      hard_chain_standup_sent_ = false;
      hard_next_drive_ = now;
      saw_standup_progress_ = false;
    }
    return;
  }
  if (ts == kHandingOver && !hard_chain_active_) {
    if (soft_stage_ == 0) {
      if (fall_stand_) fall_stand_->sendCommand(FallStandCommand::kPrepare);
      soft_stage_ = 1;
      soft_next_drive_ = now;
      saw_standup_progress_ = false;
    } else if (soft_stage_ == 2 && fs == kFsHolding) {
      if (fall_stand_) fall_stand_->sendCommand(FallStandCommand::kStandUp);
      soft_stage_ = 3;
    }
    return;
  }
  if (ts != kInactive) return;

  const std::string ctrl =
      controller_manager_ ? controller_manager_->getCurrentControllerName() : "";
  if (ctrl != kFallStandController) return;
  if (fs == kFsIdle) {
    if (fall_stand_) fall_stand_->sendCommand(FallStandCommand::kPrepare);
    solo_chain_active_ = true;
    saw_standup_progress_ = false;
  } else if (fs == kFsHolding) {
    if (hand_grip_stage_ != HandGripStage::kIdle) {
      RL_LOGI("TransportFallStandScheduler: wait for hand grip before stand-up");
      return;
    }
    if (fall_stand_) fall_stand_->sendCommand(FallStandCommand::kStandUp);
    solo_chain_active_ = true;
  }
}

void TransportFallStandScheduler::resetChains() {
  saw_standup_progress_ = false;
  hard_chain_active_ = false;
  hard_chain_prepare_done_ = false;
  hard_chain_standup_sent_ = false;
  soft_stage_ = 0;
  resetSoloChain();
}

void TransportFallStandScheduler::resetSoloChain() {
  solo_chain_active_ = false;
}

void TransportFallStandScheduler::beginNormalExit(double now) {
  if (exit_stage_ != ExitStage::kIdle) {
    return;
  }
  if (exit_from_ready_) {
    exit_stage_ = ExitStage::kWaitBlend;
    next_wait_log_ = now;
    RL_LOGI("TransportFallStandScheduler: ready-cancel exit");
  } else {
    requestAmpAndDelay(now);
    RL_LOGI("TransportFallStandScheduler: lock exit, switch amp");
  }
}

void TransportFallStandScheduler::beginFallPathExit(double now) {
  if (exit_stage_ != ExitStage::kIdle) {
    return;
  }
  if (static_cast<int>(transport_->state()) == kHandingOver) {
    pending_exit_voice_ = kVoiceExitTransport;
  }
  requestAmpAndDelay(now);
  RL_LOGI("TransportFallStandScheduler: fall-path exit, switch amp");
}

void TransportFallStandScheduler::requestAmpAndDelay(double now) {
  double delay = kPostAmpSwitchSlackSec;
  if (controller_manager_) {
    controller_manager_->requestSwitch(kAmpController, now);
    delay = controller_manager_->switchDuration() + kPostAmpSwitchSlackSec;
  }
  exit_stage_ = ExitStage::kPostSwitchDelay;
  stage_until_ = now + delay;
  next_wait_log_ = now;
}

void TransportFallStandScheduler::tickExitSequencer(double now) {
  if (exit_stage_ == ExitStage::kIdle) return;
  const std::string ctrl =
      controller_manager_ ? controller_manager_->getCurrentControllerName() : "";
  const bool transitioning = controller_manager_ && controller_manager_->isTransitioning();
  if (exit_stage_ == ExitStage::kPostSwitchDelay && now >= stage_until_) {
    exit_stage_ = ExitStage::kWaitBlend;
    next_wait_log_ = now;
  } else if (exit_stage_ == ExitStage::kWaitBlend) {
    if (ctrl == kAmpController && !transitioning) {
      std::string msg;
      if (!controller_manager_ || controller_manager_->setArmMode(ArmControlMode::kAuto, msg)) {
        exit_stage_ = ExitStage::kSendExit;
      } else if (now >= next_wait_log_) {
        RL_LOGW("TransportFallStandScheduler: setArmMode(auto) failed: %s", msg.c_str());
        next_wait_log_ = now + kWaitAmpLogIntervalSec;
      }
    } else if (now >= next_wait_log_) {
      RL_LOGI("TransportFallStandScheduler: waiting amp blend (current='%s' transitioning=%d)",
              ctrl.c_str(), transitioning ? 1 : 0);
      next_wait_log_ = now + kWaitAmpLogIntervalSec;
    }
  } else if (exit_stage_ == ExitStage::kSendExit) {
    if (static_cast<int>(transport_->state()) == kHandingOver) {
      transport_->sendCommand(TransportModeCommand::kExit);
      if (!pending_exit_voice_.empty()) playVoice(pending_exit_voice_);
    }
    exit_stage_ = ExitStage::kIdle;
    exit_from_ready_ = false;
    pending_exit_voice_.clear();
    RL_LOGI("TransportFallStandScheduler: exit done");
  }
}

}  // namespace runtime
}  // namespace leju
