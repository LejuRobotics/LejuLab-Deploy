#include "leju-rl-controller/runtime/fall_stand_coordinator.h"

#include <algorithm>
#include <cctype>

#include "leju-rl-controller/controllers/controller_manager.h"
#include "leju-rl-controller/controllers/generic_rl_controller.h"
#include "leju-rl-controller/rl_log.h"

namespace leju {
namespace runtime {

namespace {

/// 心跳周期 [s]：状态无变化时的最低发布频率（对齐研杨激活期间持续发布）
constexpr double kHeartbeatPeriod = 0.1;

std::string ToUpper(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  return s;
}

}  // namespace

bool FallStandCoordinator::initialize(dds::domain::DomainParticipant& participant) {
  try {
    state_pub_ = std::make_unique<leju::dds_common::TopicPublisher<leju::msgs::Float64>>(
        participant, leju::dds_topics::kFallStandState);

    auto self = this;
    command_sub_ = std::make_unique<leju::dds_common::TopicSubscriber<leju::msgs::StringData>>(
        participant,
        leju::dds_topics::kFallStandCommand,
        [self](const leju::msgs::StringData& msg) {
          self->onCommand(msg.data());
        });

    RL_LOGI("FallStandCoordinator: DDS topics initialized");
    return true;
  } catch (const std::exception& e) {
    RL_LOG_FAILURE("FallStandCoordinator: failed to initialize DDS: %s", e.what());
    return false;
  }
}

void FallStandCoordinator::onCommand(const std::string& command) {
  const std::string cmd = ToUpper(command);
  if (cmd != FallStandCommand::kPrepare && cmd != FallStandCommand::kStandUp) {
    RL_LOGW("FallStandCoordinator: unknown command '%s' (expect PREPARE/STAND_UP; "
            "RESET is not implemented)", command.c_str());
    return;
  }
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_command_ = cmd;
  }
  RL_LOGI("FallStandCoordinator: command '%s' pending", cmd.c_str());
}

void FallStandCoordinator::tick(ControllerManager& controller_manager,
                                bool control_output_allowed, double now) {
  const std::string current = controller_manager.getCurrentControllerName();
  auto* grl = (current == "mimic_fall_stand" && !controller_manager.isTransitioning())
                  ? dynamic_cast<GenericRLController*>(controller_manager.getCurrentController())
                  : nullptr;
  const int fs_state = grl ? grl->getFallStandState() : -1;
  last_fs_state_ = fs_state;

  if (control_output_allowed) {
    // 消费命令（阶段门控，对齐研杨；命令时机由调度层掌握，运控层只执行）
    const std::string cmd = takePendingCommand();
    if (!cmd.empty()) {
      if (!grl) {
        RL_LOGW("FallStandCoordinator: command '%s' ignored, current controller='%s'",
                cmd.c_str(), current.c_str());
      } else if (cmd == FallStandCommand::kPrepare) {
        if (fs_state != 0) {
          RL_LOGW("FallStandCoordinator: PREPARE ignored, state=%d (need 0=FALL_DOWN)",
                  fs_state);
        } else {
          RL_LOGI("FallStandCoordinator: PREPARE -> prepareToMotionStart");
          controller_manager.prepareToMotionStart();
        }
      } else if (cmd == FallStandCommand::kStandUp) {
        if (fs_state != 2) {
          RL_LOGW("FallStandCoordinator: STAND_UP ignored, state=%d (need 2=READY_FOR_STAND_UP)",
                  fs_state);
        } else {
          RL_LOGI("FallStandCoordinator: STAND_UP -> startMotion");
          controller_manager.startMotion();
        }
      }
    }
  }

  publishState(fs_state, now);
}

void FallStandCoordinator::publishState(int state, double now) {
  if (!state_pub_) {
    return;
  }
  if (state < 0) {
    // fall-stand 控制器未激活：不发布，重置 on-change 记录以便下次激活时立即发布
    last_published_state_ = -1;
    return;
  }
  const bool changed = (state != last_published_state_);
  const bool heartbeat_due = (now - last_publish_time_ >= kHeartbeatPeriod);
  if (!changed && !heartbeat_due) {
    return;
  }
  leju::msgs::Float64 state_msg;
  state_msg.data(static_cast<double>(state));
  state_pub_->publish(state_msg);
  if (changed) {
    RL_LOGI("FallStandCoordinator: state %d -> %d", last_published_state_, state);
  }
  last_published_state_ = state;
  last_publish_time_ = now;
}

std::string FallStandCoordinator::takePendingCommand() {
  std::lock_guard<std::mutex> lock(pending_mutex_);
  std::string cmd;
  cmd.swap(pending_command_);
  return cmd;
}

void FallStandCoordinator::clearPendingCommand() {
  std::lock_guard<std::mutex> lock(pending_mutex_);
  pending_command_.clear();
}

}  // namespace runtime
}  // namespace leju
