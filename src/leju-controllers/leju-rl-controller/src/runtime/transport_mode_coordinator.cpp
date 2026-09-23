#include "leju-rl-controller/runtime/transport_mode_coordinator.h"

#include <algorithm>
#include <cctype>

#include "leju-rl-controller/controllers/controller_base.h"
#include "leju-rl-controller/controllers/controller_manager.h"
#include "leju-rl-controller/rl/multi_mode_arm_controller.h"
#include "leju-rl-controller/rl_log.h"

namespace leju {
namespace runtime {

namespace {

/// 手臂插值到零位（自然下垂）的固定时长 [s]，起止速度为零
constexpr double kTransportArmMoveDuration = 2.0;

std::string ToUpper(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  return s;
}

}  // namespace

bool TransportModeCoordinator::initialize(dds::domain::DomainParticipant& participant) {
  try {
    state_pub_ = std::make_unique<leju::dds_common::TopicPublisher<leju::msgs::Float64>>(
        participant, leju::dds_topics::kTransportModeState);

    auto self = this;
    command_sub_ = std::make_unique<leju::dds_common::TopicSubscriber<leju::msgs::StringData>>(
        participant,
        leju::dds_topics::kTransportModeCommand,
        [self](const leju::msgs::StringData& msg) {
          self->onCommand(msg.data());
        });

    RL_LOGI("TransportModeCoordinator: DDS topics initialized");
    return true;
  } catch (const std::exception& e) {
    RL_LOG_FAILURE("TransportModeCoordinator: failed to initialize DDS: %s", e.what());
    return false;
  }
}

void TransportModeCoordinator::onCommand(const std::string& command) {
  const std::string cmd = ToUpper(command);
  const auto current = state();

  if (cmd == TransportModeCommand::kEnter) {
    if (current != TransportModeState::kInactive) {
      RL_LOGW("TransportModeCoordinator: ENTER ignored, current state=%s",
              stateToString(current).c_str());
      return;
    }
    // 门控前置（对齐研杨基线：进入前校验，不满足则不进入）：仅 amp + 站立 + 手臂可用
    auto* ctrl = controller_manager_ ? controller_manager_->getCurrentController() : nullptr;
    if (!ctrl || controller_manager_->getCurrentControllerName() != "amp" ||
        !ctrl->isStanding() || !ctrl->getArmController()) {
      RL_LOGW("TransportModeCoordinator: ENTER rejected, need amp controller + standing");
      return;
    }
    setState(TransportModeState::kInterpolating);
    RL_LOGI("TransportModeCoordinator: ENTER -> INTERPOLATING");
  } else if (cmd == TransportModeCommand::kLock) {
    if (current != TransportModeState::kReady) {
      RL_LOGW("TransportModeCoordinator: LOCK ignored, current state=%s",
              stateToString(current).c_str());
      return;
    }
    // 门控前置：站立 + hold_pose 控制器可用（速度指令已由搬运门控）
    auto* ctrl = controller_manager_ ? controller_manager_->getCurrentController() : nullptr;
    if (!ctrl || !ctrl->isStanding() ||
        !controller_manager_->hasController("hold_pose")) {
      RL_LOGW("TransportModeCoordinator: LOCK rejected, need standing + hold_pose controller");
      return;
    }
    // 对齐研杨 lock_pending：状态保持 READY，待 tick 切 hold_pose 提交后才翻 ACTIVE
    lock_pending_.store(true);
    RL_LOGI("TransportModeCoordinator: LOCK pending (switch to hold_pose)");
  } else if (cmd == TransportModeCommand::kHandOver) {
    if (current != TransportModeState::kReady && current != TransportModeState::kActive) {
      RL_LOGW("TransportModeCoordinator: HAND_OVER ignored, current state=%s",
              stateToString(current).c_str());
      return;
    }
    // 零副作用；取消未完成的进入动作（LOCK/FALL_DOWN pending）
    lock_pending_.store(false);
    fall_down_pending_.store(false);
    setState(TransportModeState::kHandingOver);
    RL_LOGI("TransportModeCoordinator: HAND_OVER -> HANDING_OVER");
  } else if (cmd == TransportModeCommand::kFallDown) {
    if (current != TransportModeState::kActive) {
      RL_LOGW("TransportModeCoordinator: FALL_DOWN ignored, current state=%s",
              stateToString(current).c_str());
      return;
    }
    // 门控前置：站立 + mimic_fall_stand 控制器可用
    auto* ctrl = controller_manager_ ? controller_manager_->getCurrentController() : nullptr;
    if (!ctrl || !ctrl->isStanding() ||
        !controller_manager_->hasController("mimic_fall_stand")) {
      RL_LOGW("TransportModeCoordinator: FALL_DOWN rejected, need standing + "
              "mimic_fall_stand controller");
      return;
    }
    // 状态保持 ACTIVE，待 tick 切到 mimic_fall_stand 后才翻 HANDING_OVER
    fall_down_pending_.store(true);
    RL_LOGI("TransportModeCoordinator: FALL_DOWN pending (handoff to fall-stand)");
  } else if (cmd == TransportModeCommand::kExit) {
    if (current != TransportModeState::kHandingOver) {
      RL_LOGW("TransportModeCoordinator: EXIT ignored, current state=%s (need HAND_OVER first)",
              stateToString(current).c_str());
      return;
    }
    // 无条件接受（对齐研杨）：回切 amp + 手臂复位由调度层在 EXIT 前完成
    setState(TransportModeState::kInactive);
    RL_LOGI("TransportModeCoordinator: EXIT -> INACTIVE");
  } else {
    RL_LOGW("TransportModeCoordinator: unknown command '%s'", command.c_str());
  }
}

void TransportModeCoordinator::tick(ControllerManager& controller_manager,
                                    bool running, double now) {
  // 状态发布不受 Running 限制（对齐研杨：非 Running 期间也持续发布状态）
  if (state_pub_) {
    leju::msgs::Float64 state_msg;
    state_msg.data(static_cast<double>(state_.load()));
    state_pub_->publish(state_msg);
  }

  const auto tm_state = state();
  if (tm_state != last_state_) {
    RL_LOGI("TransportModeCoordinator: state %s -> %s", stateToString(last_state_).c_str(),
            stateToString(tm_state).c_str());
    last_state_ = tm_state;
    if (tm_state == TransportModeState::kInterpolating) {
      arm_move_requested_ = false;
    }
  }

  if (!running) {
    return;
  }

  if (tm_state == TransportModeState::kInterpolating) {
    handleInterpolating(controller_manager);
  }
  if (lock_pending_.load()) {
    handleLockPending(controller_manager, now);
  }
  if (fall_down_pending_.load()) {
    handleFallDownPending(controller_manager, now);
  }
}

void TransportModeCoordinator::handleInterpolating(ControllerManager& controller_manager) {
  // ENTER 门控已在接受时完成，此处只执行副作用：手臂切 kExternal 归零，到位自动 READY，不检查不退出
  auto* arm_ctrl = controller_manager.getCurrentController()->getArmController();
  if (!arm_move_requested_) {
    if (arm_ctrl->getMode() != ArmControlMode::kExternal) {
      std::string msg;
      // 不可达：ENTER 门控已校验 amp+手臂可用，搬运期间无切换发起（不 transitioning）
      if (!controller_manager.setArmMode(ArmControlMode::kExternal, msg)) {
        RL_LOGE("TransportModeCoordinator: setArmMode(kExternal) failed unexpectedly: %s",
                msg.c_str());
        return;  // 防御路径：不置标记，下周期重试（不退出状态机）
      }
    }
    arm_move_requested_ = true;
    arm_ctrl->moveToExternalTarget(
        Eigen::VectorXd::Zero(arm_ctrl->getDesiredPosition().size()),
        kTransportArmMoveDuration);
    RL_LOGI("TransportModeCoordinator: arm -> kExternal, min-jerk to zero pose over %.1fs",
            kTransportArmMoveDuration);
  }
  // 到位检测 -> READY
  if (arm_ctrl->getMode() == ArmControlMode::kExternal &&
      !arm_ctrl->isModeTransitioning() && !arm_ctrl->isApproaching()) {
    setState(TransportModeState::kReady);
    RL_LOGI("TransportModeCoordinator: pose ready -> READY");
  }
}

void TransportModeCoordinator::handleLockPending(ControllerManager& controller_manager,
                                                 double now) {
  const std::string current = controller_manager.getCurrentControllerName();
  if (current == "hold_pose" && !controller_manager.isTransitioning()) {
    lock_pending_.store(false);
    setState(TransportModeState::kActive);
    RL_LOGI("TransportModeCoordinator: LOCK -> ACTIVE (hold_pose committed, whole-body freeze)");
    return;
  }
  if (controller_manager.isTransitioning()) {
    return;  // 切换进行中：等确认完成，不重复触发
  }
  // 单次尝试：接受时已校验，此处失败不可达；失败清 pending 保持 READY，再按 LOCK 可重触发
  // 跳过混合：LOCK 应立刻按当前反馈位冻住；混合会把身体拖向 hold 标称/中间态再捕获
  if (!controller_manager.requestSwitch("hold_pose", now, /*auto_start_motion=*/false,
                                        /*instant_commit=*/true)) {
    lock_pending_.store(false);
    RL_LOGE("TransportModeCoordinator: LOCK switch failed unexpectedly (gated at accept); "
            "state stays READY, press LOCK again");
    return;
  }
  RL_LOGI("TransportModeCoordinator: LOCK -> instant switch %s -> hold_pose", current.c_str());
}

void TransportModeCoordinator::handleFallDownPending(ControllerManager& controller_manager,
                                                     double now) {
  const std::string current = controller_manager.getCurrentControllerName();
  if (current == "mimic_fall_stand") {
    fall_down_pending_.store(false);
    setState(TransportModeState::kHandingOver);
    RL_LOGI("TransportModeCoordinator: FALL_DOWN -> HANDING_OVER (handoff to fall-stand)");
    return;
  }
  if (controller_manager.isTransitioning()) {
    return;  // 切换进行中：等确认完成，不重复触发
  }
  // 单次尝试：被拒不自动重试，清 pending 保持 ACTIVE，再按 FALL_DOWN 可重新触发
  // instant commit 切 mimic_fall_stand 即进瘫软（limp 零力矩，等人摆放/倒地静止）；
  // 不自动 prepare、不起播——PREPARE/STAND_UP 由调度层或外部客户端经 /rt/fall_stand_command 驱动
  if (!controller_manager.requestSwitch("mimic_fall_stand", now, /*auto_start_motion=*/false,
                                        /*instant_commit=*/true)) {
    fall_down_pending_.store(false);
    RL_LOGW("TransportModeCoordinator: FALL_DOWN rejected, giving up (state ACTIVE); "
            "press FALL_DOWN again to retry");
    return;
  }
  RL_LOGI("TransportModeCoordinator: FALL_DOWN -> switching %s -> mimic_fall_stand",
          current.c_str());
}

void TransportModeCoordinator::setState(TransportModeState new_state) {
  state_.store(static_cast<int>(new_state));
}

std::string TransportModeCoordinator::stateToString(TransportModeState state) const {
  switch (state) {
    case TransportModeState::kInactive:
      return "INACTIVE";
    case TransportModeState::kInterpolating:
      return "INTERPOLATING";
    case TransportModeState::kReady:
      return "READY";
    case TransportModeState::kActive:
      return "ACTIVE";
    case TransportModeState::kHandingOver:
      return "HANDING_OVER";
  }
  return "UNKNOWN";
}

}  // namespace runtime
}  // namespace leju
