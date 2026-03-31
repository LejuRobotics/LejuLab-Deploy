#include "leju-rl-controller/runtime/lifecycle.h"

#include <magic_enum/magic_enum.hpp>

#include "leju-rl-controller/runtime/input/action_trigger.h"
#include "leju-rl-controller/rl_log.h"

namespace leju {
namespace runtime {

static const char* LifecycleStateToString(LifecycleState state) {
  auto name = magic_enum::enum_name(state);
  return name.empty() ? "Unknown" : name.data();
}

void Lifecycle::update(bool ready, const std::vector<ActionTrigger>& triggers) {
  // 检查是否存在 Start 或 Quit 触发器
  bool has_start = false;
  bool has_quit = false;
  for (const auto& trigger : triggers) {
    if (trigger.type == ActionType::Start) has_start = true;
    if (trigger.type == ActionType::Quit) has_quit = true;
  }

  // 规则：quit 在任意状态下都触发退出
  if (has_quit && state_ != LifecycleState::kExiting) {
    state_ = LifecycleState::kExiting;
    LOG_BANNER("Lifecycle => 退出中");
    RL_LOG_SUCCESS("Lifecycle: Quit triggered, exiting program from state %s",
                   LifecycleStateToString(state_));
    return;
  }

  // 规则：任意状态 + ready=false -> kWaitingForReady
  if (!ready) {
    if (state_ != LifecycleState::kWaitingForReady) {
      RL_LOG_WARNING("Lifecycle: Robot not ready, falling back to WaitingForReady from %s",
                     LifecycleStateToString(state_));
      state_ = LifecycleState::kWaitingForReady;
    }
    return;
  }

  // 正常状态转换
  switch (state_) {
    case LifecycleState::kWaitingForReady:
      // 规则：kWaitingForReady + ready=true -> kWaitingForStart
      state_ = LifecycleState::kWaitingForStart;
      LOG_BANNER("Lifecycle => 机器人就绪");
      RL_LOG_SUCCESS("Lifecycle: 机器人就绪，等待`start`启动信号");
      // 在 WaitingForReady 状态下忽略 start（必须先进入 WaitingForStart）
      if (has_start) {
        RL_LOG_WARNING("Lifecycle: Start trigger ignored, robot not ready yet");
      }
      break;

    case LifecycleState::kWaitingForStart:
      // 规则：kWaitingForStart + "start" trigger -> kRunning
      if (has_start) {
        state_ = LifecycleState::kRunning;
        LOG_BANNER("Lifecycle => 运行中");
        RL_LOG_SUCCESS("Lifecycle: Start triggered, entering running state");
      }
      break;

    case LifecycleState::kRunning:
    case LifecycleState::kExiting:
      // Running: 忽略 start 防止重复触发
      // Exiting: 保持状态不变，忽略所有触发器
      if (has_start) {
        RL_LOG_WARNING("Lifecycle: Start trigger ignored (state: %s)",
                       LifecycleStateToString(state_));
      }
      break;
  }
}

}  // namespace runtime
}  // namespace leju
