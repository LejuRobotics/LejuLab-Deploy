/**
 * @file quest_vr_abs_fsm.cpp
 * @brief Quest VR 绝对式手臂模式状态机实现
 */

#include "leju-vr-control/quest_vr_abs_fsm.h"

namespace leju {
namespace vr_control {

QuestVrAbsFSMAction QuestVrAbsFSM::update(const QuestJoystickData& joy,
                                          int current_mode) {
  QuestVrAbsFSMAction action;
  action.arm_mode = current_mode;

  const bool xa_now = isXAPressed(joy);
  const bool xb_now = isXBPressed(joy);
  const bool xy_now = isXYPressed(joy);
  const bool y_now = isYPressed(joy);  
  const bool xy_touched_now = isXYTouched(joy);

  // X+Y = 退出程序（边沿触发一次，避免按住时重复 publishStopRobot）。
  if (xy_now && !xy_prev_) {
    action.request_quit = true;
  }

  // 优先判定 X+B（X+B 用右次按钮，与 X+A 互斥），与增量 FSM 保持同样的判定顺序。
  if (xb_now && !xb_prev_) {
    action.arm_mode = (current_mode == 0) ? 1 : 0;  // 保持⇄自动；外部→保持
    action.request_set_arm_mode = true;
    action.xb_pressed_event = true;
  } else if (xa_now && !xa_prev_) {
    action.arm_mode = (current_mode == 2) ? 1 : 2;  // 外部⇄自动；保持→外部直达
    action.request_set_arm_mode = true;
    action.xa_pressed_event = true;
  }
  // X + Y 触摸判定
  if (xy_touched_now) {
    // 可在节点中用作特殊状态判定（目前用于squat判定）
    action.xy_touched_event = true;
  }
  // Y 按下判定
  if (y_now && !y_prev_ && !xy_now) {
    action.y_pressed_event = true;
  }
  xa_prev_ = xa_now;
  xb_prev_ = xb_now;
  xy_prev_ = xy_now;
  y_prev_ = y_now;

  return action;
}

}  // namespace vr_control
}  // namespace leju
