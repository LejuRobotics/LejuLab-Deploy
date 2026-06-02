/**
 * @file quest_vr_fsm.h
 * @brief Quest VR joystick state machine (unlock arm, switch arm mode)
 */

#ifndef LEJU_VR_CONTROL_QUEST_VR_FSM_H_
#define LEJU_VR_CONTROL_QUEST_VR_FSM_H_

#include <lejusdk-vr/data_types.h>
#include <cstdint>

#include "leju-vr-control/quest_combo.h"

namespace leju {
namespace vr_control {

using leju::vr::QuestJoystickData;

/**
 * @brief FSM action output
 */
struct QuestVrFSMAction {
  bool request_set_arm_mode = false;
  int arm_mode = 0;  // 0=KeepPose, 1=Auto, 2=External
  bool xa_pressed_event = false;  // X+A 按下（切换手臂模式）
  bool xb_pressed_event = false;  // X+B 按下（关闭手臂控制）
};

/**
 * @brief Quest VR joystick FSM
 *
 * 对齐 kuavo-ros-control：仅用 X+A / X+B 控制，无 trigger 解锁、无 grip 锁定。
 *
 * - Switch arm mode: X + A (left first + right first button) 在 Auto(1)/External(2) 间切换
 * - KeepPose toggle: X + B 在 KeepPose(0)/External(2) 间切换
 */
class QuestVrFSM {
 public:
  QuestVrFSM() = default;

  /**
   * @brief Update FSM with new joystick data
   * @param joy Current joystick state
   * @param dt_ms Time since last update (ms)
   * @return Action to take
   */
  QuestVrFSMAction update(const QuestJoystickData& joy, int dt_ms);

 private:
  int arm_mode_ = 1;  // 站立默认 1（自动摆臂），X+A 一次切到 2（外部控制）
  bool xa_prev_ = false;
  bool xb_prev_ = false;
};

}  // namespace vr_control
}  // namespace leju

#endif  // LEJU_VR_CONTROL_QUEST_VR_FSM_H_
