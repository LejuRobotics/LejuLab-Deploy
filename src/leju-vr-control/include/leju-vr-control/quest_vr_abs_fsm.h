/**
 * @file quest_vr_abs_fsm.h
 * @brief Quest VR 绝对式手臂控制的按键状态机（X+A / X+B 切换手臂模式）
 *
 * 与增量式 QuestVrFSM 的区别（故单独分岔，互不影响）：
 *   - X+A：外部(2) ⇄ 自动(1)，且「保持(0) → 外部(2)」可直达（保持下直接进外部控制）。
 *   - X+B：保持(0) ⇄ 自动(1)，外部(2) → 保持(0)。
 *
 * 绝对式下「双手扳机 OK 手势」与「进入外部控制」是等效的：OK 手势本身即进入
 * kExternal 并开始 IK 跟随，无需先按 X+A。本 FSM 只负责 X+A / X+B 的模式切换，
 * OK 手势进外部由节点检测 arm_mapper.isRunning() 完成。
 *
 * 本 FSM 不持有权威模式：权威模式由节点的原子量维护（OK 手势与按键两条路径都会改它），
 * update() 以「当前模式」为入参算出目标模式，自身只保留 X+A/X+B 的边沿检测状态。
 */

#ifndef LEJU_VR_CONTROL_QUEST_VR_ABS_FSM_H_
#define LEJU_VR_CONTROL_QUEST_VR_ABS_FSM_H_

#include <lejusdk-vr/data_types.h>

#include "leju-vr-control/quest_combo.h"

namespace leju {
namespace vr_control {

using leju::vr::QuestJoystickData;

/**
 * @brief 绝对式 FSM 动作输出
 */
struct QuestVrAbsFSMAction {
  bool request_set_arm_mode = false;
  int arm_mode = 1;               // 0=KeepPose, 1=Auto, 2=External
  bool xa_pressed_event = false;  // X+A 按下
  bool xb_pressed_event = false;  // X+B 按下
  bool request_quit = false;      // X+Y 按下（退出程序，边沿触发一次）
};

/**
 * @brief Quest VR 绝对式手臂模式状态机
 *
 * 三模式：保持(0) / 自动摆手(1) / 外部控制(2)。边沿触发（按下瞬间）：
 *   - X+A (左主+右主)：mode = (mode == 2) ? 1 : 2   // 外部⇄自动；保持→外部直达
 *   - X+B (左主+右次)：mode = (mode == 0) ? 1 : 0   // 保持⇄自动；外部→保持
 */
class QuestVrAbsFSM {
 public:
  QuestVrAbsFSM() = default;

  /**
   * @brief 用最新手柄数据更新 FSM
   * @param joy          当前手柄状态
   * @param current_mode 当前权威手臂模式（0=KeepPose/1=Auto/2=External）
   * @return 需要执行的动作（是否切模式 + 目标模式）
   */
  QuestVrAbsFSMAction update(const QuestJoystickData& joy, int current_mode);

 private:
  bool xa_prev_ = false;
  bool xb_prev_ = false;
  bool xy_prev_ = false;
};

}  // namespace vr_control
}  // namespace leju

#endif  // LEJU_VR_CONTROL_QUEST_VR_ABS_FSM_H_
