/**
 * @file quest_combo.h
 * @brief Quest 手柄组合键谓词（增量/绝对 FSM 共享，避免重复）
 *
 * 按键命名：X = 左主按钮(left_first)，Y = 左次按钮(left_second)，
 *           A = 右主按钮(right_first)，B = 右次按钮(right_second)。
 * X 为辅助键，与右手 A/B 或左手 Y 组合分别表达不同语义。
 */

#ifndef LEJU_VR_CONTROL_QUEST_COMBO_H_
#define LEJU_VR_CONTROL_QUEST_COMBO_H_

#include <lejusdk-vr/data_types.h>

namespace leju {
namespace vr_control {

using leju::vr::QuestJoystickData;

/// X+A：左主 + 右主
inline bool isXAPressed(const QuestJoystickData& joy) {
  return joy.left_first_button_pressed && joy.right_first_button_pressed;
}

/// X+B：左主 + 右次
inline bool isXBPressed(const QuestJoystickData& joy) {
  return joy.left_first_button_pressed && joy.right_second_button_pressed;
}

/// X+Y：左主 + 左次（退出程序）
inline bool isXYPressed(const QuestJoystickData& joy) {
  return joy.left_first_button_pressed && joy.left_second_button_pressed;
}

}  // namespace vr_control
}  // namespace leju

#endif  // LEJU_VR_CONTROL_QUEST_COMBO_H_
