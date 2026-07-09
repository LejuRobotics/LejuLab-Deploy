/**
 * @file quest_to_ik_converter.h
 * @brief Convert Quest data (lejusdk-vr) to leju-ik types
 */

#ifndef LEJU_VR_CONTROL_QUEST_TO_IK_CONVERTER_H_
#define LEJU_VR_CONTROL_QUEST_TO_IK_CONVERTER_H_

#include <leju-ik/ik_types.h>
#include <lejusdk-vr/data_types.h>

namespace leju {
namespace vr_control {

using leju::vr::QuestBonePosesData;
using leju::vr::QuestJoystickData;

/**
 * @brief Convert QuestBonePosesData to PoseInfoList for leju-ik
 */
leju::ik::PoseInfoList toPoseInfoList(const QuestBonePosesData& quest_poses);

/**
 * @brief Convert QuestJoystickData to JoyStickData for leju-ik
 */
leju::ik::JoyStickData toJoyStickData(const QuestJoystickData& quest_joy);

}  // namespace vr_control
}  // namespace leju

#endif  // LEJU_VR_CONTROL_QUEST_TO_IK_CONVERTER_H_
