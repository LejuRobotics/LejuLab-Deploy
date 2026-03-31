/**
 * @file head_solver.h
 * @brief Compute head yaw/pitch from Quest bone poses (Chest, Head)
 */

#ifndef LEJU_VR_CONTROL_HEAD_SOLVER_H_
#define LEJU_VR_CONTROL_HEAD_SOLVER_H_

#include <lejusdk-vr/data_types.h>
#include <vector>

namespace leju {
namespace vr_control {

using leju::vr::QuestBonePosesData;

/**
 * @brief Compute head joint target [pitch, yaw] (rad) from Quest bone poses.
 *
 * Uses poses[23] (Chest) and poses[25] (Head) to compute relative orientation.
 * @param quest_poses Quest bone poses
 * @param out_q Output: [pitch, yaw] in rad, or empty if invalid
 * @return true if successful
 */
bool computeHeadFromBones(const QuestBonePosesData& quest_poses,
                          std::vector<double>& out_q);

}  // namespace vr_control
}  // namespace leju

#endif  // LEJU_VR_CONTROL_HEAD_SOLVER_H_
