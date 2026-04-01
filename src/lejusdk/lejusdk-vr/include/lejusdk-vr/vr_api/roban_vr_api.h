/**
 * @file roban_vr_api.h
 * @brief Roban 机器人 VR 控制 API
 */

#ifndef LEJUSDK_VR_ROBAN_VR_API_H_
#define LEJUSDK_VR_ROBAN_VR_API_H_

#include "vr_base.h"

namespace leju {
namespace vr {

/**
 * @brief Roban 机器人 VR 控制 API
 *
 * 提供与 Roban 机器人的 VR 控制通信接口。
 * 继承自 VRBaseAPI，通过指定 RobotVersion 标识机器人类型。
 */
class RobanVRAPI : public VRBaseAPI {
 public:
  /**
   * @brief 构造函数
   * @param version 机器人版本，默认为 ROBAN2_BASE
   */
  explicit RobanVRAPI(const RobotVersion& version = RobotVersions::ROBAN2_BASE);
  ~RobanVRAPI() override = default;

  /// @brief 禁用拷贝
  RobanVRAPI(const RobanVRAPI&) = delete;
  RobanVRAPI& operator=(const RobanVRAPI&) = delete;
};

}  // namespace vr
}  // namespace leju

#endif  // LEJUSDK_VR_ROBAN_VR_API_H_
