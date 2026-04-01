/**
 * @file kuavo_vr_api.h
 * @brief Kuavo 机器人 VR 控制 API
 */

#ifndef LEJUSDK_VR_KUAVO_VR_API_H_
#define LEJUSDK_VR_KUAVO_VR_API_H_

#include "vr_base.h"

namespace leju {
namespace vr {

/**
 * @brief Kuavo 机器人 VR 控制 API
 *
 * 提供与 Kuavo 机器人的 VR 控制通信接口。
 * 继承自 VRBaseAPI，通过指定 RobotVersion 标识机器人类型。
 */
class KuavoVRAPI : public VRBaseAPI {
 public:
  /**
   * @brief 构造函数
   * @param version 机器人版本，默认为 KUAVO5_BASE
   */
  explicit KuavoVRAPI(const RobotVersion& version = RobotVersions::KUAVO5_BASE);
  ~KuavoVRAPI() override = default;

  /// @brief 禁用拷贝
  KuavoVRAPI(const KuavoVRAPI&) = delete;
  KuavoVRAPI& operator=(const KuavoVRAPI&) = delete;
};

}  // namespace vr
}  // namespace leju

#endif  // LEJUSDK_VR_KUAVO_VR_API_H_
