/**
 * @file kuavo_humanoid.h
 * @brief Kuavo 人形机器人接口
 */

#ifndef LEJUSDK_ROBOT_KUAVO_HUMANOID_H_
#define LEJUSDK_ROBOT_KUAVO_HUMANOID_H_

#include "lejusdk-lowlevel/robot_api/robot_base.h"
#include <memory>

namespace leju {

class GlobalRobot;

/**
 * @class KuavoHumanoid
 * @brief Kuavo 人形机器人接口类
 *
 * 实现 Kuavo 系列机器人的具体通信接口，通过 GlobalRobot 统一管理。
 */
class KuavoHumanoid : public RobotBaseAPI {
public:
  KuavoHumanoid(const KuavoHumanoid&) = delete;
  KuavoHumanoid& operator=(const KuavoHumanoid&) = delete;
  ~KuavoHumanoid() override = default;

  /// @brief 获取关节数量
  uint8_t getMotorNumber() const override;

  /// @brief 获取所有关节名称
  std::vector<std::string> getMotorNames() const override;

  /// @brief 获取手臂关节名称
  std::vector<std::string> getArmJointNames() const override;

  /// @brief 获取腰部关节名称
  std::vector<std::string> getWaistJointNames() const override;

  /// @brief 获取头部关节名称
  std::vector<std::string> getHeadJointNames() const override;

  /// @brief 获取腿部关节名称
  std::vector<std::string> getLegJointNames() const override;

/** @cond INTERNAL */
private:
  explicit KuavoHumanoid(const RobotVersion& version);
  friend class GlobalRobot;
/** @endcond */
};

}  // namespace leju

#endif  // LEJUSDK_ROBOT_KUAVO_HUMANOID_H_