/**
 * @file roban_humanoid.h
 * @brief Roban 人形机器人接口
 */

#ifndef LEJUSDK_ROBOT_ROBAN_HUMANOID_H_
#define LEJUSDK_ROBOT_ROBAN_HUMANOID_H_

#include "lejusdk-lowlevel/robot_api/robot_base.h"

namespace leju {

class GlobalRobot;

/**
 * @class RobanHumanoid
 * @brief Roban 人形机器人接口类
 *
 * 实现 Roban 系列机器人的具体通信接口，通过 GlobalRobot 统一管理。
 */
class RobanHumanoid : public RobotBaseAPI {
public:
  RobanHumanoid(const RobanHumanoid&) = delete;
  RobanHumanoid& operator=(const RobanHumanoid&) = delete;
  ~RobanHumanoid() override = default;

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
  explicit RobanHumanoid(const RobotVersion& version);
  friend class GlobalRobot;
/** @endcond */
};

}  // namespace leju

#endif  // LEJUSDK_ROBOT_ROBAN_HUMANOID_H_