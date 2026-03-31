#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <magic_enum/magic_enum.hpp>

#include "lejusdk-vr/data_types.h"
#include "leju-rl-controller/rl/multi_mode_arm_controller.h"
#include "leju-rl-controller/rl/waist_controller.h"
#include "leju-rl-controller/rl/rl_controller_types.h"
#include "leju-rl-controller/runtime/input/action_trigger.h"

namespace leju {
namespace runtime {

// 引入控制器中定义的模式枚举
using ArmControlMode = leju::ArmControlMode;      ///< 手臂控制模式
using WaistControlMode = leju::WaistControlMode;  ///< 腰部控制模式

// ============================================================================
// 基础类型定义
// ============================================================================

// 使用 lejusdk-vr 的 JointTrajectoryPoint 作为外部关节目标
using ExternalJointTarget = ::leju::vr::JointTrajectoryPoint;

/**
 * @brief 将字符串转换为 ArmControlMode
 * @param str 模式名称字符串 (如 "kKeepPose", "keep_pose", "0")
 * @return 手臂控制模式，无效值返回 std::nullopt
 */
inline std::optional<ArmControlMode> StringToArmControlMode(const std::string& str) {
  // 尝试直接匹配枚举名 (kKeepPose, kAuto, kExternal)
  if (auto result = magic_enum::enum_cast<ArmControlMode>(str); result.has_value()) {
    return result.value();
  }
  // 尝试匹配小写版本 (keep_pose, auto, external) 或数字字符串
  if (str == "keep_pose" || str == std::to_string(static_cast<int>(ArmControlMode::kKeepPose))) {
    return ArmControlMode::kKeepPose;
  }
  if (str == "auto" || str == std::to_string(static_cast<int>(ArmControlMode::kAuto))) {
    return ArmControlMode::kAuto;
  }
  if (str == "external" || str == std::to_string(static_cast<int>(ArmControlMode::kExternal))) {
    return ArmControlMode::kExternal;
  }
  return std::nullopt;
}

/**
 * @brief 将字符串转换为 WaistControlMode
 * @param str 模式名称字符串 (如 "kAuto", "auto", "1")
 * @return 腰部控制模式，无效值返回 std::nullopt
 */
inline std::optional<WaistControlMode> StringToWaistControlMode(const std::string& str) {
  // 尝试直接匹配枚举名 (kAuto, kExternal)
  if (auto result = magic_enum::enum_cast<WaistControlMode>(str); result.has_value()) {
    return result.value();
  }
  // 尝试匹配小写版本 (auto, external) 或数字字符串
  if (str == "auto" || str == std::to_string(static_cast<int>(WaistControlMode::kAuto))) {
    return WaistControlMode::kAuto;
  }
  if (str == "external" || str == std::to_string(static_cast<int>(WaistControlMode::kExternal))) {
    return WaistControlMode::kExternal;
  }
  return std::nullopt;
}

/**
 * @brief 运动控制指令 (用于 cmd_vel)
 *
 * 包含线速度、角速度和有效性标志
 */
struct MotionCommand {
  double linear_x = 0.0;    ///< 前进速度 [m/s]
  double linear_y = 0.0;    ///< 侧向速度 [m/s]
  double angular_z = 0.0;   ///< 旋转角速度 [rad/s]
  bool valid = false;       ///< 是否有效

  void setZero() {
    linear_x = linear_y = angular_z = 0.0;
  }

  /**
   * @brief 检查速度是否接近零（在死区内）
   * @param x_threshold x方向线速度阈值，默认 0.01
   * @param y_threshold y方向线速度阈值，默认 0.01
   * @param angular_threshold 角速度阈值，默认 0.01
   * @return 如果所有速度分量绝对值都小于对应阈值，返回 true
   *
   * 用途：判断输入源是否处于"静止"状态，如果是，可以让其他输入源接管控制
   */
  bool isNearZero(double x_threshold = 0.01, double y_threshold = 0.01,
                  double angular_threshold = 0.01) const {
    return std::abs(linear_x) < x_threshold &&
           std::abs(linear_y) < y_threshold &&
           std::abs(angular_z) < angular_threshold;
  }
};

} // namespace runtime
} // namespace leju
