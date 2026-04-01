/**
 * @file robot_base.h
 * @brief 机器人基础API接口定义
 */

#ifndef LEJUSDK_ROBOT_ROBOT_BASE_H_
#define LEJUSDK_ROBOT_ROBOT_BASE_H_
#include "lejusdk-lowlevel/data_types.h"
#include "lejusdk-utils/robot_version.hpp"
#include <functional>
#include <string>

namespace leju {

/** @cond INTERNAL */
namespace internal {
  template<typename CallbackType>
  struct CallbackVector;
}
/** @endcond */

/// @name 回调函数类型定义
/// @{
using RobotStateCallback = std::function<void(const RobotStateConstPtr&)>;
using RobotCmdCallback = std::function<void(const RobotCmdConstPtr&)>;
using ImuDataCallback = std::function<void(const ImuDataConstPtr&)>;
using JoyDataCallback = std::function<void(const JoyDataConstPtr&)>;
using HwStateCallback = std::function<void(const StringDataConstPtr&)>;
using StringDataCallback = std::function<void(const StringDataConstPtr&)>;
/// @}

/**
 * @class RobotBaseAPI
 * @brief 机器人基础API接口类
 *
 * 定义机器人通用接口，包括数据订阅和控制指令发布。
 */
class RobotBaseAPI {
public:
  /// @brief 构造函数
  /// @param version 机器人版本信息，参见 @ref RobotVersion
  explicit RobotBaseAPI(const RobotVersion& version);

  virtual ~RobotBaseAPI();

  /// @brief 获取关节数量
  virtual uint8_t getMotorNumber() const = 0;

  /// @brief 获取所有关节名称
  virtual std::vector<std::string> getMotorNames() const = 0;

  /// @brief 获取机器人版本信息
  const RobotVersion& getRobotVersion() const { return robot_version_; }

public:
  /// @name 用户端接口
  /// @brief 控制程序常用接口：订阅传感器数据，发布控制指令
  /// @{

  /// @brief 订阅 IMU 数据
  virtual void subscribeImuData(const ImuDataCallback& callback);

  /// @brief 订阅机器人状态
  virtual void subscribeRobotState(const RobotStateCallback& callback);

  /// @brief 订阅手柄数据
  virtual void subscribeJoyData(const JoyDataCallback& callback);

  /**
   * @brief 订阅硬件状态
   *
   * @param callback 回调函数，当接收到硬件状态时被调用。
   *                 接收到的硬件状态字符串包括: "unknown", "initializing", "ready_ok", "error"
   *                 可使用 @ref String2HwState 函数将字符串转换为 @ref HardwareState 枚举值
   */
  virtual void subscribeHardwareState(const HwStateCallback& callback);

  /**
   * @brief 发布机器人控制指令
   *
   * @param cmd 控制指令，参见 @ref RobotCmd
   * @return 发布成功返回 true
   *
   * @note 只有当硬件状态为 @ref HardwareState::READY_OK 时，发布的控制指令才会生效。
   *       可通过 @ref subscribeHardwareState 订阅硬件状态变化。
   */
  virtual bool publishRobotCmd(const RobotCmd& cmd);

  /// @brief 发布停止指令
  virtual bool publishStopRobot();

  /// @}

  /// @name 驱动端接口
  /// @brief 底层驱动或仿真程序使用：发布传感器数据，订阅控制指令
  /// @{

  /// @brief 订阅控制指令
  virtual void subscribeRobotCmd(const RobotCmdCallback& callback);

  /// @brief 发布硬件状态
  virtual bool publishHardwareState(HardwareState hw_state);

  /// @brief 发布 IMU 数据
  virtual bool publishImuData(const ImuData& imu_data);

  /// @brief 发布机器人状态
  virtual bool publishRobotState(const RobotState& state);

  /// @brief 发布手柄数据
  virtual bool publishJoyData(const JoyData& joy);

  /// @}

  /// @brief 停止通讯并释放资源
  virtual void shutdown();

/** @cond INTERNAL */
protected:
  ///////////////////////////////////////////////////////////////////////////
  // 回调函数成员变量，供子类使用
  ///////////////////////////////////////////////////////////////////////////
  internal::CallbackVector<ImuDataCallback>* imu_data_callbacks_;
  internal::CallbackVector<RobotStateCallback>* robot_state_callbacks_;
  internal::CallbackVector<JoyDataCallback>* joy_data_callbacks_;
  internal::CallbackVector<HwStateCallback>* hw_state_callbacks_;
  internal::CallbackVector<RobotCmdCallback>* robot_cmd_callbacks_;

  ///////////////////////////////////////////////////////////////////////////
  // 机器人版本信息
  ///////////////////////////////////////////////////////////////////////////
  RobotVersion robot_version_;
/** @endcond */
};

}  // namespace leju

#endif  // LEJUSDK_ROBOT_ROBOT_BASE_H_