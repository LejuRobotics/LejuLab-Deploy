/**
 * @file vr_base.h
 * @brief VR 控制 API 基类定义
 */

#ifndef LEJUSDK_VR_VR_BASE_H_
#define LEJUSDK_VR_VR_BASE_H_

#include <string>

#include <lejusdk-utils/robot_version.hpp>

#include "lejusdk-vr/data_types.h"

namespace leju {
namespace vr {

/**
 * @brief VR 控制 API 基类
 *
 * 提供 VR 控制的通用接口实现，通过 VRCommunication 单例与机器人通信。
 * 主要功能包括：
 * - 关节指令发布/订阅（手臂、头部、腰部）
 * - 速度指令发布
 * - 控制器切换与模式设置
 *
 * 子类（KuavoVRAPI、RobanVRAPI）通过指定 RobotVersion 来区分机器人类型。
 */
class VRBaseAPI {
 public:
  /**
   * @brief 构造函数
   * @param version 机器人版本
   */
  explicit VRBaseAPI(const RobotVersion& version);

  virtual ~VRBaseAPI();

  /// @brief 禁用拷贝
  VRBaseAPI(const VRBaseAPI&) = delete;
  VRBaseAPI& operator=(const VRBaseAPI&) = delete;

  // 初始化与生命周期

  /**
   * @brief 初始化 VR API
   * @return true 成功，false 失败
   */
  bool initialize();

  /**
   * @brief 关闭 VR API，释放资源
   */
  void shutdown();

  /**
   * @brief 检查是否已初始化
   * @return true 已初始化，false 未初始化
   */
  bool isInitialized() const;

  // 关节指令发布

  /**
   * @brief 发布手臂关节指令
   * @param cmd 关节指令（位置、速度、加速度）
   * @return true 成功，false 失败
   */
  bool publishArmJointCmd(const JointTrajectoryPoint& cmd);

  /**
   * @brief 发布头部关节指令
   * @param cmd 关节指令（位置、速度、加速度）
   * @return true 成功，false 失败
   */
  bool publishHeadJointCmd(const JointTrajectoryPoint& cmd);

  /**
   * @brief 发布腰部关节指令
   * @param cmd 关节指令（位置、速度、加速度）
   * @return true 成功，false 失败
   */
  bool publishWaistJointCmd(const JointTrajectoryPoint& cmd);

  /**
   * @brief 发布底盘速度指令
   * @param cmd 控制器最终速度指令（线速度、角速度）
   * @return true 成功，false 失败
   */
  bool publishVelocityCmd(const VelocityCmd& cmd);

  /**
   * @brief 发布 VR 速度输入
   * @param cmd VR 输入速度指令（线速度、角速度）
   * @return true 成功，false 失败
   */
  bool publishVrVelocityCmd(const VrVelocityCmd& cmd);

  /**
   * @brief 发布 Quest3 骨骼位姿数据
   * @param data Quest 骨骼位姿
   * @return true 成功，false 失败
   */
  bool publishQuestBonePoses(const QuestBonePosesData& data);

  /**
   * @brief 发布 Quest3 手柄数据
   * @param data Quest 手柄数据
   * @return true 成功，false 失败
   */
  bool publishQuestJoystickData(const QuestJoystickData& data);

  // 指令订阅

  /**
   * @brief 订阅手臂关节指令
   * @param callback 接收到指令时的回调函数
   */
  void subscribeArmJointCmd(ArmTrajectoryCallback callback);

  /**
   * @brief 订阅头部关节指令
   * @param callback 接收到指令时的回调函数
   */
  void subscribeHeadJointCmd(HeadTrajectoryCallback callback);

  /**
   * @brief 订阅腰部关节指令
   * @param callback 接收到指令时的回调函数
   */
  void subscribeWaistJointCmd(WaistTrajectoryCallback callback);

  /**
   * @brief 订阅速度指令
   * @param callback 接收到指令时的回调函数
   */
  void subscribeVelocityCmd(VelocityCmdCallback callback);

  /**
   * @brief 订阅 VR 速度指令
   * @param callback 接收到指令时的回调函数
   */
  void subscribeVrVelocityCmd(VrVelocityCmdCallback callback);

  /**
   * @brief 订阅 Quest3 骨骼位姿数据
   * @param callback 接收到数据时的回调函数
   */
  void subscribeQuestBonePoses(QuestBonePosesCallback callback);

  /**
   * @brief 订阅 Quest3 手柄数据
   * @param callback 接收到数据时的回调函数
   */
  void subscribeQuestJoystickData(QuestJoystickDataCallback callback);

  // 控制器服务

  /**
   * @brief 切换控制器
   * @param controller_name 目标控制器名称
   * @param[out] message 服务端返回的消息（成功或失败原因），可为 nullptr
   * @param timeout_ms RPC 超时时间（毫秒），默认 3000ms
   * @return true 成功，false 失败
   */
  bool switchController(const std::string& controller_name,
                        std::string* message = nullptr,
                        int timeout_ms = 3000);

  /**
   * @brief 设置手臂控制模式
   * @param mode 控制模式（kKeepPose/kAuto/kExternal）
   * @param timeout_ms RPC 超时时间（毫秒），默认 3000ms
   * @return true 成功，false 失败
   */
  bool setArmMode(ControlMode mode, int timeout_ms = 3000);

  /**
   * @brief 设置头部控制模式
   * @param mode 控制模式（kKeepPose/kAuto/kExternal）
   * @param timeout_ms RPC 超时时间（毫秒），默认 3000ms
   * @return true 成功，false 失败
   */
  bool setHeadMode(ControlMode mode, int timeout_ms = 3000);

  /**
   * @brief 设置腰部控制模式
   * @param mode 控制模式（kKeepPose/kAuto/kExternal）
   * @param timeout_ms RPC 超时时间（毫秒），默认 3000ms
   * @return true 成功，false 失败
   */
  bool setWaistMode(ControlMode mode, int timeout_ms = 3000);

  /**
   * @brief 获取控制器状态
   * @param[out] state 返回的控制器状态
   * @param timeout_ms RPC 超时时间（毫秒），默认 3000ms
   * @return true 成功，false 失败
   */
  bool getControllerState(ControllerState& state, int timeout_ms = 3000);

  // RPC 服务端（控制器端使用）

  /**
   * @brief 注册切换控制器的处理函数
   * @param handler 处理函数，返回是否成功及消息
   */
  void registerSwitchControllerHandler(SwitchControllerHandler handler);

  /**
   * @brief 注册设置手臂模式的处理函数
   * @param handler 处理函数，返回是否成功及消息
   */
  void registerSetArmModeHandler(SetModeHandler handler);

  /**
   * @brief 注册设置头部模式的处理函数
   * @param handler 处理函数，返回是否成功及消息
   */
  void registerSetHeadModeHandler(SetModeHandler handler);

  /**
   * @brief 注册设置腰部模式的处理函数
   * @param handler 处理函数，返回是否成功及消息
   */
  void registerSetWaistModeHandler(SetModeHandler handler);

  /**
   * @brief 注册获取控制器状态的处理函数
   * @param handler 处理函数，返回控制器状态
   */
  void registerGetStateHandler(GetStateHandler handler);

  /**
   * @brief 获取机器人版本
   * @return 机器人版本
   */
  const RobotVersion& getRobotVersion() const { return robot_version_; }

 protected:
  RobotVersion robot_version_;  ///< 机器人版本
};

}  // namespace vr
}  // namespace leju

#endif  // LEJUSDK_VR_VR_BASE_H_
