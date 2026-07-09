/**
 * @file data_types.h
 * @brief VR SDK 数据类型定义
 */

#ifndef LEJUSDK_VR_DATA_TYPES_H_
#define LEJUSDK_VR_DATA_TYPES_H_

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace leju {
namespace vr {

/**
 * @brief 位姿数据
 *
 * 表示单个骨骼节点在机器人坐标系下的位置与四元数姿态。
 * 
 */
struct Pose {
  float x = 0.0f;   ///< 位置 X 分量
  float y = 0.0f;   ///< 位置 Y 分量
  float z = 0.0f;   ///< 位置 Z 分量
  float qx = 0.0f;  ///< 姿态四元数 X 分量
  float qy = 0.0f;  ///< 姿态四元数 Y 分量
  float qz = 0.0f;  ///< 姿态四元数 Z 分量
  float qw = 0.0f;  ///< 姿态四元数 W 分量
};

/**
 * @brief Quest 骨骼位姿数据
 *
 * 包含时间戳、置信度以及整帧骨骼节点位姿数组。
 */
struct QuestBonePosesData {
  int32_t header_sec = 0;              ///< 头部秒时间戳
  uint32_t header_nanosec = 0;         ///< 头部纳秒时间戳
  int64_t timestamp_ms = 0;            ///< Quest 原始毫秒时间戳
  bool is_high_confidence = false;     ///< 本帧是否高置信度
  bool is_hand_tracking = false;       ///< 本帧是否来自手势跟踪模式
  std::vector<Pose> poses;             ///< 骨骼位姿数组
};

/**
 * @brief Quest 摇杆数据
 *
 * 包含左右手摇杆、扳机、握把和按钮状态。
 */
struct QuestJoystickData {
  int32_t header_sec = 0;                   ///< 头部秒时间戳
  uint32_t header_nanosec = 0;              ///< 头部纳秒时间戳
  float left_x = 0.0f;                      ///< 左手摇杆 X 轴
  float left_y = 0.0f;                      ///< 左手摇杆 Y 轴
  float left_trigger = 0.0f;                ///< 左手扳机值
  float left_grip = 0.0f;                   ///< 左手握把值
  bool left_first_button_pressed = false;   ///< 左手主按钮按下状态
  bool left_second_button_pressed = false;  ///< 左手次按钮按下状态
  bool left_first_button_touched = false;   ///< 左手主按钮触摸状态
  bool left_second_button_touched = false;  ///< 左手次按钮触摸状态
  float right_x = 0.0f;                     ///< 右手摇杆 X 轴
  float right_y = 0.0f;                     ///< 右手摇杆 Y 轴
  float right_trigger = 0.0f;               ///< 右手扳机值
  float right_grip = 0.0f;                  ///< 右手握把值
  bool right_first_button_pressed = false;  ///< 右手主按钮按下状态
  bool right_second_button_pressed = false; ///< 右手次按钮按下状态
  bool right_first_button_touched = false;  ///< 右手主按钮触摸状态
  bool right_second_button_touched = false; ///< 右手次按钮触摸状态
};

/**
 * @brief 关节轨迹点
 *
 * 表示单个时刻的关节目标状态，包含位置、速度和加速度。
 */
struct JointTrajectoryPoint {
  std::vector<double> q;    ///< 目标关节位置（rad）
  std::vector<double> v;    ///< 目标关节速度（rad/s）
  std::vector<double> acc;  ///< 目标关节加速度（rad/s²）
};

/**
 * @brief 速度指令
 *
 * 用于底盘速度控制，采用机器人坐标系。
 */
struct VelocityCmd {
  double linear_x = 0.0;   ///< X 方向线速度（m/s），正向前进
  double linear_y = 0.0;   ///< Y 方向线速度（m/s），正向左
  double angular_z = 0.0;  ///< Z 轴角速度（rad/s），正为逆时针
  double timestamp = 0.0;  ///< 时间戳，单位: s
};

/**
 * @brief VR 输入速度指令
 *
 * 用于 vr 的速度指令，采用机器人坐标系。
 */
struct VrVelocityCmd {
  double linear_x = 0.0;   ///< X 方向线速度（m/s），正向前进
  double linear_y = 0.0;   ///< Y 方向线速度（m/s），正向左
  double angular_z = 0.0;  ///< Z 轴角速度（rad/s），正为逆时针
  double timestamp = 0.0;  ///< 时间戳，单位: s
};

/**
 * @brief 部位控制模式
 */
enum class ControlMode : int {
  kKeepPose = 0,  ///< 保持当前姿态
  kAuto = 1,      ///< 自动模式（RL 控制）
  kExternal = 2,  ///< 外部控制模式
};

/**
 * @brief 控制器状态
 */
struct ControllerState {
  std::string current_controller;  ///< 当前控制器名称
  ControlMode arm_mode = ControlMode::kAuto;    ///< 手臂控制模式
  ControlMode head_mode = ControlMode::kAuto;   ///< 头部控制模式
  ControlMode waist_mode = ControlMode::kAuto;  ///< 腰部控制模式
  std::vector<std::string> available_controllers;  ///< 可用控制器列表
};

using ArmTrajectoryCallback = std::function<void(const JointTrajectoryPoint&)>;
using HeadTrajectoryCallback = std::function<void(const JointTrajectoryPoint&)>;
using WaistTrajectoryCallback = std::function<void(const JointTrajectoryPoint&)>;
using VelocityCmdCallback = std::function<void(const VelocityCmd&)>;
using VrVelocityCmdCallback = std::function<void(const VrVelocityCmd&)>;
using QuestBonePosesCallback = std::function<void(const QuestBonePosesData&)>;
using QuestJoystickDataCallback = std::function<void(const QuestJoystickData&)>;
using SwitchControllerHandler =
    std::function<bool(const std::string& name, std::string& message)>;
using SetModeHandler =
    std::function<bool(ControlMode mode, std::string& message)>;
using GetStateHandler = std::function<ControllerState()>;

}  // namespace vr
}  // namespace leju

#endif  // LEJUSDK_VR_DATA_TYPES_H_
