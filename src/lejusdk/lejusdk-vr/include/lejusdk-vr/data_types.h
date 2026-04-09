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
 * @brief 底层硬件状态
 *
 * 对齐 runtime 中 RobotData 持有的硬件状态，用于把原始硬件阶段
 * 暴露给群控和外部控制接口，不包含业务组合语义。
 */
enum class HardwareState : int {
  kUnknown = 0,       ///< 未知状态
  kInitializing = 1, ///< 硬件初始化中
  kReadyOk = 2,      ///< 硬件 ready，可进入下一阶段
  kError = 3,        ///< 硬件异常
  kStopped = 4,      ///< 硬件已停止
};

/**
 * @brief Runtime 生命周期状态
 *
 * 对齐 runtime::LifecycleState，用于暴露控制系统当前所处阶段。
 */
enum class LifecycleState : int {
  kWaitingForReady = 0,  ///< 等待硬件和数据 ready
  kWaitingForStart = 1,  ///< 已 ready，等待 start
  kRunning = 2,          ///< 控制主循环运行中
  kExiting = 3,          ///< 退出流程中
};

/**
 * @brief Runtime 原始状态
 *
 * 仅描述 runtime 当前原始阶段，不包含“已站稳”“可跳舞”等组合业务含义。
 */
struct RuntimeState {
  bool data_ready = false;  ///< 数据流是否 ready
  HardwareState hardware_state = HardwareState::kUnknown;  ///< 原始硬件状态
  LifecycleState lifecycle_state = LifecycleState::kWaitingForReady;  ///< 生命周期状态
};

/**
 * @brief 控制器状态
 */
struct ControllerState {
  std::string current_controller;  ///< 当前控制器名称
  ControlMode arm_mode = ControlMode::kAuto;    ///< 手臂控制模式
  ControlMode head_mode = ControlMode::kAuto;   ///< 头部控制模式
  ControlMode waist_mode = ControlMode::kAuto;  ///< 腰部控制模式
  bool controller_transitioning = false;        ///< 控制器是否处于切换过渡中
  std::vector<std::string> available_controllers;  ///< 可用控制器列表
};

/**
 * @brief Motion 原始状态
 *
 * 当当前控制器支持 motion 播放时，返回动作播放状态和可用动作列表。
 */
struct MotionState {
  bool supported = false;  ///< 当前控制器是否支持 motion 查询
  bool motion_playing = false;  ///< 当前是否正在播放 motion
  std::string current_motion_name;  ///< 当前 motion 名称
  std::vector<std::string> available_motion_names;  ///< 可播放 motion 列表
};

using ArmTrajectoryCallback = std::function<void(const JointTrajectoryPoint&)>;
using HeadTrajectoryCallback = std::function<void(const JointTrajectoryPoint&)>;
using WaistTrajectoryCallback = std::function<void(const JointTrajectoryPoint&)>;
using VelocityCmdCallback = std::function<void(const VelocityCmd&)>;
using VrVelocityCmdCallback = std::function<void(const VrVelocityCmd&)>;
using QuestBonePosesCallback = std::function<void(const QuestBonePosesData&)>;
using QuestJoystickDataCallback = std::function<void(const QuestJoystickData&)>;

/**
 * @brief 切换控制器 RPC 处理器
 * @param name 目标控制器名称
 * @param message 返回消息
 * @return 是否接受该请求
 */
using SwitchControllerHandler =
    std::function<bool(const std::string& name, std::string& message)>;

/**
 * @brief 控制模式设置 RPC 处理器
 * @param mode 目标模式
 * @param message 返回消息
 * @return 是否接受该请求
 */
using SetModeHandler =
    std::function<bool(ControlMode mode, std::string& message)>;

/**
 * @brief 无输入参数的 RPC 处理器
 *
 * 适用于 start/stop 这类 trigger 型服务。
 */
using TriggerHandler = std::function<bool(std::string& message)>;

/**
 * @brief 带字符串输入参数的 RPC 处理器
 *
 * 适用于 motion 名称之类的简单字符串参数服务。
 */
using SetStringHandler =
    std::function<bool(const std::string& data, std::string& message)>;

/**
 * @brief Runtime 状态查询 RPC 处理器
 * @return runtime 原始状态
 */
using GetRuntimeStateHandler = std::function<RuntimeState()>;

/**
 * @brief Controller 状态查询 RPC 处理器
 * @return controller 原始状态
 */
using GetControllerStateHandler = std::function<ControllerState()>;

/**
 * @brief Motion 状态查询 RPC 处理器
 * @return motion 原始状态
 */
using GetMotionStateHandler = std::function<MotionState()>;

}  // namespace vr
}  // namespace leju

#endif  // LEJUSDK_VR_DATA_TYPES_H_
