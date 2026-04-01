/**
 * @file data_types.h
 * @brief 机器人底层SDK数据类型定义
 * @author Leju Robotics
 *
 * @details 本文件定义了机器人底层控制所需的核心数据结构，包括：
 * - 电机控制模式枚举
 * - 硬件状态枚举
 * - 机器人状态数据结构
 * - 机器人控制指令数据结构
 * - IMU传感器数据结构
 * - 手柄输入数据结构
 *
 */

#ifndef _LEJU_SDK_LOW_LEVEL_DATAT_YPES_H_
#define _LEJU_SDK_LOW_LEVEL_DATAT_YPES_H_

#include <string>
#include <vector>
#include <cstdint>
#include <array>
#include <memory>
#include <iosfwd>

namespace leju {

/**
 * @enum MotorControlMode
 * @brief 电机控制模式枚举
 *
 * 定义了机器人电机支持的三种控制模式，用于指定电机的运行方式。
 *
 * @warning CST 模式下，kp 和 kd 必须设置为 0，否则会叠加 PD 控制律的输出。
 */
enum class MotorControlMode : uint8_t {
  CST = 0,  ///< 力矩控制模式，直接发送目标力矩，kp/kd 必须为 0
  CSV = 1,  ///< 速度控制模式，发送目标速度
  CSP = 2,  ///< 位置控制模式，发送目标位置、Kp、Kd，内部计算力矩
};

/// @brief 硬件状态枚举
enum class HardwareState {
    UNKNOWN = 0,    ///< 未知状态
    STOPPED,        ///< 已停止
    ERROR,          ///< 错误状态
    INITIALIZING,   ///< 初始化中
    READY_OK,       ///< 就绪，可发送控制指令
};

/// @brief 字符串转硬件状态
/// @param hw_state_str "unknown", "initializing", "ready_ok", "error"
HardwareState String2HwState(const std::string& hw_state_str);

/// @brief 硬件状态转字符串
const char* HardwareState2String(HardwareState hw_state);

/**
 * @struct RobotState
 * @brief 机器人状态结构体
 *
 * 用于封装机器人关节的状态信息，包括位置、速度、加速度和力矩等反馈数据。
 *
 * @note 所有向量的维度应与机器人关节数量一致，可通过 resize() 方法调整。
 *
 * @see RobotCmd 机器人控制指令结构体
 */
struct RobotState {
  std::vector<double> q;        ///< 关节位置数组，单位: rad
  std::vector<double> v;        ///< 关节速度数组，单位: rad/s
  std::vector<double> vd;       ///< 关节加速度数组，单位: rad/s²
  std::vector<double> tau;      ///< 关节力矩数组，单位: N·m
  double timestamp;             ///< 时间戳，单位: s

  /// @brief 默认构造函数
  RobotState() : timestamp(0.0) {}

  /// @brief 构造函数，指定关节数量
  /// @param num 关节数量
  explicit RobotState(size_t num) {
    resize(num);
  }

  /// @brief 调整所有向量的维度
  /// @param num 目标关节数量
  void resize(size_t num) {
    q.resize(num, 0.0);
    v.resize(num, 0.0);
    vd.resize(num, 0.0);
    tau.resize(num, 0.0);
  }

  /// @brief 检查数据维度是否一致
  /// @return 所有向量维度一致返回 true
  bool isValid() const {
    size_t size = q.size();
    return (v.size() == size && vd.size() == size &&
            tau.size() == size);
  }
};

using RobotStatePtr = std::shared_ptr<leju::RobotState>;
using RobotStateConstPtr = std::shared_ptr<const leju::RobotState>;

/**
 * @struct RobotCmd
 * @brief 机器人关节控制指令结构体
 *
 * 用于封装机器人关节的控制参数，包括目标位置、速度、力矩以及控制增益等。
 * 支持三种控制模式：@ref MotorControlMode::CST "CST"(力矩控制)、
 * @ref MotorControlMode::CSV "CSV"(速度控制)、@ref MotorControlMode::CSP "CSP"(位置控制)。
 *
 * @note 各向量维度必须与机器人关节数量一致。
 *
 * @see RobotState 机器人状态结构体
 * @see MotorControlMode 电机控制模式枚举
 */
struct RobotCmd {
  std::vector<double> q;        ///< 目标位置数组，单位: rad
  std::vector<double> v;        ///< 目标速度数组，单位: rad/s
  std::vector<double> tau;      ///< 前馈力矩数组，单位: N·m
  std::vector<double> kp;       ///< 位置增益 (Kp)，单位: N·m/rad
  std::vector<double> kd;       ///< 速度增益 (Kd)，单位: N·m·s/rad
  std::vector<uint8_t> modes;   ///< 控制模式数组，参见 MotorControlMode
  double timestamp;             ///< 时间戳，单位: s

  /// @brief 默认构造函数
  RobotCmd() : timestamp(0.0) {}

  /// @brief 构造函数，指定数量
  /// @param num 数量
  explicit RobotCmd(size_t num) : timestamp(0.0) {
    resize(num);
  }

  /// @brief 调整所有向量的维度
  /// @param num 目标关节数量
  void resize(size_t num) {
    q.resize(num, 0.0);
    v.resize(num, 0.0);
    tau.resize(num, 0.0);
    kp.resize(num, 0.0);
    kd.resize(num, 0.0);
    modes.resize(num, 0);
  }

  /// @brief 检查数据维度是否一致
  /// @return 所有向量维度一致返回 true
  bool isValid() const {
    size_t size = q.size();
    return (v.size() == size &&
            tau.size() == size &&
            kp.size() == size &&
            kd.size() == size &&
            modes.size() == size);
  }
};

using RobotCmdPtr = std::shared_ptr<leju::RobotCmd>;
using RobotCmdConstPtr = std::shared_ptr<const leju::RobotCmd>;

/**
 * @struct ImuData
 * @brief IMU传感器数据结构体
 *
 * 用于封装惯性测量单元(IMU)的状态信息，包括角速度、加速度和姿态四元数。
 *
 */
struct ImuData {
  std::array<double, 3> gyro;     ///< 角速度 [x, y, z]，单位: rad/s
  std::array<double, 3> acc;      ///< 加速度 [x, y, z]，含重力，单位: m/s²
  std::array<double, 3> free_acc; ///< 无重力加速度 [x, y, z]，单位: m/s²
  std::array<double, 4> quat;     ///< 姿态四元数 [w, x, y, z]
  double timestamp;               ///< 时间戳，单位: s

  /// @brief 默认构造函数，四元数初始化为单位四元数 [1, 0, 0, 0]
  ImuData() : timestamp(0.0) {
    gyro.fill(0.0);
    acc.fill(0.0);
    free_acc.fill(0.0);
    quat.fill(0.0);
    quat[0] = 1.0;  // w = 1, 单位四元数
  }
};

using ImuDataPtr = std::shared_ptr<leju::ImuData>;
using ImuDataConstPtr = std::shared_ptr<const leju::ImuData>;

/**
 * @struct JoyData
 * @brief 手柄输入数据结构体
 *
 * 用于封装游戏手柄的输入数据，包括摇杆、扳机和按钮状态。
 * 数据格式与 DDS/SDL 保持一致，支持 Xbox 和 PlayStation 风格手柄。
 *
 */
struct JoyData {
  /**
   * @struct Axes
   * @brief 摇杆和扳机轴数据结构体
   *
   * 包含左右摇杆的 X/Y 轴数值以及左右扳机的数值。
   * 摇杆数值范围为 [-1.0, 1.0]，扳机数值范围为 [0.0, 1.0]。
   */
  struct Axes {
    float left_x = 0.0f;         ///< 左摇杆 X 轴 [-1.0, 1.0]
    float left_y = 0.0f;         ///< 左摇杆 Y 轴 [-1.0, 1.0]
    float right_x = 0.0f;        ///< 右摇杆 X 轴 [-1.0, 1.0]
    float right_y = 0.0f;        ///< 右摇杆 Y 轴 [-1.0, 1.0]
    float left_trigger = 0.0f;   ///< 左扳机 (LT/L2) [0.0, 1.0]
    float right_trigger = 0.0f;  ///< 右扳机 (RT/R2) [0.0, 1.0]
  };

  /**
   * @struct Buttons
   * @brief 按钮状态结构体
   *
   * 包含手柄上所有按钮的状态。
   * 按钮状态值：0 表示未按下，非0 表示按下。
   */
  struct Buttons {
    int32_t south = 0;          ///< 底部面键 (Xbox A / PS Cross)
    int32_t east = 0;           ///< 右侧面键 (Xbox B / PS Circle)
    int32_t west = 0;           ///< 左侧面键 (Xbox X / PS Square)
    int32_t north = 0;          ///< 顶部面键 (Xbox Y / PS Triangle)
    int32_t back = 0;           ///< 返回键 (Xbox Back / PS Select)
    int32_t guide = 0;          ///< 指南键 (Xbox 按钮 / PS 按钮)
    int32_t start = 0;          ///< 开始键 (Xbox Start / PS Start)
    int32_t left_stick = 0;     ///< 左摇杆按下 (L3)
    int32_t right_stick = 0;    ///< 右摇杆按下 (R3)
    int32_t left_shoulder = 0;  ///< 左肩键 (LB / L1)
    int32_t right_shoulder = 0; ///< 右肩键 (RB / R1)
    int32_t dpad_up = 0;        ///< 方向键上
    int32_t dpad_down = 0;      ///< 方向键下
    int32_t dpad_left = 0;      ///< 方向键左
    int32_t dpad_right = 0;     ///< 方向键右
    int32_t misc1 = 0;          ///< 额外按键1
  };

  double timestamp = 0.0;       ///< 时间戳，单位: s
  Axes axes;                    ///< 摇杆和扳机数据
  Buttons buttons;              ///< 按钮状态数据

  /// @brief 默认构造函数
  JoyData() = default;
};

using JoyDataPtr = std::shared_ptr<leju::JoyData>;
using JoyDataConstPtr = std::shared_ptr<const leju::JoyData>;

/**
 * @struct StringData
 * @brief 字符串数据结构体
 *
 * 用于传输通用字符串消息，如可将 json 数据转换为字符串，用于传输等。
 */
struct StringData {
  double timestamp;             ///< 时间戳，单位: s
  std::string data;             ///< 字符串数据内容

  /// @brief 默认构造函数
  StringData() : timestamp(0.0) {}
};

using StringDataPtr = std::shared_ptr<leju::StringData>;
using StringDataConstPtr = std::shared_ptr<const leju::StringData>;


/** @cond INTERNAL */
std::ostream& operator<<(std::ostream& os, const MotorControlMode& mode);
std::ostream& operator<<(std::ostream& os, const RobotState& state);
std::ostream& operator<<(std::ostream& os, const RobotCmd& cmd);
std::ostream& operator<<(std::ostream& os, const ImuData& imu);
std::ostream& operator<<(std::ostream& os, const JoyData& joy);
/** @endcond */

} // namespace leju

#endif // _LEJU_SDK_LOW_LEVEL_DATAT_YPES_H_