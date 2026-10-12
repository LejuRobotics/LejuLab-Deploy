#pragma once

#include <cstdint>
#include <memory>
#include <ostream>
#include <string>

namespace leju {
namespace runtime {

// ============================================================================
// 动作类型枚举
// ============================================================================

/**
 * @brief 动作类型枚举
 *
 * 定义所有支持的 ActionTrigger 类型
 */
enum class ActionType : uint8_t {
  None = 0,           ///< 无动作
  Start,              ///< 启动系统
  Quit,               ///< 退出系统
  SwitchController,   ///< 切换控制器
  SetArmMode,         ///< 设置手臂模式
  SetWaistMode,       ///< 设置腰部模式
  MotionCommand,      ///< 动作控制命令（Start）
  SetInputMask,       ///< 屏蔽/恢复遥杆与方向键输入（args.name: on/off/toggle）
  TransportFallStand, ///< 搬运/倒地起身调度事件（args.name: transport.* / fallstand.*）
  QuitSquat,          ///< 先下蹲再退出（由 ControlLoop 消费，本 tick 执行）
};

inline std::ostream& operator<<(std::ostream& os, ActionType type) {
    switch (type) {
        case ActionType::None:               os << "None"; break;
        case ActionType::Start:              os << "Start"; break;
        case ActionType::Quit:               os << "Quit"; break;
        case ActionType::SwitchController:   os << "SwitchController"; break;
        case ActionType::SetArmMode:         os << "SetArmMode"; break;
        case ActionType::SetWaistMode:       os << "SetWaistMode"; break;
        case ActionType::MotionCommand:      os << "MotionCommand"; break;
        case ActionType::SetInputMask:       os << "SetInputMask"; break;
        case ActionType::TransportFallStand: os << "TransportFallStand"; break;
        case ActionType::QuitSquat:          os << "QuitSquat"; break;
        default:                             os << "Unknown(" << static_cast<int>(type) << ")"; break;
    }
    return os;
}

// ============================================================================
// 动作参数继承体系
// ============================================================================

/**
 * @brief 动作参数基类
 *
 * 所有动作参数的基类，提供虚析构函数
 */
struct ActionArgs {
  virtual ~ActionArgs() = default;
};

/**
 * @brief 命名参数
 *
 * 包含单个 name 字段的参数，用于大多数动作类型
 */
struct NamedArgs : public ActionArgs {
  std::string name;  ///< 名称参数
  bool auto_start_motion = false;  ///< 切换后是否自动起播默认动作（SwitchController 专用）

  NamedArgs() = default;
  explicit NamedArgs(const std::string& n, bool auto_start = false)
      : name(n), auto_start_motion(auto_start) {}
};

/**
 * @brief 动作控制命令参数
 *
 * 统一的动作控制命令，支持 Start
 */
struct MotionCommandArgs : public ActionArgs {
  enum class Operation : uint8_t {
    Start,    ///< 开始新动作（必须指定 motion_name）
  };

  Operation op;            ///< 操作类型（当前仅支持 Start）
  std::string motion_name; ///< 动作名称（可选）
  std::string music;       ///< 配套音乐文件名（可选, 自定义组合键动作用）
  double music_delay = 0.0; ///< 音乐起播延迟[s]（可选, 用于对齐舞蹈起播的就位/blend 时间）
  bool block_velocity_during_motion = false; ///< M1/M2 动作播放期间屏蔽摇杆行走

  MotionCommandArgs() = default;
  explicit MotionCommandArgs(Operation o, std::string name = "", std::string music_name = "",
                             double music_delay_s = 0.0)
      : op(o), motion_name(std::move(name)), music(std::move(music_name)),
        music_delay(music_delay_s) {}

  bool isStart() const { return op == Operation::Start; }
};

/**
 * @brief 先下蹲再退出（QuitSquat）参数
 *
 * 由 Joy 适配器在 BACK+START 时产出，ControlLoop 消费并执行下蹲→退出序列。
 */
struct QuitSquatArgs : public ActionArgs {
  double squat_height = -0.15;  ///< 下蹲高度 [m]，负值=下蹲
  double duration_sec = 1.5;    ///< 下蹲保持时长 [s]，之后退出

  QuitSquatArgs() = default;
  QuitSquatArgs(double height, double duration)
      : squat_height(height), duration_sec(duration) {}
};

// ============================================================================
// 动作触发器
// ============================================================================

/**
 * @brief 新的动作触发器（类型安全版本）
 *
 * 使用 ActionType 枚举替代字符串 name，支持类型安全的参数传递
 */
struct ActionTrigger {
  ActionType type;                     ///< 动作类型
  std::shared_ptr<ActionArgs> args;    ///< 动作参数

  ActionTrigger() : type(ActionType::None) {}

  explicit ActionTrigger(ActionType t) : type(t) {}

  ActionTrigger(ActionType t, std::shared_ptr<ActionArgs> a)
      : type(t), args(std::move(a)) {}

  /**
   * @brief 检查是否为有效触发器
   */
  bool IsValid() const { return type != ActionType::None; }

  /**
   * @brief 重置为无效状态
   */
  void Reset() {
    type = ActionType::None;
    args.reset();
  }
};

// ============================================================================
// 辅助函数声明
// ============================================================================

/**
 * @brief 将字符串转换为 ActionType（仅支持 PascalCase）
 * @param str 输入字符串（如 "SwitchController"）
 * @return 对应的 ActionType，如果失败返回 None
 */
ActionType ParseActionType(const std::string& str);

/**
 * @brief 将 ActionType 转换为字符串（PascalCase）
 * @param type 动作类型
 * @return 对应的字符串表示
 */
std::string ActionTypeToString(ActionType type);


/**
 * @brief 创建 NamedArgs 参数对象的工厂函数
 * @param name 名称参数
 * @return 指向 NamedArgs 的 shared_ptr
 */
std::shared_ptr<ActionArgs> CreateNamedArgs(const std::string& name);

/**
 * @brief 便捷函数：创建切换控制器的触发器
 * @param controller_name 目标控制器名称
 * @return ActionTrigger 对象
 */
ActionTrigger MakeSwitchControllerTrigger(const std::string& controller_name,
                                          bool auto_start_motion = false);

/**
 * @brief 便捷函数：创建设置手臂模式的触发器
 * @param mode_name 模式名称
 * @return ActionTrigger 对象
 */
ActionTrigger MakeSetArmModeTrigger(const std::string& mode_name);

/**
 * @brief 便捷函数：创建设置腰部模式的触发器
 * @param mode_name 模式名称
 * @return ActionTrigger 对象
 */
ActionTrigger MakeSetWaistModeTrigger(const std::string& mode_name);

/**
 * @brief 便捷函数：创建退出触发器
 * @return ActionTrigger 对象
 */
ActionTrigger MakeQuitTrigger();

/**
 * @brief 便捷函数：创建「先下蹲再退出」触发器
 * @param squat_height 下蹲高度 [m]，负值=下蹲
 * @param duration_sec 下蹲保持时长 [s]
 * @return ActionTrigger 对象
 */
ActionTrigger MakeQuitSquatTrigger(double squat_height, double duration_sec);

/**
 * @brief 便捷函数：创建动作控制命令触发器
 * @param op 操作类型
 * @param motion_name 动作名称（仅 Start 时需要）
 * @return ActionTrigger 对象
 */
ActionTrigger MakeMotionCommandTrigger(MotionCommandArgs::Operation op,
                                       const std::string& motion_name = "",
                                       const std::string& music = "",
                                       double music_delay = 0.0);

/**
 * @brief 便捷函数：创建输入屏蔽触发器
 * @param mode 屏蔽模式，"on" / "off" / "toggle"
 * @return ActionTrigger 对象
 */
ActionTrigger MakeSetInputMaskTrigger(const std::string& mode);

/**
 * @brief 便捷函数：创建搬运/倒地起身调度事件触发器
 * @param event_name 事件名（如 transport.enter / fallstand.standup）
 */
ActionTrigger MakeTransportFallStandTrigger(const std::string& event_name);

}  // namespace runtime
}  // namespace leju
