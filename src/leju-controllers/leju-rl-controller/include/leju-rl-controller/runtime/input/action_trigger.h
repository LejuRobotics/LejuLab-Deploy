#pragma once

#include <cstdint>
#include <memory>
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
};

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

  NamedArgs() = default;
  explicit NamedArgs(const std::string& n) : name(n) {}
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

  MotionCommandArgs() = default;
  explicit MotionCommandArgs(Operation o, std::string name = "")
      : op(o), motion_name(std::move(name)) {}

  bool isStart() const { return op == Operation::Start; }
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
ActionTrigger MakeSwitchControllerTrigger(const std::string& controller_name);

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
 * @brief 便捷函数：创建动作控制命令触发器
 * @param op 操作类型
 * @param motion_name 动作名称（仅 Start 时需要）
 * @return ActionTrigger 对象
 */
ActionTrigger MakeMotionCommandTrigger(MotionCommandArgs::Operation op,
                                       const std::string& motion_name = "");

}  // namespace runtime
}  // namespace leju
