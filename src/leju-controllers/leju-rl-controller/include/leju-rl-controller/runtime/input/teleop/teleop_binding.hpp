#pragma once

#include <algorithm>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "leju-rl-controller/runtime/input/action_trigger.h"

namespace leju {
namespace runtime {

// ============================================================================
// 组合键定义
// ============================================================================

// 前向声明
struct ComboKey;

/**
 * @brief 将组合键转换为查找 key（按钮排序后用 '+' 连接）
 * @param combo 组合键
 * @return 排序后的字符串，如 "LB+X"
 */
inline std::string ComboKeyToString(const ComboKey& combo);

/**
 * @brief 组合键表示
 *
 * 表示一个组合键，包含多个按钮的集合
 */
struct ComboKey {
  std::vector<std::string> buttons;  ///< 按钮名称列表，如 ["LB", "X"]

  /**
   * @brief 检查是否为空组合
   */
  bool empty() const { return buttons.empty(); }

  /**
   * @brief 比较两个组合键是否相等（不考虑顺序）
   */
  inline bool operator==(const ComboKey& other) const;

  /**
   * @brief 比较两个组合键是否不相等
   */
  bool operator!=(const ComboKey& other) const { return !(*this == other); }
};

// ============================================================================
// 内联实现
// ============================================================================

inline std::string ComboKeyToString(const ComboKey& combo) {
  if (combo.buttons.empty()) return "";
  std::vector<std::string> sorted = combo.buttons;
  std::sort(sorted.begin(), sorted.end());
  std::string key;
  for (size_t i = 0; i < sorted.size(); ++i) {
    if (i > 0) key += "+";
    key += sorted[i];
  }
  return key;
}

inline bool ComboKey::operator==(const ComboKey& other) const {
  return ComboKeyToString(*this) == ComboKeyToString(other);
}

// ============================================================================
// 绑定条目
// ============================================================================

/**
 * @brief 触发边沿
 *
 * - kPress:   组合键按下瞬间触发（默认，对应文档「按下响应」）
 * - kRelease: 组合键松开瞬间触发（对应文档「松开响应」，如 LT/RT 预设动作）
 */
enum class TriggerEdge : uint8_t {
  kPress = 0,
  kRelease,
};

/**
 * @brief 单个组合键绑定配置
 *
 * 表示一个组合键到动作触发器的映射，使用类型安全的 ActionTrigger
 */
struct TeleopBinding {
  ComboKey combo;              ///< 组合键
  ActionTrigger action;        ///< 动作触发器（包含类型和参数）
  TriggerEdge edge = TriggerEdge::kPress;  ///< 触发边沿（按下/松开）
  bool block_when_walking = false;         ///< 走路（遥杆/方向键产生运动）时是否禁止触发
  double hold_duration = 0.0;  ///< 长按触发时长 [s]；>0 时忽略 edge，组合键持续按住满时长触发一次
};

// ============================================================================
// 设备绑定集合
// ============================================================================

/**
 * @brief 设备类型
 */
enum class TeleopDeviceType : uint8_t {
  kJoy = 0,            ///< 手柄
  kQuest,              ///< Quest VR
};

/**
 * @brief 设备绑定配置
 *
 * 使用哈希表存储绑定，查找复杂度 O(1)
 */
struct DeviceBindingConfig {
  TeleopDeviceType device_type = TeleopDeviceType::kJoy;  ///< 设备类型（默认初始化避免 UB）

  /**
   * @brief 添加绑定
   * @param binding 绑定配置
   */
  inline void addBinding(const TeleopBinding& binding) {
    auto key = ComboKeyToString(binding.combo);
    bindings_[key] = binding;
  }

  /**
   * @brief 查找匹配的组合键绑定
   * @param current_combo 当前按下的组合键
   * @return 找到的绑定，未找到返回 nullptr
   */
  inline const TeleopBinding* findBinding(const ComboKey& current_combo) const {
    if (current_combo.empty()) {
      return nullptr;
    }
    auto key = ComboKeyToString(current_combo);
    auto it = bindings_.find(key);
    return it != bindings_.end() ? &it->second : nullptr;
  }

  /**
   * @brief 清空所有绑定
   */
  inline void clear() { bindings_.clear(); }

  /**
   * @brief 获取绑定数量
   * @return 绑定数量
   */
  inline size_t size() const { return bindings_.size(); }

 private:
  std::unordered_map<std::string, TeleopBinding> bindings_;  ///< 绑定查找表
};

} // namespace runtime
} // namespace leju
