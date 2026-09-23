/**
 * @file test_teleop_binding.cpp
 * @brief TeleopBinding 模块单元测试
 */

#include <gtest/gtest.h>

#include "leju-rl-controller/runtime/input/teleop/teleop_binding.hpp"

using namespace leju::runtime;

// ============================================================================
// ComboKey 测试
// ============================================================================

TEST(ComboKeyTest, DefaultConstructor) {
  ComboKey key;
  EXPECT_TRUE(key.empty());
  EXPECT_EQ(key.buttons.size(), 0);
}

TEST(ComboKeyTest, EmptyReturnsTrueForEmptyButtons) {
  ComboKey key;
  EXPECT_TRUE(key.empty());

  key.buttons.push_back("A");
  EXPECT_FALSE(key.empty());
}

TEST(ComboKeyTest, EqualitySameOrder) {
  ComboKey key1;
  key1.buttons = {"RB", "X"};

  ComboKey key2;
  key2.buttons = {"RB", "X"};

  EXPECT_EQ(key1, key2);
}

TEST(ComboKeyTest, EqualityOrderIndependent) {
  ComboKey key1;
  key1.buttons = {"RB", "X"};

  ComboKey key2;
  key2.buttons = {"X", "RB"};

  EXPECT_EQ(key1, key2);
}

TEST(ComboKeyTest, InequalityDifferentSize) {
  ComboKey key1;
  key1.buttons = {"RB"};

  ComboKey key2;
  key2.buttons = {"RB", "X"};

  EXPECT_NE(key1, key2);
}

TEST(ComboKeyTest, InequalityDifferentButtons) {
  ComboKey key1;
  key1.buttons = {"RB", "X"};

  ComboKey key2;
  key2.buttons = {"RB", "Y"};

  EXPECT_NE(key1, key2);
}

// ============================================================================
// ComboKeyToString 测试
// ============================================================================

TEST(ComboKeyToStringTest, Empty) {
  ComboKey key;
  EXPECT_EQ(ComboKeyToString(key), "");
}

TEST(ComboKeyToStringTest, SingleButton) {
  ComboKey key;
  key.buttons = {"A"};
  EXPECT_EQ(ComboKeyToString(key), "A");
}

TEST(ComboKeyToStringTest, MultipleButtonsSorted) {
  ComboKey key;
  key.buttons = {"RB", "X"};  // RB < X
  EXPECT_EQ(ComboKeyToString(key), "RB+X");
}

TEST(ComboKeyToStringTest, MultipleButtonsUnsorted) {
  ComboKey key;
  key.buttons = {"X", "RB"};  // X > RB，但结果应排序
  EXPECT_EQ(ComboKeyToString(key), "RB+X");
}

TEST(ComboKeyToStringTest, ThreeButtons) {
  ComboKey key;
  key.buttons = {"X", "LB", "RB"};  // 乱序输入
  EXPECT_EQ(ComboKeyToString(key), "LB+RB+X");  // 排序后
}

// ============================================================================
// TeleopBinding 测试
// ============================================================================

TEST(TeleopBindingTest, DefaultConstructor) {
  TeleopBinding binding;
  EXPECT_TRUE(binding.combo.empty());
  EXPECT_FALSE(binding.action.IsValid());
}

TEST(TeleopBindingTest, DirectConstruction) {
  ComboKey combo;
  combo.buttons = {"RB", "X"};

  TeleopBinding binding;
  binding.combo = combo;
  binding.action = MakeSetArmModeTrigger("keep_pose");

  EXPECT_EQ(binding.combo, combo);
  EXPECT_EQ(binding.action.type, ActionType::SetArmMode);
  EXPECT_TRUE(binding.action.IsValid());
}

// ============================================================================
// DeviceBindingConfig 测试
// ============================================================================

TEST(DeviceBindingConfigTest, DefaultConstructor) {
  DeviceBindingConfig config;
  EXPECT_EQ(config.device_type, TeleopDeviceType::kJoy);
  EXPECT_EQ(config.size(), 0);
}

TEST(DeviceBindingConfigTest, AddBinding) {
  DeviceBindingConfig config;

  TeleopBinding binding;
  binding.combo.buttons = {"RB", "X"};
  binding.action = MakeSetArmModeTrigger("keep_pose");

  config.addBinding(binding);

  EXPECT_EQ(config.size(), 1);
}

TEST(DeviceBindingConfigTest, AddMultipleBindings) {
  DeviceBindingConfig config;

  TeleopBinding binding1;
  binding1.combo.buttons = {"RB", "X"};
  binding1.action = MakeSetArmModeTrigger("keep_pose");
  config.addBinding(binding1);

  TeleopBinding binding2;
  binding2.combo.buttons = {"LB", "A"};
  binding2.action = MakeSetArmModeTrigger("auto");
  config.addBinding(binding2);

  TeleopBinding binding3;
  binding3.combo.buttons = {"BACK"};
  binding3.action = MakeQuitTrigger();
  config.addBinding(binding3);

  EXPECT_EQ(config.size(), 3);
}

TEST(DeviceBindingConfigTest, ClearBindings) {
  DeviceBindingConfig config;

  TeleopBinding binding;
  binding.combo.buttons = {"RB", "X"};
  binding.action = MakeSetArmModeTrigger("keep_pose");
  config.addBinding(binding);

  EXPECT_EQ(config.size(), 1);

  config.clear();
  EXPECT_EQ(config.size(), 0);
}

TEST(DeviceBindingConfigTest, FindBindingExisting) {
  DeviceBindingConfig config;

  TeleopBinding binding;
  binding.combo.buttons = {"RB", "X"};
  binding.action = MakeSetArmModeTrigger("keep_pose");
  config.addBinding(binding);

  ComboKey search_key;
  search_key.buttons = {"X", "RB"};  // 不同顺序

  const auto* found = config.findBinding(search_key);
  ASSERT_NE(found, nullptr);
  EXPECT_EQ(found->action.type, ActionType::SetArmMode);
}

TEST(DeviceBindingConfigTest, FindBindingNotFound) {
  DeviceBindingConfig config;

  TeleopBinding binding;
  binding.combo.buttons = {"RB", "X"};
  binding.action = MakeSetArmModeTrigger("keep_pose");
  config.addBinding(binding);

  ComboKey search_key;
  search_key.buttons = {"LB", "A"};

  const auto* found = config.findBinding(search_key);
  EXPECT_EQ(found, nullptr);
}

TEST(DeviceBindingConfigTest, FindBindingEmptyCombo) {
  DeviceBindingConfig config;

  TeleopBinding binding;
  binding.combo.buttons = {"RB", "X"};
  binding.action = MakeSetArmModeTrigger("keep_pose");
  config.addBinding(binding);

  ComboKey empty_key;
  const auto* found = config.findBinding(empty_key);
  EXPECT_EQ(found, nullptr);
}

TEST(DeviceBindingConfigTest, FindBindingEmptyConfig) {
  DeviceBindingConfig config;

  ComboKey search_key;
  search_key.buttons = {"RB", "X"};

  const auto* found = config.findBinding(search_key);
  EXPECT_EQ(found, nullptr);
}

TEST(DeviceBindingConfigTest, FindBindingSingleButton) {
  DeviceBindingConfig config;

  TeleopBinding binding;
  binding.combo.buttons = {"START"};
  binding.action = ActionTrigger(ActionType::Start);
  config.addBinding(binding);

  ComboKey search_key;
  search_key.buttons = {"START"};

  const auto* found = config.findBinding(search_key);
  ASSERT_NE(found, nullptr);
  EXPECT_EQ(found->action.type, ActionType::Start);
}

TEST(DeviceBindingConfigTest, FindBindingOrderIndependent) {
  // 测试哈希表查找的顺序无关性
  DeviceBindingConfig config;

  // 添加绑定: LB + X
  TeleopBinding binding;
  binding.combo.buttons = {"LB", "X"};
  binding.action = MakeSetArmModeTrigger("keep_pose");
  config.addBinding(binding);

  // 多种顺序查找都应该找到
  ComboKey key1;
  key1.buttons = {"LB", "X"};
  EXPECT_NE(config.findBinding(key1), nullptr);

  ComboKey key2;
  key2.buttons = {"X", "LB"};
  EXPECT_NE(config.findBinding(key2), nullptr);
}

// ============================================================================
// TeleopDeviceType 测试
// ============================================================================

TEST(TeleopDeviceTypeTest, EnumValues) {
  EXPECT_EQ(static_cast<uint8_t>(TeleopDeviceType::kJoy), 0);
  EXPECT_EQ(static_cast<uint8_t>(TeleopDeviceType::kQuest), 1);
}

// ============================================================================
// 复杂场景测试
// ============================================================================

TEST(TeleopBindingIntegrationTest, FullBindingScenario) {
  // 创建一个完整的绑定配置
  DeviceBindingConfig config;
  config.device_type = TeleopDeviceType::kJoy;

  // 添加多个绑定
  TeleopBinding binding1;
  binding1.combo.buttons = {"RB", "A"};
  binding1.action = MakeSwitchControllerTrigger("amp");
  config.addBinding(binding1);

  TeleopBinding binding2;
  binding2.combo.buttons = {"RB", "B"};
  binding2.action = MakeSwitchControllerTrigger("mimic");
  config.addBinding(binding2);

  TeleopBinding binding3;
  binding3.combo.buttons = {"LB", "X"};
  binding3.action = MakeSetArmModeTrigger("keep_pose");
  config.addBinding(binding3);

  TeleopBinding binding4;
  binding4.combo.buttons = {"LB", "Y"};
  binding4.action = MakeSetArmModeTrigger("auto");
  config.addBinding(binding4);

  TeleopBinding binding5;
  binding5.combo.buttons = {"START"};
  binding5.action = ActionTrigger(ActionType::Start);
  config.addBinding(binding5);

  TeleopBinding binding6;
  binding6.combo.buttons = {"BACK"};
  binding6.action = MakeQuitTrigger();
  config.addBinding(binding6);

  EXPECT_EQ(config.size(), 6);

  // 验证每个绑定都能找到（顺序不同）
  ComboKey key1;
  key1.buttons = {"A", "RB"};  // 顺序不同
  EXPECT_NE(config.findBinding(key1), nullptr);

  ComboKey key2;
  key2.buttons = {"LB", "Y"};
  const auto* found2 = config.findBinding(key2);
  ASSERT_NE(found2, nullptr);
  EXPECT_EQ(found2->action.type, ActionType::SetArmMode);

  // 未定义的 combo
  ComboKey key3;
  key3.buttons = {"A", "B"};
  EXPECT_EQ(config.findBinding(key3), nullptr);
}

// ============================================================================
// 主函数
// ============================================================================

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
