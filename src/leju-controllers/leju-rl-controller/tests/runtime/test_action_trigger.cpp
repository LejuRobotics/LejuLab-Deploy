/**
 * @file test_action_trigger.cpp
 * @brief ActionTrigger 模块单元测试
 */

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>

#include "leju-rl-controller/runtime/input/action_trigger.h"
#include "leju-rl-controller/runtime/input/teleop/teleop_binding_config.h"

using namespace leju::runtime;

// 辅助函数：创建临时 YAML 文件
static std::string CreateTempYamlFile(const std::string& content) {
  std::string path = "/tmp/test_action_trigger_" + std::to_string(getpid()) + ".yaml";
  std::ofstream f(path);
  f << content;
  f.close();
  return path;
}

// ============================================================================
// ActionType 枚举测试
// ============================================================================

TEST(ActionTypeTest, EnumValuesAreUnique) {
  // 确保所有枚举值都是唯一的
  EXPECT_NE(ActionType::None, ActionType::Start);
  EXPECT_NE(ActionType::None, ActionType::Quit);
  EXPECT_NE(ActionType::None, ActionType::SwitchController);
  EXPECT_NE(ActionType::None, ActionType::SetArmMode);
  EXPECT_NE(ActionType::None, ActionType::SetWaistMode);
  EXPECT_NE(ActionType::None, ActionType::StartMotion);
}

TEST(ActionTypeTest, EnumToString) {
  EXPECT_EQ(ActionTypeToString(ActionType::None), "None");
  EXPECT_EQ(ActionTypeToString(ActionType::Start), "Start");
  EXPECT_EQ(ActionTypeToString(ActionType::Quit), "Quit");
  EXPECT_EQ(ActionTypeToString(ActionType::SwitchController), "SwitchController");
  EXPECT_EQ(ActionTypeToString(ActionType::SetArmMode), "SetArmMode");
  EXPECT_EQ(ActionTypeToString(ActionType::SetWaistMode), "SetWaistMode");
  EXPECT_EQ(ActionTypeToString(ActionType::StartMotion), "StartMotion");
}

// ============================================================================
// ParseActionType 测试 - PascalCase
// ============================================================================

TEST(ParseActionTypeTest, ValidTypesPascalCase) {
  EXPECT_EQ(ParseActionType("Start"), ActionType::Start);
  EXPECT_EQ(ParseActionType("Quit"), ActionType::Quit);
  EXPECT_EQ(ParseActionType("SwitchController"), ActionType::SwitchController);
  EXPECT_EQ(ParseActionType("SetArmMode"), ActionType::SetArmMode);
  EXPECT_EQ(ParseActionType("SetWaistMode"), ActionType::SetWaistMode);
  EXPECT_EQ(ParseActionType("StartMotion"), ActionType::StartMotion);
}

TEST(ParseActionTypeTest, InvalidType) {
  EXPECT_EQ(ParseActionType("Invalid"), ActionType::None);
  EXPECT_EQ(ParseActionType(""), ActionType::None);
  EXPECT_EQ(ParseActionType("Unknown"), ActionType::None);
}

TEST(ParseActionTypeTest, CaseSensitive) {
  // 大小写敏感测试
  EXPECT_EQ(ParseActionType("start"), ActionType::None);  // 小写不应匹配
  EXPECT_EQ(ParseActionType("START"), ActionType::None);  // 全大写不应匹配
  EXPECT_EQ(ParseActionType("Switchcontroller"), ActionType::None);  // 大小写混合不应匹配
}

// ============================================================================
// RoundTrip 测试
// ============================================================================

TEST(RoundTripTest, ActionTypeToStringAndBack) {
  // Start -> "Start" -> ActionType::Start
  ActionTrigger trigger(ActionType::Start);
  EXPECT_TRUE(trigger.IsValid());

  // 转回字符串 (PascalCase 格式)
  std::string output = ActionTypeToString(trigger.type);
  EXPECT_EQ(output, "Start");
}

TEST(RoundTripTest, SwitchControllerWithArgs) {
  // 带参数的 SwitchController
  auto args = CreateNamedArgs("amp_controller");
  ActionTrigger trigger(ActionType::SwitchController, args);

  EXPECT_TRUE(trigger.IsValid());
  EXPECT_EQ(trigger.type, ActionType::SwitchController);

  // 验证参数
  auto* named_args = dynamic_cast<NamedArgs*>(trigger.args.get());
  ASSERT_NE(named_args, nullptr);
  EXPECT_EQ(named_args->name, "amp_controller");
}

// ============================================================================
// TeleopBindingConfig 测试（使用 loadFromFile）
// ============================================================================

TEST(TeleopBindingConfigTest, LoadFromFile_ValidYaml) {
  std::string yaml = R"(
joy_bindings:
  - buttons: ["RB", "X"]
    action:
      type: SetArmMode
      args:
        name: keep_pose
)";
  std::string path = CreateTempYamlFile(yaml);
  TeleopBindingConfig config;
  EXPECT_TRUE(config.loadFromFile(path));
  EXPECT_TRUE(config.isLoaded());

  const auto& joy_config = config.getJoyConfig();
  EXPECT_EQ(joy_config.size(), 1);

  // 验证可以通过查找找到绑定
  ComboKey key;
  key.buttons = {"RB", "X"};
  auto* binding = joy_config.findBinding(key);
  ASSERT_NE(binding, nullptr);
  EXPECT_EQ(binding->action.type, ActionType::SetArmMode);

  std::remove(path.c_str());
}

TEST(TeleopBindingConfigTest, LoadFromFile_InvalidYaml) {
  std::string yaml = R"(
joy_bindings:
  - buttons: ["RB", "X"
    action:
      type: SetArmMode
)";
  std::string path = CreateTempYamlFile(yaml);
  TeleopBindingConfig config;
  EXPECT_FALSE(config.loadFromFile(path));
  EXPECT_FALSE(config.isLoaded());

  std::remove(path.c_str());
}

TEST(TeleopBindingConfigTest, LoadFromFile_MissingArgs) {
  // 测试缺少 args 的 action
  std::string yaml = R"(
joy_bindings:
  - buttons: ["START"]
    action:
      type: Quit
)";
  std::string path = CreateTempYamlFile(yaml);
  TeleopBindingConfig config;
  EXPECT_TRUE(config.loadFromFile(path));

  const auto& joy_config = config.getJoyConfig();
  ASSERT_EQ(joy_config.size(), 1);

  ComboKey key;
  key.buttons = {"START"};
  auto* binding = joy_config.findBinding(key);
  ASSERT_NE(binding, nullptr);
  EXPECT_EQ(binding->action.type, ActionType::Quit);

  std::remove(path.c_str());
}

TEST(TeleopBindingConfigTest, LoadFromFile_DualConfig) {
  // 测试 Joy 和 Quest 双配置
  std::string yaml = R"(
joy_bindings:
  - buttons: ["RB", "X"]
    action:
      type: SetArmMode
      args:
        name: keep_pose
  - buttons: ["LB", "A"]
    action:
      type: SetArmMode
      args:
        name: auto

quest_bindings:
  - buttons: ["RIGHT_TRIGGER", "RIGHT_FIRST"]
    action:
      type: SetArmMode
      args:
        name: keep_pose
)";
  std::string path = CreateTempYamlFile(yaml);
  TeleopBindingConfig config;
  EXPECT_TRUE(config.loadFromFile(path));

  const auto& joy_config = config.getJoyConfig();
  const auto& quest_config = config.getQuestConfig();

  EXPECT_EQ(joy_config.size(), 2);
  EXPECT_EQ(quest_config.size(), 1);

  std::remove(path.c_str());
}

TEST(TeleopBindingConfigTest, LoadFromFile_AllActionTypes) {
  // 测试所有支持的 action 类型
  std::string yaml = R"(
joy_bindings:
  - buttons: ["START"]
    action:
      type: Start
  - buttons: ["BACK"]
    action:
      type: Quit
  - buttons: ["RB", "A"]
    action:
      type: SwitchController
      args:
        name: amp
  - buttons: ["LB", "X"]
    action:
      type: SetArmMode
      args:
        name: keep_pose
  - buttons: ["LB", "Y"]
    action:
      type: SetWaistMode
      args:
        name: external
  - buttons: ["RB", "B"]
    action:
      type: StartMotion
      args:
        name: dance_a
)";
  std::string path = CreateTempYamlFile(yaml);
  TeleopBindingConfig config;
  EXPECT_TRUE(config.loadFromFile(path));

  const auto& joy_config = config.getJoyConfig();
  EXPECT_EQ(joy_config.size(), 6);

  // 验证每个 action 类型
  ComboKey key1;
  key1.buttons = {"START"};
  EXPECT_EQ(joy_config.findBinding(key1)->action.type, ActionType::Start);

  ComboKey key2;
  key2.buttons = {"BACK"};
  EXPECT_EQ(joy_config.findBinding(key2)->action.type, ActionType::Quit);

  ComboKey key3;
  key3.buttons = {"RB", "A"};
  EXPECT_EQ(joy_config.findBinding(key3)->action.type, ActionType::SwitchController);

  ComboKey key4;
  key4.buttons = {"LB", "X"};
  EXPECT_EQ(joy_config.findBinding(key4)->action.type, ActionType::SetArmMode);

  ComboKey key5;
  key5.buttons = {"LB", "Y"};
  EXPECT_EQ(joy_config.findBinding(key5)->action.type, ActionType::SetWaistMode);

  ComboKey key6;
  key6.buttons = {"RB", "B"};
  EXPECT_EQ(joy_config.findBinding(key6)->action.type, ActionType::StartMotion);

  std::remove(path.c_str());
}

TEST(TeleopBindingConfigTest, LoadFromFile_EmptyConfig) {
  // 测试空配置
  std::string yaml = "";
  std::string path = CreateTempYamlFile(yaml);
  TeleopBindingConfig config;
  EXPECT_TRUE(config.loadFromFile(path));
  EXPECT_TRUE(config.isLoaded());

  const auto& joy_config = config.getJoyConfig();
  const auto& quest_config = config.getQuestConfig();
  EXPECT_EQ(joy_config.size(), 0);
  EXPECT_EQ(quest_config.size(), 0);

  std::remove(path.c_str());
}

TEST(TeleopBindingConfigTest, DirectConfigManipulation) {
  // 测试直接操作 DeviceBindingConfig
  DeviceBindingConfig joy_config;
  joy_config.device_type = TeleopDeviceType::kJoy;

  TeleopBinding binding;
  binding.combo.buttons = {"RB", "A"};
  binding.action = MakeSwitchControllerTrigger("amp");
  joy_config.addBinding(binding);

  EXPECT_EQ(joy_config.size(), 1);

  ComboKey key;
  key.buttons = {"RB", "A"};
  auto* found = joy_config.findBinding(key);
  ASSERT_NE(found, nullptr);
  EXPECT_EQ(found->action.type, ActionType::SwitchController);
}

TEST(TeleopBindingConfigTest, CopyConstructor) {
  std::string yaml = R"(
joy_bindings:
  - buttons: ["RB", "X"]
    action:
      type: SetArmMode
      args:
        name: keep_pose
)";
  std::string path = CreateTempYamlFile(yaml);
  TeleopBindingConfig config1;
  ASSERT_TRUE(config1.loadFromFile(path));

  // 使用拷贝构造函数
  TeleopBindingConfig config2(config1);

  EXPECT_TRUE(config2.isLoaded());
  EXPECT_EQ(config2.getJoyConfig().size(), 1);

  std::remove(path.c_str());
}

TEST(TeleopBindingConfigTest, AssignmentOperator) {
  std::string yaml = R"(
joy_bindings:
  - buttons: ["RB", "X"]
    action:
      type: SetArmMode
      args:
        name: keep_pose
)";
  std::string path = CreateTempYamlFile(yaml);
  TeleopBindingConfig config1;
  ASSERT_TRUE(config1.loadFromFile(path));

  TeleopBindingConfig config2;
  config2 = config1;

  EXPECT_TRUE(config2.isLoaded());
  EXPECT_EQ(config2.getJoyConfig().size(), 1);

  std::remove(path.c_str());
}

// ============================================================================
// 主函数
// ============================================================================

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
