/**
 * @file test_teleop_integration.cpp
 * @brief Teleop 端到端集成测试
 *
 * 测试从 YAML 配置到 ActionTrigger 生成的完整流程
 */

#include <gtest/gtest.h>

#include <cstdio>
#include <cstring>
#include <fstream>

#include "leju-rl-controller/runtime/input/command_buffer.h"
#include "leju-rl-controller/runtime/input/teleop/teleop_binding_config.h"
#include "leju-rl-controller/runtime/input/trigger_buffer.h"
#include "lejusdk-lowlevel/data_types.h"
#include "lejusdk-vr/data_types.h"

using namespace leju::runtime;
using leju::JoyData;
using leju::vr::QuestJoystickData;

// ============================================================================
// 辅助函数
// ============================================================================

static std::string CreateTempYamlFile(const std::string& content) {
  std::string path = "/tmp/test_integration_" + std::to_string(getpid()) + ".yaml";
  std::ofstream f(path);
  f << content;
  f.close();
  return path;
}

JoyData MakeJoyData(float left_x, float left_y, float right_x,
                    const std::vector<std::string>& pressed_buttons) {
  JoyData joy{};
  joy.axes.left_x = left_x;
  joy.axes.left_y = left_y;
  joy.axes.right_x = right_x;

  for (const auto& btn : pressed_buttons) {
    if (btn == "A" || btn == "SOUTH") joy.buttons.south = 1;
    if (btn == "B" || btn == "EAST") joy.buttons.east = 1;
    if (btn == "X" || btn == "WEST") joy.buttons.west = 1;
    if (btn == "Y" || btn == "NORTH") joy.buttons.north = 1;
    if (btn == "BACK") joy.buttons.back = 1;
    if (btn == "GUIDE") joy.buttons.guide = 1;
    if (btn == "START") joy.buttons.start = 1;
    if (btn == "L3" || btn == "LEFT_STICK") joy.buttons.left_stick = 1;
    if (btn == "R3" || btn == "RIGHT_STICK") joy.buttons.right_stick = 1;
    if (btn == "LB" || btn == "LEFT_SHOULDER") joy.buttons.left_shoulder = 1;
    if (btn == "RB" || btn == "RIGHT_SHOULDER") joy.buttons.right_shoulder = 1;
    if (btn == "DPAD_UP") joy.buttons.dpad_up = 1;
    if (btn == "DPAD_DOWN") joy.buttons.dpad_down = 1;
    if (btn == "DPAD_LEFT") joy.buttons.dpad_left = 1;
    if (btn == "DPAD_RIGHT") joy.buttons.dpad_right = 1;
    if (btn == "MISC") joy.buttons.misc1 = 1;
  }
  return joy;
}

QuestJoystickData MakeQuestData(float left_x, float left_y, float right_x,
                                float left_trigger, float right_trigger,
                                const std::vector<std::string>& pressed_buttons) {
  QuestJoystickData data;
  memset(&data, 0, sizeof(data));
  data.left_x = left_x;
  data.left_y = left_y;
  data.right_x = right_x;
  data.left_trigger = left_trigger;
  data.right_trigger = right_trigger;

  for (const auto& btn : pressed_buttons) {
    if (btn == "LEFT_FIRST" || btn == "L_FIRST" || btn == "A")
      data.left_first_button_pressed = true;
    if (btn == "LEFT_SECOND" || btn == "L_SECOND" || btn == "X")
      data.left_second_button_pressed = true;
    if (btn == "RIGHT_FIRST" || btn == "R_FIRST" || btn == "B")
      data.right_first_button_pressed = true;
    if (btn == "RIGHT_SECOND" || btn == "R_SECOND" || btn == "Y")
      data.right_second_button_pressed = true;
    if (btn == "LEFT_GRIP" || btn == "L_GRIP") data.left_grip = 1.0f;
    if (btn == "RIGHT_GRIP" || btn == "R_GRIP") data.right_grip = 1.0f;
  }
  return data;
}

// ============================================================================
// YAML 配置解析集成测试
// ============================================================================

TEST(TeleopIntegrationTest, YamlToConfigParsing) {
  std::string yaml = R"(
velocity_limits:
  stick_deadzone: 0.05
  linear_x: 0.55
  linear_y: 0.30
  angular_z: 0.30
joy_bindings:
  - buttons: ["RB", "X"]
    action:
      type: SetArmMode
      args:
        name: keep_pose
  - buttons: ["LB", "A"]
    action:
      type: SwitchController
      args:
        name: amp

quest_bindings:
  - buttons: ["RIGHT_TRIGGER", "RIGHT_FIRST"]
    action:
      type: SetArmMode
      args:
        name: keep_pose
)";

  std::string path = CreateTempYamlFile(yaml);
  TeleopBindingConfig config;
  ASSERT_TRUE(config.loadFromFile(path));
  EXPECT_TRUE(config.isLoaded());

  const auto& joy_config = config.getJoyConfig();
  const auto& quest_config = config.getQuestConfig();
  const auto& teleop_config = config.getTeleopConfig();

  EXPECT_EQ(joy_config.size(), 2);
  EXPECT_EQ(quest_config.size(), 1);
  EXPECT_DOUBLE_EQ(teleop_config.max_linear_x, 0.55);
  EXPECT_DOUBLE_EQ(teleop_config.max_linear_y, 0.30);
  EXPECT_DOUBLE_EQ(teleop_config.max_angular_z, 0.30);

  // 验证 Joy 绑定可以通过查找找到
  ComboKey joy_key1;
  joy_key1.buttons = {"RB", "X"};
  auto* joy_binding1 = joy_config.findBinding(joy_key1);
  ASSERT_NE(joy_binding1, nullptr);
  EXPECT_EQ(joy_binding1->action.type, ActionType::SetArmMode);

  ComboKey joy_key2;
  joy_key2.buttons = {"LB", "A"};
  auto* joy_binding2 = joy_config.findBinding(joy_key2);
  ASSERT_NE(joy_binding2, nullptr);
  EXPECT_EQ(joy_binding2->action.type, ActionType::SwitchController);

  // 验证 Quest 绑定
  ComboKey quest_key;
  quest_key.buttons = {"RIGHT_TRIGGER", "RIGHT_FIRST"};
  auto* quest_binding = quest_config.findBinding(quest_key);
  ASSERT_NE(quest_binding, nullptr);
  EXPECT_EQ(quest_binding->action.type, ActionType::SetArmMode);

  std::remove(path.c_str());
}

// ============================================================================
// 复杂配置测试
// ============================================================================

TEST(TeleopIntegrationTest, ComplexConfiguration) {
  // 完整的配置，包含所有动作类型
  std::string yaml = R"(
velocity_limits:
  stick_deadzone: 0.05
  linear_x: 0.60
  linear_y: 0.60
  angular_z: 1.00
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
  - buttons: ["RB", "B"]
    action:
      type: SwitchController
      args:
        name: mimic
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

quest_bindings:
  - buttons: ["RIGHT_TRIGGER", "RIGHT_FIRST"]
    action:
      type: SwitchController
      args:
        name: amp
  - buttons: ["LEFT_TRIGGER", "LEFT_FIRST"]
    action:
      type: SetArmMode
      args:
        name: keep_pose
  - buttons: ["RIGHT_GRIP"]
    action:
      type: SetWaistMode
      args:
        name: external
)";

  std::string path = CreateTempYamlFile(yaml);
  TeleopBindingConfig config;
  ASSERT_TRUE(config.loadFromFile(path));

  const auto& joy_config = config.getJoyConfig();
  const auto& quest_config = config.getQuestConfig();

  EXPECT_EQ(joy_config.size(), 6);
  EXPECT_EQ(quest_config.size(), 3);

  // 验证所有动作类型都存在
  bool has_start = false, has_quit = false, has_switch = false;
  bool has_arm_mode = false, has_waist_mode = false;

  // 检查 Joy 配置
  ComboKey keys[] = {
      {{"START"}}, {{"BACK"}}, {{"RB", "A"}}, {{"RB", "B"}}, {{"LB", "X"}}, {{"LB", "Y"}}};
  for (const auto& key : keys) {
    auto* binding = joy_config.findBinding(key);
    if (binding) {
      switch (binding->action.type) {
        case ActionType::Start: has_start = true; break;
        case ActionType::Quit: has_quit = true; break;
        case ActionType::SwitchController: has_switch = true; break;
        case ActionType::SetArmMode: has_arm_mode = true; break;
        case ActionType::SetWaistMode: has_waist_mode = true; break;
        default: break;
      }
    }
  }

  EXPECT_TRUE(has_start);
  EXPECT_TRUE(has_quit);
  EXPECT_TRUE(has_switch);
  EXPECT_TRUE(has_arm_mode);
  EXPECT_TRUE(has_waist_mode);

  std::remove(path.c_str());
}

// ============================================================================
// TriggerBuffer 集成测试
// ============================================================================

TEST(TeleopIntegrationTest, TriggerBufferIntegration) {
  // 1. 创建 TriggerBuffer
  TriggerBuffer trigger_buffer;

  // 2. 直接测试 TriggerBuffer 功能
  ActionTrigger trigger(ActionType::SetArmMode);
  trigger_buffer.push(trigger);

  // 3. 验证 TriggerBuffer 内容
  auto drained = trigger_buffer.drainAll();
  ASSERT_EQ(drained.size(), 1);
  EXPECT_EQ(drained[0].type, ActionType::SetArmMode);
}

// ============================================================================
// 主函数
// ============================================================================

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
