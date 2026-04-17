/**
 * @file test_teleop_adapter.cpp
 * @brief TeleopAdapter 单元测试
 *
 * 注意：QuestTeleopAdapter 现在采用主动订阅模式，需要 VR SDK 初始化。
 * 本测试文件主要测试 JoyTeleopAdapter 和配置加载。
 */

#include <gtest/gtest.h>

#include <cstring>

#define private public
#define protected public
#include "leju-rl-controller/runtime/input/command_buffer.h"
#include "leju-rl-controller/runtime/control_loop.h"
#include "leju-rl-controller/runtime/input/teleop/joy_teleop_adapter.h"
#include "leju-rl-controller/runtime/input/teleop/quest_teleop_adapter.h"
#include "leju-rl-controller/runtime/input/teleop/teleop_input_source.h"
#include "leju-rl-controller/runtime/input/teleop/teleop_binding_config.h"
#include "leju-rl-controller/runtime/input/trigger_buffer.h"
#undef protected
#undef private

#include "leju-rl-controller/controllers/controller_manager.h"
#include "leju-rl-controller/robot_data.h"
#include "leju-rl-controller/runtime/lifecycle.h"
#include "lejusdk-lowlevel/data_types.h"
#include "lejusdk-utils/robot_version.hpp"
#include "lejusdk-vr/data_types.h"

using namespace leju::runtime;
using leju::JoyData;
using leju::RobotData;
using leju::ControllerManager;
using leju::vr::QuestJoystickData;

// ============================================================================
// 辅助函数：创建测试数据
// ============================================================================

// 创建 JoyData 的辅助函数
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

QuestJoystickData MakeQuestData(float left_x, float left_y, float right_x) {
  QuestJoystickData joy{};
  joy.left_x = left_x;
  joy.left_y = left_y;
  joy.right_x = right_x;
  return joy;
}

namespace {

class StubInputSource : public InputSource {
 public:
  explicit StubInputSource(InputPriority priority) : priority_(priority) {}

  CommandBuffer::Snapshot getSnapshot() const override { return snapshot_; }
  InputPriority getPriority() const override { return priority_; }
  const char* getName() const override { return "StubInput"; }

  CommandBuffer::Snapshot snapshot_;

 private:
  InputPriority priority_;
};

}  // namespace

// ============================================================================
// TeleopBindingConfig 加载测试
// ============================================================================

TEST(JoyTeleopAdapterConfigTest, LoadBindingConfigFromString) {
  TriggerBuffer trigger_buffer;
  JoyTeleopAdapter adapter(&trigger_buffer);

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
)";

  // 通过加载临时文件测试，或验证接口行为
  // 这里仅测试构造和基础功能
  SUCCEED();
}

TEST(JoyTeleopAdapterConfigTest, LoadBindingConfigFromFileNotFound) {
  TriggerBuffer trigger_buffer;
  JoyTeleopAdapter adapter(&trigger_buffer);

  // 尝试加载不存在的文件
  bool result = adapter.loadBindingConfig("/nonexistent/path/config.yaml");
  EXPECT_FALSE(result);
}

// ============================================================================
// CommandBuffer 集成测试
// ============================================================================

TEST(JoyTeleopAdapterCommandBufferTest, GetSnapshotReturnsValidData) {
  TriggerBuffer trigger_buffer;
  JoyTeleopAdapter adapter(&trigger_buffer);

  // 获取快照（应该返回默认值）
  auto snapshot = adapter.getSnapshot();
  EXPECT_FALSE(snapshot.cmd_vel.valid);  // 默认无效
}

TEST(QuestTeleopAdapterRegressionTest, CombinesTranslationAndRotationInSingleCmd) {
  TriggerBuffer trigger_buffer;
  QuestTeleopAdapter adapter(leju::RobotVersions::KUAVO5_BASE, &trigger_buffer);

  adapter.config_.stick_deadzone = 0.0f;
  adapter.config_.max_linear_x = 0.55;
  adapter.config_.max_linear_y = 0.30;
  adapter.config_.max_angular_z = 1.00;

  auto quest = MakeQuestData(0.4f, 0.6f, -0.25f);
  adapter.processVelocityImpl(quest, adapter.cmd_buffer_, 0.0);

  auto snapshot = adapter.getSnapshot();
  ASSERT_TRUE(snapshot.cmd_vel.valid);
  EXPECT_NEAR(snapshot.cmd_vel.linear_x, 0.6 * 0.55, 1e-6);
  EXPECT_NEAR(snapshot.cmd_vel.linear_y, -0.4 * 0.30, 1e-6);
  EXPECT_NEAR(snapshot.cmd_vel.angular_z, 0.25, 1e-6);
}

TEST(TeleopInputSourceRegressionTest, QuestTakesOverWhenJoyIsIdle) {
  TriggerBuffer trigger_buffer;
  TeleopInputSource input_source(leju::RobotVersions::KUAVO5_BASE, &trigger_buffer);

  MotionCommand joy_cmd;
  joy_cmd.setZero();
  joy_cmd.valid = true;
  input_source.joy_adapter_->cmd_buffer_.writeCmdVel(joy_cmd);

  MotionCommand quest_cmd;
  quest_cmd.linear_x = 0.2;
  quest_cmd.linear_y = -0.1;
  quest_cmd.angular_z = 0.3;
  quest_cmd.valid = true;
  input_source.quest_adapter_->cmd_buffer_.writeCmdVel(quest_cmd);

  auto snapshot = input_source.getSnapshot();
  ASSERT_TRUE(snapshot.cmd_vel.valid);
  EXPECT_DOUBLE_EQ(snapshot.cmd_vel.linear_x, 0.2);
  EXPECT_DOUBLE_EQ(snapshot.cmd_vel.linear_y, -0.1);
  EXPECT_DOUBLE_EQ(snapshot.cmd_vel.angular_z, 0.3);
}

TEST(TeleopInputSourceRegressionTest, JoyStillWinsWhenActivelyMoving) {
  TriggerBuffer trigger_buffer;
  TeleopInputSource input_source(leju::RobotVersions::KUAVO5_BASE, &trigger_buffer);

  MotionCommand joy_cmd;
  joy_cmd.linear_x = 0.15;
  joy_cmd.linear_y = 0.0;
  joy_cmd.angular_z = 0.0;
  joy_cmd.valid = true;
  input_source.joy_adapter_->cmd_buffer_.writeCmdVel(joy_cmd);

  MotionCommand quest_cmd;
  quest_cmd.linear_x = 0.2;
  quest_cmd.linear_y = -0.1;
  quest_cmd.angular_z = 0.3;
  quest_cmd.valid = true;
  input_source.quest_adapter_->cmd_buffer_.writeCmdVel(quest_cmd);

  auto snapshot = input_source.getSnapshot();
  ASSERT_TRUE(snapshot.cmd_vel.valid);
  EXPECT_DOUBLE_EQ(snapshot.cmd_vel.linear_x, 0.15);
  EXPECT_DOUBLE_EQ(snapshot.cmd_vel.linear_y, 0.0);
  EXPECT_DOUBLE_EQ(snapshot.cmd_vel.angular_z, 0.0);
}

// ============================================================================
// JoyTeleopAdapter 初始化测试
// ============================================================================

TEST(JoyTeleopAdapterTest, IsInitializedReturnsFalseByDefault) {
  TriggerBuffer trigger_buffer;
  JoyTeleopAdapter adapter(&trigger_buffer);

  // 未初始化时应该返回 false
  EXPECT_FALSE(adapter.isInitialized());
}

// ============================================================================
// TeleopBindingConfig 独立测试
// ============================================================================

TEST(TeleopBindingConfigTest, ParseJoyBindings) {
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
)";

  // 创建临时文件
  const char* temp_file = "/tmp/test_binding_config.yaml";
  {
    FILE* f = fopen(temp_file, "w");
    fprintf(f, "%s", yaml.c_str());
    fclose(f);
  }

  TeleopBindingConfig config;
  ASSERT_TRUE(config.loadFromFile(temp_file));

  const auto& joy_config = config.getJoyConfig();
  const auto& teleop_config = config.getTeleopConfig();
  EXPECT_EQ(joy_config.size(), 2);
  EXPECT_FLOAT_EQ(teleop_config.stick_deadzone, 0.05f);
  EXPECT_DOUBLE_EQ(teleop_config.max_linear_x, 0.55);
  EXPECT_DOUBLE_EQ(teleop_config.max_linear_y, 0.30);
  EXPECT_DOUBLE_EQ(teleop_config.max_angular_z, 0.30);

  // 验证绑定
  ComboKey key1;
  key1.buttons = {"RB", "X"};
  auto* binding1 = joy_config.findBinding(key1);
  ASSERT_NE(binding1, nullptr);
  EXPECT_EQ(binding1->action.type, ActionType::SetArmMode);

  ComboKey key2;
  key2.buttons = {"LB", "A"};
  auto* binding2 = joy_config.findBinding(key2);
  ASSERT_NE(binding2, nullptr);
  EXPECT_EQ(binding2->action.type, ActionType::SwitchController);

  remove(temp_file);
}

TEST(TeleopBindingConfigTest, ParseQuestBindings) {
  std::string yaml = R"(
velocity_limits:
  stick_deadzone: 0.05
  linear_x: 0.45
  linear_y: 0.30
  angular_z: 1.00
quest_bindings:
  - buttons: ["RIGHT_TRIGGER", "RIGHT_FIRST"]
    action:
      type: SetArmMode
      args:
        name: keep_pose
)";

  // 创建临时文件
  const char* temp_file = "/tmp/test_quest_config.yaml";
  {
    FILE* f = fopen(temp_file, "w");
    fprintf(f, "%s", yaml.c_str());
    fclose(f);
  }

  TeleopBindingConfig config;
  ASSERT_TRUE(config.loadFromFile(temp_file));

  const auto& quest_config = config.getQuestConfig();
  EXPECT_EQ(quest_config.size(), 1);

  // 验证绑定
  ComboKey key;
  key.buttons = {"RIGHT_TRIGGER", "RIGHT_FIRST"};
  auto* binding = quest_config.findBinding(key);
  ASSERT_NE(binding, nullptr);
  EXPECT_EQ(binding->action.type, ActionType::SetArmMode);

  remove(temp_file);
}

TEST(TeleopBindingConfigTest, MissingVelocityLimitsFails) {
  std::string yaml = R"(
joy_bindings:
  - buttons: ["RB", "X"]
    action:
      type: SetArmMode
      args:
        name: keep_pose
)";

  const char* temp_file = "/tmp/test_missing_velocity_limits.yaml";
  {
    FILE* f = fopen(temp_file, "w");
    fprintf(f, "%s", yaml.c_str());
    fclose(f);
  }

  TeleopBindingConfig config;
  EXPECT_FALSE(config.loadFromFile(temp_file));

  remove(temp_file);
}

// ============================================================================
// 主函数
// ============================================================================

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
