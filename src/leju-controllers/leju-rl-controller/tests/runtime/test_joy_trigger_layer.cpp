/**
 * @file test_joy_trigger_layer.cpp
 * @brief v17 手柄触发层单元测试
 *
 * 覆盖：LT/RT 扳机虚拟按键、松开响应(edge)、走路门控(block_when_walking)、
 *       十字键切档不输出速度、LB+A 输入屏蔽(SetInputMask)、配置解析、v17 实配加载。
 *
 * 需求来源：需求文档-Roban2.2手柄控制功能调整需求文档V2.0--3588版.pdf
 */

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>

#define private public
#define protected public
#include "leju-rl-controller/runtime/input/teleop/joy_teleop_adapter.h"
#include "leju-rl-controller/runtime/input/teleop/teleop_binding_config.h"
#include "leju-rl-controller/runtime/input/trigger_buffer.h"
#undef protected
#undef private

#include "lejusdk-lowlevel/data_types.h"

using namespace leju::runtime;
using leju::JoyData;

// ============================================================================
// 辅助函数
// ============================================================================

// 完整版 JoyData 构造：支持摇杆、扳机、方向键、misc
JoyData MakeJoy(float left_x, float left_y, float right_x, float right_y,
                float left_trigger, float right_trigger,
                const std::vector<std::string>& buttons) {
  JoyData joy{};
  joy.axes.left_x = left_x;
  joy.axes.left_y = left_y;
  joy.axes.right_x = right_x;
  joy.axes.right_y = right_y;
  joy.axes.left_trigger = left_trigger;
  joy.axes.right_trigger = right_trigger;
  for (const auto& b : buttons) {
    if (b == "A") joy.buttons.south = 1;
    if (b == "B") joy.buttons.east = 1;
    if (b == "X") joy.buttons.west = 1;
    if (b == "Y") joy.buttons.north = 1;
    if (b == "BACK") joy.buttons.back = 1;
    if (b == "START") joy.buttons.start = 1;
    if (b == "GUIDE") joy.buttons.guide = 1;
    if (b == "L3") joy.buttons.left_stick = 1;
    if (b == "R3") joy.buttons.right_stick = 1;
    if (b == "LB") joy.buttons.left_shoulder = 1;
    if (b == "RB") joy.buttons.right_shoulder = 1;
    if (b == "DPAD_UP") joy.buttons.dpad_up = 1;
    if (b == "DPAD_DOWN") joy.buttons.dpad_down = 1;
    if (b == "DPAD_LEFT") joy.buttons.dpad_left = 1;
    if (b == "DPAD_RIGHT") joy.buttons.dpad_right = 1;
    if (b == "MISC") joy.buttons.misc1 = 1;
    if (b == "MISC2") joy.buttons.misc2 = 1;
  }
  return joy;
}

// 简化：仅按键
JoyData MakeBtn(const std::vector<std::string>& buttons) {
  return MakeJoy(0, 0, 0, 0, 0, 0, buttons);
}

static std::string WriteTempYaml(const std::string& content) {
  std::string path = "/tmp/test_joy_trigger_" + std::to_string(getpid()) + ".yaml";
  std::ofstream f(path);
  f << content;
  f.close();
  return path;
}

// 用一段 YAML 配置一个 adapter
static void LoadConfig(JoyTeleopAdapter& adapter, const std::string& yaml) {
  std::string path = WriteTempYaml(yaml);
  ASSERT_TRUE(adapter.loadBindingConfig(path));
  std::remove(path.c_str());
}

// 驱动一帧并返回本帧产生的触发器
static std::vector<ActionTrigger> Feed(JoyTeleopAdapter& adapter,
                                       TriggerBuffer& tb,
                                       const JoyData& joy,
                                       const JoyData& prev) {
  adapter.onJoyData(joy, prev.buttons);
  return tb.drainAll();
}

// ============================================================================
// F1: LT/RT 扳机 → 虚拟按键
// ============================================================================

TEST(JoyTriggerLayer_F1, LeftTriggerBecomesVirtualButton) {
  TriggerBuffer tb;
  JoyTeleopAdapter adapter(&tb);  // config_ 用默认值，trigger_threshold=0.5

  auto combo = adapter.detectCurrentComboImpl(MakeJoy(0, 0, 0, 0, 0.8f, 0.0f, {}));
  EXPECT_EQ(ComboKeyToString(combo), "LT");
}

TEST(JoyTriggerLayer_F1, RightTriggerBecomesVirtualButton) {
  TriggerBuffer tb;
  JoyTeleopAdapter adapter(&tb);
  auto combo = adapter.detectCurrentComboImpl(MakeJoy(0, 0, 0, 0, 0.0f, 0.8f, {}));
  EXPECT_EQ(ComboKeyToString(combo), "RT");
}

TEST(JoyTriggerLayer_F1, TriggerBelowThresholdIgnored) {
  TriggerBuffer tb;
  JoyTeleopAdapter adapter(&tb);
  auto combo = adapter.detectCurrentComboImpl(MakeJoy(0, 0, 0, 0, 0.3f, 0.0f, {}));
  EXPECT_TRUE(combo.empty());
}

TEST(JoyTriggerLayer_F1, TriggerCombinesWithFaceButton) {
  TriggerBuffer tb;
  JoyTeleopAdapter adapter(&tb);
  auto combo = adapter.detectCurrentComboImpl(MakeJoy(0, 0, 0, 0, 0.8f, 0.0f, {"A"}));
  EXPECT_EQ(ComboKeyToString(combo), "A+LT");
}

// ============================================================================
// F2: 松开响应 / 按下响应
// ============================================================================

static const char* kEdgeYaml = R"(
velocity_limits:
  stick_deadzone: 0.05
  linear_x: 0.6
  linear_y: 0.6
  angular_z: 1.0
  trigger_threshold: 0.5
joy_bindings:
  - buttons: ["LT", "A"]
    edge: release
    action:
      type: MotionCommand
      args:
        op: Start
        name: baoquan
  - buttons: ["X"]
    edge: press
    action:
      type: SwitchController
      args:
        name: amp
)";

TEST(JoyTriggerLayer_F2, ReleaseEdgeFiresOnRelease) {
  TriggerBuffer tb;
  JoyTeleopAdapter adapter(&tb);
  LoadConfig(adapter, kEdgeYaml);

  JoyData empty = MakeBtn({});
  JoyData lt = MakeJoy(0, 0, 0, 0, 0.8f, 0, {});          // 仅按住 LT
  JoyData lt_a = MakeJoy(0, 0, 0, 0, 0.8f, 0, {"A"});      // LT + A

  // prime
  EXPECT_TRUE(Feed(adapter, tb, empty, empty).empty());
  // 按住 LT：{}->{LT}，无绑定
  EXPECT_TRUE(Feed(adapter, tb, lt, empty).empty());
  // 按下 A：{LT}->{LT,A}，release 绑定不在按下时触发
  EXPECT_TRUE(Feed(adapter, tb, lt_a, lt).empty());
  // 松开 A：{LT,A}->{LT}，release 触发 baoquan
  auto fired = Feed(adapter, tb, lt, lt_a);
  ASSERT_EQ(fired.size(), 1u);
  EXPECT_EQ(fired[0].type, ActionType::MotionCommand);
  auto* args = dynamic_cast<MotionCommandArgs*>(fired[0].args.get());
  ASSERT_NE(args, nullptr);
  EXPECT_EQ(args->motion_name, "baoquan");
}

TEST(JoyTriggerLayer_F2, PressEdgeFiresOnPressNotRelease) {
  TriggerBuffer tb;
  JoyTeleopAdapter adapter(&tb);
  LoadConfig(adapter, kEdgeYaml);

  JoyData empty = MakeBtn({});
  JoyData x = MakeBtn({"X"});

  EXPECT_TRUE(Feed(adapter, tb, empty, empty).empty());  // prime
  // 按下 X：press 触发
  auto pressed = Feed(adapter, tb, x, empty);
  ASSERT_EQ(pressed.size(), 1u);
  EXPECT_EQ(pressed[0].type, ActionType::SwitchController);
  // 松开 X：press 绑定不在松开时触发
  EXPECT_TRUE(Feed(adapter, tb, empty, x).empty());
}

// ============================================================================
// F3: 走路门控
// ============================================================================

static const char* kWalkGateYaml = R"(
velocity_limits:
  stick_deadzone: 0.05
  linear_x: 0.6
  linear_y: 0.6
  angular_z: 1.0
joy_bindings:
  - buttons: ["A"]
    edge: press
    block_when_walking: true
    action:
      type: MotionCommand
      args:
        op: Start
        name: gated
  - buttons: ["RT", "A"]
    edge: release
    action:
      type: MotionCommand
      args:
        op: Start
        name: fist_bump
)";

TEST(JoyTriggerLayer_F3, BlockedComboFiresWhenIdle) {
  TriggerBuffer tb;
  JoyTeleopAdapter adapter(&tb);
  LoadConfig(adapter, kWalkGateYaml);

  JoyData empty = MakeBtn({});
  JoyData a = MakeBtn({"A"});  // 轴全 0 → 静止

  EXPECT_TRUE(Feed(adapter, tb, empty, empty).empty());  // prime
  auto fired = Feed(adapter, tb, a, empty);
  ASSERT_EQ(fired.size(), 1u);
  EXPECT_EQ(fired[0].type, ActionType::MotionCommand);
}

TEST(JoyTriggerLayer_F3, BlockedComboSuppressedWhenWalking) {
  TriggerBuffer tb;
  JoyTeleopAdapter adapter(&tb);
  LoadConfig(adapter, kWalkGateYaml);

  JoyData empty = MakeBtn({});
  JoyData a_walk = MakeJoy(0, 0.8f, 0, 0, 0, 0, {"A"});  // 左摇杆推动=走路

  EXPECT_TRUE(Feed(adapter, tb, empty, empty).empty());  // prime
  auto fired = Feed(adapter, tb, a_walk, empty);
  EXPECT_TRUE(fired.empty());  // 走路时被门控屏蔽
}

TEST(JoyTriggerLayer_F3, PresetActionFiresWhileWalking) {
  // LT/RT 预设动作 block_when_walking=false（默认）→ 边走边做
  TriggerBuffer tb;
  JoyTeleopAdapter adapter(&tb);
  LoadConfig(adapter, kWalkGateYaml);

  JoyData walk = MakeJoy(0, 0.8f, 0, 0, 0, 0, {});            // 走路
  JoyData walk_rt_a = MakeJoy(0, 0.8f, 0, 0, 0, 0.8f, {"A"}); // 走路 + RT + A

  EXPECT_TRUE(Feed(adapter, tb, walk, walk).empty());  // prime（走路态）
  EXPECT_TRUE(Feed(adapter, tb, walk_rt_a, walk).empty());  // 按下 A（release 绑定，不在按下触发）
  auto fired = Feed(adapter, tb, walk, walk_rt_a);  // 松开 A，走路中仍触发
  ASSERT_EQ(fired.size(), 1u);
  auto* args = dynamic_cast<MotionCommandArgs*>(fired[0].args.get());
  ASSERT_NE(args, nullptr);
  EXPECT_EQ(args->motion_name, "fist_bump");
}

// ============================================================================
// F4: 十字键仅切前进速度档，不输出速度
// ============================================================================

static void EnableAmpHandGear(JoyTeleopAdapter& adapter) {
  adapter.config_.amp_hand_posture_axis.enabled = true;
  adapter.config_.amp_hand_cmd_vel_line_x_gear.enabled = true;
  adapter.config_.amp_hand_cmd_vel_line_x_gear.policy_linear_x = 0.75;
  adapter.config_.amp_hand_cmd_vel_line_x_gear.policy_linear_x_low = 0.4;
  adapter.config_.amp_hand_cmd_vel_line_x_gear.policy_linear_x_up = 0.95;
  adapter.config_.amp_hand_cmd_vel_line_x_gear.policy_linear_x_negative = 0.6;
}

TEST(JoyTriggerLayer_F4, DpadSwitchesGearWithoutVelocity) {
  TriggerBuffer tb;
  JoyTeleopAdapter adapter(&tb);
  LoadConfig(adapter, kEdgeYaml);
  EnableAmpHandGear(adapter);

  JoyData empty = MakeBtn({});
  JoyData dpad_up = MakeBtn({"DPAD_UP"});

  Feed(adapter, tb, empty, empty);  // prime
  EXPECT_EQ(adapter.cmd_vel_line_x_limit_level_, 1);

  Feed(adapter, tb, dpad_up, empty);
  EXPECT_EQ(adapter.cmd_vel_line_x_limit_level_, 2);
  EXPECT_DOUBLE_EQ(adapter.getSnapshot().cmd_vel.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(adapter.getSnapshot().cmd_vel.linear_y, 0.0);
  EXPECT_DOUBLE_EQ(adapter.getSnapshot().cmd_vel.angular_z, 0.0);
}

TEST(JoyTriggerLayer_F4, ForwardGearLimitIsDirectPolicyValue) {
  TriggerBuffer tb;
  JoyTeleopAdapter adapter(&tb);
  LoadConfig(adapter, kEdgeYaml);
  EnableAmpHandGear(adapter);
  adapter.config_.max_linear_x = 1.0;

  adapter.cmd_vel_line_x_limit_level_ = 0;
  EXPECT_DOUBLE_EQ(adapter.getEffectiveMaxLinearXForward(), 0.4);

  adapter.cmd_vel_line_x_limit_level_ = 1;
  EXPECT_DOUBLE_EQ(adapter.getEffectiveMaxLinearXForward(), 0.75);

  adapter.cmd_vel_line_x_limit_level_ = 2;
  EXPECT_DOUBLE_EQ(adapter.getEffectiveMaxLinearXForward(), 0.95);

  adapter.config_.amp_hand_cmd_vel_line_x_gear.policy_linear_x_low = 0.5;
  adapter.cmd_vel_line_x_limit_level_ = 0;
  EXPECT_DOUBLE_EQ(adapter.getEffectiveMaxLinearXForward(), 0.5);
}

TEST(JoyTriggerLayer_F4, BackwardLimitIsDirectPolicyValue) {
  TriggerBuffer tb;
  JoyTeleopAdapter adapter(&tb);
  LoadConfig(adapter, kEdgeYaml);
  EnableAmpHandGear(adapter);

  EXPECT_DOUBLE_EQ(adapter.getEffectiveMaxLinearXBackward(), 0.6);

  adapter.config_.amp_hand_cmd_vel_line_x_gear.policy_linear_x_negative = 0.5;
  EXPECT_DOUBLE_EQ(adapter.getEffectiveMaxLinearXBackward(), 0.5);
}

// ============================================================================
// F5: LB+A 输入屏蔽
// ============================================================================

static const char* kMaskYaml = R"(
velocity_limits:
  stick_deadzone: 0.05
  linear_x: 0.6
  linear_y: 0.6
  angular_z: 1.0
joy_bindings:
  - buttons: ["LB", "A"]
    edge: press
    action:
      type: SetInputMask
      args:
        name: toggle
)";

TEST(JoyTriggerLayer_F5, MaskTogglesVelocityAndIsNotForwarded) {
  TriggerBuffer tb;
  JoyTeleopAdapter adapter(&tb);
  LoadConfig(adapter, kMaskYaml);

  JoyData empty = MakeBtn({});
  JoyData lb_a = MakeBtn({"LB", "A"});
  JoyData stick = MakeJoy(0, 0.8f, 0, 0, 0, 0, {});  // 仅推左摇杆

  // prime
  Feed(adapter, tb, empty, empty);
  // 未屏蔽：摇杆生效
  Feed(adapter, tb, stick, empty);
  EXPECT_LT(adapter.getSnapshot().cmd_vel.linear_x, -0.1);

  // 按 LB+A 开启屏蔽：SetInputMask 不下发
  auto fired = Feed(adapter, tb, lb_a, stick);
  EXPECT_TRUE(fired.empty()) << "SetInputMask 应被 adapter 内部消费，不进入 TriggerBuffer";
  EXPECT_TRUE(adapter.input_masked_);

  // 屏蔽态下推摇杆 → 速度被清零
  Feed(adapter, tb, stick, lb_a);
  EXPECT_DOUBLE_EQ(adapter.getSnapshot().cmd_vel.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(adapter.getSnapshot().cmd_vel.linear_y, 0.0);

  // 再按 LB+A 关闭屏蔽
  Feed(adapter, tb, lb_a, stick);
  EXPECT_FALSE(adapter.input_masked_);
  Feed(adapter, tb, stick, lb_a);
  EXPECT_LT(adapter.getSnapshot().cmd_vel.linear_x, -0.1);  // 恢复
}

TEST(JoyTriggerLayer_F5, EntryGuardCountsOnlyRealNeutralJoyFrames) {
  TriggerBuffer tb;
  JoyTeleopAdapter adapter(&tb);
  LoadConfig(adapter, kEdgeYaml);
  adapter.config_.velocity_entry_neutral_guard.enabled = true;
  adapter.config_.velocity_entry_neutral_guard.neutral_frames = 3;

  JoyData neutral = MakeBtn({});
  JoyData start_with_stick = MakeJoy(0, 0.8f, 0, 0, 0, 0, {"START"});
  JoyData stick = MakeJoy(0, 0.8f, 0, 0, 0, 0, {});
  Feed(adapter, tb, neutral, neutral);  // prime

  auto start_triggers = Feed(adapter, tb, start_with_stick, neutral);
  ASSERT_EQ(start_triggers.size(), 1u);
  EXPECT_EQ(start_triggers[0].type, ActionType::Start);
  EXPECT_TRUE(adapter.velocity_neutral_guard_active_);
  EXPECT_DOUBLE_EQ(adapter.getSnapshot().cmd_vel.linear_x, 0.0);

  // 非回中真实帧会持续清零，并重置计数。
  Feed(adapter, tb, neutral, start_with_stick);
  Feed(adapter, tb, neutral, neutral);
  Feed(adapter, tb, stick, neutral);
  EXPECT_TRUE(adapter.velocity_neutral_guard_active_);
  EXPECT_EQ(adapter.velocity_neutral_frames_, 0);
  EXPECT_DOUBLE_EQ(adapter.getSnapshot().cmd_vel.linear_x, 0.0);

  // 必须再收到三帧真实回中数据才解锁；之后新的推杆帧才可下发。
  Feed(adapter, tb, neutral, stick);
  Feed(adapter, tb, neutral, neutral);
  EXPECT_TRUE(adapter.velocity_neutral_guard_active_);
  Feed(adapter, tb, neutral, neutral);
  EXPECT_FALSE(adapter.velocity_neutral_guard_active_);
  Feed(adapter, tb, stick, neutral);
  EXPECT_LT(adapter.getSnapshot().cmd_vel.linear_x, -0.1);
}

TEST(JoyTriggerLayer_F5, SwitchToAmpArmsEntryGuard) {
  TriggerBuffer tb;
  JoyTeleopAdapter adapter(&tb);
  LoadConfig(adapter, kEdgeYaml);
  adapter.config_.velocity_entry_neutral_guard.enabled = true;
  adapter.config_.velocity_entry_neutral_guard.neutral_frames = 3;

  JoyData neutral = MakeBtn({});
  JoyData x = MakeBtn({"X"});
  Feed(adapter, tb, neutral, neutral);  // prime
  auto triggers = Feed(adapter, tb, x, neutral);

  ASSERT_EQ(triggers.size(), 1u);
  EXPECT_EQ(triggers[0].type, ActionType::SwitchController);
  EXPECT_TRUE(adapter.velocity_neutral_guard_active_);
  EXPECT_EQ(adapter.velocity_neutral_frames_, 1);
}

// ============================================================================
// F6: 配置解析
// ============================================================================

TEST(JoyTriggerLayer_F6, ParsesEdgeBlockAndThreshold) {
  std::string yaml = R"(
velocity_limits:
  stick_deadzone: 0.05
  linear_x: 0.6
  linear_y: 0.6
  angular_z: 1.0
  trigger_threshold: 0.42
joy_bindings:
  - buttons: ["LT", "A"]
    edge: release
    block_when_walking: false
    action:
      type: MotionCommand
      args:
        op: Start
        name: baoquan
  - buttons: ["MISC", "A"]
    edge: release
    block_when_walking: true
    action:
      type: MotionCommand
      args:
        op: Start
        name: custom
)";
  std::string path = WriteTempYaml(yaml);
  TeleopBindingConfig config;
  ASSERT_TRUE(config.loadFromFile(path));
  std::remove(path.c_str());

  EXPECT_FLOAT_EQ(config.getTeleopConfig().trigger_threshold, 0.42f);

  ComboKey k1; k1.buttons = {"LT", "A"};
  const auto* b1 = config.getJoyConfig().findBinding(k1);
  ASSERT_NE(b1, nullptr);
  EXPECT_EQ(b1->edge, TriggerEdge::kRelease);
  EXPECT_FALSE(b1->block_when_walking);

  ComboKey k2; k2.buttons = {"MISC", "A"};
  const auto* b2 = config.getJoyConfig().findBinding(k2);
  ASSERT_NE(b2, nullptr);
  EXPECT_TRUE(b2->block_when_walking);
}

TEST(JoyTriggerLayer_F6, DefaultEdgeIsPress) {
  std::string yaml = R"(
velocity_limits:
  stick_deadzone: 0.05
  linear_x: 0.6
  linear_y: 0.6
  angular_z: 1.0
joy_bindings:
  - buttons: ["X"]
    action:
      type: SwitchController
      args:
        name: amp
)";
  std::string path = WriteTempYaml(yaml);
  TeleopBindingConfig config;
  ASSERT_TRUE(config.loadFromFile(path));
  std::remove(path.c_str());

  ComboKey k; k.buttons = {"X"};
  const auto* b = config.getJoyConfig().findBinding(k);
  ASSERT_NE(b, nullptr);
  EXPECT_EQ(b->edge, TriggerEdge::kPress);
  EXPECT_FALSE(b->block_when_walking);
}

TEST(JoyTriggerLayer_F6, ParsesVelocityEntryNeutralGuard) {
  std::string yaml = R"(
HumanoidRobotCfg:
  env:
    velocity_entry_neutral_guard:
      enabled: true
      neutral_frames: 7
)";
  std::string path = WriteTempYaml(yaml);
  TeleopConfig config;
  ASSERT_TRUE(ParseAmpHandTeleopFromControllerConfig(path, config));
  std::remove(path.c_str());

  EXPECT_TRUE(config.velocity_entry_neutral_guard.enabled);
  EXPECT_EQ(config.velocity_entry_neutral_guard.neutral_frames, 7);
}

TEST(JoyTriggerLayer_F6, ParsesQuitSquatConfig) {
  std::string yaml = R"(
HumanoidRobotCfg:
  env:
    quit_squat:
      enabled: true
      squat_height: -0.15
      duration_sec: 1.5
)";
  std::string path = WriteTempYaml(yaml);
  TeleopConfig config;
  ASSERT_TRUE(ParseAmpHandTeleopFromControllerConfig(path, config));
  std::remove(path.c_str());

  EXPECT_TRUE(config.quit_squat.enabled);
  EXPECT_DOUBLE_EQ(config.quit_squat.squat_height, -0.15);
  EXPECT_DOUBLE_EQ(config.quit_squat.duration_sec, 1.5);

  // 缺失 quit_squat 段时保持默认关闭，不影响原有解析
  std::string yaml2 = R"(
HumanoidRobotCfg:
  env:
    amp_hand_posture_axis:
      enabled: true
)";
  std::string path2 = WriteTempYaml(yaml2);
  TeleopConfig config2;
  ASSERT_TRUE(ParseAmpHandTeleopFromControllerConfig(path2, config2));
  std::remove(path2.c_str());
  EXPECT_FALSE(config2.quit_squat.enabled);
}

// v17 实配文件可加载，LT/RT 预设齐全
TEST(JoyTriggerLayer_F6, RealV17BindingsLoad) {
#ifndef JOY_V17_BINDINGS_PATH
  GTEST_SKIP() << "JOY_V17_BINDINGS_PATH 未定义";
#else
  TeleopBindingConfig config;
  ASSERT_TRUE(config.loadFromFile(JOY_V17_BINDINGS_PATH))
      << "无法加载 v17 teleop_bindings.yaml";
  const auto& joy = config.getJoyConfig();

  // 8 个 LT/RT 预设动作均应存在且为 release 边沿
  const std::vector<std::vector<std::string>> presets = {
      {"LT", "A"}, {"LT", "B"}, {"LT", "X"}, {"LT", "Y"},
      {"RT", "A"}, {"RT", "B"}, {"RT", "X"}, {"RT", "Y"}};
  for (const auto& combo : presets) {
    ComboKey k; k.buttons = combo;
    const auto* b = joy.findBinding(k);
    ASSERT_NE(b, nullptr) << "缺少绑定 " << ComboKeyToString(k);
    EXPECT_EQ(b->edge, TriggerEdge::kRelease) << ComboKeyToString(k) << " 应为松开响应";
    EXPECT_EQ(b->action.type, ActionType::MotionCommand)
        << ComboKeyToString(k) << " 应为 MotionCommand";
    const auto* args = dynamic_cast<const MotionCommandArgs*>(b->action.args.get());
    ASSERT_NE(args, nullptr) << ComboKeyToString(k) << " 缺少 MotionCommand 参数";
    EXPECT_FALSE(args->motion_name.empty()) << ComboKeyToString(k) << " 动作名不能为空";
  }
#endif
}

// ============================================================================
// F7: LB+B 手臂模式 toggle（adapter 内部，复用 SetArmMode 接口）
// ============================================================================

TEST(JoyTriggerLayer_F7, ArmModeToggleEmitsToggleKeepPose) {
  TriggerBuffer tb;
  JoyTeleopAdapter adapter(&tb);
  LoadConfig(adapter, kEdgeYaml);  // LB+B 非配置绑定，由 adapter 内部处理

  JoyData empty = MakeBtn({});
  JoyData lb_b = MakeBtn({"LB", "B"});

  Feed(adapter, tb, empty, empty);  // prime

  // 每次 LB+B 都发 toggle_keep_pose，由 ControlLogic 基于实际模式解析，
  // adapter 不维护本地翻转状态（本地状态会与 TactPlayer 等改模式的组件失步）
  auto t1 = Feed(adapter, tb, lb_b, empty);
  ASSERT_EQ(t1.size(), 1u);
  EXPECT_EQ(t1[0].type, ActionType::SetArmMode);
  auto* args1 = dynamic_cast<NamedArgs*>(t1[0].args.get());
  ASSERT_NE(args1, nullptr);
  EXPECT_EQ(args1->name, "toggle_keep_pose");

  Feed(adapter, tb, empty, lb_b);  // 松开，不重复触发
  EXPECT_TRUE(Feed(adapter, tb, empty, empty).empty());

  // 第二次 LB+B → 同样是 toggle_keep_pose
  auto t2 = Feed(adapter, tb, lb_b, empty);
  ASSERT_EQ(t2.size(), 1u);
  EXPECT_EQ(t2[0].type, ActionType::SetArmMode);
  auto* args2 = dynamic_cast<NamedArgs*>(t2[0].args.get());
  ASSERT_NE(args2, nullptr);
  EXPECT_EQ(args2->name, "toggle_keep_pose");
}

// ============================================================================
// F6: RT 按住时 posture 门控（与头控轴互斥）
// ============================================================================

static void EnableAmpHandPosture(JoyTeleopAdapter& adapter) {
  adapter.config_.amp_hand_posture_axis.enabled = true;
  adapter.config_.amp_hand_posture_axis.axis_threshold = 0.4f;
  adapter.config_.amp_hand_posture_axis.squat_height_min = -0.21;
  adapter.config_.trigger_threshold = 0.50f;
}

TEST(JoyTriggerLayer_F6, RtBlocksPostureEnterWhileHeadControl) {
  TriggerBuffer tb;
  JoyTeleopAdapter adapter(&tb);
  LoadConfig(adapter, kEdgeYaml);
  EnableAmpHandPosture(adapter);

  JoyData neutral = MakeJoy(0, 0, 0, 0, 0, 0, {});
  JoyData rt_pitch = MakeJoy(0, 0, 0, -1.0f, 0, 0.8f, {});

  Feed(adapter, tb, neutral, neutral);
  EXPECT_FALSE(adapter.posture_control_mode_);

  Feed(adapter, tb, rt_pitch, neutral);
  EXPECT_FALSE(adapter.posture_control_mode_);
  EXPECT_EQ(adapter.getSnapshot().cmd_vel.cmd_stance_mode, 0);
  EXPECT_DOUBLE_EQ(adapter.getSnapshot().cmd_vel.angular_z, 0.0);
}

TEST(JoyTriggerLayer_F6, RtFreezesSquatHeightInPostureMode) {
  TriggerBuffer tb;
  JoyTeleopAdapter adapter(&tb);
  LoadConfig(adapter, kEdgeYaml);
  EnableAmpHandPosture(adapter);

  JoyData neutral = MakeJoy(0, 0, 0, 0, 0, 0, {});
  JoyData squat = MakeJoy(0, 0, 0, 1.0f, 0, 0, {});  // 下推为正 → 下蹲
  JoyData rt_move = MakeJoy(0, 0, 1.0f, -1.0f, 0, 0.8f, {});

  Feed(adapter, tb, neutral, neutral);
  Feed(adapter, tb, squat, neutral);
  ASSERT_TRUE(adapter.posture_control_mode_);
  const double squat_before_rt = adapter.getSnapshot().cmd_vel.angular_z;
  ASSERT_LT(squat_before_rt, -0.01);

  Feed(adapter, tb, rt_move, squat);
  EXPECT_TRUE(adapter.posture_control_mode_);
  EXPECT_EQ(adapter.getSnapshot().cmd_vel.cmd_stance_mode, 1);
  EXPECT_DOUBLE_EQ(adapter.getSnapshot().cmd_vel.angular_z, squat_before_rt);
  EXPECT_DOUBLE_EQ(adapter.frozen_squat_cmd_, squat_before_rt);
}

// ============================================================================
int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
