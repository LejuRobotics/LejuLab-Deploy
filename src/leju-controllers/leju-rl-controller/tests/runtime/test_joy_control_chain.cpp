/**
 * @file test_joy_control_chain.cpp
 * @brief 虚拟手柄 → 触发 → 控制链路/状态机 端到端集成测试
 *
 * 链路：虚拟 JoyData → JoyTeleopAdapter(真实) → TriggerBuffer
 *       → drain → ControlLogic / Lifecycle / ControllerManager(dummy 控制器)
 *
 * 验证三个状态机/路由：
 *   1. Lifecycle 生命周期状态机（START → Running，BACK+START → Exiting）
 *   2. 控制器切换状态机（X → SwitchController → requestSwitch → commitSwitch）
 *   3. Motion 触发路由（A → MotionCommand → ControllerManager.startMotion → 控制器）
 *
 * 不依赖 GlobalRobot/SDK/ONNX：用最小 DummyController 注册到 ControllerManager，
 * 通过 commitSwitch() 确定性推进切换状态机。
 */

#include <gtest/gtest.h>

#include <cstdio>
#include <string>
#include <vector>

#define private public
#define protected public
#include "leju-rl-controller/controllers/controller_manager.h"
#include "leju-rl-controller/rl/multi_mode_arm_controller.h"
#include "leju-rl-controller/robot_data.h"
#include "leju-rl-controller/runtime/control_loop.h"
#include "leju-rl-controller/runtime/control_logic.h"
#include "leju-rl-controller/runtime/input/teleop/joy_teleop_adapter.h"
#include "leju-rl-controller/runtime/input/trigger_buffer.h"
#include "leju-rl-controller/runtime/lifecycle.h"
#undef protected
#undef private

#include "lejusdk-lowlevel/data_types.h"

using namespace leju::runtime;
using leju::ControllerBase;
using leju::ControllerManager;
using leju::ControllerState;
using leju::ImuData;
using leju::JoyData;
using leju::RobotCmd;
using leju::RobotState;

// ============================================================================
// 最小 Dummy 控制器：实现 ControllerBase 纯虚接口，记录 startMotion 调用
// ============================================================================

namespace {

// 直立姿态 IMU（单位四元数），供 handleActionTriggers 倒地门控使用
ImuData MakeUprightImu() { return ImuData{}; }

class DummyController : public ControllerBase {
 public:
  explicit DummyController(std::string name) {
    name_ = std::move(name);
    state_ = ControllerState::kPaused;
  }

  bool initialize() override {
    state_ = ControllerState::kPaused;
    return true;
  }

  // 覆写无参 startMotion（基类虚函数），记录被调用
  bool startMotion() override {
    motion_started_ = true;
    return true;
  }

  bool motion_started_ = false;

 protected:
  bool updateImpl(double, const RobotState&, const ImuData&, RobotCmd&) override {
    return true;
  }
  bool loadPolicy(const std::string&) override { return true; }
  void computeObservation() override {}
  void computeActions() override {}
  void updateRobotCmd(RobotCmd&) override {}
};

// 构造 JoyData：摇杆 + 扳机 + 按键名列表
JoyData MakeJoy(const std::vector<std::string>& pressed, float lt = 0.0f, float rt = 0.0f) {
  JoyData joy{};
  joy.axes.left_trigger = lt;
  joy.axes.right_trigger = rt;
  for (const auto& b : pressed) {
    if (b == "A") joy.buttons.south = 1;
    else if (b == "B") joy.buttons.east = 1;
    else if (b == "X") joy.buttons.west = 1;
    else if (b == "Y") joy.buttons.north = 1;
    else if (b == "LB") joy.buttons.left_shoulder = 1;
    else if (b == "RB") joy.buttons.right_shoulder = 1;
    else if (b == "START") joy.buttons.start = 1;
    else if (b == "BACK") joy.buttons.back = 1;
  }
  return joy;
}

bool LoadYaml(JoyTeleopAdapter& adapter, const std::string& yaml, const char* path) {
  FILE* f = fopen(path, "w");
  if (!f) return false;
  fprintf(f, "%s", yaml.c_str());
  fclose(f);
  bool ok = adapter.loadBindingConfig(path);
  remove(path);
  return ok;
}

// 端到端绑定：X→切AMP, A→无名Motion, RT+B→松开Motion
const char* kE2EYaml = R"(
velocity_limits:
  stick_deadzone: 0.05
  linear_x: 0.6
  linear_y: 0.6
  angular_z: 1.0
  trigger_threshold: 0.5
joy_bindings:
  - buttons: ["X"]
    action:
      type: SwitchController
      args:
        name: amp
  - buttons: ["A"]
    action:
      type: MotionCommand
      args:
        op: Start
)";

// 顺序喂帧驱动器（维护 prev buttons）
class Feeder {
 public:
  explicit Feeder(JoyTeleopAdapter& a) : adapter_(a) {}
  void feed(const JoyData& joy) {
    adapter_.onJoyData(joy, prev_);
    prev_ = joy.buttons;
  }
 private:
  JoyTeleopAdapter& adapter_;
  JoyData::Buttons prev_{};
};

}  // namespace

// ============================================================================
// 1. Lifecycle 生命周期状态机（虚拟 START / BACK+START）
// ============================================================================

TEST(JoyControlChain, LifecycleStartThenQuitViaVirtualJoy) {
  TriggerBuffer tb;
  JoyTeleopAdapter adapter(&tb);
  ASSERT_TRUE(LoadYaml(adapter, kE2EYaml, "/tmp/test_jcc_life.yaml"));
  Lifecycle lifecycle;
  Feeder feeder(adapter);

  // 初始：等待就绪
  EXPECT_EQ(lifecycle.state(), LifecycleState::kWaitingForReady);

  // ready=true，中性帧 → 等待启动
  feeder.feed(MakeJoy({}));
  lifecycle.update(true, tb.drainAll());
  EXPECT_EQ(lifecycle.state(), LifecycleState::kWaitingForStart);

  // 虚拟 START → Running
  feeder.feed(MakeJoy({"START"}));
  auto start_triggers = tb.drainAll();
  ASSERT_FALSE(start_triggers.empty());
  EXPECT_EQ(start_triggers[0].type, ActionType::Start);
  lifecycle.update(true, start_triggers);
  EXPECT_TRUE(lifecycle.isRunning());

  // 虚拟 BACK+START → Quit → Exiting
  // （quit_squat 默认 enabled=false：保持「按下即 Quit」原行为，回归用）
  feeder.feed(MakeJoy({"BACK", "START"}));
  auto quit_triggers = tb.drainAll();
  ASSERT_FALSE(quit_triggers.empty());
  EXPECT_EQ(quit_triggers[0].type, ActionType::Quit);
  lifecycle.update(true, quit_triggers);
  EXPECT_TRUE(lifecycle.shouldExit());
}

// ============================================================================
// 1.1 BACK+START 先下蹲再退出（适配器只发 QuitSquat，ControlLoop 执行时序）
// ============================================================================

TEST(JoyControlChain, QuitSquatEmitsTriggerFromAdapter) {
  TriggerBuffer tb;
  JoyTeleopAdapter adapter(&tb);
  ASSERT_TRUE(LoadYaml(adapter, kE2EYaml, "/tmp/test_jcc_quitsquat_emit.yaml"));
  adapter.config_.quit_squat.enabled = true;
  adapter.config_.quit_squat.squat_height = -0.15;
  adapter.config_.quit_squat.duration_sec = 1.5;
  Feeder feeder(adapter);

  // 按下 BACK+START：适配器只发 QuitSquat 请求（含参数），不写 cmd_buffer、不管理时序
  feeder.feed(MakeJoy({"BACK", "START"}));
  auto triggers = tb.drainAll();
  ASSERT_FALSE(triggers.empty());
  EXPECT_EQ(triggers[0].type, ActionType::QuitSquat);
  const auto* args = dynamic_cast<const QuitSquatArgs*>(triggers[0].args.get());
  ASSERT_NE(args, nullptr);
  EXPECT_DOUBLE_EQ(args->squat_height, -0.15);
  EXPECT_DOUBLE_EQ(args->duration_sec, 1.5);

  // 适配器不写本地下蹲状态（无 cmd_stance 覆盖）
  EXPECT_FALSE(adapter.getSnapshot().cmd_vel.cmd_stance_valid);
}

namespace {

// 构造最小 ControlLoop（不跑 tick，直接测 QuitSquat 消费/覆写/超时方法）
struct QuitSquatLoopFixture {
  TriggerBuffer tb;
  leju::RobotData rd;
  Lifecycle lifecycle;
  ControllerManager mgr;
  std::vector<InputSource*> sources;
  ControlLoop loop;

  QuitSquatLoopFixture() : loop(rd, tb, sources, lifecycle, mgr) {}
};

}  // namespace

TEST(JoyControlChain, ConsumeQuitSquatActivatesAndIgnoresDuplicate) {
  QuitSquatLoopFixture f;
  // 下蹲序列仅在「运行态 + AMP」激活：先进 kRunning，再挂 amp 为活跃控制器
  f.lifecycle.update(true, {});
  f.lifecycle.update(true, {ActionTrigger(ActionType::Start)});
  ASSERT_TRUE(f.lifecycle.allowsControlOutput());
  f.mgr.addController("amp", std::make_unique<DummyController>("amp"));
  f.mgr.active_index_ = 0;
  ASSERT_EQ(f.mgr.getCurrentControllerName(), "amp");

  double now = 100.0;
  std::vector<ActionTrigger> triggers{MakeQuitSquatTrigger(-0.15, 1.5)};
  f.loop.consumeQuitSquatTriggers(triggers, now);
  EXPECT_TRUE(f.loop.quit_squat_active_);
  EXPECT_DOUBLE_EQ(f.loop.quit_squat_height_, -0.15);
  EXPECT_DOUBLE_EQ(f.loop.quit_squat_deadline_, 101.5);
  EXPECT_TRUE(triggers.empty());  // QuitSquat 已消费

  // 重复 QuitSquat：忽略，不刷新 deadline（防重按续期）
  std::vector<ActionTrigger> triggers2{MakeQuitSquatTrigger(-0.2, 5.0)};
  f.loop.consumeQuitSquatTriggers(triggers2, now + 0.5);
  EXPECT_DOUBLE_EQ(f.loop.quit_squat_deadline_, 101.5);
  EXPECT_TRUE(triggers2.empty());
}

// 未运行（就绪/等待启动）：BACK+START 走原「立即退出」路径，不下蹲
TEST(JoyControlChain, ConsumeQuitSquatDegradesToImmediateQuitWhenNotRunning) {
  QuitSquatLoopFixture f;  // lifecycle 默认 kWaitingForReady → 未运行

  std::vector<ActionTrigger> triggers{MakeQuitSquatTrigger(-0.15, 1.5)};
  f.loop.consumeQuitSquatTriggers(triggers, 100.0);
  EXPECT_FALSE(f.loop.quit_squat_active_);          // 不激活下蹲
  ASSERT_EQ(triggers.size(), 1u);                    // 降级为立即 Quit
  EXPECT_EQ(triggers[0].type, ActionType::Quit);
}

// 运行态但非 AMP（舞蹈/搬运/倒地）：同样立即退出，不空等 1.5s
TEST(JoyControlChain, ConsumeQuitSquatDegradesToImmediateQuitWhenRunningNonAmp) {
  QuitSquatLoopFixture f;
  f.lifecycle.update(true, {});
  f.lifecycle.update(true, {ActionTrigger(ActionType::Start)});
  ASSERT_TRUE(f.lifecycle.allowsControlOutput());
  f.mgr.addController("mimic_dance", std::make_unique<DummyController>("mimic_dance"));
  f.mgr.active_index_ = 0;  // 非 AMP 活跃

  std::vector<ActionTrigger> triggers{MakeQuitSquatTrigger(-0.15, 1.5)};
  f.loop.consumeQuitSquatTriggers(triggers, 100.0);
  EXPECT_FALSE(f.loop.quit_squat_active_);          // 不激活下蹲
  ASSERT_EQ(triggers.size(), 1u);                    // 立即退出
  EXPECT_EQ(triggers[0].type, ActionType::Quit);
}

TEST(JoyControlChain, ConsumeQuitSquatQuitWinsAndInvalidParamsDegradeToQuit) {
  QuitSquatLoopFixture f;

  // 同批 Quit + QuitSquat：Quit 优先，消费掉 QuitSquat，保留 Quit
  std::vector<ActionTrigger> triggers{MakeQuitTrigger(), MakeQuitSquatTrigger(-0.15, 1.5)};
  f.loop.consumeQuitSquatTriggers(triggers, 100.0);
  EXPECT_FALSE(f.loop.quit_squat_active_);
  ASSERT_EQ(triggers.size(), 1u);
  EXPECT_EQ(triggers[0].type, ActionType::Quit);

  // 无 args：fail-safe 降级为立即 Quit（避免 runtime 不退、等 monitor 8s 强杀）
  std::vector<ActionTrigger> bad1{ActionTrigger(ActionType::QuitSquat)};
  f.loop.consumeQuitSquatTriggers(bad1, 200.0);
  EXPECT_FALSE(f.loop.quit_squat_active_);
  ASSERT_EQ(bad1.size(), 1u);
  EXPECT_EQ(bad1[0].type, ActionType::Quit);

  // duration<=0：降级为立即 Quit
  std::vector<ActionTrigger> bad2{MakeQuitSquatTrigger(-0.15, 0.0)};
  f.loop.consumeQuitSquatTriggers(bad2, 200.0);
  EXPECT_FALSE(f.loop.quit_squat_active_);
  ASSERT_EQ(bad2.size(), 1u);
  EXPECT_EQ(bad2[0].type, ActionType::Quit);

  // squat_height>=0（非下蹲）：降级为立即 Quit
  std::vector<ActionTrigger> bad3{MakeQuitSquatTrigger(0.05, 1.5)};
  f.loop.consumeQuitSquatTriggers(bad3, 200.0);
  EXPECT_FALSE(f.loop.quit_squat_active_);
  ASSERT_EQ(bad3.size(), 1u);
  EXPECT_EQ(bad3[0].type, ActionType::Quit);
}

TEST(JoyControlChain, ApplyQuitSquatOverrideSetsSquatCommand) {
  QuitSquatLoopFixture f;
  f.loop.quit_squat_active_ = true;
  f.loop.quit_squat_height_ = -0.15;

  MotionCommand cmd;
  cmd.setZero();
  f.loop.applyQuitSquatOverride(cmd);
  EXPECT_TRUE(cmd.valid);
  EXPECT_TRUE(cmd.cmd_stance_valid);
  EXPECT_EQ(cmd.cmd_stance_mode, 1);
  EXPECT_DOUBLE_EQ(cmd.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(cmd.linear_y, 0.0);
  EXPECT_DOUBLE_EQ(cmd.angular_z, -0.15);
}

TEST(JoyControlChain, MaybeExpireQuitSquatPushesQuitAfterDeadline) {
  QuitSquatLoopFixture f;
  f.loop.quit_squat_active_ = true;
  f.loop.quit_squat_deadline_ = 101.5;

  // 未到期：不推 Quit
  std::vector<ActionTrigger> triggers;
  f.loop.maybeExpireQuitSquat(101.4, triggers);
  EXPECT_TRUE(triggers.empty());
  EXPECT_TRUE(f.loop.quit_squat_active_);

  // 到期：本 tick 本地 triggers 加 Quit，状态复位
  f.loop.maybeExpireQuitSquat(101.5, triggers);
  ASSERT_EQ(triggers.size(), 1u);
  EXPECT_EQ(triggers[0].type, ActionType::Quit);
  EXPECT_FALSE(f.loop.quit_squat_active_);
}

// 状态数据不可用时保持 beta 的先读状态语义：请求留在 TriggerBuffer，等待状态恢复。
TEST(JoyControlChain, TickDefersQuitSquatRequestWhenStateUnavailable) {
  QuitSquatLoopFixture f;
  f.tb.push(MakeQuitSquatTrigger(-0.15, 1.5));
  f.loop.tick();
  EXPECT_FALSE(f.loop.quit_squat_active_);
  auto triggers = f.tb.drainAll();
  ASSERT_EQ(triggers.size(), 1u);
  EXPECT_EQ(triggers[0].type, ActionType::QuitSquat);
}

// 真实 tick() 路径：状态不可用 + deadline 已到期 → 本 tick 直接退出（Fix1）
TEST(JoyControlChain, TickExitsViaQuitSquatEvenWhenStateUnavailable) {
  QuitSquatLoopFixture f;
  f.loop.quit_squat_active_ = true;
  f.loop.quit_squat_deadline_ = -1.0;  // 已到期（过去时间）
  f.loop.tick();
  EXPECT_TRUE(f.lifecycle.shouldExit());  // 走 kExiting 退出路径
}

// ============================================================================
// 2. 控制器切换状态机（虚拟 X → SwitchController → commitSwitch）
// ============================================================================

TEST(JoyControlChain, SwitchControllerStateMachineViaVirtualJoy) {
  TriggerBuffer tb;
  JoyTeleopAdapter adapter(&tb);
  ASSERT_TRUE(LoadYaml(adapter, kE2EYaml, "/tmp/test_jcc_switch.yaml"));

  // 注册 dummy 控制器，初始活跃为 mimic_dance
  ControllerManager mgr;
  mgr.addController("amp", std::make_unique<DummyController>("amp"));
  mgr.addController("mimic_dance", std::make_unique<DummyController>("mimic_dance"));
  mgr.active_index_ = 1;  // mimic_dance
  ASSERT_EQ(mgr.getCurrentControllerName(), "mimic_dance");

  Lifecycle lifecycle;
  ControlLogic logic;
  Feeder feeder(adapter);

  // 虚拟按下 X → 产出 SwitchController(amp)
  feeder.feed(MakeJoy({}));
  feeder.feed(MakeJoy({"X"}));
  auto triggers = tb.drainAll();
  ASSERT_EQ(triggers.size(), 1u);
  EXPECT_EQ(triggers[0].type, ActionType::SwitchController);

  // 经 ControlLogic 路由到 ControllerManager → 进入切换过渡
  CommandBuffer::Snapshot snap;
  logic.handleActionTriggers(MakeUprightImu(), triggers, lifecycle, mgr, snap, 0.0);
  EXPECT_TRUE(mgr.isTransitioning());
  EXPECT_EQ(mgr.transition_.to_controller, "amp");

  // 提交切换 → 活跃控制器变为 amp
  mgr.commitSwitch();
  EXPECT_FALSE(mgr.isTransitioning());
  EXPECT_EQ(mgr.getCurrentControllerName(), "amp");
}

// ============================================================================
// 3. Motion 触发路由到控制器（虚拟 A → MotionCommand → startMotion）
// ============================================================================

TEST(JoyControlChain, MotionCommandRoutesToActiveControllerViaVirtualJoy) {
  TriggerBuffer tb;
  JoyTeleopAdapter adapter(&tb);
  ASSERT_TRUE(LoadYaml(adapter, kE2EYaml, "/tmp/test_jcc_motion.yaml"));

  ControllerManager mgr;
  mgr.addController("amp", std::make_unique<DummyController>("amp"));
  mgr.active_index_ = 0;

  auto* dummy = dynamic_cast<DummyController*>(mgr.getCurrentController());
  ASSERT_NE(dummy, nullptr);
  EXPECT_FALSE(dummy->motion_started_);

  Lifecycle lifecycle;
  ControlLogic logic;
  Feeder feeder(adapter);

  // 虚拟按下 A → 无名 MotionCommand
  feeder.feed(MakeJoy({}));
  feeder.feed(MakeJoy({"A"}));
  auto triggers = tb.drainAll();
  ASSERT_EQ(triggers.size(), 1u);
  EXPECT_EQ(triggers[0].type, ActionType::MotionCommand);

  // 经 ControlLogic 路由 → ControllerManager.startMotion() → 控制器记录
  CommandBuffer::Snapshot snap;
  logic.handleActionTriggers(MakeUprightImu(), triggers, lifecycle, mgr, snap, 0.0);
  EXPECT_TRUE(dummy->motion_started_);
}

// ============================================================================
// 4. 完整链路串联：切换 → Motion，状态机连续正确
// ============================================================================

TEST(JoyControlChain, FullChainSwitchThenMotion) {
  TriggerBuffer tb;
  JoyTeleopAdapter adapter(&tb);
  ASSERT_TRUE(LoadYaml(adapter, kE2EYaml, "/tmp/test_jcc_full.yaml"));

  ControllerManager mgr;
  mgr.addController("amp", std::make_unique<DummyController>("amp"));
  mgr.addController("mimic_dance", std::make_unique<DummyController>("mimic_dance"));
  mgr.active_index_ = 1;  // 起始 mimic_dance

  Lifecycle lifecycle;
  ControlLogic logic;
  Feeder feeder(adapter);
  CommandBuffer::Snapshot snap;

  // X → 切到 amp 并提交
  feeder.feed(MakeJoy({}));
  feeder.feed(MakeJoy({"X"}));
  logic.handleActionTriggers(MakeUprightImu(), tb.drainAll(), lifecycle, mgr, snap, 0.0);
  mgr.commitSwitch();
  ASSERT_EQ(mgr.getCurrentControllerName(), "amp");

  // A → 在 amp 上触发 Motion
  feeder.feed(MakeJoy({}));
  feeder.feed(MakeJoy({"A"}));
  logic.handleActionTriggers(MakeUprightImu(), tb.drainAll(), lifecycle, mgr, snap, 0.0);
  auto* amp = dynamic_cast<DummyController*>(mgr.getCurrentController());
  ASSERT_NE(amp, nullptr);
  EXPECT_TRUE(amp->motion_started_);
}

// ============================================================================
// 5. MotionCommand 携带 music → 触发注入的音乐播放回调
// ============================================================================

TEST(JoyControlChain, MotionCommandMusicCallbackFires) {
  ControllerManager mgr;
  mgr.addController("amp", std::make_unique<DummyController>("amp"));
  mgr.active_index_ = 0;

  Lifecycle lifecycle;
  ControlLogic logic;
  CommandBuffer::Snapshot snap;

  std::vector<std::string> played;
  logic.setMusicPlayer([&](const std::string& m) { played.push_back(m); });

  // 带 music 的 MotionCommand → 回调收到文件名
  std::vector<ActionTrigger> with_music = {
      MakeMotionCommandTrigger(MotionCommandArgs::Operation::Start, "newdance", "dance.wav")};
  logic.handleActionTriggers(MakeUprightImu(), with_music, lifecycle, mgr, snap, 0.0);
  ASSERT_EQ(played.size(), 1u);
  EXPECT_EQ(played[0], "dance.wav");

  // 不带 music 的 MotionCommand → 回调不触发
  std::vector<ActionTrigger> no_music = {
      MakeMotionCommandTrigger(MotionCommandArgs::Operation::Start, "newdance", "")};
  logic.handleActionTriggers(MakeUprightImu(), no_music, lifecycle, mgr, snap, 0.0);
  EXPECT_EQ(played.size(), 1u);  // 未增加
}

// 未注入回调时，带 music 的 MotionCommand 不应崩溃（退化为日志）
TEST(JoyControlChain, MotionCommandMusicNoCallbackNoCrash) {
  ControllerManager mgr;
  mgr.addController("amp", std::make_unique<DummyController>("amp"));
  mgr.active_index_ = 0;

  Lifecycle lifecycle;
  ControlLogic logic;  // 未注入 music_player_
  CommandBuffer::Snapshot snap;

  std::vector<ActionTrigger> with_music = {
      MakeMotionCommandTrigger(MotionCommandArgs::Operation::Start, "newdance", "dance.wav")};
  // 未注入 music_player_ 时也应安全返回（不崩溃、无 DDS 依赖）
  logic.handleActionTriggers(MakeUprightImu(), with_music, lifecycle, mgr, snap, 0.0);
  SUCCEED();
}

// 带 music_delay 的 MotionCommand → 延迟到点才触发（对齐舞蹈起播）
TEST(JoyControlChain, MotionCommandMusicDelayedFires) {
  ControllerManager mgr;
  mgr.addController("amp", std::make_unique<DummyController>("amp"));
  mgr.active_index_ = 0;

  Lifecycle lifecycle;
  ControlLogic logic;
  CommandBuffer::Snapshot snap;

  std::vector<std::string> played;
  logic.setMusicPlayer([&](const std::string& m) { played.push_back(m); });

  // 延迟 1.0s 起播，now=10.0 → 登记到 11.0
  std::vector<ActionTrigger> trig = {MakeMotionCommandTrigger(
      MotionCommandArgs::Operation::Start, "newdance", "dance.wav", 1.0)};
  logic.handleActionTriggers(MakeUprightImu(), trig, lifecycle, mgr, snap, 10.0);
  EXPECT_TRUE(played.empty());  // 触发瞬间不播

  logic.firePendingMusic(10.5);
  EXPECT_TRUE(played.empty());  // 未到点仍不播

  logic.firePendingMusic(11.0);
  ASSERT_EQ(played.size(), 1u);  // 到点播放
  EXPECT_EQ(played[0], "dance.wav");

  // 到点后不再重复触发
  logic.firePendingMusic(12.0);
  EXPECT_EQ(played.size(), 1u);
}

// ============================================================================
// 6. 共享启动键 + 按控制器配置的配乐回退（多支舞蹈复用同一个播放键）
// ============================================================================

// 触发器未带 music 时，回退到当前激活控制器在 controller_manager.yaml 里配置的 music
TEST(JoyControlChain, MotionCommandFallsBackToControllerMusic) {
  ControllerManager mgr;
  mgr.addController("mimic_dance", std::make_unique<DummyController>("mimic_dance"));
  mgr.active_index_ = 0;
  mgr.setControllerMusic("mimic_dance", "dance_lonelydance.wav", 0.0);

  Lifecycle lifecycle;
  ControlLogic logic;
  CommandBuffer::Snapshot snap;

  std::vector<std::string> played;
  logic.setMusicPlayer([&](const std::string& m) { played.push_back(m); });

  // 不带 music/motion_name 的共享启动键（对应 RB+B）
  std::vector<ActionTrigger> trig = {
      MakeMotionCommandTrigger(MotionCommandArgs::Operation::Start, "", "")};
  logic.handleActionTriggers(MakeUprightImu(), trig, lifecycle, mgr, snap, 0.0);
  ASSERT_EQ(played.size(), 1u);
  EXPECT_EQ(played[0], "dance_lonelydance.wav");
}

// 触发器显式指定 music 时，优先于控制器配置的回退值
TEST(JoyControlChain, MotionCommandExplicitMusicOverridesControllerFallback) {
  ControllerManager mgr;
  mgr.addController("mimic_dance", std::make_unique<DummyController>("mimic_dance"));
  mgr.active_index_ = 0;
  mgr.setControllerMusic("mimic_dance", "dance_lonelydance.wav", 0.0);

  Lifecycle lifecycle;
  ControlLogic logic;
  CommandBuffer::Snapshot snap;

  std::vector<std::string> played;
  logic.setMusicPlayer([&](const std::string& m) { played.push_back(m); });

  std::vector<ActionTrigger> trig = {
      MakeMotionCommandTrigger(MotionCommandArgs::Operation::Start, "", "explicit.wav")};
  logic.handleActionTriggers(MakeUprightImu(), trig, lifecycle, mgr, snap, 0.0);
  ASSERT_EQ(played.size(), 1u);
  EXPECT_EQ(played[0], "explicit.wav");
}

// 当前控制器未配置 music 时，共享启动键不应播放任何东西（无回退可用）
TEST(JoyControlChain, MotionCommandNoFallbackWhenControllerHasNoMusic) {
  ControllerManager mgr;
  mgr.addController("amp", std::make_unique<DummyController>("amp"));
  mgr.active_index_ = 0;  // 未调用 setControllerMusic

  Lifecycle lifecycle;
  ControlLogic logic;
  CommandBuffer::Snapshot snap;

  std::vector<std::string> played;
  logic.setMusicPlayer([&](const std::string& m) { played.push_back(m); });

  std::vector<ActionTrigger> trig = {
      MakeMotionCommandTrigger(MotionCommandArgs::Operation::Start, "", "")};
  logic.handleActionTriggers(MakeUprightImu(), trig, lifecycle, mgr, snap, 0.0);
  EXPECT_TRUE(played.empty());
}

// ============================================================================
// LB+B 冻结翻转基于实际模式解析（toggle_keep_pose）
//
// 回归场景：tact 播放（kExternal）中按 LB+B，若翻转基于手柄侧本地状态，
// 状态与真实模式失步时会误发 auto → 手臂立刻复位。修复后由 ControlLogic
// 查询 getCurrentArmMode()：非 kKeepPose 一律冻结，kKeepPose 才解锁。
// ============================================================================

namespace {

// 带手臂控制器的 Dummy：LB+B 链路需要 MultiModeArmController 实例
class ArmDummyController : public DummyController {
 public:
  explicit ArmDummyController(std::string name) : DummyController(std::move(name)) {
    leju::MultiModeArmControllerConfig cfg;
    arm_controller_ = std::make_unique<leju::MultiModeArmController>(cfg);
    arm_controller_->init(4, Eigen::VectorXd::Zero(4), 0.001);
  }
};

// 按下一次 LB+B 并经 ControlLogic 路由
void PressLbB(Feeder& feeder, TriggerBuffer& tb, ControlLogic& logic,
              Lifecycle& lifecycle, ControllerManager& mgr) {
  feeder.feed(MakeJoy({}));
  feeder.feed(MakeJoy({"LB", "B"}));
  auto triggers = tb.drainAll();
  ASSERT_EQ(triggers.size(), 1u);
  ASSERT_EQ(triggers[0].type, ActionType::SetArmMode);
  CommandBuffer::Snapshot snap;
  logic.handleActionTriggers(MakeUprightImu(), triggers, lifecycle, mgr, snap, 0.0);
}

}  // namespace

TEST(JoyControlChain, LbBFreezesEvenAfterModeChangedElsewhere) {
  TriggerBuffer tb;
  JoyTeleopAdapter adapter(&tb);
  ASSERT_TRUE(LoadYaml(adapter, kE2EYaml, "/tmp/test_jcc_lbb_ext.yaml"));

  ControllerManager mgr;
  mgr.addController("amp", std::make_unique<ArmDummyController>("amp"));
  mgr.active_index_ = 0;

  int freeze_count = 0;
  mgr.setArmFreezeCallback([&]() { ++freeze_count; });

  Lifecycle lifecycle;
  ControlLogic logic;
  Feeder feeder(adapter);
  std::string msg;

  // 还原用户实测序列：
  // 1. tact A 播放中（kExternal）按 LB+B → 冻结
  ASSERT_TRUE(mgr.setArmMode(leju::ArmControlMode::kExternal, msg));
  PressLbB(feeder, tb, logic, lifecycle, mgr);
  ASSERT_EQ(mgr.getCurrentArmMode(), leju::ArmControlMode::kKeepPose);
  EXPECT_EQ(freeze_count, 1);

  // 2. 再播放 tact B：TactPlayer 切 External，播完自动恢复 Auto
  //    （手柄侧对这两次模式变化一无所知）
  ASSERT_TRUE(mgr.setArmMode(leju::ArmControlMode::kExternal, msg));
  ASSERT_TRUE(mgr.setArmMode(leju::ArmControlMode::kAuto, msg));

  // 3. tact C 播放中（kExternal）再按 LB+B → 必须仍是冻结。
  //    旧实现基于手柄本地翻转状态，此时会误发 auto → 手臂立刻复位。
  ASSERT_TRUE(mgr.setArmMode(leju::ArmControlMode::kExternal, msg));
  PressLbB(feeder, tb, logic, lifecycle, mgr);
  EXPECT_EQ(mgr.getCurrentArmMode(), leju::ArmControlMode::kKeepPose);
  EXPECT_EQ(freeze_count, 2);
}

TEST(JoyControlChain, LbBTogglesBetweenKeepPoseAndAuto) {
  TriggerBuffer tb;
  JoyTeleopAdapter adapter(&tb);
  ASSERT_TRUE(LoadYaml(adapter, kE2EYaml, "/tmp/test_jcc_lbb_toggle.yaml"));

  ControllerManager mgr;
  mgr.addController("amp", std::make_unique<ArmDummyController>("amp"));
  mgr.active_index_ = 0;
  ASSERT_EQ(mgr.getCurrentArmMode(), leju::ArmControlMode::kAuto);

  Lifecycle lifecycle;
  ControlLogic logic;
  Feeder feeder(adapter);

  // kAuto 下第一次 LB+B → 冻结
  PressLbB(feeder, tb, logic, lifecycle, mgr);
  EXPECT_EQ(mgr.getCurrentArmMode(), leju::ArmControlMode::kKeepPose);

  // kKeepPose 下第二次 LB+B → 解锁回 kAuto
  PressLbB(feeder, tb, logic, lifecycle, mgr);
  EXPECT_EQ(mgr.getCurrentArmMode(), leju::ArmControlMode::kAuto);

  // 外部组件把模式改走（模拟 TactPlayer 播完恢复流程），再按 LB+B 仍是冻结
  std::string msg;
  ASSERT_TRUE(mgr.setArmMode(leju::ArmControlMode::kExternal, msg));
  PressLbB(feeder, tb, logic, lifecycle, mgr);
  EXPECT_EQ(mgr.getCurrentArmMode(), leju::ArmControlMode::kKeepPose);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
