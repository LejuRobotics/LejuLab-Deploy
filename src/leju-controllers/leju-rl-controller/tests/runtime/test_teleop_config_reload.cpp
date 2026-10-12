/**
 * @file test_teleop_config_reload.cpp
 * @brief 遥控器绑定配置热重载 (kReloadTeleopConfig) 验收测试
 *
 * 覆盖:
 *  - VT1  热重载后绑定被替换, 且 MotionCommand 的可选 music 字段被解析 (T5 钩子)
 *  - VT1b 未曾成功加载过配置时 reloadBindingConfig 返回 false
 *  - VT2  joy 帧读 (onJoyData) 与热重载写 (loadBindingConfig) 并发无崩溃
 *         建议在 ThreadSanitizer 下运行验证无 data race:
 *           cmake -DCMAKE_CXX_FLAGS="-fsanitize=thread -g" ... && ./test_teleop_config_reload
 *  - VT3  热重载不丢启动时从 AMP 控制器配置合并的 amp_hand 参数 (issue #4225):
 *         桌面端改键触发 reload 后, 下蹲/posture/挡位/退出下蹲仍生效
 *  - VT3b 未合并过 AMP 控制器配置时, 热重载不会凭空生成 amp_hand 参数
 */

#include <gtest/gtest.h>

#include <atomic>
#include <cstdio>
#include <fstream>
#include <string>
#include <thread>

#define private public
#define protected public
#include "leju-rl-controller/runtime/input/teleop/joy_teleop_adapter.h"
#include "leju-rl-controller/runtime/input/teleop/teleop_input_source.h"
#include "leju-rl-controller/runtime/input/teleop/teleop_binding_config.h"
#include "leju-rl-controller/runtime/input/trigger_buffer.h"
#undef protected
#undef private

#include "lejusdk-lowlevel/data_types.h"
#include "lejusdk-utils/robot_version.hpp"

using namespace leju::runtime;

namespace {

void WriteFile(const std::string& path, const std::string& content) {
  std::ofstream ofs(path, std::ios::trunc);
  ofs << content;
}

// X -> SwitchController amp
const char* kYamlA =
    "velocity_limits:\n"
    "  stick_deadzone: 0.05\n"
    "  linear_x: 0.60\n"
    "  linear_y: 0.00\n"
    "  angular_z: 0.60\n"
    "\n"
    "joy_bindings:\n"
    "  - buttons: [\"X\"]\n"
    "    action:\n"
    "      type: SwitchController\n"
    "      args:\n"
    "        name: amp\n";

// X -> MotionCommand start dance_b + music
const char* kYamlB =
    "velocity_limits:\n"
    "  stick_deadzone: 0.05\n"
    "  linear_x: 0.60\n"
    "  linear_y: 0.00\n"
    "  angular_z: 0.60\n"
    "\n"
    "joy_bindings:\n"
    "  - buttons: [\"X\"]\n"
    "    action:\n"
    "      type: MotionCommand\n"
    "      args:\n"
    "        op: Start\n"
    "        name: dance_b\n"
    "        music: dance_b.wav\n";

ComboKey ComboX() {
  ComboKey c;
  c.buttons.push_back("X");
  return c;
}

// 最小 AMP 控制器配置：只保留 ParseAmpHandTeleopFromControllerConfig 关心的 env 段。
// 这些参数不在 teleop_bindings.yaml 里, 启动时由 main 从控制器配置合并进 adapter 的
// TeleopConfig; 桌面端改键触发的热重载必须把它们保住 (issue #4225)。
const char* kAmpControllerYaml =
    "HumanoidRobotCfg:\n"
    "  env:\n"
    "    velocity_scale:\n"
    "      walking:\n"
    "        linear_x: 0.75\n"
    "        linear_x_low: 0.4\n"
    "        linear_x_up: 0.95\n"
    "        linear_x_negative_scale: 0.45\n"
    "        angular_z: 1.2\n"
    "    velocity_entry_neutral_guard:\n"
    "      enabled: true\n"
    "      neutral_frames: 5\n"
    "    amp_hand_posture_axis:\n"
    "      enabled: true\n"
    "      axis_threshold: 0.3\n"
    "      squat_height_min: -0.25\n"
    "    quit_squat:\n"
    "      enabled: true\n"
    "      squat_height: -0.15\n"
    "      duration_sec: 1.5\n";

}  // namespace

// VT1: 热重载切换绑定 + music 字段解析
TEST(TeleopConfigReload, ReloadSwitchesBindingAndParsesMusic) {
  const std::string path = "/tmp/test_teleop_reload_vt1.yaml";
  WriteFile(path, kYamlA);

  TriggerBuffer tb;
  TeleopInputSource src(leju::RobotVersions::KUAVO5_BASE, &tb);

  ASSERT_TRUE(src.loadBindingConfig(path));
  {
    const TeleopBinding* b =
        src.joy_adapter_->binding_config_.getJoyConfig().findBinding(ComboX());
    ASSERT_NE(b, nullptr);
    EXPECT_EQ(b->action.type, ActionType::SwitchController);
  }

  // 改写配置文件后热重载, 不重建对象
  WriteFile(path, kYamlB);
  ASSERT_TRUE(src.reloadBindingConfig());
  {
    const TeleopBinding* b =
        src.joy_adapter_->binding_config_.getJoyConfig().findBinding(ComboX());
    ASSERT_NE(b, nullptr);
    EXPECT_EQ(b->action.type, ActionType::MotionCommand);
    auto* args = dynamic_cast<MotionCommandArgs*>(b->action.args.get());
    ASSERT_NE(args, nullptr);
    EXPECT_EQ(args->motion_name, "dance_b");
    EXPECT_EQ(args->music, "dance_b.wav");  // T5: music 钩子字段被解析
  }

  std::remove(path.c_str());
}

// VT1b: 未加载过配置时 reload 优雅失败
TEST(TeleopConfigReload, ReloadWithoutPriorLoadFails) {
  TriggerBuffer tb;
  TeleopInputSource src(leju::RobotVersions::KUAVO5_BASE, &tb);
  EXPECT_FALSE(src.reloadBindingConfig());
}

// VT2: joy 帧读 与 热重载写 并发安全 (无崩溃; 建议 TSAN 验证)
TEST(TeleopConfigReload, ConcurrentReadWhileReloadingIsSafe) {
  const std::string path_a = "/tmp/test_teleop_reload_vt2_a.yaml";
  const std::string path_b = "/tmp/test_teleop_reload_vt2_b.yaml";
  WriteFile(path_a, kYamlA);
  WriteFile(path_b, kYamlB);

  TriggerBuffer tb;
  TeleopInputSource src(leju::RobotVersions::KUAVO5_BASE, &tb);
  ASSERT_TRUE(src.loadBindingConfig(path_a));

  std::atomic<bool> stop{false};

  // 读线程: 模拟 joy 回调逐帧处理, 交替按下/松开 X 以每帧都触发 binding 查找
  std::thread reader([&]() {
    leju::JoyData joy{};
    leju::JoyData::Buttons prev{};
    int frame = 0;
    while (!stop.load(std::memory_order_relaxed)) {
      joy.buttons.west = (frame++ & 1);  // 切换 X 制造组合键边沿
      src.joy_adapter_->onJoyData(joy, prev);
      prev = joy.buttons;
    }
  });

  // 写线程: 反复热重载, A/B 交替替换 binding_config_
  std::thread writer([&]() {
    for (int i = 0; i < 3000; ++i) {
      src.loadBindingConfig((i & 1) ? path_b : path_a);
    }
    stop.store(true, std::memory_order_relaxed);
  });

  writer.join();
  reader.join();

  std::remove(path_a.c_str());
  std::remove(path_b.c_str());
  SUCCEED();
}

// VT3: 热重载后 amp_hand 参数仍在 (issue #4225 回归)
// 复现路径: 启动 merge AMP 控制器配置 -> 桌面端改键触发热重载 -> 下蹲/挡位/退出下蹲
// 都必须还在. 修复前 reload 会用 binding 文件里的 TeleopConfig 整体覆盖 config_,
// amp_hand_* / quit_squat / neutral_guard 全部回落到 enabled=false。
TEST(TeleopConfigReload, ReloadKeepsAmpHandTeleopFromControllerConfig) {
  const std::string bind_path = "/tmp/test_teleop_reload_vt3_bind.yaml";
  const std::string amp_path = "/tmp/test_teleop_reload_vt3_amp.yaml";
  WriteFile(bind_path, kYamlA);
  WriteFile(amp_path, kAmpControllerYaml);

  TriggerBuffer tb;
  TeleopInputSource src(leju::RobotVersions::KUAVO5_BASE, &tb);

  ASSERT_TRUE(src.loadBindingConfig(bind_path));

  // 热重载前先把 AMP 控制器配置合并进来, 与 main() 启动顺序一致
  ASSERT_TRUE(src.mergeAmpHandTeleopFromControllerConfig(amp_path));
  const TeleopConfig& before = src.joy_adapter_->config_;
  ASSERT_TRUE(before.amp_hand_posture_axis.enabled);
  ASSERT_TRUE(before.amp_hand_cmd_vel_line_x_gear.enabled);
  ASSERT_TRUE(before.velocity_entry_neutral_guard.enabled);
  ASSERT_TRUE(before.quit_squat.enabled);

  // 桌面端改键: 绑定表内容变化后热重载
  WriteFile(bind_path, kYamlB);
  ASSERT_TRUE(src.reloadBindingConfig());

  const TeleopConfig& after = src.joy_adapter_->config_;
  EXPECT_TRUE(after.amp_hand_posture_axis.enabled);           // 右摇杆下蹲
  EXPECT_FLOAT_EQ(after.amp_hand_posture_axis.axis_threshold,
                  before.amp_hand_posture_axis.axis_threshold);
  EXPECT_DOUBLE_EQ(after.amp_hand_posture_axis.squat_height_min,
                   before.amp_hand_posture_axis.squat_height_min);
  EXPECT_TRUE(after.amp_hand_cmd_vel_line_x_gear.enabled);    // 十字键切速度挡
  EXPECT_DOUBLE_EQ(after.amp_hand_cmd_vel_line_x_gear.policy_linear_x_low,
                   before.amp_hand_cmd_vel_line_x_gear.policy_linear_x_low);
  EXPECT_DOUBLE_EQ(after.amp_hand_cmd_vel_line_x_gear.policy_linear_x_negative,
                   before.amp_hand_cmd_vel_line_x_gear.policy_linear_x_negative);
  EXPECT_TRUE(after.velocity_entry_neutral_guard.enabled);
  EXPECT_EQ(after.velocity_entry_neutral_guard.neutral_frames,
            before.velocity_entry_neutral_guard.neutral_frames);
  EXPECT_TRUE(after.quit_squat.enabled);                      // BACK+START 先下蹲再退出
  EXPECT_DOUBLE_EQ(after.quit_squat.squat_height,
                   before.quit_squat.squat_height);
  EXPECT_DOUBLE_EQ(after.quit_squat.duration_sec,
                   before.quit_squat.duration_sec);

  // 绑定本身仍被热重载替换 (修 amp_hand 不能妨碍绑定更新)
  const TeleopBinding* b =
      src.joy_adapter_->binding_config_.getJoyConfig().findBinding(ComboX());
  ASSERT_NE(b, nullptr);
  EXPECT_EQ(b->action.type, ActionType::MotionCommand);

  std::remove(bind_path.c_str());
  std::remove(amp_path.c_str());
}

// VT3b: 从未合并过 AMP 控制器配置时, 热重载不应凭空生成 amp_hand 参数
TEST(TeleopConfigReload, ReloadWithoutAmpMergeKeepsDefaults) {
  const std::string path = "/tmp/test_teleop_reload_vt3b.yaml";
  WriteFile(path, kYamlA);

  TriggerBuffer tb;
  TeleopInputSource src(leju::RobotVersions::KUAVO5_BASE, &tb);

  ASSERT_TRUE(src.loadBindingConfig(path));
  ASSERT_TRUE(src.reloadBindingConfig());

  const TeleopConfig& after = src.joy_adapter_->config_;
  EXPECT_FALSE(after.amp_hand_posture_axis.enabled);
  EXPECT_FALSE(after.amp_hand_cmd_vel_line_x_gear.enabled);
  EXPECT_FALSE(after.quit_squat.enabled);

  std::remove(path.c_str());
}
