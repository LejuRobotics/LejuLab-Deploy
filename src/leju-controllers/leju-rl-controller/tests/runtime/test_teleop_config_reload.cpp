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
