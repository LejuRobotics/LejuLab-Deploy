/**
 * @file reload_smoketest.cpp
 * @brief 遥控器配置热重载 DDS 联调工具 (不驱动机器人)
 *
 * 用途: 真机 DDS 层联调, 验证 kReloadTeleopConfig 信号从 NX 经 DDS 总线到达 RK3588,
 *       并触发 rl-controller 侧的 teleop_bindings.yaml 热重载. 仅初始化 SDK 通信 +
 *       订阅 reload topic, 不起控制环、不发关节命令, 零运动风险.
 *
 * 用法:
 *   ./reload_smoketest <teleop_bindings.yaml 路径>
 * 然后从 NX 发布 StringData 到 /rt/teleop/reload_config (空 data=重读该路径).
 * 每次重载, 框架会打印 "已加载 N 个 joy 绑定" —— 改 yaml 增删绑定后该计数随之变化,
 * 即证明 DDS 信号触发了热重载并生效.
 */

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>

#include "leju-rl-controller/runtime/input/teleop/teleop_input_source.h"
#include "leju-rl-controller/runtime/input/trigger_buffer.h"
#include "lejusdk-lowlevel/data_types.h"
#include "lejusdk-lowlevel/leju_sdk.h"
#include "lejusdk-utils/robot_version.hpp"

using namespace leju;
using namespace leju::runtime;

static std::atomic<bool> g_run{true};
static void OnSignal(int) { g_run.store(false); }

int main(int argc, char** argv) {
  if (argc < 2) {
    fprintf(stderr, "用法: %s <teleop_bindings.yaml 路径>\n", argv[0]);
    return 2;
  }
  const std::string config_path = argv[1];

  std::signal(SIGINT, OnSignal);
  std::signal(SIGTERM, OnSignal);

  RobotVersion version = RobotVersion::from_env();
  if (!GlobalRobot::init_env(version)) {
    fprintf(stderr, "GlobalRobot::init_env 失败\n");
    return 1;
  }
  printf("[reload_smoketest] SDK 初始化完成, robot version=%s\n",
         version.version_name().c_str());

  TriggerBuffer tb;
  TeleopInputSource teleop(version, &tb);
  if (!teleop.loadBindingConfig(config_path)) {
    fprintf(stderr, "首次加载配置失败: %s\n", config_path.c_str());
    return 1;
  }
  printf("[reload_smoketest] 已加载初始配置: %s (上方日志含 joy 绑定数)\n",
         config_path.c_str());

  std::atomic<int> count{0};
  GlobalRobot::getInstance().subscribeReloadTeleopConfig(
      [&](const StringDataConstPtr& msg) {
        std::string path = msg ? msg->data : std::string();
        bool ok = path.empty() ? teleop.reloadBindingConfig()
                               : teleop.loadBindingConfig(path);
        int n = ++count;
        printf("\n[reload_smoketest] #%d 收到 DDS reload 信号 (data='%s') -> %s "
               "(绑定数见上方框架日志)\n",
               n, path.c_str(), ok ? "重载成功" : "重载失败");
        fflush(stdout);
      });

  printf("[reload_smoketest] 正在监听 DDS topic /rt/teleop/reload_config ... "
         "(Ctrl-C 退出)\n");
  fflush(stdout);
  while (g_run.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  printf("[reload_smoketest] 退出, 共收到 %d 次信号\n", count.load());
  return 0;
}
