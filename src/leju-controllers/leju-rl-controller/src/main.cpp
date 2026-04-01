#include "leju-rl-controller/controllers/controller_manager.h"
#include "leju-rl-controller/joy_handler.h"
#include "leju-rl-controller/rl_log.h"
#include "leju-rl-controller/velocity_manager.h"
#include "lejusdk-lowlevel/leju_sdk.h"

#include <chrono>
#include <iostream>
#include <string>

void printUsage(const char* program_name) {
  std::cerr << "Usage: " << program_name << " <config_file>" << std::endl;
  std::cerr << "  config_file: Path to controller_manager.yaml" << std::endl;
  std::cerr << std::endl;
  std::cerr << "Example:" << std::endl;
  std::cerr << "  " << program_name << " config/46/controller_manager.yaml" << std::endl;
}

int main(int argc, char** argv) {
  if (argc < 2) {
    printUsage(argv[0]);
    return 1;
  }
  std::string config_file = argv[1];

  try {
    // 1. SDK 初始化
    leju::RobotVersion version = leju::RobotVersion::from_env();
    if (!leju::GlobalRobot::init_env(version)) {
      RL_LOGE("Failed to initialize robot environment");
      return 1;
    }
    RL_LOGI("Robot version: %s", version.version_name().c_str());

    // 2. 控制器管理器初始化
    leju::ControllerManager manager;
    leju::VelocityManager velocity_manager;
    if (!velocity_manager.initializeFromControllerManagerConfig(config_file)) {
      RL_LOGE("Failed to initialize VelocityManager");
      return 1;
    }
    manager.setVelocityManager(&velocity_manager);
    if (!manager.initialize(config_file)) {
      RL_LOGE("Failed to initialize ControllerManager");
      return 1;
    }
    RL_LOGI("ControllerManager initialized, %zu controllers loaded", manager.getControllerCount());

    // 3. 手柄输入
    leju::JoyHandler joy;
    joy.setCallback([&](const leju::JoyData& data, const leju::JoyData::Buttons& prev) {
      velocity_manager.onJoyData(data);
      manager.dispatchJoyInput(data, prev);
    });
    joy.start();

    // 4. 阻塞等待启动信号
    manager.starting();

    // 5. 主控制循环
    while (manager.isRunning()) {
      auto t = std::chrono::steady_clock::now();
      velocity_manager.publishResolvedCmdVel();
      manager.update();
      manager.waitNextCycle(t);
    }

    joy.stop();

  } catch (const std::exception& e) {
    RL_LOGE("Exception: %s", e.what());
    return 1;
  } catch (...) {
    RL_LOGE("Unknown exception");
    return 1;
  }

  RL_LOGI("Controller manager exited");
  return 0;
}
