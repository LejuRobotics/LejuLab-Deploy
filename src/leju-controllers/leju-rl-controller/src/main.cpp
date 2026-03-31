/**
 * @file main.cpp
 * @brief 主函数 - 按照设计文档简化实现
 *
 * 设计文档：
 * - docs/framework-controller/main_and_control_loop_design.md
 * - docs/framework-controller/controllermanager_controllloop_boundary_design.md
 *
 * main() 职责：
 * 1. 初始化基础设施
 * 2. 创建模块对象
 * 3. 初始化各模块（包括 TeleopAdapter，内部管理手柄数据订阅）
 * 4. 初始化外部接口（VR/SDK）
 * 5. 创建并运行 ControlLoop
 */

#include "leju-rl-controller/controllers/controller_manager.h"
#include "leju-rl-controller/runtime/input/action_trigger.h"
#include "leju-rl-controller/runtime/control_loop.h"
#include "leju-rl-controller/runtime/data_types.hpp"
#include "leju-rl-controller/runtime/input/external_interface.h"
#include "leju-rl-controller/runtime/input/input_source.h"
#include "leju-rl-controller/runtime/lifecycle.h"
#include "leju-rl-controller/runtime/input/teleop/teleop_input_source.h"
#include "leju-rl-controller/runtime/input/trigger_buffer.h"
#include "leju-rl-controller/rl_log.h"
#include "lejusdk-lowlevel/leju_sdk.h"
#include "lejusdk-utils/robot_version.hpp"

#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using namespace leju;
using namespace leju::runtime;

// 全局控制循环指针，用于信号处理
static runtime::ControlLoop* g_control_loop = nullptr;

// 信号处理函数
void signalHandler(int signal) {
  if (signal == SIGINT || signal == SIGTERM) {
    RL_LOGI("Received signal %d, stopping control loop...", signal);
    if (g_control_loop != nullptr) {
      g_control_loop->stop();
    }
  }
}

void printUsage(const char* program_name) {
  std::cerr << "Usage: " << program_name << " [options]" << std::endl;
  std::cerr << std::endl;
  std::cerr << "Options:" << std::endl;
  std::cerr << "  -c, --config          Path to controller_manager.yaml (required)" << std::endl;
  std::cerr << "  -t, --teleop-config   Path to teleop_bindings.yaml" << std::endl;
  std::cerr << "                        (default: <config_dir>/teleop_bindings.yaml)" << std::endl;
  std::cerr << "  -h, --help            Show this help message" << std::endl;
  std::cerr << std::endl;
  std::cerr << "Examples:" << std::endl;
  std::cerr << "  " << program_name << " -c config/46/controller_manager.yaml" << std::endl;
  std::cerr << "  " << program_name << " -c config/46/controller_manager.yaml -t /custom/path/teleop_bindings.yaml" << std::endl;
}

// ============================================================================
// 主函数
// ============================================================================
int main(int argc, char** argv) {
  std::string config_file;
  std::string teleop_config_path;

  // 解析命令行参数
  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
      printUsage(argv[0]);
      return 0;
    } else if ((strcmp(argv[i], "-c") == 0 || strcmp(argv[i], "--config") == 0) && i + 1 < argc) {
      config_file = argv[++i];
    } else if ((strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--teleop-config") == 0) && i + 1 < argc) {
      teleop_config_path = argv[++i];
    } else if (strncmp(argv[i], "__", 2) == 0) {
      // 忽略 ROS 内部参数（如 __name:=leju_rl_controller）
      continue;
    } else if (argv[i][0] != '-') {
      // 非选项参数也作为 config_file（向后兼容）
      if (config_file.empty()) {
        config_file = argv[i];
      } else {
        std::cerr << "Error: Unexpected argument: " << argv[i] << std::endl;
        printUsage(argv[0]);
        return 1;
      }
    } else {
      std::cerr << "Error: Unknown option: " << argv[i] << std::endl;
      printUsage(argv[0]);
      return 1;
    }
  }

  if (config_file.empty()) {
    std::cerr << "Error: config_file is required" << std::endl;
    printUsage(argv[0]);
    return 1;
  }

  try {
    // ========================================================================
    // 1. 初始化基础设施
    // ========================================================================
    RobotVersion version = RobotVersion::from_env();
    if (!GlobalRobot::init_env(version)) {
      RL_LOGE("Failed to initialize robot environment");
      return 1;
    }
    RL_LOGI("Robot version: %s", version.version_name().c_str());

    // ========================================================================
    // 2. 创建模块对象
    // ========================================================================
    RobotData robot_data;
    TriggerBuffer trigger_buffer;
    Lifecycle lifecycle;
    ControllerManager controller_manager;

    // ========================================================================
    // 3. 初始化各模块
    // ========================================================================

    // 3.1 初始化 RobotData（传感器数据订阅）
    if (!robot_data.initialize()) {
      RL_LOGE("Failed to initialize RobotData");
      return 1;
    }

    // 3.2 初始化 ControllerManager（加载控制器）
    if (!controller_manager.initialize(config_file)) {
      RL_LOGE("Failed to initialize ControllerManager");
      return 1;
    }
    RL_LOGI("ControllerManager initialized, %zu controllers loaded",
            controller_manager.getControllerCount());

    // 3.4 创建遥操作输入源（内部自动管理 Joy 和 Quest 适配器）
    TeleopInputSource teleop_input(version, &trigger_buffer);
    if (!teleop_input.initialize()) {
      RL_LOGE("Failed to initialize TeleopInputSource");
      return 1;
    }

    // 3.5 加载遥操作绑定配置
    if (!teleop_config_path.empty()) {
      RL_LOGI("Loading teleop config from: %s", teleop_config_path.c_str());
      if (!teleop_input.loadBindingConfig(teleop_config_path)) {
        RL_LOGE("Failed to load teleop bindings from %s", teleop_config_path.c_str());
        return 1;
      }
    }

    // ========================================================================
    // 4. 初始化外部接口（VR/SDK）
    // ========================================================================
    ExternalInterface external_interface;
    if (!external_interface.initialize(version, trigger_buffer, controller_manager, lifecycle)) {
      RL_LOGW("Failed to initialize ExternalInterface, continuing without VR support");
    } else {
      RL_LOGI("ExternalInterface initialized");
    }

    // ========================================================================
    // 5. 创建并运行 ControlLoop
    // ========================================================================

    // 准备输入源列表：遥操作输入源 + 外部接口
    std::vector<InputSource*> input_sources = {
      &teleop_input,      // 遥操作输入（内部 Joy > Quest）
      &external_interface // External
    };

    ControlLoop control_loop(
                             robot_data,
                             trigger_buffer,
                             input_sources,
                             lifecycle,
                             controller_manager);
    g_control_loop = &control_loop;

    // 注册信号处理
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // 6. 启动 ControllerManager（设置 running_ 标志）
    controller_manager.Start();

    // 7. 等待传感器数据就绪
    if (!controller_manager.waitForDataReady()) {
      RL_LOGE("Failed to wait for data ready, exiting");
      return 1;
    }

    // 8. 移动到默认位置（在启动 ControlLoop 前）
    RL_LOGI("Moving to default position...");
    auto* initial_controller = controller_manager.getCurrentController();
    if (initial_controller) {
      RobotState current_state;
      if (robot_data.getRobotState(current_state)) {
        initial_controller->moveToDefaultPos(current_state, 3.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      } else {
        RL_LOGW("Failed to get robot state, skipping moveToDefaultPos");
      }
    } else {
      RL_LOGW("No initial controller, skipping moveToDefaultPos");
    }

    // 9. 启动 ControlLoop（阻塞直到 stop() 被调用）
    RL_LOGI("Starting ControlLoop...");
    RL_LOGI("Press 'start' button to begin, 'back' button to exit program");

    control_loop.run();

    // ========================================================================
    // 9. 清理
    // ========================================================================
    teleop_input.shutdown();
    g_control_loop = nullptr;

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
