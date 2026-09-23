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
#include "leju-rl-controller/runtime/dexterous_hand_poses.h"
#include "leju-rl-controller/runtime/data_types.hpp"
#include "leju-rl-controller/runtime/input/external_interface.h"
#include "leju-rl-controller/runtime/input/input_source.h"
#include "leju-rl-controller/runtime/lifecycle.h"
#include "leju-rl-controller/runtime/input/teleop/teleop_input_source.h"
#include "leju-rl-controller/runtime/fall_stand_coordinator.h"
#include "leju-rl-controller/runtime/transport_mode_coordinator.h"
#include "leju-rl-controller/runtime/transport_fall_stand_scheduler.h"
#include "leju-rl-controller/runtime/input/trigger_buffer.h"
#include "leju-rl-controller/rl_log.h"
#include "lejusdk-lowlevel/leju_sdk.h"
#include "lejusdk-utils/robot_version.hpp"
#include "lejusdk-utils/cpu_affinity.hpp"
#include "lejusdk-utils/time_utils.hpp"

#include <dds/dds.hpp>
#include "lejusdk-dds-idl/StringData.hpp"
#include "lejusdk-topic-pubsub/topic_names.h"
#include "lejusdk-topic-pubsub/topic_publisher.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace leju;
using namespace leju::runtime;

// 全局指针，用于信号处理
static runtime::ControlLoop* g_control_loop = nullptr;
static ControllerManager* g_controller_manager = nullptr;

// 未显式指定 -t 时，优先读取现场 lejuconfig；首次安装尚未生成该文件时，
// 回退到 controller_manager.yaml 同目录下随版本交付的标准绑定表。
static std::string resolveDefaultTeleopConfig(const std::string& config_file) {
  const char* home = std::getenv("HOME");
  if (home != nullptr && home[0] != '\0') {
    const auto runtime_config =
        std::filesystem::path(home) / ".config/lejuconfig/teleop_bindings.yaml";
    if (std::filesystem::is_regular_file(runtime_config)) {
      return runtime_config.string();
    }
  }

  const auto repository_config =
      std::filesystem::path(config_file).parent_path() / "teleop_bindings.yaml";
  if (std::filesystem::is_regular_file(repository_config)) {
    return repository_config.string();
  }
  return {};
}

// 信号处理函数：只能做 async-signal-safe 操作——仅原子置位，禁止调用
// 日志/加锁函数(否则信号可能在主线程持锁时到达 → 死锁，正是之前 SIGTERM
// 卡死、需 kill -9 的原因)。同时停 ControlLoop(使 run() 退出)和
// ControllerManager(使 waitForDataReady() 等阻塞循环退出)。
void signalHandler(int signal) {
  if (signal == SIGINT || signal == SIGTERM) {
    if (g_control_loop != nullptr) {
      g_control_loop->requestStop();
    }
    if (g_controller_manager != nullptr) {
      g_controller_manager->requestStop();
    }
  }
}

void printUsage(const char* program_name) {
  std::cerr << "Usage: " << program_name << " [options]" << std::endl;
  std::cerr << std::endl;
  std::cerr << "Options:" << std::endl;
  std::cerr << "  -c, --config               Path to controller_manager.yaml (required)" << std::endl;
  std::cerr << "  -u, --urdf-path            Path to robot URDF (for arm gravity compensation)" << std::endl;
  std::cerr << "  -t, --teleop-config        Path to teleop_bindings.yaml" << std::endl;
  std::cerr << "                             (default: <config_dir>/teleop_bindings.yaml)" << std::endl;
  std::cerr << "  -d, --default-controller   Override yaml default_controller" << std::endl;
  std::cerr << "      --pre-start-fall-recovery" << std::endl;
  std::cerr << "                             Replay startup LB+RB+X after runtime launch" << std::endl;
  std::cerr << "  -h, --help                 Show this help message" << std::endl;
  std::cerr << std::endl;
  std::cerr << "Examples:" << std::endl;
  std::cerr << "  " << program_name << " -c config/46/controller_manager.yaml" << std::endl;
  std::cerr << "  " << program_name << " -c config/46/controller_manager.yaml -u /path/to/biped_s17.urdf" << std::endl;
  std::cerr << "  " << program_name << " -c config/46/controller_manager.yaml -t /custom/path/teleop_bindings.yaml" << std::endl;
  std::cerr << "  " << program_name << " -c config/17/controller_manager.yaml -d mimic_fall_stand" << std::endl;
}

// ============================================================================
// 主函数
// ============================================================================
int main(int argc, char** argv) {
  std::string config_file;
  std::string teleop_config_path;
  std::string urdf_path;
  std::string default_controller_override;
  bool pre_start_fall_recovery = false;

  // 解析命令行参数
  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
      printUsage(argv[0]);
      return 0;
    } else if ((strcmp(argv[i], "-c") == 0 || strcmp(argv[i], "--config") == 0) && i + 1 < argc) {
      config_file = argv[++i];
    } else if ((strcmp(argv[i], "-u") == 0 || strcmp(argv[i], "--urdf-path") == 0) && i + 1 < argc) {
      urdf_path = argv[++i];
    } else if ((strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--teleop-config") == 0) && i + 1 < argc) {
      teleop_config_path = argv[++i];
    } else if ((strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--default-controller") == 0) &&
               i + 1 < argc) {
      default_controller_override = argv[++i];
    } else if (strcmp(argv[i], "--pre-start-fall-recovery") == 0) {
      pre_start_fall_recovery = true;
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

  if (teleop_config_path.empty()) {
    teleop_config_path = resolveDefaultTeleopConfig(config_file);
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
    if (!controller_manager.initialize(config_file, urdf_path)) {
      RL_LOGE("Failed to initialize ControllerManager");
      return 1;
    }
    if (!default_controller_override.empty()) {
      if (!controller_manager.setDefaultController(default_controller_override)) {
        RL_LOGE("Failed to override default controller to '%s'",
                default_controller_override.c_str());
        return 1;
      }
    }
    RL_LOGI("ControllerManager initialized, %zu controllers loaded, active='%s'",
            controller_manager.getControllerCount(),
            controller_manager.getCurrentControllerName().c_str());

    // 3.4 创建遥操作输入源（内部自动管理 Joy 和 Quest 适配器）
    TeleopInputSource teleop_input(version, &trigger_buffer);
    if (!teleop_input.initialize()) {
      RL_LOGE("Failed to initialize TeleopInputSource");
      return 1;
    }
    // 蹲起状态查询：External 处于下蹲/姿态模式（cmd_stance=1）时，遥操作行走命令
    // 让路给 External 的蹲起高度指令，避免 mergeAllCmdVel 用 mode=0 覆盖下蹲。
    teleop_input.setStanceActiveQuery(
        [&controller_manager]() -> bool {
          return controller_manager.getCurrentCmdStanceMode() == 1;
        });

    // 3.4.1 搬运/倒地起身协调器（仅 Roban 2.2；指针为空时 ControlLoop 不参与）。
    // 与下方 audio publisher 共享 DomainParticipant。
    std::shared_ptr<runtime::TransportModeCoordinator> transport_coordinator;
    std::shared_ptr<runtime::FallStandCoordinator> fall_stand_coordinator;
    dds::domain::DomainParticipant mc_participant(0);
    if (IS_ROBAN2_2_LEGGED(version)) {
      transport_coordinator = std::make_shared<runtime::TransportModeCoordinator>();
      if (!transport_coordinator->initialize(mc_participant)) {
        RL_LOGW("Failed to initialize TransportModeCoordinator, continuing without transport mode support");
        transport_coordinator.reset();
      } else {
        RL_LOGI("TransportModeCoordinator initialized");
      }

      fall_stand_coordinator = std::make_shared<runtime::FallStandCoordinator>();
      if (!fall_stand_coordinator->initialize(mc_participant)) {
        RL_LOGW("Failed to initialize FallStandCoordinator, continuing without fall-stand DDS support");
        fall_stand_coordinator.reset();
      } else {
        RL_LOGI("FallStandCoordinator initialized");
      }
    }

    // 3.5 加载遥操作绑定配置
    if (!teleop_config_path.empty()) {
      RL_LOGI("Loading teleop config from: %s", teleop_config_path.c_str());
      if (!teleop_input.loadBindingConfig(teleop_config_path)) {
        RL_LOGE("Failed to load teleop bindings from %s", teleop_config_path.c_str());
        return 1;
      }
    }

    const std::string amp_config_path =
        controller_manager.getControllerConfigPath("amp");
    if (!amp_config_path.empty()) {
      if (!teleop_input.mergeAmpHandTeleopFromControllerConfig(amp_config_path)) {
        RL_LOGW("No amp_hand teleop config merged from %s",
                amp_config_path.c_str());
      }
    }

    // 3.5.1 订阅遥控器绑定配置热重载信号
    // 上位机修改 teleop_bindings.yaml 后, 通过 DDS topic kReloadTeleopConfig 通知本控制器
    // 重读配置并热应用 (无需重启). 信号 StringData.data 可选携带新配置路径; 为空则重读
    // 上次加载的路径. 适配器内部 setBindingConfig 持锁, 与 joy/quest 帧处理线程安全互斥.
    GlobalRobot::getInstance().subscribeReloadTeleopConfig(
        [&teleop_input](const StringDataConstPtr& msg) {
          std::string path = msg ? msg->data : std::string();
          bool ok = path.empty() ? teleop_input.reloadBindingConfig()
                                  : teleop_input.loadBindingConfig(path);
          if (ok) {
            RL_LOGI("Teleop binding config hot-reloaded successfully");
          } else {
            RL_LOGW("Teleop binding config hot-reload failed");
          }
        });

    // 上位机修改 controller_manager.yaml 后, 通过 DDS topic kReloadControllerConfig
    // 通知本控制器热重载 (无需重启). 仅添加新增的控制器, 不影响已运行的.
    GlobalRobot::getInstance().subscribeReloadControllerConfig(
        [&controller_manager](const StringDataConstPtr& msg) {
          (void)msg;  // data 暂不使用, 始终重读已缓存的 config_file_
          bool ok = controller_manager.reloadControllersFromConfig();
          if (ok) {
            RL_LOGI("Controller config hot-reloaded successfully");
          } else {
            RL_LOGW("Controller config hot-reload failed");
          }
        });

    // ========================================================================
    // 4. 初始化外部接口（VR/SDK）
    // ========================================================================
    ExternalInterface external_interface;
    if (!external_interface.initialize(version, trigger_buffer, controller_manager, lifecycle)) {
      RL_LOGW("Failed to initialize ExternalInterface, continuing without VR support");
    } else {
      RL_LOGI("ExternalInterface initialized");
      if (!teleop_config_path.empty() &&
          !external_interface.loadVelocityLimitsFromTeleopConfig(teleop_config_path)) {
        RL_LOGW("Failed to load External velocity limits from %s, using TeleopConfig"
                " defaults",
                teleop_config_path.c_str());
      }
      if (!amp_config_path.empty()) {
        external_interface.loadPostureHeightConfig(amp_config_path);
      }
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
    g_controller_manager = &controller_manager;

    // 5.1 音乐播放接线：MotionCommand{Start} 带 music 时，ControlLogic 通过此回调
    // 把文件名发布到 DDS /rt/audio_play_file（非阻塞，控制环安全），由 leju-audio 的
    // audio_player_node 订阅后按名解码播放。与协调器共享 mc_participant，
    // publisher 生命周期覆盖 run()。
    leju::dds_common::TopicPublisher<leju::msgs::StringData> audio_play_file_pub(
        mc_participant, leju::dds_topics::kAudioPlayFile);
    auto play_audio_file = [&audio_play_file_pub](const std::string& music_name) {
      leju::msgs::StringData msg;
      msg.data(music_name);
      audio_play_file_pub.publish(msg);
    };
    control_loop.setMusicPlayer(play_audio_file);

    // 5.1.1 搬运/倒地起身接线：协调器先注入控制环，调度器再绑定两者（仅初始化成功的机型参与）
    if (transport_coordinator) {
      // ENTER 门控前置需要查询当前控制器状态（仅 amp+站立可进入）
      transport_coordinator->setControllerManager(&controller_manager);
      control_loop.setTransportModeCoordinator(transport_coordinator);
    }
    if (fall_stand_coordinator) {
      control_loop.setFallStandCoordinator(fall_stand_coordinator);
    }
    if (transport_coordinator && fall_stand_coordinator) {
      auto scheduler = std::make_shared<runtime::TransportFallStandScheduler>();
      scheduler->bind(transport_coordinator.get(), fall_stand_coordinator.get(),
                      &controller_manager);
      scheduler->setAudioPlayer(play_audio_file);

      // 灵巧手存在性仅在本会话首份反馈时判定一次。未收到反馈默认视为无手，
      // 不做中途掉线处理，也不将反馈作为状态机门控。
      auto hand_presence_resolved = std::make_shared<std::atomic_bool>(false);
      auto dexterous_hands_available = std::make_shared<std::atomic_bool>(false);
      leju::GlobalRobot::getInstance().subscribeHandState(
          [hand_presence_resolved, dexterous_hands_available](const HandStateConstPtr& state) {
            bool expected = false;
            if (!hand_presence_resolved->compare_exchange_strong(expected, true)) {
              return;
            }
            const bool available = state && state->left_valid && state->right_valid;
            dexterous_hands_available->store(available);
            RL_LOGI("Dexterous hands %s for this session", available ? "available" : "unavailable");
          });
      scheduler->setHandsAvailableChecker(
          [dexterous_hands_available]() { return dexterous_hands_available->load(); });
      scheduler->setHandPreGripSetter([]() {
        leju::HandCmd cmd;
        for (std::size_t i = 0; i < runtime::kDexterousHandDof; ++i) {
          cmd.position[i] = runtime::kFullyOpenHandPose[i];
          cmd.position[runtime::kDexterousHandDof + i] = runtime::kFullyOpenHandPose[i];
        }
        cmd.timestamp = leju::common::GetUnixTimestampS();
        return leju::GlobalRobot::getInstance().publishHandCmd(cmd);
      });
      scheduler->setFourFingerGripSetter([]() {
        leju::HandCmd cmd;
        for (std::size_t i = 0; i < runtime::kDexterousHandDof; ++i) {
          cmd.position[i] = runtime::kTransportFourFingerGripPose[i];
          cmd.position[runtime::kDexterousHandDof + i] =
              runtime::kTransportFourFingerGripPose[i];
        }
        cmd.timestamp = leju::common::GetUnixTimestampS();
        return leju::GlobalRobot::getInstance().publishHandCmd(cmd);
      });
      scheduler->setHandGripSetter([](bool closed) {
        leju::HandCmd cmd;
        const auto& pose = closed ? runtime::kTransportClosedHandPose
                                  : runtime::kTransportReleaseHandPose;
        for (std::size_t i = 0; i < runtime::kDexterousHandDof; ++i) {
          cmd.position[i] = pose[i];
          cmd.position[runtime::kDexterousHandDof + i] = pose[i];
        }
        cmd.timestamp = leju::common::GetUnixTimestampS();
        return leju::GlobalRobot::getInstance().publishHandCmd(cmd);
      });
      control_loop.setTransportFallStandScheduler(scheduler);
    }

    // 注册信号处理(须在 waitForDataReady 之前,使启动阶段也能被信号中断退出)
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // 6. 启动 ControllerManager（设置 running_ 标志）
    controller_manager.Start();

    // 7. 等待传感器数据就绪
    if (!controller_manager.waitForDataReady()) {
      RL_LOGE("Failed to wait for data ready, exiting");
      return 1;
    }

    // 启动阶段保持默认控制器；普通 START 后的倾倒检测由 ControlLogic 处理。
    // 启动前 LB+RB+X 是显式倒地起身请求，不依赖 IMU，直接切入 FALL_DOWN。
    bool pre_start_fall_stand_active = false;
    if (pre_start_fall_recovery) {
      if (!IS_ROBAN2_2_LEGGED(version)) {
        RL_LOGI("Ignore pre-start fall recovery on non-Roban2.2 robot");
      } else if (!controller_manager.hasController("mimic_fall_stand")) {
        RL_LOGW("Ignore pre-start fall recovery: mimic_fall_stand unavailable");
      } else {
        auto* old_controller = controller_manager.getCurrentController();
        if (controller_manager.setDefaultController("mimic_fall_stand")) {
          if (old_controller) {
            old_controller->pause();
          }
          if (auto* fall_stand_controller = controller_manager.getCurrentController()) {
            fall_stand_controller->resume();
          }
          pre_start_fall_stand_active = true;
          trigger_buffer.push(runtime::ActionTrigger(runtime::ActionType::Start));
          trigger_buffer.push(
              runtime::MakeTransportFallStandTrigger("fallstand.standup"));
          RL_LOGI("Startup LB+RB+X replay queued for fall-stand startup");
        }
      }
    }

    // 8. 常规启动插值；倒地起身由 PREPARE 自己插值到准备姿态。
    if (pre_start_fall_stand_active) {
      RL_LOGI("Skip default-pose interpolation for pre-start fall recovery");
    } else {
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
    }

    // 9. 启动 ControlLoop（阻塞直到 stop() 被调用）
    RL_LOGI("Starting ControlLoop...");
    RL_LOGI("Press 'start' button to begin, 'back' button to exit program");

    // 9. 启动 ControlLoop — CPU 大核绑定
    if (leju::cpu::bindCurrentThreadToBigCores()) {
      RL_LOGI("RL control thread bound to big cores %d-%d, current CPU=%d",
              leju::cpu::kRk3588BigCoreFirst, leju::cpu::kRk3588BigCoreLast,
              leju::cpu::getCurrentCpu());
    } else {
      RL_LOGW("Failed to bind RL control thread to big cores");
    }

    control_loop.run();

    // ========================================================================
    // 9. 清理
    // ========================================================================
    teleop_input.shutdown();
    g_control_loop = nullptr;
    g_controller_manager = nullptr;

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
