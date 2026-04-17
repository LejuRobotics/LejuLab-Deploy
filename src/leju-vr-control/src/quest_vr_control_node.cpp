/**
 * @file quest_vr_control_node.cpp
 * @brief Unified VR control node - subscribes Quest data, runs IK, publishes via lejusdk-vr
 * IK mode: USE_INCREMENTAL_IK env (from launch use_incremental_ik) - true=增量IK, false=绝对IK
 */

#include "leju-vr-control/head_solver.h"
#include "leju-vr-control/arm_ctrl_mode_fsm.h"
#include "leju-vr-control/quest_to_ik_converter.h"
#include "leju-vr-control/quest_vr_fsm.h"

#include <leju-ik/Quest3IkAPI.h>
#include <leju-ik/Quest3IkIncrementalAPI.h>
#include <lejusdk-lowlevel/leju_sdk.h>
#include <lejusdk-vr/lejusdk_vr.h>
#include <lejusdk-utils/robot_version.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

namespace {

std::atomic<bool> g_running{true};

void signalHandler(int) { g_running = false; }

std::string getEnv(const char* name) {
  const char* env = std::getenv(name);
  if (env && env[0] != '\0') return std::string(env);
  return "";
}

bool getEnvBool(const char* name, bool default_val) {
  std::string s = getEnv(name);
  if (s.empty()) return default_val;
  if (s == "1" || s == "true" || s == "True" || s == "TRUE" || s == "yes") return true;
  if (s == "0" || s == "false" || s == "False" || s == "FALSE" || s == "no") return false;
  return default_val;
}

void triggerEmergencyStopAndExit(leju::vr::KuavoVRAPI& vr_api) {
  auto& robot = leju::GlobalRobot::getInstance();

  // 1) 先把底盘速度归零，尽快停止外部速度输入
  leju::vr::VrVelocityCmd stop_vel;
  stop_vel.linear_x = 0.0;
  stop_vel.linear_y = 0.0;
  stop_vel.angular_z = 0.0;
  vr_api.publishVrVelocityCmd(stop_vel);

  // 2) 下发 StopRobot
  robot.publishStopRobot();

  // 3) 主循环退出，进入统一析构流程
  g_running = false;
}

void restoreControlModesToAutoOnExit(leju::vr::KuavoVRAPI& vr_api, int timeout_ms = 1000) {
  const bool arm_ok = vr_api.setArmMode(leju::vr::ControlMode::kAuto, timeout_ms);
  std::cout << "[VR] Arm mode set to Auto: " << (arm_ok ? "success" : "fail") << std::endl;

  const bool head_ok = vr_api.setHeadMode(leju::vr::ControlMode::kAuto, timeout_ms);
  std::cout << "[VR] Head mode set to Auto: " << (head_ok ? "success" : "fail") << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, signalHandler);

  leju::RobotVersion robot_version;
  try {
    robot_version = leju::RobotVersion::from_env();
  } catch (const std::exception& e) {
    std::cerr << "[VR] Failed to read ROBOT_VERSION: " << e.what() << std::endl;
    return 1;
  }

  std::cout << "[VR] Robot version detected: " << robot_version.version_name_short() << std::endl;

  if (IS_ROBAN(robot_version)) {
    std::cerr << "[VR] Roban robots (including 14/17) are not supported by leju-vr-control, exiting..." << std::endl;
    return 1;
  }

  if (!IS_KUAVO5_LEGGED(robot_version) && !IS_KUAVO4PRO_LEGGED(robot_version)) {
    std::cerr << "[VR] Unsupported robot version for leju-vr-control: "
              << robot_version.version_name_short() << std::endl;
    return 1;
  }

  if (!leju::GlobalRobot::init_env(robot_version)) {
    std::cerr << "Failed to initialize lejusdk-lowlevel" << std::endl;
    return 1;
  }

  leju::vr::KuavoVRAPI vr_api;
  if (!vr_api.initialize()) {
    std::cerr << "Failed to initialize lejusdk-vr" << std::endl;
    return 1;
  }

  bool use_incremental_ik = getEnvBool("USE_INCREMENTAL_IK", true);
  std::string models_base = getEnv("KUAVO_MODELS_PATH");
  std::string config_base = getEnv("KUAVO_CONFIG_PATH");
  std::string quest3_yaml_path = getEnv("QUEST3_IK_YAML_PATH");
  const uint16_t major = robot_version.major();
  const uint16_t minor = robot_version.minor();

  std::unique_ptr<leju::ik::Quest3IkAPI> ik_absolute;
  std::unique_ptr<leju::ik::Quest3IkIncrementalAPI> ik_incremental;

  if (models_base.empty() || config_base.empty()) {
    std::cerr << "KUAVO_MODELS_PATH or KUAVO_CONFIG_PATH not set. IK will not be initialized."
              << std::endl;
    std::cerr << "Set KUAVO_MODELS_PATH (e.g. $(find leju_assets)) and KUAVO_CONFIG_PATH "
                 "(e.g. $(find leju-hardware))" << std::endl;
  } else {
    if (use_incremental_ik) {
      ik_incremental = std::make_unique<leju::ik::Quest3IkIncrementalAPI>();
      if (!ik_incremental->init(models_base, config_base, major, minor, {}, quest3_yaml_path)) {
        std::cerr << "Failed to initialize leju-ik (incremental)" << std::endl;
        ik_incremental.reset();
      } else {
        std::cout << "leju-ik (incremental) initialized: models=" << models_base
                  << " config=" << config_base << " version=" << major << "." << minor << std::endl;
        ik_incremental->setArmMode(1);  // 站立默认 1（自动摆臂），X+A 一次切到 2
      }
    } else {
      ik_absolute = std::make_unique<leju::ik::Quest3IkAPI>();
      if (!ik_absolute->init(models_base, config_base, major, minor)) {
        std::cerr << "Failed to initialize leju-ik (absolute)" << std::endl;
        ik_absolute.reset();
      } else {
        std::cout << "leju-ik (absolute) initialized: models=" << models_base
                  << " config=" << config_base << " version=" << major << "." << minor << std::endl;
        ik_absolute->setArmMode(1);  // 站立默认 1（自动摆臂），X+A 一次切到 2
      }
    }
  }

  std::mutex bone_mutex;
  std::mutex joy_mutex;
  leju::vr::QuestBonePosesData latest_bones;
  leju::vr::QuestJoystickData latest_joy;
  bool has_bones = false;
  bool has_joy = false;

  leju::vr_control::QuestVrFSM fsm;
  leju::vr_control::ArmCtrlModeFSM arm_ctrl_mode_fsm;
  int current_arm_mode = 1;

  auto last_joy_time = std::chrono::steady_clock::now();
  constexpr float GRIP_THRESHOLD = 0.5f;
  bool prev_left_grip = false;
  bool prev_right_grip = false;

  vr_api.subscribeQuestBonePoses([&](const leju::vr::QuestBonePosesData& data) {
    std::lock_guard<std::mutex> lock(bone_mutex);
    latest_bones = data;
    has_bones = true;
  });

  vr_api.subscribeQuestJoystickData([&](const leju::vr::QuestJoystickData& data) {
    std::lock_guard<std::mutex> lock(joy_mutex);
    latest_joy = data;
    has_joy = true;
  });

  // 通过 lowlevel 订阅关节状态，供增量 IK 锚点计算（v52: 腿12 + 腰1 = 13，手臂 q[13..26]）
  const int arm_joint_start = 12 + (major >= 5 && minor >= 2 ? 1 : 0);
  leju::GlobalRobot::getInstance().subscribeRobotState([&](const leju::RobotStateConstPtr& state) {
    const auto& q = state->q;
    if (ik_incremental && q.size() >= static_cast<size_t>(arm_joint_start + 14)) {
      std::vector<double> arm_q(14);
      for (int i = 0; i < 14; ++i) arm_q[i] = q[arm_joint_start + i];
      ik_incremental->onSensorArmJoints(arm_q);
    }
  });

  auto arm_callback = [&vr_api](const std::vector<double>& q) {
    if (q.size() >= 14) {
      leju::vr::JointTrajectoryPoint cmd;
      cmd.q.assign(q.begin(), q.begin() + 14);
      cmd.v.resize(14, 0.0);
      cmd.acc.resize(14, 0.0);
      vr_api.publishArmJointCmd(cmd);
    }
  };

  auto head_body_callback = [](const leju::ik::HeadBodyPose&) {};

  if (ik_incremental) {
    ik_incremental->setArmJointCallback(arm_callback);
    ik_incremental->setHeadBodyPoseCallback(head_body_callback);
  } else if (ik_absolute) {
    ik_absolute->setArmJointCallback(arm_callback);
    ik_absolute->setHeadBodyPoseCallback(head_body_callback);
  }

  std::cout << "quest_vr_control_node running. Press Ctrl+C to exit." << std::endl;

  constexpr double rate_hz = 100.0;
  auto interval = std::chrono::microseconds(static_cast<int64_t>(1000000.0 / rate_hz));

  while (g_running) {
    auto now = std::chrono::steady_clock::now();
    int dt_ms = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_joy_time).count());
    last_joy_time = now;

    if (has_joy) {
      leju::vr::QuestJoystickData joy_copy;
      {
        std::lock_guard<std::mutex> lock(joy_mutex);
        joy_copy = latest_joy;
      }

      // X + Y (left first + left second) triggers immediate stop.
      const bool xy_pressed =
          joy_copy.left_first_button_pressed && joy_copy.left_second_button_pressed;
      if (xy_pressed) {
        std::cout << "[VR] X+Y pressed, stopping robot (RL back semantics)..." << std::endl;
        triggerEmergencyStopAndExit(vr_api);
      }
      if (!g_running) {
        continue;
      }

      auto action = fsm.update(joy_copy, dt_ms);
      bool arm_mode_changed = false;

      if (action.request_set_arm_mode) {
        const char* mode_str =
            (action.arm_mode == 0) ? "KeepPose" : (action.arm_mode == 1) ? "Auto" : "External";
        const char* event_str = action.xa_pressed_event ? "X+A 按下"
            : action.xb_pressed_event ? "X+B 按下"
            : "setArmMode";
        std::cout << "[VR] " << event_str << " 调用 setArmMode(" << mode_str << ")..."
                  << std::endl;
        bool ok = vr_api.setArmMode(static_cast<leju::vr::ControlMode>(action.arm_mode), 1000);
        std::cout << "[VR] setArmMode(" << mode_str << ") 结果: " << (ok ? "成功" : "失败")
                  << std::endl;
        if (ik_incremental) {
          ik_incremental->setArmMode(action.arm_mode);
        } else if (ik_absolute) {
          ik_absolute->setArmMode(action.arm_mode);
        }

        current_arm_mode = action.arm_mode;
        arm_mode_changed = true;
      }

      const auto vr_arm_ctrl_state =
          arm_ctrl_mode_fsm.update(current_arm_mode, arm_mode_changed);
      leju::ik::ArmControlModeState ik_arm_ctrl_state;
      ik_arm_ctrl_state.left_active = vr_arm_ctrl_state.left_active;
      ik_arm_ctrl_state.right_active = vr_arm_ctrl_state.right_active;
      ik_arm_ctrl_state.left_changed = vr_arm_ctrl_state.left_changed;
      ik_arm_ctrl_state.right_changed = vr_arm_ctrl_state.right_changed;

      bool left_grip = joy_copy.left_grip > GRIP_THRESHOLD;
      bool right_grip = joy_copy.right_grip > GRIP_THRESHOLD;
      if (left_grip != prev_left_grip) {
        std::cout << "[VR] 左手握把 " << (left_grip ? "按下" : "松开") << std::endl;
        prev_left_grip = left_grip;
      }
      if (right_grip != prev_right_grip) {
        std::cout << "[VR] 右手握把 " << (right_grip ? "按下" : "松开") << std::endl;
        prev_right_grip = right_grip;
      }

      leju::ik::JoyStickData ik_joy = leju::vr_control::toJoyStickData(joy_copy);
      if (ik_incremental) {
        ik_incremental->onArmCtrlModeState(ik_arm_ctrl_state);

        if (ik_arm_ctrl_state.left_changed || ik_arm_ctrl_state.right_changed) {
          std::cout << "[VR] 手臂控制模式 "
                    << (ik_arm_ctrl_state.left_active ? "进入 External，左右臂可控"
                                        : "退出 External，左右臂停用")
                    << std::endl;
        }

        ik_incremental->onJoystick(ik_joy);
      } else if (ik_absolute) {
        ik_absolute->onJoystick(ik_joy);
      }

      leju::vr::VrVelocityCmd vel;
      vel.linear_x = joy_copy.left_y;
      vel.linear_y = -joy_copy.left_x;
      vel.angular_z = -joy_copy.right_x;
      vr_api.publishVrVelocityCmd(vel);
    }

    if (has_bones) {
      leju::vr::QuestBonePosesData bones_copy;
      {
        std::lock_guard<std::mutex> lock(bone_mutex);
        bones_copy = latest_bones;
      }

      if (bones_copy.is_high_confidence) {
        std::vector<double> head_q;
        if (leju::vr_control::computeHeadFromBones(bones_copy, head_q)) {
          leju::vr::JointTrajectoryPoint head_cmd;
          head_cmd.q = head_q;
          head_cmd.v.resize(head_q.size(), 0.0);
          head_cmd.acc.resize(head_q.size(), 0.0);
          vr_api.publishHeadJointCmd(head_cmd);
        }

        leju::ik::PoseInfoList pil = leju::vr_control::toPoseInfoList(bones_copy);
        if (ik_incremental) {
          ik_incremental->onBonePoses(pil);
        } else if (ik_absolute) {
          ik_absolute->onBonePoses(pil);
        }
      }
    }

    if (ik_incremental) {
      ik_incremental->runOnce();
    } else if (ik_absolute) {
      ik_absolute->runOnce();
    }

    std::this_thread::sleep_for(interval);
  }

  restoreControlModesToAutoOnExit(vr_api);
  leju::GlobalRobot::getInstance().shutdown();
  vr_api.shutdown();
  std::cout << "quest_vr_control_node exited." << std::endl;
  return 0;
}
