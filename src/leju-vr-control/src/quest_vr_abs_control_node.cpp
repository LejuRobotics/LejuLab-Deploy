/**
 * @file quest_vr_abs_control_node.cpp
 * @brief Quest3 VR 绝对式手臂控制节点（Roban/biped_s17 专用）
 *
 * 功能：
 *   1. 订阅 Quest3 骨骼位姿（QuestBonePoses DDS 话题）
 *   2. 通过 QuestArmRobotMapper（内嵌 Quest3ArmInfoTransformer）将骨骼转为 IK 用手/肘目标
 *   3. 调用 ArmAbsoluteIK（基于 Drake InverseKinematics + SNOPT）求解关节角
 *   4. 通过 RobanVRAPI 发布手臂关节指令
 *
 * 对应 Python 实现：ik_ros_uni.py 中 IkRos 类（IkTypeIdx.TorsoIK, as_mc_ik=True）
 *
 * 环境变量：
 *   LEJU_ASSETS_PATH   手臂模型资源根目录，需包含
 *                      models/biped_s17/urdf/drake/biped_v3_arm.urdf
 *   ROBOT_VERSION      机器人版本（如 "17"），默认 "17"
 *   VR_CONTROL_DT      控制周期（秒），默认 0.01（100Hz）
 *   VR_STATE_FILE      若设置，将每帧 IK 结果写入该 JSON 文件（供 MuJoCo 可视化）；
 *                      可选写入缩放前躯干系手肘坐标（*_pre_scale），见 mujoco_viewer.py
 *
 * 几何标定：QuestVrCalibration（按 ROBOT_VERSION）写入 QuestArmRobotMapperConfig，再驱动映射器 + IK eef_z。
 *   "17"、"14" → roban_v14/kuavo.json；其它 → 原默认；肩 link / STL 名采用 QuestArmRobotMapper::bipedArmDefault()
 *   VR_SKIP_OK_GESTURE 设为 1 时跳过「等待 OK」阶段（联调 DDS / 无手柄时用）
 *   VR_DEBUG_JOYSTICK  设为 1 时在等待 OK 阶段周期性打印扳机/握把数值
 *
 * 启动遥操作（与 motion_capture_ik quest3_utils 手柄分支一致）：
 *   双手扳机同时按住（值 > 0.5）并保持约 50 帧骨骼处理周期（约 1～2 秒）。
 */

#include <leju-ik/ArmAbsoluteIK.h>
#include <leju-ik/QuestArmRobotMapper.h>
#include <leju-ik/QuestVrCalibration.h>
#include <leju-ik/ik_types.h>
#include <lejusdk-lowlevel/leju_sdk.h>
#include <lejusdk-vr/lejusdk_vr.h>
#include <lejusdk-utils/robot_version.hpp>
#include <nlohmann/json.hpp>

#include "leju-vr-control/quest_to_ik_converter.h"
#include "leju-vr-control/quest_vr_abs_fsm.h"
#include "leju-vr-control/head_solver.h"

#include <Eigen/Dense>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace {

std::atomic<bool> g_running{true};

void signalHandler(int) { g_running = false; }

std::string getEnv(const char* name, const std::string& default_val = "") {
  const char* val = std::getenv(name);
  if (val && val[0] != '\0') return std::string(val);
  return default_val;
}

double getEnvDouble(const char* name, double default_val) {
  std::string s = getEnv(name);
  if (s.empty()) return default_val;
  try { return std::stod(s); } catch (...) { return default_val; }
}

/// 原子性写入 IK 状态到 JSON 文件（先写 .tmp 再 rename）
/// viz_debug：可选，写入缩放前躯干系手肘坐标（供 mujoco_viewer 与 IK target 对比）
void writeStateFile(
    const std::string& path,
    const Eigen::VectorXd& joints,
    const std::optional<Eigen::Vector3d>& l_hand,
    const std::optional<Eigen::Vector3d>& r_hand,
    const std::optional<Eigen::Vector3d>& l_elbow,
    const std::optional<Eigen::Vector3d>& r_elbow,
    bool ik_success,
    const leju::ik::Quest3ArmInfoTransformer::VisualizationData* viz_debug = nullptr) {
  using json = nlohmann::json;
  json j;
  j["timestamp"] = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  j["ik_success"] = ik_success;

  if (ik_success && joints.size() > 0) {
    j["arm_joints_rad"] = std::vector<double>(joints.data(), joints.data() + joints.size());
  } else {
    j["arm_joints_rad"] = std::vector<double>(8, 0.0);
  }

  auto toArr = [](const std::optional<Eigen::Vector3d>& v) -> json {
    if (v) return {(*v)[0], (*v)[1], (*v)[2]};
    return {0.0, 0.0, 0.0};
  };
  auto v3 = [](const Eigen::Vector3d& v) -> json { return {v.x(), v.y(), v.z()}; };

  j["left_hand_target"]  = toArr(l_hand);
  j["right_hand_target"] = toArr(r_hand);
  j["left_elbow_target"] = toArr(l_elbow);
  j["right_elbow_target"]= toArr(r_elbow);

  if (viz_debug != nullptr) {
    j["left_hand_pre_scale"] = v3(viz_debug->leftHandPreScale);
    j["right_hand_pre_scale"] = v3(viz_debug->rightHandPreScale);
    j["left_elbow_pre_scale"] = v3(viz_debug->leftElbowPreScale);
    j["right_elbow_pre_scale"] = v3(viz_debug->rightElbowPreScale);
  }

  const std::string tmp = path + ".tmp";
  {
    std::ofstream ofs(tmp);
    if (ofs) ofs << j.dump(2);
  }
  std::rename(tmp.c_str(), path.c_str());
}

}  // namespace

// ============================================================
// 主函数
// ============================================================
int main(int argc, char** argv) {
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  // ------- 读取配置 -------
  const std::string robot_version_str = getEnv("ROBOT_VERSION", "17");
  const std::string assets_path = getEnv("LEJU_ASSETS_PATH");
  const double ctrl_dt = getEnvDouble("VR_CONTROL_DT", 0.01);
  const leju::ik::QuestVrCalibration cal =
      leju::ik::calibrationForRobotVersion(robot_version_str);
  const std::string state_file = getEnv("VR_STATE_FILE");  // 空字符串 = 不写文件

  const leju::ik::QuestArmRobotMapperConfig mapper_cfg =
      leju::ik::QuestArmRobotMapperConfig::fromQuestVrCalibration(cal);

  std::cout << "[VrAbsCtrl] robot_version=" << robot_version_str
            << "  ctrl_dt=" << ctrl_dt << "s" << std::endl;
  std::cout << "[VrAbsCtrl] QuestVrCalibration → mapper: upper=" << mapper_cfg.upper_arm_length
            << " lower=" << mapper_cfg.lower_arm_length
            << " shoulder_w=" << mapper_cfg.shoulder_width
            << " base_h=" << mapper_cfg.base_height_offset
            << " base_chest_x=" << mapper_cfg.base_chest_offset_x
            << " eef_z=" << mapper_cfg.eef_z_offset << std::endl;
  std::cout << "[VrAbsCtrl] mapper shoulder_frames: " << mapper_cfg.shoulder_frame_left << " / "
            << mapper_cfg.shoulder_frame_right << std::endl;
  if (!state_file.empty()) {
    std::cout << "[VrAbsCtrl] State file: " << state_file << std::endl;
  }

  // ------- 构造 URDF 路径 -------
  std::string urdf_path;
  if (!assets_path.empty()) {
    urdf_path = assets_path + "/models/biped_s" + robot_version_str +
                "/urdf/drake/biped_v3_arm.urdf";
  } else {
    std::cerr << "[VrAbsCtrl] LEJU_ASSETS_PATH not set. " << std::endl;
    return 1;
  }
  std::cout << "[VrAbsCtrl] URDF path: " << urdf_path << std::endl;

  // ------- 初始化 IK 求解器 -------
  leju::ik::ArmAbsoluteIKConfig ik_config;
  ik_config.eef_z_bias = mapper_cfg.eef_z_offset;
  std::unique_ptr<leju::ik::ArmAbsoluteIK> arm_ik;
  try {
    arm_ik = std::make_unique<leju::ik::ArmAbsoluteIK>(urdf_path, ik_config);
    arm_ik->setTorsoState(0.0, 0.0);
  } catch (const std::exception& e) {
    std::cerr << "[VrAbsCtrl] Failed to initialize ArmAbsoluteIK: " << e.what()
              << std::endl;
    return 1;
  }
  std::cout << "[VrAbsCtrl] ArmAbsoluteIK initialized. nq=" << arm_ik->numPositions()
            << std::endl;

  // ------- Quest3 骨骼 → 机器人手/肘 IK 目标（QuestArmRobotMapper）-------
  leju::ik::QuestArmRobotMapper arm_mapper(mapper_cfg);
  arm_mapper.setRunning(false);  // 等待 OK 手势

  // ------- 初始化 VR API（RobanVRAPI 用于 biped_s17）-------
  leju::RobotVersion vr_robot_version;
  try {
    vr_robot_version = leju::RobotVersion::from_env();
  } catch (...) {
    // 默认 biped_s17 = major 1 minor 7
    vr_robot_version = leju::RobotVersion(1, 7);
  }

  // 初始化 lejusdk-lowlevel（供 X+Y 退出时 publishStopRobot 让机器人程序退出，与增量节点一致）。
  if (!leju::GlobalRobot::init_env(vr_robot_version)) {
    std::cerr << "[VrAbsCtrl] Failed to initialize lejusdk-lowlevel (GlobalRobot)" << std::endl;
    return 1;
  }

  leju::vr::RobanVRAPI vr_api(vr_robot_version);
  if (!vr_api.initialize()) {
    std::cerr << "[VrAbsCtrl] Failed to initialize RobanVRAPI" << std::endl;
    return 1;
  }
  std::cout << "[VrAbsCtrl] RobanVRAPI initialized." << std::endl;

  // ------- 共享数据 -------
  std::mutex bones_mutex;
  leju::vr::QuestBonePosesData latest_bones;
  bool has_bones = false;

  std::mutex joy_mutex;
  leju::vr::QuestJoystickData latest_joy;
  bool has_joy = false;

  // 手臂模式（0=KeepPose / 1=Auto / 2=External）由 X+A / X+B 绝对式 FSM 切换；
  // 主循环仅在 External 且 OK 手势解锁(arm_mapper.isRunning())时下发外部关节指令。
  leju::vr_control::QuestVrAbsFSM arm_fsm;
  std::atomic<int> current_arm_mode{static_cast<int>(leju::vr::ControlMode::kAuto)};

  // ------- 订阅回调 -------
  vr_api.subscribeQuestBonePoses([&](const leju::vr::QuestBonePosesData& data) {
    std::lock_guard<std::mutex> lk(bones_mutex);
    latest_bones = data;
    has_bones = true;
  });

  vr_api.subscribeQuestJoystickData([&](const leju::vr::QuestJoystickData& data) {
    // 组合键统一交给绝对式 FSM 解析（X+A/X+B 切手臂模式，X+Y 退出，均边沿触发）。
    leju::vr_control::QuestVrAbsFSMAction act =
        arm_fsm.update(data, current_arm_mode.load());

    // X+Y = 退出程序（与 quest_vr_control_node 一致）：先 StopRobot 让机器人程序退出，
    // 再退本 VR 进程；统一走 shutdown 流程把手臂切回 kAuto。等待首帧/主循环两阶段都可触发。
    if (act.request_quit) {
      std::cout << "[VrAbsCtrl] X+Y pressed — StopRobot 并退出 VR 绝对控制..." << std::endl;
      leju::GlobalRobot::getInstance().publishStopRobot();
      g_running = false;
    }

    // X+A / X+B = 绝对式手臂模式切换（外部/自动/保持）。
    if (act.request_set_arm_mode) {
      const char* mode_str = (act.arm_mode == 0) ? "KeepPose"
                           : (act.arm_mode == 1) ? "Auto" : "External";
      const char* ev = act.xa_pressed_event ? "X+A" : act.xb_pressed_event ? "X+B" : "?";
      std::cout << "[VrAbsCtrl] " << ev << " → setArmMode(" << mode_str << ")" << std::endl;
      vr_api.setArmMode(static_cast<leju::vr::ControlMode>(act.arm_mode), 1000);
      // 跟随门控与模式同步：External 时开始 IK 跟随；切出 External 时停止。
      arm_mapper.setRunning(act.arm_mode ==
                            static_cast<int>(leju::vr::ControlMode::kExternal));
      current_arm_mode.store(act.arm_mode);
    }

    std::lock_guard<std::mutex> lk(joy_mutex);
    latest_joy = data;
    has_joy = true;
    arm_mapper.updateJoystickForGesture(data.left_trigger, data.left_grip,
                                        data.right_trigger, data.right_grip);
  });

  // 初始为自动摆手模式（kAuto）；进入外部控制由 X+A 切换。
  if (!vr_api.setArmMode(leju::vr::ControlMode::kAuto, 3000)) {
    std::cerr << "[VrAbsCtrl] Warning: failed to set arm mode to Auto" << std::endl;
  } else {
    std::cout << "[VrAbsCtrl] Arm mode set to Auto (按 X+A 进入外部控制)." << std::endl;
  }

  // 头部跟随：设为外部控制，主循环每帧由头显姿态驱动头部关节。
  if (!vr_api.setHeadMode(leju::vr::ControlMode::kExternal, 3000)) {
    std::cerr << "[VrAbsCtrl] Warning: failed to set head mode to External" << std::endl;
  } else {
    std::cout << "[VrAbsCtrl] Head mode set to External (头部跟随头显)." << std::endl;
  }

  // ------- 等待第一帧骨骼数据 -------
  std::cout << "[VrAbsCtrl] Waiting for first QuestBonePoses frame..." << std::endl;
  while (g_running && !has_bones) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (!g_running) {
    vr_api.setArmMode(leju::vr::ControlMode::kAuto, 1000);
    vr_api.setHeadMode(leju::vr::ControlMode::kAuto, 1000);
    vr_api.shutdown();
    leju::GlobalRobot::getInstance().shutdown();
    return 0;
  }
  std::cout << "[VrAbsCtrl] First frame received." << std::endl;
  std::cout << "[VrAbsCtrl] 操作：双手扳机同时按下（>0.5）保持约 1～2 秒做 OK 手势，"
               "即进入外部控制、手臂开始跟随；"
               "X+A 回到自动摆手（再按或保持下按可进外部）；"
               "X+B 在保持/自动间切换；X+Y 退出。"
            << std::endl;

  const bool skip_ok = (getEnv("VR_SKIP_OK_GESTURE") == "1");
  const bool dbg_joy = (getEnv("VR_DEBUG_JOYSTICK") == "1");
  if (skip_ok) {
    std::cout << "[VrAbsCtrl] VR_SKIP_OK_GESTURE=1 — 跳过 OK 手势，直接进入外部控制。"
              << std::endl;
    vr_api.setArmMode(leju::vr::ControlMode::kExternal, 1000);
    current_arm_mode.store(static_cast<int>(leju::vr::ControlMode::kExternal));
    arm_mapper.setRunning(true);
  }

  // ------- 主控制循环 -------
  // OK 手势（arm_mapper.isRunning() 变为 true）与 X+A 进外部等效：均切到 kExternal 并开始 IK 跟随。
  // 仅在 kExternal 模式下求解并下发外部关节指令；其余模式手臂交由控制器（Auto/KeepPose）处理。
  Eigen::VectorXd q_last = arm_ik->defaultPositions();
  int run_count = 0, fail_count = 0;
  double sum_time_ms = 0.0;
  bool prev_following = false;
  int idle_dbg_iter = 0;

  const auto loop_dt = std::chrono::duration<double>(ctrl_dt);

  while (g_running) {
    auto loop_start = std::chrono::steady_clock::now();

    // 1. 获取最新骨骼数据
    leju::vr::QuestBonePosesData bones_copy;
    {
      std::lock_guard<std::mutex> lk(bones_mutex);
      bones_copy = latest_bones;
    }

    // 1b. 头部跟随（与手臂模式无关，每帧由头显姿态驱动）
    if (bones_copy.is_high_confidence) {
      std::vector<double> head_q;
      if (leju::vr_control::computeHeadFromBones(bones_copy, head_q)) {
        leju::vr::JointTrajectoryPoint head_cmd;
        head_cmd.q = head_q;
        head_cmd.v.resize(head_q.size(), 0.0);
        head_cmd.acc.resize(head_q.size(), 0.0);
        vr_api.publishHeadJointCmd(head_cmd);
      }
    }

    // 2. 转换为 leju-ik PoseInfoList，并推进 OK 手势检测（即使当前非外部也要算，
    //    以便 OK 手势能把模式提升到 External）。
    leju::ik::PoseInfoList pose_list = leju::vr_control::toPoseInfoList(bones_copy);
    leju::ik::RobotArmIkTargets ik_targets;
    const bool mapped =
        arm_mapper.mapBonesToRobotTargets(pose_list, ik_targets) && ik_targets.valid;

    // 2b. OK 手势解锁 = 进入外部控制（与 X+A 进外部等效）。
    if (arm_mapper.isRunning() &&
        current_arm_mode.load() != static_cast<int>(leju::vr::ControlMode::kExternal)) {
      std::cout << "\n[VrAbsCtrl] OK 手势 → 进入外部控制，手臂开始跟随。" << std::endl;
      vr_api.setArmMode(leju::vr::ControlMode::kExternal, 1000);
      current_arm_mode.store(static_cast<int>(leju::vr::ControlMode::kExternal));
    }

    // 2c. 跟随门控：仅 External 模式下求解/下发。
    const bool following =
        current_arm_mode.load() == static_cast<int>(leju::vr::ControlMode::kExternal);
    if (!following) {
      if (prev_following) {
        std::cout << "\n[VrAbsCtrl] 退出外部控制，停止手臂跟随。" << std::endl;
        prev_following = false;
      }
      if (dbg_joy && (++idle_dbg_iter % 100 == 0)) {
        std::cout << "[VrAbsCtrl] 当前非外部模式（OK 手势 / X+A 可进外部控制）。" << std::endl;
      }
      std::this_thread::sleep_for(loop_dt);
      continue;
    }
    if (!prev_following) {
      std::cout << "\n[VrAbsCtrl] 外部控制中，手臂跟随 VR。" << std::endl;
      prev_following = true;
    }

    if (!mapped) {
      std::this_thread::sleep_for(loop_dt);
      continue;
    }

    // mapBonesToRobotTargets 成功且 valid 时四元位置均已就绪
    const std::optional<Eigen::Vector3d> l_hand_pos(ik_targets.left_hand);
    const std::optional<Eigen::Vector3d> r_hand_pos(ik_targets.right_hand);
    const std::optional<Eigen::Vector3d> l_elbow_pos(ik_targets.left_elbow);
    const std::optional<Eigen::Vector3d> r_elbow_pos(ik_targets.right_elbow);

    // 4. 执行IK求解
    const auto t0 = std::chrono::steady_clock::now();
    Eigen::VectorXd q_sol = arm_ik->computeIK(
        l_hand_pos, r_hand_pos, l_elbow_pos, r_elbow_pos, q_last);
    const double time_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0)
            .count();

    run_count++;
    sum_time_ms += time_ms;

    if (q_sol.size() == 0) {
      // IK 失败，保持上一帧
      fail_count++;
      if (run_count % 50 == 0) {
        std::cout << "\r[VrAbsCtrl] IK success rate: "
                  << 100.0 * (1.0 - static_cast<double>(fail_count) / run_count)
                  << "%, avg=" << sum_time_ms / run_count << "ms" << std::flush;
      }
      std::this_thread::sleep_for(loop_dt);
      continue;
    }

    q_last = q_sol;

    // 5. 发布关节指令（单位：弧度）
    leju::vr::JointTrajectoryPoint cmd;
    cmd.q.assign(q_sol.data(), q_sol.data() + q_sol.size());
    // 速度和加速度设为0（跟随模式下由底层插值）
    cmd.v.assign(q_sol.size(), 0.0);
    cmd.acc.assign(q_sol.size(), 0.0);
    vr_api.publishArmJointCmd(cmd);

    // 5b. 可选：写入状态文件供 MuJoCo 可视化（每帧都写，开销极低）
    if (!state_file.empty()) {
      writeStateFile(state_file, q_sol,
                     l_hand_pos, r_hand_pos, l_elbow_pos, r_elbow_pos,
                     /*ik_success=*/true,
                     &arm_mapper.transformer().getVisualizationData());
    }

    if (run_count % 100 == 0) {
      std::cout << "\r[VrAbsCtrl] IK success rate: "
                << 100.0 * (1.0 - static_cast<double>(fail_count) / run_count)
                << "%, avg=" << sum_time_ms / run_count << "ms  q_sol=" << q_sol.transpose() << "  \nl_hand_pos=" << l_hand_pos.value().transpose() << "  \nr_hand_pos=" << r_hand_pos.value().transpose() << "  l_elbow_pos=" << l_elbow_pos.value().transpose() << "  r_elbow_pos=" << r_elbow_pos.value().transpose() << std::flush;
    }

    // 6. 控制周期等待
    const auto elapsed =
        std::chrono::steady_clock::now() - loop_start;
    if (elapsed < loop_dt) {
      std::this_thread::sleep_for(loop_dt - elapsed);
    }
  }

  // ------- 退出时恢复自动模式 -------
  std::cout << "\n[VrAbsCtrl] Shutting down..." << std::endl;
  vr_api.setArmMode(leju::vr::ControlMode::kAuto, 1000);
  vr_api.setHeadMode(leju::vr::ControlMode::kAuto, 1000);
  vr_api.shutdown();
  leju::GlobalRobot::getInstance().shutdown();
  return 0;
}
