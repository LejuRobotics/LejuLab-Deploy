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
 * 人体下蹲控制（通过骨骼 Chest Z → /rt/posture_height_cmd，对标遥控器右摇杆 Y 下蹲）：
 *   VR_SQUAT_ENABLED=1     启用人体下蹲控制
 *   VR_SQUAT_STANDING_HEIGHT=0.77 机器人站立姿态高度（米）
 *   VR_SQUAT_HEIGHT_MIN=-0.21     最大下蹲偏移（米，负值=下蹲，对标 squat_height_min）
 *   VR_SQUAT_HEIGHT_MAX=0.01      最大站起偏移（米，对标 squat_height_max）
 *   VR_SQUAT_DEADZONE=0.03        下蹲死区（米），小于此值不触发
 *   VR_SQUAT_MAX=0.2              最大下蹲量限制（米）
 *   VR_SQUAT_CALIB_FRAMES=100     标定帧数，前 N 帧取平均作为站立基准高度
 *   VR_SQUAT_FILTER_ALPHA_DOWN=0.15 下蹲时低通滤波系数（快速跟随）
 *   VR_SQUAT_FILTER_ALPHA_UP=0.08   站起时低通滤波系数（缓慢回零）
 *   VR_SQUAT_OUTPUT_DEADZONE=0.02   输出死区（米），截断滤波残余小值
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
#include <lejusdk-utils/time_utils.hpp>
#include <nlohmann/json.hpp>

#include "leju-vr-control/quest_to_ik_converter.h"
#include "leju-vr-control/quest_vr_abs_fsm.h"
#include "leju-vr-control/head_solver.h"

#include <Eigen/Dense>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
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

/// 避免单独重启 VR 程序时误触发机器人急停。
std::atomic<bool> g_stop_robot_requested{false};

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
  // 手臂关节角一阶低通滤波：
  const double arm_filter_alpha = getEnvDouble("VR_ARM_FILTER_ALPHA", 0.3);
  const double arm_filter_output_deadzone = getEnvDouble("VR_ARM_FILTER_OUTPUT_DEADZONE", 0.03);
  if (arm_filter_alpha < 0.0 || arm_filter_alpha > 1.0) {
    std::cerr << "[VrAbsCtrl] VR_ARM_FILTER_ALPHA 必须在 [0,1] 内，使用默认 0.3" << std::endl;
  }

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
    // 默认从 CMake 编译期路径读取（回退）
    urdf_path = std::string(LEJU_ASSETS_URDF_PATH);
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
  // 骨骼新鲜度: 每次收到骨骼帧就刷新时间戳。Quest 断开/骨骼停发后,
  // 时间戳不再更新, 主循环据此判定骨骼失效, 停发头部指令。
  // 这样即使本进程残留(孤儿进程), 运控的 0.5s 头部超时也能触发,
  // 自动回退到手柄控制, 不再依赖进程被杀干净。
  // 初始化为 min(): 首帧到达前不过期, 避免首帧(若延迟>200ms)被新鲜度误拦。
  std::chrono::steady_clock::time_point last_bone_rx_time =
      std::chrono::steady_clock::time_point::min();
  constexpr std::chrono::milliseconds kBoneFreshWindow{200};  // < 运控 0.5s 超时, 留裕量防抖
  vr_api.subscribeQuestBonePoses([&](const leju::vr::QuestBonePosesData& data) {
    std::lock_guard<std::mutex> lk(bones_mutex);
    latest_bones = data;
    has_bones = true;
    last_bone_rx_time = std::chrono::steady_clock::now();
  });
  std::atomic<bool> squat_event_enabled{false};
  std::atomic<bool> reset_init_chest_yaw_{true};
  std::atomic<bool> first_OK_flag{false}; // 用于首次OK手势解锁手臂，随后进入自锁防止重复按双扳机解锁手臂
  bool lock_hand{false};                  // 用于锁定灵巧手
  std::atomic<bool> mask_filter{true}; // 用于首次OK手势解锁手臂，随后进入自锁防止重复按双扳机解锁手臂
  vr_api.subscribeQuestJoystickData([&](const leju::vr::QuestJoystickData& data) {
    // 组合键统一交给绝对式 FSM 解析（X+A/X+B 切手臂模式，X+Y 退出，均边沿触发）。
    leju::vr_control::QuestVrAbsFSMAction act =
        arm_fsm.update(data, current_arm_mode.load());

    // X+Y = 退出程序（与 quest_vr_control_node 一致）：先 StopRobot 让机器人程序退出，
    // 再退本 VR 进程；统一走 shutdown 流程把手臂切回 kAuto。等待首帧/主循环两阶段都可触发。
    if (act.request_quit) {
      std::cout << "[VrAbsCtrl] X+Y pressed — StopRobot 并退出 VR 绝对控制..." << std::endl;
      g_stop_robot_requested = true;  // X+Y 主动退出：允许下发 StopRobot
      leju::GlobalRobot::getInstance().publishStopRobot();
      g_running = false;
    }

    // X+A / X+B = 绝对式手臂模式切换（外部/自动/保持）。
    if (act.request_set_arm_mode && first_OK_flag.load()) {
      const char* mode_str = (act.arm_mode == 0) ? "KeepPose"
                           : (act.arm_mode == 1) ? "Auto" : "External";
      const char* ev = act.xa_pressed_event ? "X+A" : act.xb_pressed_event ? "X+B" : "?";
      std::cout << "[VrAbsCtrl] " << ev << " → setArmMode(" << mode_str << ")" << std::endl;
      vr_api.setArmMode(static_cast<leju::vr::ControlMode>(act.arm_mode), 1000);
      // 跟随门控与模式同步：External 时开始 IK 跟随；切出 External 时停止。
      arm_mapper.setRunning(act.arm_mode ==
                            static_cast<int>(leju::vr::ControlMode::kExternal));
      current_arm_mode.store(act.arm_mode);
      if (act.arm_mode == static_cast<int>(leju::vr::ControlMode::kExternal)) {
        reset_init_chest_yaw_.store(true);
        mask_filter.store(true);
      }
    }
    
    if (act.xy_touched_event) {
      squat_event_enabled.store(true);
    }else{
      squat_event_enabled.store(false);
    }

    std::lock_guard<std::mutex> lk(joy_mutex);
    latest_joy = data;
    has_joy = true;
    arm_mapper.updateJoystickForGesture(data.left_trigger, data.left_grip,
                                        data.right_trigger, data.right_grip);

    // --- 灵巧手控制: 扳机→同侧手6指全动 ---
    if (act.y_pressed_event && first_OK_flag.load()) {
      lock_hand = !lock_hand;
      std::cout << "[VrAbsCtrl] Hand is " << (lock_hand ? "locked" : "unlocked") << std::endl;
    }
    if(!lock_hand){
      static float prev_lt = -1.0f, prev_rt = -1.0f;
      if (data.left_trigger != prev_lt || data.right_trigger != prev_rt) {
          prev_lt = data.left_trigger;
          prev_rt = data.right_trigger;
          static int hand_cmd_count = 0;
          if(++hand_cmd_count % 50 == 0){
            std::cout << "[VrAbsCtrl] HandCmd: left_trigger=" << prev_lt
                      << " right_trigger=" << prev_rt << std::endl;
          }
        leju::HandCmd hand_cmd;
        for (int i = 0; i < 6; i++) {
          hand_cmd.position[i]     = static_cast<double>(prev_lt  * 100.0f);
          hand_cmd.position[6 + i] = static_cast<double>(prev_rt * 100.0f);
        }
        hand_cmd.timestamp = leju::common::GetUnixTimestampS();
        leju::GlobalRobot::getInstance().publishHandCmd(hand_cmd);
      }
    }
    // --- 灵巧手控制 end ---
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
    // 仅 X+Y 主动退出才 StopRobot；Ctrl+C 单独退出 VR 程序时不急停机器人
    if (g_stop_robot_requested.load()) {
      leju::GlobalRobot::getInstance().shutdown();
    }
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

  // ------- 人体下蹲控制参数 -------
  // 通过骨骼 Chest 节点 Z 坐标变化计算下蹲量，发布到 /rt/posture_height_cmd。
  // rl_controller 侧 ExternalInterface 接收后自动设置 cmd_stance=1 并调整身体高度，
  // 对标遥控器 X+Y 触摸右摇杆 Y 轴下蹲控制。
  const bool squat_enabled = (getEnv("VR_SQUAT_ENABLED", "0") == "1");
  const double squat_standing_height = getEnvDouble("VR_SQUAT_STANDING_HEIGHT", 0.77);     // 站立姿态高度（米）
  const double squat_height_min = getEnvDouble("VR_SQUAT_HEIGHT_MIN", -0.21);             // 最大下蹲偏移（米）
  const double squat_height_max = getEnvDouble("VR_SQUAT_HEIGHT_MAX", 0.01);              // 最大站起偏移（米）
  const double squat_deadzone = getEnvDouble("VR_SQUAT_DEADZONE", 0.03);                  // 下蹲死区（米）
  const double squat_max = getEnvDouble("VR_SQUAT_MAX", 0.2);                             // 最大下蹲量限制（米）
  const int squat_calib_frames = static_cast<int>(getEnvDouble("VR_SQUAT_CALIB_FRAMES", 100));
  const double squat_filter_alpha_down = getEnvDouble("VR_SQUAT_FILTER_ALPHA_DOWN", 0.15); // 下蹲低通
  const double squat_filter_alpha_up = getEnvDouble("VR_SQUAT_FILTER_ALPHA_UP", 0.08);     // 站起低通
  const double squat_output_deadzone = getEnvDouble("VR_SQUAT_OUTPUT_DEADZONE", 0.02);     // 输出死区（米）
  double squat_baseline_z = 0.0;  // 站立基准高度（标定后填充）
  int squat_calib_count = 0;
  double squat_depth_filtered = 0.0;  // 滤波后的下蹲量
  if (squat_enabled) {
    std::cout << "[VrAbsCtrl] 人体下蹲控制已启用 → /rt/posture_height_cmd" 
              << "(VR_SQUAT_ENABLED=1) ,右摇杆下蹲未启用"
              << " (standing=" << squat_standing_height
              << " min_offset=" << squat_height_min
              << " max_offset=" << squat_height_max
              << " deadzone=" << squat_deadzone << " max=" << squat_max
              << " filter_down=" << squat_filter_alpha_down
              << " filter_up=" << squat_filter_alpha_up << ")" << std::endl;
  } else {
    std::cout << "[VrAbsCtrl] 右摇杆下蹲控制已启用 → /rt/posture_height_cmd"
              << " (VR_SQUAT_ENABLED=0，人体下蹲控制未启用)"
              << " (standing=" << squat_standing_height
              << " min_offset=" << squat_height_min
              << " max_offset=" << squat_height_max
              << " deadzone=" << squat_deadzone << " max=" << squat_max
              << " filter_down=" << squat_filter_alpha_down
              << " filter_up=" << squat_filter_alpha_up << ")" << std::endl;
  }

  if (skip_ok) {
    std::cout << "[VrAbsCtrl] VR_SKIP_OK_GESTURE=1 — 跳过 OK 手势，直接进入外部控制。"
              << std::endl;
    vr_api.setArmMode(leju::vr::ControlMode::kExternal, 1000);
    current_arm_mode.store(static_cast<int>(leju::vr::ControlMode::kExternal));
    arm_mapper.setRunning(true);
    first_OK_flag.store(true);
    reset_init_chest_yaw_.store(true);
  }

  // ------- 主控制循环 -------
  // OK 手势（arm_mapper.isRunning() 变为 true）与 X+A 进外部等效：均切到 kExternal 并开始 IK 跟随。
  // 仅在 kExternal 模式下求解并下发外部关节指令；其余模式手臂交由控制器（Auto/KeepPose）处理。
  Eigen::VectorXd q_last = arm_ik->defaultPositions();
  Eigen::VectorXd q_filtered = q_last;  // 一阶低通滤波状态（从默认位姿起步）
  int run_count = 0, fail_count = 0;
  double sum_time_ms = 0.0;
  bool prev_following = false;
  int idle_dbg_iter = 0;

  const auto loop_dt = std::chrono::duration<double>(ctrl_dt);
  bool torso_height_cali=false;
  while (g_running) {
    auto loop_start = std::chrono::steady_clock::now();

    // 1. 获取最新骨骼数据（锁内一并读时间戳, 避免 data race）
    leju::vr::QuestBonePosesData bones_copy;
    std::chrono::steady_clock::time_point bone_rx;
    {
      std::lock_guard<std::mutex> lk(bones_mutex);
      bones_copy = latest_bones;
      bone_rx = last_bone_rx_time;
    }

    // 骨骼新鲜才下发: Quest 断开/骨骼停发后时间戳不再刷新, 超过
    // kBoneFreshWindow 即停发头部/下蹲/手臂, 让运控 0.5s 超时回退到手柄。
    // 骨骼恢复后时间戳立即刷新, 下一帧自动恢复。
    const bool bone_fresh =
        (std::chrono::steady_clock::now() - bone_rx) < kBoneFreshWindow;
    if (bone_fresh && bones_copy.is_high_confidence) {
      std::vector<double> head_q;
      if (leju::vr_control::computeHeadFromBones(bones_copy, head_q)) {
        leju::vr::JointTrajectoryPoint head_cmd;
        head_cmd.q = head_q;
        head_cmd.v.resize(head_q.size(), 0.0);
        head_cmd.acc.resize(head_q.size(), 0.0);
        vr_api.publishHeadJointCmd(head_cmd);
      }
    }

    // 1c. 人体下蹲 → /rt/posture_height_cmd（对标遥控器右摇杆 Y 下蹲控制）
    //     利用骨骼 Chest 节点 Z 坐标变化计算下蹲量，映射为机器人站立高度。
    //     ExternalInterface::buildFreshPostureCommand() 自动设置 cmd_stance=1。
    //     0.5 秒超时未收到新指令则自动恢复站立。
    //     squat_event_enabled 由 X+Y 触摸边沿触发，避免手柄抖动导致频繁开关。
    if (bone_fresh && squat_enabled && bones_copy.is_high_confidence &&
        bones_copy.poses.size() > 23 ) {
      // Root=22, Chest=23, Neck=24, Head=25
      const double chest_z = static_cast<double>(bones_copy.poses[23].z);
      if (squat_event_enabled.load()) {
        // 标定阶段：前 N 帧取平均作为站立基准高度
        if (squat_calib_count < squat_calib_frames) {
          squat_baseline_z += chest_z;
          squat_calib_count++;
          if (!torso_height_cali){
            torso_height_cali=true;
            std::cout << "[VrAbsCtrl] 下蹲基准高度标定中。请保持站立姿态，持续约 "
                      << static_cast<double>(squat_calib_frames) * ctrl_dt
                      << " 秒..." << std::endl;
          }
          if (squat_calib_count == squat_calib_frames) {
            squat_baseline_z /= squat_calib_frames;
            std::cout << "[VrAbsCtrl] 下蹲基准高度标定完成: baseline_z="
                      << squat_baseline_z << std::endl;
          }
        } else {
          // 标定完成，用 Chest 的 Z 坐标计算下蹲量
          double squat_depth = squat_baseline_z - chest_z;  // 正值=下蹲
          // 死区过滤
          if (std::abs(squat_depth) < squat_deadzone) {
            squat_depth = 0.0;
          }
          // 限幅
          squat_depth = std::clamp(squat_depth, -squat_max, squat_max);
          // 非对称低通滤波：下蹲时快速跟随，站起时缓慢回零（防止摔倒）
          const double alpha = (squat_depth > squat_depth_filtered) ? squat_filter_alpha_down
                                                                    : squat_filter_alpha_up;
          squat_depth_filtered += alpha * (squat_depth - squat_depth_filtered);
          // 输出死区：截断滤波残余小值，防止人站直后机器人缓慢漂移
          if (std::abs(squat_depth_filtered) < squat_output_deadzone) {
            squat_depth_filtered = 0.0;
          }
          // 下蹲深度 → 机器人站立高度偏移（下蹲为正 depth，需取负作为高度偏移）
          const double height_offset = std::clamp(-squat_depth_filtered,
                                                  squat_height_min, squat_height_max);
          const double target_height = squat_standing_height + height_offset;

          leju::GlobalRobot::getInstance().publishPostureHeightCmd(target_height);

          static int squat_dbg_count = 0;
          if (++squat_dbg_count % 500 == 0) {
            std::cout << "[VrAbsCtrl] squat: chest_z=" << chest_z
                      << " baseline=" << squat_baseline_z
                      << " raw_depth=" << (squat_baseline_z - chest_z)
                      << " filtered=" << squat_depth_filtered
                      << " height=" << target_height << std::endl;
          }
        }
      } else {
        if(torso_height_cali){
          std::cout << "[VrAbsCtrl] 躯干映射下蹲已取消，下次进入需重新标定基准高度。" << std::endl;
          torso_height_cali=false;
        }
        squat_calib_count = 0;
      }
    }else if(bone_fresh && !squat_enabled && bones_copy.is_high_confidence &&
        bones_copy.poses.size() > 23 && squat_event_enabled.load()){
      double right_y = 0.0f;
      {
        std::lock_guard<std::mutex> lk(joy_mutex);
        if (has_joy) right_y = latest_joy.right_y;
      }
      // 摇杆死区：|right_y| < 0.05 视为回中，站直（摇杆范围 -1~1，单位非米）
      const double raw_squat = (std::abs(right_y) < 0.05) ? 0.0 : right_y;
      // 下蹲深度 → 机器人站立高度偏移（right_y 下推为负 → 高度降低 = 下蹲）
      const double height_offset =
          std::clamp(raw_squat * squat_max, squat_height_min, squat_height_max);
      const double target_height = squat_standing_height + height_offset;

      leju::GlobalRobot::getInstance().publishPostureHeightCmd(target_height);

      static int squat_dbg_count = 0;
      if (++squat_dbg_count % 500 == 0) {
        std::cout << "[VrAbsCtrl] squat: right_y=" << right_y
                  << " raw_squat=" << raw_squat
                  << " height=" << target_height << std::endl;
      }
    } 

    // 2. 转换为 leju-ik PoseInfoList，并推进 OK 手势检测（即使当前非外部也要算，
    //    以便 OK 手势能把模式提升到 External）。
    leju::ik::PoseInfoList pose_list = leju::vr_control::toPoseInfoList(bones_copy);
    leju::ik::RobotArmIkTargets ik_targets;
    if (reset_init_chest_yaw_.load()) {
      arm_mapper.resetInitChestYaw();
      reset_init_chest_yaw_.store(false);
    }
    const bool mapped =
        arm_mapper.mapBonesToRobotTargets(pose_list, ik_targets) && ik_targets.valid;

    // 2b. OK 手势解锁 = 进入外部控制（与 X+A 进外部等效）。
    if (arm_mapper.isRunning() && !first_OK_flag &&
        current_arm_mode.load() != static_cast<int>(leju::vr::ControlMode::kExternal)) {
      std::cout << "\n[VrAbsCtrl] OK 手势 → 进入外部控制，手臂开始跟随。" << std::endl;
      vr_api.setArmMode(leju::vr::ControlMode::kExternal, 1000);
      current_arm_mode.store(static_cast<int>(leju::vr::ControlMode::kExternal));
      first_OK_flag.store(true);
      reset_init_chest_yaw_.store(true);
      //清空ik与滤波器状态
      arm_ik->resetLastSolution();
      q_last = arm_ik->defaultPositions();
      q_filtered = q_last;
    }

    // 2c. 跟随门控：仅 External 模式下求解/下发。
    const bool following =
        current_arm_mode.load() == static_cast<int>(leju::vr::ControlMode::kExternal);
    if (!following) {
      if (prev_following) {
        arm_ik->resetLastSolution();
        q_last = arm_ik->defaultPositions();
        q_filtered = q_last;
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

    if (!mapped || !bone_fresh) {
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

    // 4b. 对 q_sol 施加一阶低通滤波（
    if (mask_filter.load()) {
      // 首次解锁时直接用 IK 解算结果初始化滤波器，避免滤波器初始值为默认位姿导致抖动
      mask_filter.store(false);
    } else {
      auto q_filtered_cache = arm_filter_alpha * (q_sol - q_filtered) + q_filtered;
      // 输出死区：截断滤波残余小值，防止静止时缓慢漂移（默认 0 不启用）
      if (arm_filter_output_deadzone > 0.0 && (q_sol-q_filtered).norm() < arm_filter_output_deadzone) {
        q_sol = q_filtered;
      }else{
        q_sol = q_filtered_cache;
        q_filtered = q_filtered_cache;
      }
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
  // 仅 X+Y 主动退出才 StopRobot
  if (g_stop_robot_requested.load()) {
    leju::GlobalRobot::getInstance().shutdown();
  }
  return 0;
}
